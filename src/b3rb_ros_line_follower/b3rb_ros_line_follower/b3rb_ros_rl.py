"""
RL runner node — REINFORCE residual steering on top of the baseline controller.

Modes
-----
training=true  (Gazebo only)
    Runs episodic REINFORCE training.  At the end of each episode the policy
    gradient is applied, weights are saved to disk, and the robot is teleported
    back to its spawn pose via the gz service CLI.

training=false  (Gazebo + hardware)
    Loads saved weights and runs the policy in deterministic inference mode.
    No trajectory collection, no gradient updates, no Gazebo reset calls.

Topics
------
  Sub  /nxp_cup/lane_chains  (std_msgs/String)     — vision pipeline JSON
  Sub  /nxp_cup/wheel_odom   (nav_msgs/Odometry)   — remap to /cerebri/out/odometry in Gazebo
  Sub  /traffic_status       (synapse_msgs/TrafficStatus)
  Pub  /nxp_cup/cmd_safe     (geometry_msgs/TwistStamped)

Parameters
----------
  training       bool    True    — training vs deployment mode
  delta_max      float   0.30    — maximum RL steering correction (fraction of [-1,1] range)
  gamma          float   0.99    — discount factor
  lr             float   1e-3    — Adam learning rate
  no_lane_limit  int     20      — consecutive no-measurement frames to declare episode done
  max_steps      int     2000    — hard step cap per episode (fallback termination)
  settle_steps   int     30      — frames to hold zero after reset before collecting
  world_name     str   "default" — gz world name for the set_pose service
  weights_path   str  "~/rl_policy.pt"
  spawn_x/y/z    float           — robot spawn pose used by gz reset
"""

import os
import subprocess
from enum import Enum, auto

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from synapse_msgs.msg import TrafficStatus

from .controllers.baseline_lane_controller import BaselineLaneController
from .estimation.vehicle_state_estimator import VehicleStateEstimator
from .perception.path_measurements import PathMeasurementExtractor
from .rl.episode_manager import REINFORCETrainer
from .rl.policy import GaussianPolicy

QOS_PROFILE_DEFAULT = 10

# ---------------------------------------------------------------------- #
# State normalisation                                                      #
#   ye_cam_filt      ∈ [-1,  1]       (already normalised by BEV width)  #
#   psi_rel_cam_filt ∈ [-1,  1]       (already normalised by BEV width)  #
#   vx_recon         ∈ [ 0, ~0.6] m/s  →  ×2 brings it to [0, ~1.2]    #
#   a_long_filt      ∈ [-2,  2]  m/s²  →  ×0.5 brings it to [-1, 1]    #
# ---------------------------------------------------------------------- #
_S_SCALE = np.array([1.0, 1.0, 2.0, 0.5], dtype=np.float32)


class _EpState(Enum):
    COLLECTING = auto()
    SETTLING   = auto()


class RLRunner(Node):

    def __init__(self) -> None:
        super().__init__("rl_runner")

        # ---- declare & read parameters -------------------------------- #
        self.declare_parameter("training",      True)
        self.declare_parameter("delta_max",     0.30)
        self.declare_parameter("gamma",         0.99)
        self.declare_parameter("lr",            1e-3)
        self.declare_parameter("no_lane_limit", 20)
        self.declare_parameter("max_steps",     2000)
        self.declare_parameter("settle_steps",  30)
        self.declare_parameter("world_name",    "default")
        self.declare_parameter("weights_path",  "~/rl_policy.pt")
        self.declare_parameter("spawn_x",       -5.0)
        self.declare_parameter("spawn_y",       -2.0)
        self.declare_parameter("spawn_z",        0.05)

        self._training        = self.get_parameter("training").value
        delta_max             = float(self.get_parameter("delta_max").value)
        gamma                 = float(self.get_parameter("gamma").value)
        lr                    = float(self.get_parameter("lr").value)
        self._no_lane_limit   = int(self.get_parameter("no_lane_limit").value)
        self._max_steps       = int(self.get_parameter("max_steps").value)
        self._settle_steps    = int(self.get_parameter("settle_steps").value)
        self._world_name      = str(self.get_parameter("world_name").value)
        self._weights_path    = os.path.expanduser(
            str(self.get_parameter("weights_path").value)
        )
        self._spawn_x = float(self.get_parameter("spawn_x").value)
        self._spawn_y = float(self.get_parameter("spawn_y").value)
        self._spawn_z = float(self.get_parameter("spawn_z").value)

        # ---- subscriptions & publisher -------------------------------- #
        self.create_subscription(
            String, "/nxp_cup/lane_chains", self._lane_chains_cb, QOS_PROFILE_DEFAULT
        )
        self.create_subscription(
            Odometry, "/nxp_cup/wheel_odom", self._odom_cb, QOS_PROFILE_DEFAULT
        )
        self.create_subscription(
            TrafficStatus, "/traffic_status", self._traffic_cb, QOS_PROFILE_DEFAULT
        )
        self._pub_cmd = self.create_publisher(
            TwistStamped, "/nxp_cup/cmd_safe", QOS_PROFILE_DEFAULT
        )

        # ---- per-episode stateful components -------------------------- #
        # These are replaced wholesale on each episode reset so all filter
        # state and integral state starts clean without needing reset methods.
        self._extractor = PathMeasurementExtractor()
        self._baseline  = BaselineLaneController()
        self._estimator = VehicleStateEstimator()
        self._traffic   = TrafficStatus()

        # ---- policy --------------------------------------------------- #
        self._policy = GaussianPolicy(state_dim=4, hidden_dim=16, delta_max=delta_max)

        if self._training:
            self._trainer: REINFORCETrainer | None = REINFORCETrainer(
                self._policy, lr=lr, gamma=gamma
            )
        else:
            self._trainer = None

        if os.path.exists(self._weights_path):
            self._policy.load(self._weights_path)
            self.get_logger().info(f"[RL] loaded weights from {self._weights_path}")
        elif not self._training:
            self.get_logger().warn(
                f"[RL] no weights at {self._weights_path} — policy is uninitialised"
            )

        # ---- episode bookkeeping -------------------------------------- #
        self._ep_state    = _EpState.COLLECTING
        self._no_lane_cnt = 0
        self._step_cnt    = 0
        self._settle_cnt  = 0
        self._episode_num = 0

        mode = "TRAINING" if self._training else "DEPLOYMENT"
        self.get_logger().info(
            f"[RL] runner started — mode={mode}  delta_max={delta_max}"
        )

    # ------------------------------------------------------------------ #
    # ROS callbacks                                                        #
    # ------------------------------------------------------------------ #

    def _odom_cb(self, msg: Odometry) -> None:
        self._estimator.update_from_odometry(msg)

    def _traffic_cb(self, msg: TrafficStatus) -> None:
        self._traffic = msg

    def _lane_chains_cb(self, msg: String) -> None:
        camera  = self._extractor.extract_from_json(msg.data)
        vehicle = self._estimator.vehicle

        # ---- SETTLING: hold zero, wait for simulator state to stabilise ---- #
        if self._ep_state == _EpState.SETTLING:
            self._publish(0.0, 0.0)
            self._settle_cnt += 1
            if self._settle_cnt >= self._settle_steps:
                self._ep_state   = _EpState.COLLECTING
                self._settle_cnt = 0
                self.get_logger().info(
                    f"[RL] episode {self._episode_num} started"
                )
            return

        # ---- COLLECTING ---------------------------------------------------- #
        vx = float(vehicle.vx_recon)   if vehicle.odom_ready else 0.0
        ax = float(vehicle.a_long_filt) if vehicle.odom_ready else 0.0

        state = _build_state(camera, vx, ax)

        # Baseline steering + speed
        stop = bool(getattr(self._traffic, "stop_sign", False))
        turn_wp, speed_wp, _ = self._baseline.compute(
            camera=camera,
            stop_sign=stop,
            obstacle_detected=False,
            ramp_detected=False,
        )

        # RL residual action
        if self._training:
            delta_rl, log_prob = self._policy.sample(state)
        else:
            delta_rl  = self._policy.act(state)
            log_prob  = None

        turn_cmd = float(np.clip(turn_wp + delta_rl, -1.0, 1.0))
        self._publish(speed_wp, turn_cmd)

        # ---- training bookkeeping -------------------------------------- #
        if self._training:
            done, terminal_r = self._check_done(camera)
            reward = terminal_r if done else _reward(camera, vx, delta_rl)
            self._trainer.store(log_prob, reward)   # type: ignore[arg-type]
            self._step_cnt += 1

            if done:
                self._end_episode()

    # ------------------------------------------------------------------ #
    # Episode management                                                   #
    # ------------------------------------------------------------------ #

    def _check_done(self, camera) -> tuple:
        """Returns (done: bool, terminal_reward: float)."""
        if not camera.have_measurement:
            self._no_lane_cnt += 1
        else:
            self._no_lane_cnt = 0

        if self._no_lane_cnt >= self._no_lane_limit:
            return True, -1.0          # track departure penalty

        if self._step_cnt >= self._max_steps:
            return True, +1.0          # survived max steps — small bonus

        return False, 0.0

    def _end_episode(self) -> None:
        stats = self._trainer.update()   # type: ignore[union-attr]
        self._policy.save(self._weights_path)
        self.get_logger().info(
            f"[RL] ep={stats.episode}  steps={stats.steps}"
            f"  R={stats.total_reward:.2f}  G0={stats.return_0:.2f}"
            f"  loss={stats.loss:.4f}"
        )
        self._reset_episode()

    def _reset_episode(self) -> None:
        """Flush all episode-local state and re-enter SETTLING."""
        # Replace stateful objects entirely — cleanest way to flush filters
        self._extractor   = PathMeasurementExtractor()
        self._baseline    = BaselineLaneController()
        self._estimator   = VehicleStateEstimator()
        self._no_lane_cnt = 0
        self._step_cnt    = 0
        self._episode_num += 1
        self._ep_state    = _EpState.SETTLING
        self._settle_cnt  = 0

        if self._training:
            self._gz_reset()

    def _gz_reset(self) -> None:
        """
        Teleport the robot to spawn pose using the gz service CLI.

        The set_pose service is exposed by gz-sim 8 (Harmonic) at:
            /world/{world_name}/set_pose
        with request type gz.msgs.Pose and reply type gz.msgs.Boolean.

        Run  `gz service --list`  inside the simulation to confirm the exact
        service name if the world name differs from 'default'.

        The call is non-blocking (Popen) — the SETTLING phase gives >3 s of
        buffer time for the service to complete before data collection resumes.
        """
        req = (
            f'name: "b3rb" '
            f'position {{ x: {self._spawn_x} y: {self._spawn_y} z: {self._spawn_z} }} '
            f'orientation {{ x: 0.0 y: 0.0 z: 0.0 w: 1.0 }}'
        )
        cmd = [
            "gz", "service",
            "-s", f"/world/{self._world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", req,
        ]
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.get_logger().error(
                '[RL] "gz" binary not found — episode reset will not teleport the robot'
            )

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _publish(self, speed: float, turn: float) -> None:
        msg = TwistStamped()
        msg.twist.linear.x  = float(speed)
        msg.twist.angular.z = float(turn)
        self._pub_cmd.publish(msg)


# ---------------------------------------------------------------------- #
# Module-level pure functions (no self needed)                            #
# ---------------------------------------------------------------------- #

def _build_state(camera, vx: float, ax: float) -> np.ndarray:
    """Pack the 4-dim RL state vector and normalise to ~[-1, 1]."""
    ye      = camera.ye_cam_filt      if camera.have_measurement else 0.0
    psi_rel = camera.psi_rel_cam_filt if camera.have_measurement else 0.0
    return np.array([ye, psi_rel, vx, ax], dtype=np.float32) * _S_SCALE


def _reward(camera, vx: float, delta_rl: float) -> float:
    """
    Step reward (report eq. 66):
        β_v · vx · cos(θ_e)  — forward progress aligned with lane direction
      − β_e · |e_y|           — cross-track deviation penalty
      − β_δ · |δ_rl|          — action regularisation (intervene only when needed)
    """
    if not camera.have_measurement:
        return -1.0
    theta_e = camera.psi_rel_cam_filt
    e_y     = camera.ye_cam_filt
    return float(
        1.0  * vx * float(np.cos(theta_e))
        - 0.3 * abs(e_y)
        - 0.05 * abs(delta_rl)
    )


# ---------------------------------------------------------------------- #

def main(args=None) -> None:
    rclpy.init(args=args)
    node = RLRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

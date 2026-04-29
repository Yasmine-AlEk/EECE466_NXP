"""
RL runner node — REINFORCE residual steering on top of the MRAC controller.

Architecture
------------
Baseline PID  →  MRAC inner steering  →  +δ_rl (RL residual)  →  /nxp_cup/cmd_safe

The baseline PID is still executed every step because the MRAC adaptive law
requires the PID turn command as 'baseline_turn_cmd' input.  The RL residual
corrects the MRAC output, not the raw PID output.

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
import torch
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from synapse_msgs.msg import TrafficStatus

from . import mrac_config as cfg
from .controllers.baseline_lane_controller import BaselineLaneController
from .estimation.vehicle_state_estimator import VehicleStateEstimator
from .models.inner_lateral_yaw_reduced import InnerLateralYawReducedModel
from .models.inner_lateral_yaw_reference import InnerLateralYawReferenceModel
from .models.inner_reference_command import InnerReferenceCommandModel
from .models.inner_steering_mrac_controller import InnerSteeringMRACController
from .models.speed_scheduler import FilteredSpeedScheduler
from .mrac_utils import turn_cmd_to_delta_f_est
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


def _make_mrac_components():
    """Instantiate the MRAC inner steering chain from mrac_config constants."""
    speed_scheduler = FilteredSpeedScheduler(
        tau_s=cfg.INNER_SPEED_SCHEDULER_TAU_S,
        min_valid_vx_ms=cfg.INNER_LATERAL_YAW_MIN_VX_MS,
        max_dt_s=cfg.INNER_SPEED_SCHEDULER_MAX_DT_S,
    )
    lat_yaw_model = InnerLateralYawReducedModel(
        mass_kg=cfg.DYNAMIC_BICYCLE_MASS_KG,
        iz_kg_m2=cfg.DYNAMIC_BICYCLE_IZ_KG_M2,
        lf_m=cfg.DYNAMIC_BICYCLE_LF_M,
        lr_m=cfg.DYNAMIC_BICYCLE_LR_M,
        rear_track_width_m=cfg.REAR_TRACK_WIDTH_M,
        c_alpha_f_n_per_rad=cfg.FRONT_CORNERING_STIFFNESS_N_PER_RAD,
        c_alpha_r_n_per_rad=cfg.REAR_CORNERING_STIFFNESS_N_PER_RAD,
        min_vx_ms=cfg.INNER_LATERAL_YAW_MIN_VX_MS,
    )
    ref_cmd_model = InnerReferenceCommandModel(
        kappa_from_ye_gain_1pm=cfg.INNER_REF_KAPPA_FROM_YE_GAIN_1PM,
        kappa_from_psi_gain_1pm=cfg.INNER_REF_KAPPA_FROM_PSI_GAIN_1PM,
        kappa_sign=cfg.INNER_REF_KAPPA_SIGN,
        max_abs_kappa_1pm=cfg.INNER_REF_MAX_ABS_KAPPA_1PM,
        max_abs_r_ref_rad_s=cfg.INNER_REF_MAX_ABS_R_REF_RAD_S,
        min_vx_ms=cfg.INNER_REF_MIN_VX_MS,
    )
    lat_yaw_ref_model = InnerLateralYawReferenceModel(
        a_vy_s_inv=cfg.INNER_REFERENCE_MODEL_A_VY_S_INV,
        a_r_s_inv=cfg.INNER_REFERENCE_MODEL_A_R_S_INV,
        max_abs_uc_rad_s=cfg.INNER_REFERENCE_MODEL_MAX_ABS_UC_RAD_S,
        max_abs_vy_m_ms=cfg.INNER_REFERENCE_MODEL_MAX_ABS_VY_M_MS,
        max_abs_r_m_rad_s=cfg.INNER_REFERENCE_MODEL_MAX_ABS_R_M_RAD_S,
        max_dt_s=cfg.INNER_REFERENCE_MODEL_MAX_DT_S,
    )
    steering_mrac = InnerSteeringMRACController(
        theta_vy_initial=cfg.INNER_MRAC_THETA_VY_INITIAL,
        theta_r_initial=cfg.INNER_MRAC_THETA_R_INITIAL,
        theta_uc_initial=cfg.INNER_MRAC_THETA_UC_INITIAL,
        gamma_vy=cfg.INNER_MRAC_GAMMA_VY,
        gamma_r=cfg.INNER_MRAC_GAMMA_R,
        gamma_uc=cfg.INNER_MRAC_GAMMA_UC,
        sigma=cfg.INNER_MRAC_SIGMA,
        theta_min=cfg.INNER_MRAC_THETA_MIN,
        theta_max=cfg.INNER_MRAC_THETA_MAX,
        max_abs_theta_dot=cfg.INNER_MRAC_MAX_ABS_THETA_DOT,
        min_vx_ms=cfg.INNER_MRAC_MIN_VX_MS,
        min_phi_norm=cfg.INNER_MRAC_MIN_PHI_NORM,
        max_abs_tracking_error=cfg.INNER_MRAC_MAX_ABS_TRACKING_ERROR,
        max_dt_s=cfg.INNER_MRAC_MAX_DT_S,
        max_delta_rad=cfg.INNER_MRAC_MAX_DELTA_RAD,
        max_applied_delta_rad=cfg.INNER_MRAC_MAX_APPLIED_DELTA_RAD,
        max_delta_disagreement_rad=cfg.INNER_MRAC_MAX_DELTA_DISAGREEMENT_RAD,
        steering_rad_per_turn_cmd=cfg.STEERING_SERVO_MAX_STEER_RAD,
        max_turn_cmd=cfg.INNER_MRAC_MAX_TURN_CMD,
        error_vy_weight=cfg.INNER_MRAC_ERROR_VY_WEIGHT,
        enable_adaptation=cfg.INNER_MRAC_ENABLE_ADAPTATION,
        apply_to_steering_cmd=cfg.INNER_MRAC_APPLY_TO_STEERING_CMD,
        blend=cfg.INNER_MRAC_BLEND,
        lyapunov_q_vy=getattr(cfg, "INNER_MRAC_Q_VY", 1.0),
        lyapunov_q_r=getattr(cfg, "INNER_MRAC_Q_R", 1.0),
        reference_a_vy_s_inv=getattr(cfg, "INNER_MRAC_P_A_VY_S_INV",
                                     cfg.INNER_REFERENCE_MODEL_A_VY_S_INV),
        reference_a_r_s_inv=getattr(cfg, "INNER_MRAC_P_A_R_S_INV",
                                    cfg.INNER_REFERENCE_MODEL_A_R_S_INV),
        b_delta_vy=(cfg.FRONT_CORNERING_STIFFNESS_N_PER_RAD
                    / cfg.DYNAMIC_BICYCLE_MASS_KG),
        b_delta_r=(cfg.DYNAMIC_BICYCLE_LF_M
                   * cfg.FRONT_CORNERING_STIFFNESS_N_PER_RAD
                   / cfg.DYNAMIC_BICYCLE_IZ_KG_M2),
    )
    return speed_scheduler, lat_yaw_model, ref_cmd_model, lat_yaw_ref_model, steering_mrac


class RLRunner(Node):

    def __init__(self) -> None:
        super().__init__("rl_runner")

        # ---- declare & read parameters -------------------------------- #
        self.declare_parameter("training",      True)
        self.declare_parameter("device",        "auto")
        self.declare_parameter("delta_max",     0.30)
        self.declare_parameter("gamma",         0.99)
        self.declare_parameter("lr",            1e-3)
        self.declare_parameter("no_lane_limit", 20)
        self.declare_parameter("max_steps",     4000)
        self.declare_parameter("settle_steps",  30)
        self.declare_parameter("world_name",    "default")
        self.declare_parameter("weights_path",  "~/rl_policy.pt")
        self.declare_parameter("spawn_x",       -5.0)
        self.declare_parameter("spawn_y",       -2.0)
        self.declare_parameter("spawn_z",        0.05)

        self._training        = self.get_parameter("training").value
        requested_device      = str(self.get_parameter("device").value)

        if requested_device == "auto":
            requested_device = "cuda" if torch.cuda.is_available() else "cpu"

        if requested_device.startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warn(
                "[RL] CUDA requested but torch.cuda.is_available() is False; using CPU"
            )
            requested_device = "cpu"

        self._device          = torch.device(requested_device)
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
        # All replaced wholesale on each episode reset — no reset() methods needed.
        self._extractor = PathMeasurementExtractor()
        self._baseline  = BaselineLaneController()
        self._estimator = VehicleStateEstimator()
        self._traffic   = TrafficStatus()
        (
            self._speed_scheduler,
            self._lat_yaw_model,
            self._ref_cmd_model,
            self._lat_yaw_ref_model,
            self._steering_mrac,
        ) = _make_mrac_components()

        # ---- policy --------------------------------------------------- #
        self._policy = GaussianPolicy(state_dim=4, hidden_dim=16, delta_max=delta_max, device=str(self._device))

        if self._training:
            self._trainer: REINFORCETrainer | None = REINFORCETrainer(
                self._policy, lr=lr, gamma=gamma
            )
        else:
            self._trainer = None

        if os.path.exists(self._weights_path):
            self._policy.load(self._weights_path)
            if self._training:
                self._policy.train()
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
            f"[RL] runner started — mode={mode}  delta_max={delta_max}  device={self._device}"
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
        vx = float(vehicle.vx_recon)    if vehicle.odom_ready else 0.0
        ax = float(vehicle.a_long_filt) if vehicle.odom_ready else 0.0

        state = _build_state(camera, vx, ax)

        # 1. Baseline PID — provides speed and the turn input the MRAC
        #    adaptive law needs as baseline_turn_cmd.
        stop = bool(getattr(self._traffic, "stop_sign", False))
        turn_wp, speed_wp, dt_s = self._baseline.compute(
            camera=camera,
            stop_sign=stop,
            obstacle_detected=False,
            ramp_detected=False,
        )

        # 2. MRAC inner steering on top of baseline.
        mrac_turn = self._compute_mrac_turn(camera, vehicle, turn_wp, dt_s)

        # 3. RL residual correction on top of MRAC.
        if self._training:
            delta_rl, log_prob = self._policy.sample(state)
        else:
            delta_rl  = self._policy.act(state)
            log_prob  = None

        turn_cmd = float(np.clip(mrac_turn + delta_rl, -1.0, 1.0))
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
    # MRAC inner steering                                                  #
    # ------------------------------------------------------------------ #

    def _compute_mrac_turn(
        self, camera, vehicle, baseline_turn: float, dt_s: float
    ) -> float:
        """
        Run the MRAC inner steering chain and return the final turn command.

        Chain:
            FilteredSpeedScheduler
            → InnerLateralYawReducedModel    (lateral/yaw plant)
            → InnerReferenceCommandModel     (desired yaw rate from camera error)
            → InnerLateralYawReferenceModel  (reference model dynamics)
            → InnerSteeringMRACController    (adaptive law → turn_sat_cmd)

        Falls back to baseline_turn if MRAC has not yet activated (e.g. speed
        below min_vx_ms or first frame before odometry is ready).
        """
        vx = float(vehicle.vx_recon) if vehicle.odom_ready else 0.0
        vy = float(vehicle.vy_recon) if vehicle.odom_ready else 0.0
        r  = float(vehicle.r_recon)  if vehicle.odom_ready else 0.0

        baseline_delta_rad = turn_cmd_to_delta_f_est(baseline_turn)

        speed_sched = self._speed_scheduler.update(raw_vx_ms=vx, dt_s=dt_s)

        lat_yaw = self._lat_yaw_model.build(
            vy_ms=vy,
            r_rad_s=r,
            vx0_ms=speed_sched.vx0_ms,
            delta_f_rad=baseline_delta_rad,
            dfx_n=0.0,          # no torque vectoring in the RL runner
        )

        ref_cmd = self._ref_cmd_model.build(
            vx0_ms=lat_yaw.vx0_safe_ms,
            have_camera_measurement=camera.have_measurement,
            ye_cam_filt=camera.ye_cam_filt,
            psi_rel_cam_filt=camera.psi_rel_cam_filt,
        )

        uc    = float(getattr(ref_cmd, "uc_rad_s", 0.0)) if ref_cmd is not None else 0.0
        valid = bool(getattr(ref_cmd,  "valid",    False)) if ref_cmd is not None else False

        lat_yaw_ref = self._lat_yaw_ref_model.step(
            uc_rad_s=uc, dt_s=dt_s, command_valid=valid
        )

        inner_mrac = self._steering_mrac.update(
            vx_ms=vx,
            vy_ms=vy,
            r_rad_s=r,
            reference_command=ref_cmd,
            reference_model=lat_yaw_ref,
            baseline_delta_rad=baseline_delta_rad,
            baseline_turn_cmd=baseline_turn,
            dt_s=dt_s,
        )

        return _get_first_float(
            inner_mrac,
            ["turn_sat_cmd", "turn_final_cmd", "turn_raw_cmd"],
            default=baseline_turn,
        )

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
        self._extractor = PathMeasurementExtractor()
        self._baseline  = BaselineLaneController()
        self._estimator = VehicleStateEstimator()
        (
            self._speed_scheduler,
            self._lat_yaw_model,
            self._ref_cmd_model,
            self._lat_yaw_ref_model,
            self._steering_mrac,
        ) = _make_mrac_components()
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

def _get_first_float(obj, names: list, default: float = 0.0) -> float:
    """Return the first matching attribute from obj, or default."""
    if obj is None:
        return float(default)
    for name in names:
        if hasattr(obj, name):
            return float(getattr(obj, name))
    return float(default)


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

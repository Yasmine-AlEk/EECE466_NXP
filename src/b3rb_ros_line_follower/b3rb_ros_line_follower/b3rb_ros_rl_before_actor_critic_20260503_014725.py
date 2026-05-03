"""
RL runner node — REINFORCE residual steering on top of the MRAC controller.

Architecture
------------
Baseline PID  →  MRAC inner steering  →  +δ_rl (RL residual)  →  /nxp_cup/cmd_safe

Modes
-----
training=true  (Gazebo only)
    Runs episodic REINFORCE training.

training=false  (Gazebo + hardware)
    Deterministic inference mode.

Topics  (mirrors runner_mrac — no bridge nodes required)
------
  Sub  /edge_vectors          (synapse_msgs/EdgeVectors)   — vision pipeline (FIXED)
  Sub  /cerebri/out/odometry  (nav_msgs/Odometry)          — vehicle state   (FIXED)
  Sub  /traffic_status        (synapse_msgs/TrafficStatus)
  Pub  /nxp_cup/cmd_safe      (geometry_msgs/TwistStamped)

Parameters
----------
  training       bool    True
  delta_max      float   0.30    — max steering correction
  speed_up       float   0.25    — max speed increase the RL can add (m/s)
  speed_dn       float   0.05    — max speed decrease the RL can apply (m/s)
  gamma          float   0.99
  lr             float   1e-3
  no_lane_limit  int     20
  max_steps      int     4000
  settle_steps   int     30
  world_name     str     "default"
  weights_path   str     "~/rl_policy.pt"
  spawn_x/y/z    float
  spawn_yaw      float   0.0  — radians, converted to quaternion for gz set_pose
"""

import math
import os
import subprocess
from enum import Enum, auto

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from synapse_msgs.msg import EdgeVectors, TrafficStatus

from . import mrac_config as cfg
from .controllers.baseline_lane_controller import BaselineLaneController
from .estimation.vehicle_state_estimator import VehicleStateEstimator
from .models.inner_lateral_yaw_reduced import InnerLateralYawReducedModel
from .models.inner_lateral_yaw_reference import InnerLateralYawReferenceModel
from .models.inner_reference_command import InnerReferenceCommandModel
from .models.inner_steering_mrac_controller import InnerSteeringMRACController
from .models.speed_scheduler import FilteredSpeedScheduler
from .mrac_utils import turn_cmd_to_delta_f_est
from .perception.camera_measurements import CameraMeasurementExtractor   # FIX: use same extractor as MRAC
from .rl.episode_manager import REINFORCETrainer
from .rl.policy import GaussianPolicy

QOS_PROFILE_DEFAULT = 10

# State scaling: ye, psi_rel already in [-1,1]; vx*2 and ax*0.5 normalise to ~[-1,1]
_S_SCALE = np.array([1.0, 1.0, 2.0, 0.5], dtype=np.float32)


class _EpState(Enum):
    COLLECTING = auto()
    SETTLING   = auto()


def _make_mrac_components():
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
        b_delta_vy=(cfg.FRONT_CORNERING_STIFFNESS_N_PER_RAD / cfg.DYNAMIC_BICYCLE_MASS_KG),
        b_delta_r=(cfg.DYNAMIC_BICYCLE_LF_M * cfg.FRONT_CORNERING_STIFFNESS_N_PER_RAD
                   / cfg.DYNAMIC_BICYCLE_IZ_KG_M2),
    )
    return speed_scheduler, lat_yaw_model, ref_cmd_model, lat_yaw_ref_model, steering_mrac


class RLRunner(Node):

    def __init__(self) -> None:
        super().__init__("rl_runner")

        self.declare_parameter("training",      True)
        self.declare_parameter("delta_max",     0.30)
        self.declare_parameter("speed_up",      0.25)
        self.declare_parameter("speed_dn",      0.05)
        self.declare_parameter("gamma",         0.99)
        self.declare_parameter("lr",            3e-4)
        self.declare_parameter("no_lane_limit", 20)
        self.declare_parameter("max_steps",     2000)
        self.declare_parameter("settle_steps",  80)
        self.declare_parameter("world_name",    "default")
        self.declare_parameter("weights_path",  "~/rl_policy.pt")
        self.declare_parameter("spawn_x",       -5.0)
        self.declare_parameter("spawn_y",       -2.0)
        self.declare_parameter("spawn_z",        0.05)
        self.declare_parameter("spawn_yaw",      0.0)
        self.declare_parameter("device",         "cpu")
        self.declare_parameter("debug_every_n",  50)

        # REINFORCE-MC reward/benchmark parameters
        self.declare_parameter("ema_alpha",                0.10)
        self.declare_parameter("mrac_avg_speed_reference", 0.0)
        self.declare_parameter("terminal_speed_gain",      2000.0)
        self.declare_parameter("track_exit_penalty",       800.0)

        self._training      = self.get_parameter("training").value
        delta_max           = float(self.get_parameter("delta_max").value)
        speed_up            = float(self.get_parameter("speed_up").value)
        speed_dn            = float(self.get_parameter("speed_dn").value)
        self._delta_max     = delta_max
        self._speed_up      = speed_up
        self._speed_dn      = speed_dn
        gamma               = float(self.get_parameter("gamma").value)
        lr                  = float(self.get_parameter("lr").value)
        self._no_lane_limit = int(self.get_parameter("no_lane_limit").value)
        self._max_steps     = int(self.get_parameter("max_steps").value)
        self._settle_steps  = int(self.get_parameter("settle_steps").value)
        self._world_name    = str(self.get_parameter("world_name").value)
        self._weights_path  = os.path.expanduser(
            str(self.get_parameter("weights_path").value))
        self._spawn_x   = float(self.get_parameter("spawn_x").value)
        self._spawn_y   = float(self.get_parameter("spawn_y").value)
        self._spawn_z   = float(self.get_parameter("spawn_z").value)
        self._spawn_yaw = float(self.get_parameter("spawn_yaw").value)

        self._debug_every_n = int(self.get_parameter("debug_every_n").value)
        self._ema_alpha = float(self.get_parameter("ema_alpha").value)
        self._mrac_avg_speed_reference = float(
            self.get_parameter("mrac_avg_speed_reference").value
        )
        self._terminal_speed_gain = float(
            self.get_parameter("terminal_speed_gain").value
        )
        self._track_exit_penalty = float(
            self.get_parameter("track_exit_penalty").value
        )

        device_req = str(self.get_parameter("device").value).strip().lower()
        if device_req.startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warn(
                "[RL] device=cuda requested but CUDA is not available; falling back to CPU"
            )
            self._device = "cpu"
        else:
            self._device = device_req

        # ------------------------------------------------------------------ #
        # FIX 1: subscribe to /edge_vectors directly (same as runner_mrac).
        #         The old /nxp_cup/lane_chains subscription required the
        #         lane_bridge node to be running; if it wasn't, camera.have_measurement
        #         was always False and the car had no steering at all.
        # ------------------------------------------------------------------ #
        self.create_subscription(
            EdgeVectors, "/edge_vectors", self._edge_vectors_cb, QOS_PROFILE_DEFAULT
        )
        # ------------------------------------------------------------------ #
        # FIX 2: subscribe to /cerebri/out/odometry (same as runner_mrac).
        #         The old /nxp_cup/wheel_odom topic is not published by Gazebo,
        #         so vehicle.odom_ready was always False → vx always 0 →
        #         MRAC never activated → car went straight off every turn.
        # ------------------------------------------------------------------ #
        self.create_subscription(
            Odometry, "/cerebri/out/odometry", self._odom_cb, QOS_PROFILE_DEFAULT
        )
        self.create_subscription(
            TrafficStatus, "/traffic_status", self._traffic_cb, QOS_PROFILE_DEFAULT
        )
        self._pub_cmd = self.create_publisher(
            TwistStamped, "/nxp_cup/cmd_safe", QOS_PROFILE_DEFAULT
        )

        self._half_width: float = 200.0   # updated from first EdgeVectors message

        # Per-episode components — replaced wholesale on reset
        self._extractor = CameraMeasurementExtractor()   # FIX 3: use CameraMeasurementExtractor
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

        # Policy — hidden_dim 32 gives enough capacity without overfitting
        self._policy = GaussianPolicy(
            state_dim=4, hidden_dim=64,
            delta_max=delta_max, speed_up=speed_up, speed_dn=speed_dn,
            device=self._device,
        )

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

        self._ep_state    = _EpState.COLLECTING
        self._no_lane_cnt = 0
        self._step_cnt    = 0
        self._settle_cnt  = 0
        self._episode_num = 0
        self._debug_cnt   = 0

        # Episode-level metrics used for terminal reward.
        self._ep_vx_sum = 0.0
        self._ep_count = 0
        self._ep_distance = 0.0
        self._ep_lane_loss_steps = 0

        # Successful-speed benchmark.
        # This is intentionally only updated by successful max_steps episodes.
        self._success_speed_ema = None
        self._best_success_avg_speed = 0.0

        # Last episode summary values for logging.
        self._last_done_reason = "none"
        self._last_avg_speed = 0.0
        self._last_speed_benchmark = None
        self._last_speed_improvement = 0.0
        self._last_distance = 0.0
        self._last_lane_loss_steps = 0

        mode = "TRAINING" if self._training else "DEPLOYMENT"
        self.get_logger().info(
            f"[RL] runner started — mode={mode}  device={self._device}"
            f"  delta_max={delta_max:.3f}  speed_up={speed_up:.3f}  speed_dn={speed_dn:.3f}"
            f"  lr={lr:.1e}  gamma={gamma:.3f}"
            f"  mrac_avg_speed_ref={self._mrac_avg_speed_reference:.3f}"
        )

    # ------------------------------------------------------------------ #
    # ROS callbacks                                                        #
    # ------------------------------------------------------------------ #

    def _odom_cb(self, msg: Odometry) -> None:
        self._estimator.update_from_odometry(msg)

    def _traffic_cb(self, msg: TrafficStatus) -> None:
        self._traffic = msg

    def _edge_vectors_cb(self, msg: EdgeVectors) -> None:
        # Mirror runner_mrac: derive half_width from message header
        if msg.image_width > 0:
            self._half_width = float(msg.image_width) / 2.0

        camera  = self._extractor.extract(msg, self._half_width)
        vehicle = self._estimator.vehicle

        # ---- SETTLING phase ---- #
        if self._ep_state == _EpState.SETTLING:
            self._publish(0.0, 0.0)
            self._settle_cnt += 1
            if self._settle_cnt >= self._settle_steps:
                self._ep_state   = _EpState.COLLECTING
                self._settle_cnt = 0
                self.get_logger().info(f"[RL] episode {self._episode_num} started")
            return

        # ---- COLLECTING phase ---- #
        vx = float(vehicle.vx_recon)    if vehicle.odom_ready else 0.0
        ax = float(vehicle.a_long_filt) if vehicle.odom_ready else 0.0

        state = _build_state(camera, vx, ax)

        stop = bool(getattr(self._traffic, "stop_sign", False))
        turn_wp, speed_wp, dt_s = self._baseline.compute(
            camera=camera,
            stop_sign=stop,
            obstacle_detected=False,
            ramp_detected=False,
        )

        mrac_turn = self._compute_mrac_turn(camera, vehicle, turn_wp, dt_s)

        if self._training:
            (delta_steer, delta_speed), log_prob = self._policy.sample(state)
        else:
            delta_steer, delta_speed = self._policy.act(state)
            log_prob = None

        turn_cmd  = float(np.clip(mrac_turn + delta_steer, -1.0, 1.0))
        # RL speed residual: only applied when lane is visible.
        speed_cmd = float(speed_wp + delta_speed) if camera.have_measurement else float(speed_wp)
        self._publish(speed_cmd, turn_cmd)

        self._debug_cnt += 1
        reward = float("nan")
        done = False

        if self._training:
            self._accumulate_episode_stats(
                vx=vx,
                dt_s=dt_s,
                lane_visible=bool(camera.have_measurement),
            )

            done, terminal_r = self._check_done(camera)
            step_reward = _reward(
                camera=camera,
                vx=vx,
                delta_speed=float(delta_speed),
                speed_baseline=float(speed_wp),
                delta_steer=float(delta_steer),
            )

            # If the episode ends, terminal reward replaces the final step reward.
            reward = terminal_r if done else step_reward
            self._trainer.store(log_prob, reward)   # type: ignore[arg-type]
            self._step_cnt += 1

        if self._debug_every_n > 0 and (self._debug_cnt % self._debug_every_n == 0):
            ye_dbg = float(camera.ye_cam_filt) if camera.have_measurement else 0.0
            psi_dbg = float(camera.psi_rel_cam_filt) if camera.have_measurement else 0.0
            self.get_logger().info(
                f"[RL dbg] step={self._step_cnt} lane={int(camera.have_measurement)}"
                f" ye={ye_dbg:.3f} psi={psi_dbg:.3f} vx={vx:.3f} ax={ax:.3f}"
                f" speed_base={float(speed_wp):.3f} speed_cmd={speed_cmd:.3f}"
                f" d_speed={float(delta_speed):.3f} d_steer={float(delta_steer):.3f}"
                f" mrac_turn={float(mrac_turn):.3f} final_turn={turn_cmd:.3f}"
                f" reward={reward:.3f}"
            )

        if self._training and done:
            self._end_episode()

    # ------------------------------------------------------------------ #
    # MRAC inner steering chain                                            #
    # ------------------------------------------------------------------ #

    def _compute_mrac_turn(
        self, camera, vehicle, baseline_turn: float, dt_s: float
    ) -> float:
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
            dfx_n=0.0,
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

    def _reset_episode_stats(self) -> None:
        self._ep_vx_sum = 0.0
        self._ep_count = 0
        self._ep_distance = 0.0
        self._ep_lane_loss_steps = 0
        self._last_done_reason = "none"
        self._last_avg_speed = 0.0
        self._last_speed_benchmark = None
        self._last_speed_improvement = 0.0
        self._last_distance = 0.0
        self._last_lane_loss_steps = 0

    def _accumulate_episode_stats(
        self,
        vx: float,
        dt_s: float,
        lane_visible: bool,
    ) -> None:
        v_forward = max(0.0, float(vx))
        dt_safe = float(dt_s) if np.isfinite(dt_s) and 0.0 < float(dt_s) < 1.0 else 0.0

        self._ep_vx_sum += v_forward
        self._ep_count += 1
        self._ep_distance += v_forward * dt_safe

        if not lane_visible:
            self._ep_lane_loss_steps += 1

    def _episode_avg_speed(self) -> float:
        if self._ep_count <= 0:
            return 0.0
        return float(self._ep_vx_sum / self._ep_count)

    def _speed_benchmark(self):
        candidates = []
        if self._mrac_avg_speed_reference > 0.0:
            candidates.append(float(self._mrac_avg_speed_reference))
        if self._success_speed_ema is not None:
            candidates.append(float(self._success_speed_ema))
        if not candidates:
            return None
        return max(candidates)

    def _terminal_reward_for_success(self) -> float:
        avg_speed = self._episode_avg_speed()
        benchmark = self._speed_benchmark()

        if benchmark is None:
            improvement = 0.0
        else:
            improvement = max(0.0, avg_speed - float(benchmark))

        terminal_reward = self._terminal_speed_gain * improvement

        # Monotonic EMA update:
        # slower successful episodes do NOT lower the benchmark.
        if self._success_speed_ema is None:
            self._success_speed_ema = avg_speed
        elif avg_speed > self._success_speed_ema:
            alpha = float(np.clip(self._ema_alpha, 0.0, 1.0))
            self._success_speed_ema = (
                (1.0 - alpha) * self._success_speed_ema
                + alpha * avg_speed
            )

        if avg_speed > self._best_success_avg_speed:
            self._best_success_avg_speed = avg_speed

        self._last_avg_speed = avg_speed
        self._last_speed_benchmark = benchmark
        self._last_speed_improvement = improvement
        self._last_distance = self._ep_distance
        self._last_lane_loss_steps = self._ep_lane_loss_steps

        return float(terminal_reward)

    def _check_done(self, camera) -> tuple:
        self._last_avg_speed = self._episode_avg_speed()
        self._last_distance = self._ep_distance
        self._last_lane_loss_steps = self._ep_lane_loss_steps

        if not camera.have_measurement:
            self._no_lane_cnt += 1
        else:
            self._no_lane_cnt = 0

        if self._no_lane_cnt >= self._no_lane_limit:
            self._last_done_reason = "lane_lost"
            return True, -self._track_exit_penalty

        # Fixed-horizon success proxy:
        # if we reach max_steps, we treat this as surviving the route.
        if (self._step_cnt + 1) >= self._max_steps:
            self._last_done_reason = "max_steps_success"
            return True, self._terminal_reward_for_success()

        return False, 0.0

    def _end_episode(self) -> None:
        stats = self._trainer.update()   # type: ignore[union-attr]
        self._policy.save(self._weights_path)

        bench_str = (
            "none" if self._last_speed_benchmark is None
            else f"{float(self._last_speed_benchmark):.3f}"
        )
        ema_str = (
            "none" if self._success_speed_ema is None
            else f"{float(self._success_speed_ema):.3f}"
        )

        self.get_logger().info(
            f"[RL] ep={stats.episode}  reason={self._last_done_reason}"
            f"  steps={stats.steps}  R={stats.total_reward:.2f}"
            f"  G0={stats.return_0:.2f}  loss={stats.loss:.4f}"
            f"  avg_v={self._last_avg_speed:.3f}"
            f"  bench={bench_str}  improve={self._last_speed_improvement:.3f}"
            f"  ema_success={ema_str}  best_v={self._best_success_avg_speed:.3f}"
            f"  dist={self._last_distance:.2f}"
            f"  lane_loss_steps={self._last_lane_loss_steps}"
        )
        self._reset_episode()

    def _reset_episode(self) -> None:
        self._extractor = CameraMeasurementExtractor()
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
        self._debug_cnt   = 0
        self._reset_episode_stats()
        self._episode_num += 1
        self._ep_state    = _EpState.SETTLING
        self._settle_cnt  = 0

        if self._training:
            self._gz_reset()

    def _gz_reset(self) -> None:
        """
        Teleport the robot to spawn pose and WAIT for Gazebo to reply.

        The old version used subprocess.Popen(), which could fail silently or
        complete after the next episode had already started. This synchronous
        version makes reset behavior visible in the logs.
        """
        half_yaw = self._spawn_yaw / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        req = (
            f'name: "b3rb" '
            f'position {{ x: {self._spawn_x} y: {self._spawn_y} z: {self._spawn_z} }} '
            f'orientation {{ x: 0.0 y: 0.0 z: {qz:.6f} w: {qw:.6f} }}'
        )

        cmd = [
            "gz", "service",
            "-s", f"/world/{self._world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000",
            "--req", req,
        ]

        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=7.0,
                check=False,
            )
        except FileNotFoundError:
            self.get_logger().error(
                '[RL] "gz" binary not found — episode reset will not teleport the robot'
            )
            return
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                "[RL] Gazebo reset timed out — robot may not be back at spawn"
            )
            return

        stdout = (result.stdout or "").strip()
        stderr = (result.stderr or "").strip()

        if result.returncode != 0:
            self.get_logger().error(
                f"[RL] Gazebo reset failed returncode={result.returncode}"
                f" stdout={stdout[:200]!r} stderr={stderr[:200]!r}"
            )
            return

        self.get_logger().info(
            f"[RL] Gazebo reset sent to spawn"
            f" x={self._spawn_x:.2f} y={self._spawn_y:.2f} yaw={self._spawn_yaw:.2f}"
        )

    def _publish(self, speed: float, turn: float) -> None:
        msg = TwistStamped()
        msg.twist.linear.x  = float(speed)
        msg.twist.angular.z = float(turn)
        self._pub_cmd.publish(msg)


# ---------------------------------------------------------------------- #
# Module-level pure functions                                              #
# ---------------------------------------------------------------------- #

def _get_first_float(obj, names: list, default: float = 0.0) -> float:
    if obj is None:
        return float(default)
    for name in names:
        if hasattr(obj, name):
            return float(getattr(obj, name))
    return float(default)


def _build_state(camera, vx: float, ax: float) -> np.ndarray:
    ye      = camera.ye_cam_filt      if camera.have_measurement else 0.0
    psi_rel = camera.psi_rel_cam_filt if camera.have_measurement else 0.0
    return np.array([ye, psi_rel, vx, ax], dtype=np.float32) * _S_SCALE


def _reward(
    camera,
    vx: float,
    delta_speed: float,
    speed_baseline: float,
    delta_steer: float,
) -> float:
    """
    Step shaping reward for residual RL.

    Important:
      - no direct delta_speed reward
      - no smoothness penalty
      - no speed-gating using lane/heading error
      - main speed objective is handled at terminal success only
    """
    if not camera.have_measurement:
        return -2.0

    e_y = float(camera.ye_cam_filt)
    theta_e = float(camera.psi_rel_cam_filt)

    return float(
        + 0.02                         # small valid-lane survival shaping
        - 1.50 * (e_y ** 2)            # lateral error penalty
        - 0.50 * (theta_e ** 2)        # heading error penalty
        - 0.04 * (delta_steer ** 2)    # discourage unnecessary steering bias
        - 0.06 * (delta_speed ** 2)    # discourage constant speed bias
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RLRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

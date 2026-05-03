"""
RL runner node — n-step actor-critic residual controller on top of MRAC.

Architecture
------------
Baseline speed/turn -> MRAC steering -> RL residual steering/speed -> /nxp_cup/cmd_safe

The RL contribution limits are controlled only by:
    delta_max, speed_up, speed_dn

This file intentionally protects the learned policy:
    - successful faster laps get positive terminal reward
    - failed episodes can skip gradient update
    - best checkpoint is saved separately
    - after lane loss, the best checkpoint can be reloaded
"""

import json
import math
import os
import subprocess
from enum import Enum, auto
from pathlib import Path

import numpy as np
import rclpy
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
from .perception.camera_measurements import CameraMeasurementExtractor
from .rl.episode_manager import ActorCriticTrainer
from .rl.policy import ActorCriticPolicy

QOS_PROFILE_DEFAULT = 10

# ye, psi_rel are already approximately bounded. vx and ax are scaled.
_S_SCALE = np.array([1.0, 1.0, 2.0, 0.5], dtype=np.float32)


class _EpState(Enum):
    COLLECTING = auto()
    SETTLING = auto()


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
        reference_a_vy_s_inv=getattr(
            cfg,
            "INNER_MRAC_P_A_VY_S_INV",
            cfg.INNER_REFERENCE_MODEL_A_VY_S_INV,
        ),
        reference_a_r_s_inv=getattr(
            cfg,
            "INNER_MRAC_P_A_R_S_INV",
            cfg.INNER_REFERENCE_MODEL_A_R_S_INV,
        ),
        b_delta_vy=(
            cfg.FRONT_CORNERING_STIFFNESS_N_PER_RAD
            / cfg.DYNAMIC_BICYCLE_MASS_KG
        ),
        b_delta_r=(
            cfg.DYNAMIC_BICYCLE_LF_M
            * cfg.FRONT_CORNERING_STIFFNESS_N_PER_RAD
            / cfg.DYNAMIC_BICYCLE_IZ_KG_M2
        ),
    )

    return speed_scheduler, lat_yaw_model, ref_cmd_model, lat_yaw_ref_model, steering_mrac


class RLRunner(Node):
    def __init__(self) -> None:
        super().__init__("rl_runner")

        self.declare_parameter("training", True)

        # RL max contribution parameters — intentionally left unchanged.
        self.declare_parameter("delta_max", 0.30)
        self.declare_parameter("speed_up", 0.25)
        self.declare_parameter("speed_dn", 0.05)

        self.declare_parameter("gamma", 0.99)
        self.declare_parameter("lr", 3e-4)
        self.declare_parameter("n_step", 32)
        self.declare_parameter("value_coef", 0.50)
        self.declare_parameter("entropy_coef", 0.0015)
        self.declare_parameter("grad_clip", 1.0)

        self.declare_parameter("no_lane_limit", 20)
        self.declare_parameter("max_steps", 2000)
        self.declare_parameter("settle_steps", 80)
        self.declare_parameter("debug_every_n", 500)

        self.declare_parameter("world_name", "default")
        self.declare_parameter("weights_path", "~/rl_policy_actor_critic_latest.pt")
        self.declare_parameter("best_weights_path", "~/rl_policy_actor_critic_best.pt")
        self.declare_parameter("metrics_path", "~/rl_actor_critic_metrics.jsonl")

        self.declare_parameter("device", "cpu")

        self.declare_parameter("spawn_x", -5.0)
        self.declare_parameter("spawn_y", -2.0)
        self.declare_parameter("spawn_z", 0.05)
        self.declare_parameter("spawn_yaw", 0.0)

        # Reward / safety parameters.
        self.declare_parameter("success_bonus", 50.0)
        self.declare_parameter("terminal_speed_gain", 1000.0)
        self.declare_parameter("track_exit_penalty", 250.0)
        self.declare_parameter("ema_alpha", 0.10)
        self.declare_parameter("min_success_improve", 0.001)
        self.declare_parameter("skip_failed_updates", True)
        self.declare_parameter("reload_best_on_failure", True)

        self._training = bool(self.get_parameter("training").value)

        delta_max = float(self.get_parameter("delta_max").value)
        speed_up = float(self.get_parameter("speed_up").value)
        speed_dn = float(self.get_parameter("speed_dn").value)

        gamma = float(self.get_parameter("gamma").value)
        lr = float(self.get_parameter("lr").value)
        n_step = int(self.get_parameter("n_step").value)
        value_coef = float(self.get_parameter("value_coef").value)
        entropy_coef = float(self.get_parameter("entropy_coef").value)
        grad_clip = float(self.get_parameter("grad_clip").value)

        self._no_lane_limit = int(self.get_parameter("no_lane_limit").value)
        self._max_steps = int(self.get_parameter("max_steps").value)
        self._settle_steps = int(self.get_parameter("settle_steps").value)
        self._debug_every_n = int(self.get_parameter("debug_every_n").value)

        self._world_name = str(self.get_parameter("world_name").value)
        self._weights_path = os.path.expanduser(str(self.get_parameter("weights_path").value))
        self._best_weights_path = os.path.expanduser(
            str(self.get_parameter("best_weights_path").value)
        )
        self._metrics_path = os.path.expanduser(str(self.get_parameter("metrics_path").value))

        self._device = str(self.get_parameter("device").value)

        self._spawn_x = float(self.get_parameter("spawn_x").value)
        self._spawn_y = float(self.get_parameter("spawn_y").value)
        self._spawn_z = float(self.get_parameter("spawn_z").value)
        self._spawn_yaw = float(self.get_parameter("spawn_yaw").value)

        self._success_bonus = float(self.get_parameter("success_bonus").value)
        self._terminal_speed_gain = float(self.get_parameter("terminal_speed_gain").value)
        self._track_exit_penalty = float(self.get_parameter("track_exit_penalty").value)
        self._ema_alpha = float(self.get_parameter("ema_alpha").value)
        self._min_success_improve = float(self.get_parameter("min_success_improve").value)
        self._skip_failed_updates = bool(self.get_parameter("skip_failed_updates").value)
        self._reload_best_on_failure = bool(self.get_parameter("reload_best_on_failure").value)

        self.create_subscription(
            EdgeVectors,
            "/edge_vectors",
            self._edge_vectors_cb,
            QOS_PROFILE_DEFAULT,
        )

        self.create_subscription(
            Odometry,
            "/cerebri/out/odometry",
            self._odom_cb,
            QOS_PROFILE_DEFAULT,
        )

        self.create_subscription(
            TrafficStatus,
            "/traffic_status",
            self._traffic_cb,
            QOS_PROFILE_DEFAULT,
        )

        self._pub_cmd = self.create_publisher(
            TwistStamped,
            "/nxp_cup/cmd_safe",
            QOS_PROFILE_DEFAULT,
        )

        self._half_width = 200.0

        self._extractor = CameraMeasurementExtractor()
        self._baseline = BaselineLaneController()
        self._estimator = VehicleStateEstimator()
        self._traffic = TrafficStatus()

        (
            self._speed_scheduler,
            self._lat_yaw_model,
            self._ref_cmd_model,
            self._lat_yaw_ref_model,
            self._steering_mrac,
        ) = _make_mrac_components()

        self._policy = ActorCriticPolicy(
            state_dim=4,
            hidden_dim=64,
            delta_max=delta_max,
            speed_up=speed_up,
            speed_dn=speed_dn,
            device=self._device,
        )

        self._trainer = None
        if self._training:
            self._trainer = ActorCriticTrainer(
                self._policy,
                lr=lr,
                gamma=gamma,
                n_step=n_step,
                value_coef=value_coef,
                entropy_coef=entropy_coef,
                grad_clip=grad_clip,
            )

        if os.path.exists(self._weights_path):
            loaded = self._policy.load(self._weights_path)
            if loaded:
                self.get_logger().info(f"[RL] loaded latest weights from {self._weights_path}")
        elif os.path.exists(self._best_weights_path):
            loaded = self._policy.load(self._best_weights_path)
            if loaded:
                self.get_logger().info(f"[RL] loaded best weights from {self._best_weights_path}")
        elif not self._training:
            self.get_logger().warn("[RL] no checkpoint found; deployment policy is fresh")

        self._policy.train(self._training)

        self._ep_state = _EpState.COLLECTING
        self._no_lane_cnt = 0
        self._step_cnt = 0
        self._settle_cnt = 0
        self._episode_num = 0

        self._ep_reward_sum = 0.0
        self._ep_v_sum = 0.0
        self._ep_v_count = 0
        self._ep_dist = 0.0
        self._ep_lane_loss_steps = 0

        self._success_ema = None
        self._best_avg_v = -1.0
        self._last_done_reason = "none"
        self._last_terminal_reward = 0.0

        mode = "TRAINING" if self._training else "DEPLOYMENT"
        self.get_logger().info(
            "[RL] runner started — "
            f"mode={mode} device={self._device} "
            f"delta_max={delta_max:.3f} speed_up={speed_up:.3f} speed_dn={speed_dn:.3f} "
            f"lr={lr:.1e} gamma={gamma:.3f} n_step={n_step} "
            f"terminal_speed_gain={self._terminal_speed_gain:.1f} "
            f"track_exit_penalty={self._track_exit_penalty:.1f}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._estimator.update_from_odometry(msg)

    def _traffic_cb(self, msg: TrafficStatus) -> None:
        self._traffic = msg

    def _edge_vectors_cb(self, msg: EdgeVectors) -> None:
        if msg.image_width > 0:
            self._half_width = float(msg.image_width) / 2.0

        camera = self._extractor.extract(msg, self._half_width)
        vehicle = self._estimator.vehicle

        if self._ep_state == _EpState.SETTLING:
            self._publish(0.0, 0.0)
            self._settle_cnt += 1

            if self._settle_cnt >= self._settle_steps:
                self._ep_state = _EpState.COLLECTING
                self._settle_cnt = 0
                self.get_logger().info(f"[RL] episode {self._episode_num} started")

            return

        vx = float(vehicle.vx_recon) if vehicle.odom_ready else 0.0
        ax = float(vehicle.a_long_filt) if vehicle.odom_ready else 0.0

        state = _build_state(camera, vx, ax)

        stop = bool(getattr(self._traffic, "stop_sign", False))

        baseline_turn, baseline_speed, dt_s = self._baseline.compute(
            camera=camera,
            stop_sign=stop,
            obstacle_detected=False,
            ramp_detected=False,
        )

        mrac_turn = self._compute_mrac_turn(camera, vehicle, baseline_turn, dt_s)

        if self._training:
            (d_steer, d_speed), log_prob, value, entropy = self._policy.sample(state)
        else:
            d_steer, d_speed = self._policy.act(state)
            log_prob = value = entropy = None

        final_turn = float(np.clip(mrac_turn + d_steer, -1.0, 1.0))
        speed_cmd = float(max(0.0, baseline_speed + d_speed))

        self._publish(speed_cmd, final_turn)

        self._step_cnt += 1
        self._ep_v_sum += max(0.0, vx)
        self._ep_v_count += 1
        self._ep_dist += max(0.0, vx) * max(0.0, float(dt_s))

        if not camera.have_measurement:
            self._ep_lane_loss_steps += 1

        done, reason, terminal_reward = self._check_done(camera)

        step_reward = _reward(camera, vx, d_steer, d_speed)
        reward = step_reward + terminal_reward if done else step_reward
        self._ep_reward_sum += reward

        if self._training and self._trainer is not None:
            self._trainer.store(log_prob, value, entropy, reward)

            if self._debug_every_n > 0 and self._step_cnt % self._debug_every_n == 0:
                self.get_logger().info(
                    "[RL dbg] "
                    f"step={self._step_cnt} lane={int(camera.have_measurement)} "
                    f"ye={float(getattr(camera, 'ye_cam_filt', 0.0)):.3f} "
                    f"psi={float(getattr(camera, 'psi_rel_cam_filt', 0.0)):.3f} "
                    f"vx={vx:.3f} ax={ax:.3f} "
                    f"speed_base={baseline_speed:.3f} speed_cmd={speed_cmd:.3f} "
                    f"d_speed={d_speed:.3f} d_steer={d_steer:.3f} "
                    f"mrac_turn={mrac_turn:.3f} final_turn={final_turn:.3f} "
                    f"reward={reward:.3f}"
                )

            if done:
                self._end_episode(reason)

    def _compute_mrac_turn(self, camera, vehicle, baseline_turn: float, dt_s: float) -> float:
        vx = float(vehicle.vx_recon) if vehicle.odom_ready else 0.0
        vy = float(vehicle.vy_recon) if vehicle.odom_ready else 0.0
        r = float(vehicle.r_recon) if vehicle.odom_ready else 0.0

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

        uc = float(getattr(ref_cmd, "uc_rad_s", 0.0)) if ref_cmd is not None else 0.0
        valid = bool(getattr(ref_cmd, "valid", False)) if ref_cmd is not None else False

        lat_yaw_ref = self._lat_yaw_ref_model.step(
            uc_rad_s=uc,
            dt_s=dt_s,
            command_valid=valid,
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

    def _episode_avg_v(self) -> float:
        if self._ep_v_count <= 0:
            return 0.0
        return float(self._ep_v_sum / self._ep_v_count)

    def _success_terminal_reward(self) -> float:
        avg_v = self._episode_avg_v()

        if self._success_ema is None:
            improvement = 0.0
        else:
            improvement = max(0.0, avg_v - self._success_ema)

        return float(self._success_bonus + self._terminal_speed_gain * improvement)

    def _check_done(self, camera):
        if not camera.have_measurement:
            self._no_lane_cnt += 1
        else:
            self._no_lane_cnt = 0

        if self._no_lane_cnt >= self._no_lane_limit:
            self._last_done_reason = "lane_lost"
            self._last_terminal_reward = -float(self._track_exit_penalty)
            return True, self._last_done_reason, self._last_terminal_reward

        if self._step_cnt >= self._max_steps:
            self._last_done_reason = "max_steps_success"
            self._last_terminal_reward = self._success_terminal_reward()
            return True, self._last_done_reason, self._last_terminal_reward

        return False, "running", 0.0

    def _end_episode(self, reason: str) -> None:
        success = reason == "max_steps_success"
        avg_v = self._episode_avg_v()

        skipped = False

        if not success and self._skip_failed_updates and self._trainer is not None:
            stats = self._trainer.clear(skipped_update=True)
            skipped = True
        else:
            stats = self._trainer.update() if self._trainer is not None else None

        if success:
            old_ema = self._success_ema

            if self._success_ema is None:
                self._success_ema = avg_v
            else:
                self._success_ema = (
                    (1.0 - self._ema_alpha) * self._success_ema
                    + self._ema_alpha * avg_v
                )

            improved_best = avg_v > (self._best_avg_v + self._min_success_improve)

            self._policy.save(self._weights_path)

            if improved_best:
                self._best_avg_v = avg_v
                self._policy.save(self._best_weights_path)
                best_msg = " best_saved=1"
            else:
                best_msg = " best_saved=0"

            ema_before = old_ema if old_ema is not None else avg_v

        else:
            best_msg = " best_saved=0"
            ema_before = self._success_ema if self._success_ema is not None else 0.0

            if self._reload_best_on_failure and os.path.exists(self._best_weights_path):
                loaded = self._policy.load(self._best_weights_path)
                self._policy.train(self._training)
                if loaded:
                    self.get_logger().warn(
                        f"[RL] failure detected; reloaded best checkpoint from {self._best_weights_path}"
                    )

        actor_loss = getattr(stats, "actor_loss", 0.0) if stats is not None else 0.0
        critic_loss = getattr(stats, "critic_loss", 0.0) if stats is not None else 0.0
        entropy = getattr(stats, "entropy", 0.0) if stats is not None else 0.0
        total_reward = getattr(stats, "total_reward", self._ep_reward_sum) if stats is not None else self._ep_reward_sum
        loss = getattr(stats, "loss", 0.0) if stats is not None else 0.0
        return_0 = getattr(stats, "return_0", 0.0) if stats is not None else 0.0

        self.get_logger().info(
            "[RL] "
            f"ep={self._episode_num} reason={reason} steps={self._step_cnt} "
            f"R={total_reward:.2f} G0={return_0:.2f} loss={loss:.4f} "
            f"actor_loss={actor_loss:.4f} critic_loss={critic_loss:.4f} entropy={entropy:.4f} "
            f"avg_v={avg_v:.3f} ema_success={float(ema_before):.3f} "
            f"best_v={self._best_avg_v:.3f} dist={self._ep_dist:.2f} "
            f"terminal_reward={self._last_terminal_reward:.2f} "
            f"lane_loss_steps={self._ep_lane_loss_steps} skipped_update={int(skipped)}"
            f"{best_msg}"
        )

        self._write_metrics_jsonl(
            {
                "episode": self._episode_num,
                "reason": reason,
                "steps": self._step_cnt,
                "reward": float(total_reward),
                "return_0": float(return_0),
                "loss": float(loss),
                "actor_loss": float(actor_loss),
                "critic_loss": float(critic_loss),
                "entropy": float(entropy),
                "avg_v": float(avg_v),
                "ema_success": float(ema_before),
                "best_v": float(self._best_avg_v),
                "dist": float(self._ep_dist),
                "terminal_reward": float(self._last_terminal_reward),
                "lane_loss_steps": int(self._ep_lane_loss_steps),
                "skipped_update": bool(skipped),
                "success": bool(success),
            }
        )

        self._reset_episode()

    def _write_metrics_jsonl(self, row: dict) -> None:
        try:
            path = Path(self._metrics_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as exc:
            self.get_logger().warn(f"[RL] could not write metrics jsonl: {exc}")

    def _reset_episode(self) -> None:
        self._extractor = CameraMeasurementExtractor()
        self._baseline = BaselineLaneController()
        self._estimator = VehicleStateEstimator()

        (
            self._speed_scheduler,
            self._lat_yaw_model,
            self._ref_cmd_model,
            self._lat_yaw_ref_model,
            self._steering_mrac,
        ) = _make_mrac_components()

        self._no_lane_cnt = 0
        self._step_cnt = 0
        self._settle_cnt = 0

        self._ep_reward_sum = 0.0
        self._ep_v_sum = 0.0
        self._ep_v_count = 0
        self._ep_dist = 0.0
        self._ep_lane_loss_steps = 0

        self._episode_num += 1
        self._ep_state = _EpState.SETTLING

        if self._training:
            self._gz_reset()

    def _gz_reset(self) -> None:
        half_yaw = self._spawn_yaw / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        req = (
            f'name: "b3rb" '
            f'position {{ x: {self._spawn_x} y: {self._spawn_y} z: {self._spawn_z} }} '
            f'orientation {{ x: 0.0 y: 0.0 z: {qz:.6f} w: {qw:.6f} }}'
        )

        cmd = [
            "gz",
            "service",
            "-s",
            f"/world/{self._world_name}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            req,
        ]

        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(
                f"[RL] Gazebo reset sent to spawn x={self._spawn_x:.2f} "
                f"y={self._spawn_y:.2f} yaw={self._spawn_yaw:.2f}"
            )
        except FileNotFoundError:
            self.get_logger().error("[RL] gz binary not found; reset failed")

    def _publish(self, speed: float, turn: float) -> None:
        msg = TwistStamped()
        msg.twist.linear.x = float(speed)
        msg.twist.angular.z = float(turn)
        self._pub_cmd.publish(msg)


def _get_first_float(obj, names: list, default: float = 0.0) -> float:
    if obj is None:
        return float(default)

    for name in names:
        if hasattr(obj, name):
            return float(getattr(obj, name))

    return float(default)


def _build_state(camera, vx: float, ax: float) -> np.ndarray:
    ye = camera.ye_cam_filt if camera.have_measurement else 0.0
    psi_rel = camera.psi_rel_cam_filt if camera.have_measurement else 0.0

    return np.array([ye, psi_rel, vx, ax], dtype=np.float32) * _S_SCALE


def _reward(camera, vx: float, d_steer: float, d_speed: float) -> float:
    """
    Stable per-step reward.

    Good lane-following should be mildly positive.
    Bad cornering should be negative, but not catastrophically negative.
    The large success/failure signal is handled by terminal reward.
    """
    if not camera.have_measurement:
        return -0.25

    e_y = float(camera.ye_cam_filt)
    psi = float(camera.psi_rel_cam_filt)

    aligned_progress = max(0.0, float(vx)) * float(np.cos(np.clip(psi, -1.2, 1.2)))

    reward = (
        0.020
        + 0.020 * aligned_progress
        - 0.45 * (e_y ** 2)
        - 0.20 * (psi ** 2)
        - 0.03 * (d_steer ** 2)
        - 0.01 * (d_speed ** 2)
    )

    return float(np.clip(reward, -0.50, 0.08))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RLRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

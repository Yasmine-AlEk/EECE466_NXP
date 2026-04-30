#!/usr/bin/env python3
"""
Compare MRAC-only vs MRAC+RL evaluation rosbag runs.

Inputs:
  --mrac-bag path/to/mrac_only_.../bag
  --rl-bag   path/to/mrac_plus_rl_.../bag

Outputs:
  ~/mrac_rl_compare/plots_compare_<timestamp>/
    01_path_xy.png
    02_distance_progress.png
    03_speed_vx.png
    04_actual_speed_command.png
    05_actual_steering_command.png
    06_steering_rate_abs.png
    07_yaw_rate.png
    08_lane_error_proxy.png       if edge-vector extraction works
    09_heading_error_proxy.png    if edge-vector extraction works
    summary_metrics.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# Try to reuse the same camera extractor as the controller.
try:
    from b3rb_ros_line_follower.perception.camera_measurements import CameraMeasurementExtractor
except Exception:
    CameraMeasurementExtractor = None


@dataclass
class Series:
    t: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)


@dataclass
class BagData:
    name: str

    # Odometry-derived
    odom_t: list[float] = field(default_factory=list)
    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)
    yaw: list[float] = field(default_factory=list)
    vx_body: list[float] = field(default_factory=list)
    yaw_rate: list[float] = field(default_factory=list)
    distance: list[float] = field(default_factory=list)

    # Actual command received by vehicle
    joy_speed: Series = field(default_factory=Series)
    joy_turn: Series = field(default_factory=Series)

    # RL command before bridge, if present
    cmd_safe_speed: Series = field(default_factory=Series)
    cmd_safe_turn: Series = field(default_factory=Series)

    # Lane/camera proxy
    lane_t: list[float] = field(default_factory=list)
    ye: list[float] = field(default_factory=list)
    psi: list[float] = field(default_factory=list)
    have_lane: list[float] = field(default_factory=list)


def yaw_from_quat(q: Any) -> float:
    # ROS quaternion: x, y, z, w
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalise_time(values: list[float]) -> list[float]:
    if not values:
        return values
    t0 = values[0]
    return [v - t0 for v in values]


def get_topic_types(bag_path: Path) -> dict[str, str]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def extract_camera_measurement(extractor: Any, msg: Any) -> Any | None:
    if extractor is None:
        return None

    for method_name in (
        "extract_from_edge_vectors",
        "extract",
        "extract_from_msg",
    ):
        method = getattr(extractor, method_name, None)
        if callable(method):
            try:
                return method(msg)
            except Exception:
                return None

    return None


def read_bag(bag_path: Path, name: str) -> BagData:
    data = BagData(name=name)

    topic_types = get_topic_types(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    extractor = CameraMeasurementExtractor() if CameraMeasurementExtractor is not None else None

    raw_odom = []
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in topic_types:
            continue

        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(raw, msg_type)
        t = stamp_ns * 1.0e-9

        if topic == "/cerebri/out/odometry":
            pos = msg.pose.pose.position
            yaw = yaw_from_quat(msg.pose.pose.orientation)
            raw_odom.append((t, float(pos.x), float(pos.y), yaw))

        elif topic == "/cerebri/in/joy":
            axes = list(msg.axes)
            if len(axes) >= 4:
                data.joy_speed.t.append(t)
                data.joy_speed.y.append(float(axes[1]))
                data.joy_turn.t.append(t)
                data.joy_turn.y.append(float(axes[3]))

        elif topic == "/nxp_cup/cmd_safe":
            data.cmd_safe_speed.t.append(t)
            data.cmd_safe_speed.y.append(float(msg.twist.linear.x))
            data.cmd_safe_turn.t.append(t)
            data.cmd_safe_turn.y.append(float(msg.twist.angular.z))

        elif topic == "/edge_vectors":
            meas = extract_camera_measurement(extractor, msg)
            if meas is not None:
                data.lane_t.append(t)
                data.ye.append(float(getattr(meas, "ye_cam_filt", 0.0)))
                data.psi.append(float(getattr(meas, "psi_rel_cam_filt", 0.0)))
                data.have_lane.append(1.0 if bool(getattr(meas, "have_measurement", False)) else 0.0)

    # Process odometry after reading all samples.
    if raw_odom:
        raw_odom.sort(key=lambda r: r[0])
        t_arr = np.array([r[0] for r in raw_odom], dtype=float)
        x_arr = np.array([r[1] for r in raw_odom], dtype=float)
        y_arr = np.array([r[2] for r in raw_odom], dtype=float)
        yaw_arr = np.unwrap(np.array([r[3] for r in raw_odom], dtype=float))

        dist = np.zeros_like(t_arr)
        vx_body = np.zeros_like(t_arr)
        yaw_rate = np.zeros_like(t_arr)

        for i in range(1, len(t_arr)):
            dt = max(t_arr[i] - t_arr[i - 1], 1.0e-9)
            dx = x_arr[i] - x_arr[i - 1]
            dy = y_arr[i] - y_arr[i - 1]
            dist[i] = dist[i - 1] + math.hypot(dx, dy)

            vx_w = dx / dt
            vy_w = dy / dt
            c = math.cos(yaw_arr[i])
            s = math.sin(yaw_arr[i])
            vx_body[i] = c * vx_w + s * vy_w

            yaw_rate[i] = (yaw_arr[i] - yaw_arr[i - 1]) / dt

        data.odom_t = list(t_arr)
        data.x = list(x_arr)
        data.y = list(y_arr)
        data.yaw = list(yaw_arr)
        data.vx_body = list(vx_body)
        data.yaw_rate = list(yaw_rate)
        data.distance = list(dist)

    # Normalise all time vectors independently.
    for attr in ("odom_t", "lane_t"):
        setattr(data, attr, normalise_time(getattr(data, attr)))

    for s in (data.joy_speed, data.joy_turn, data.cmd_safe_speed, data.cmd_safe_turn):
        s.t = normalise_time(s.t)

    return data


def rms(values: list[float] | np.ndarray) -> float:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(arr ** 2)))


def mean_abs(values: list[float] | np.ndarray) -> float:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return float("nan")
    return float(np.mean(np.abs(arr)))


def steering_rate_abs(series: Series) -> tuple[list[float], list[float]]:
    if len(series.t) < 2:
        return [], []
    t = np.asarray(series.t, dtype=float)
    y = np.asarray(series.y, dtype=float)
    dt = np.diff(t)
    dy = np.diff(y)
    valid = dt > 1.0e-9
    rate_t = t[1:][valid]
    rate = np.abs(dy[valid] / dt[valid])
    return list(rate_t), list(rate)


def save_plot(
    out_dir: Path,
    filename: str,
    title: str,
    xlabel: str,
    ylabel: str,
    curves: list[tuple[list[float], list[float], str]],
) -> None:
    plt.figure(figsize=(9, 5))

    plotted = 0
    for t, y, label in curves:
        if len(t) == 0 or len(y) == 0:
            continue
        n = min(len(t), len(y))
        plt.plot(t[:n], y[:n], label=label)
        plotted += 1

    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True, alpha=0.3)
    if plotted > 1:
        plt.legend()
    plt.tight_layout()

    path = out_dir / filename
    plt.savefig(path, dpi=160)
    plt.close()
    print(f"[plot] {path}")


def save_path_plot(out_dir: Path, mrac: BagData, rl: BagData) -> None:
    plt.figure(figsize=(7, 7))

    if mrac.x and mrac.y:
        plt.plot(mrac.x, mrac.y, label="MRAC-only")
    if rl.x and rl.y:
        plt.plot(rl.x, rl.y, label="MRAC+RL")

    plt.title("Path comparison")
    plt.xlabel("x position [m]")
    plt.ylabel("y position [m]")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    path = out_dir / "01_path_xy.png"
    plt.savefig(path, dpi=160)
    plt.close()
    print(f"[plot] {path}")


def metric_row(data: BagData) -> dict[str, float | str]:
    duration = data.odom_t[-1] if data.odom_t else float("nan")
    path_len = data.distance[-1] if data.distance else float("nan")

    rate_t, rate_abs = steering_rate_abs(data.joy_turn)

    return {
        "run": data.name,
        "duration_s": duration,
        "path_length_m": path_len,
        "mean_vx_body_mps": float(np.nanmean(data.vx_body)) if data.vx_body else float("nan"),
        "max_vx_body_mps": float(np.nanmax(data.vx_body)) if data.vx_body else float("nan"),
        "rms_yaw_rate_rad_s": rms(data.yaw_rate),
        "mean_abs_joy_turn": mean_abs(data.joy_turn.y),
        "max_abs_joy_turn": float(np.nanmax(np.abs(data.joy_turn.y))) if data.joy_turn.y else float("nan"),
        "mean_abs_steering_rate": mean_abs(rate_abs),
        "mean_joy_speed_cmd": float(np.nanmean(data.joy_speed.y)) if data.joy_speed.y else float("nan"),
        "rms_lane_ye_proxy": rms(data.ye),
        "rms_lane_psi_proxy": rms(data.psi),
        "lane_available_percent": 100.0 * float(np.nanmean(data.have_lane)) if data.have_lane else float("nan"),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mrac-bag", required=True, type=Path)
    parser.add_argument("--rl-bag", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    if args.out_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.out_dir = Path.home() / "mrac_rl_compare" / f"plots_compare_{stamp}"

    args.out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[read] MRAC bag: {args.mrac_bag}")
    mrac = read_bag(args.mrac_bag, "MRAC-only")

    print(f"[read] RL bag:   {args.rl_bag}")
    rl = read_bag(args.rl_bag, "MRAC+RL")

    save_path_plot(args.out_dir, mrac, rl)

    save_plot(
        args.out_dir,
        "02_distance_progress.png",
        "Distance progress",
        "time [s]",
        "distance traveled [m]",
        [(mrac.odom_t, mrac.distance, "MRAC-only"), (rl.odom_t, rl.distance, "MRAC+RL")],
    )

    save_plot(
        args.out_dir,
        "03_speed_vx.png",
        "Body-frame speed comparison",
        "time [s]",
        "reconstructed vx [m/s]",
        [(mrac.odom_t, mrac.vx_body, "MRAC-only"), (rl.odom_t, rl.vx_body, "MRAC+RL")],
    )

    save_plot(
        args.out_dir,
        "04_actual_speed_command.png",
        "Actual speed command sent to vehicle",
        "time [s]",
        "Joy axis speed command",
        [(mrac.joy_speed.t, mrac.joy_speed.y, "MRAC-only"), (rl.joy_speed.t, rl.joy_speed.y, "MRAC+RL")],
    )

    save_plot(
        args.out_dir,
        "05_actual_steering_command.png",
        "Actual steering command sent to vehicle",
        "time [s]",
        "Joy axis steering command",
        [(mrac.joy_turn.t, mrac.joy_turn.y, "MRAC-only"), (rl.joy_turn.t, rl.joy_turn.y, "MRAC+RL")],
    )

    mrac_rate_t, mrac_rate_abs = steering_rate_abs(mrac.joy_turn)
    rl_rate_t, rl_rate_abs = steering_rate_abs(rl.joy_turn)

    save_plot(
        args.out_dir,
        "06_steering_rate_abs.png",
        "Steering command smoothness",
        "time [s]",
        "|d steering / dt|",
        [(mrac_rate_t, mrac_rate_abs, "MRAC-only"), (rl_rate_t, rl_rate_abs, "MRAC+RL")],
    )

    save_plot(
        args.out_dir,
        "07_yaw_rate.png",
        "Yaw-rate comparison",
        "time [s]",
        "yaw rate [rad/s]",
        [(mrac.odom_t, mrac.yaw_rate, "MRAC-only"), (rl.odom_t, rl.yaw_rate, "MRAC+RL")],
    )

    if mrac.ye and rl.ye:
        save_plot(
            args.out_dir,
            "08_lane_error_proxy.png",
            "Lane-center error proxy",
            "time [s]",
            "ye_cam_filt proxy",
            [(mrac.lane_t, mrac.ye, "MRAC-only"), (rl.lane_t, rl.ye, "MRAC+RL")],
        )

    if mrac.psi and rl.psi:
        save_plot(
            args.out_dir,
            "09_heading_error_proxy.png",
            "Heading error proxy",
            "time [s]",
            "psi_rel_cam_filt proxy",
            [(mrac.lane_t, mrac.psi, "MRAC-only"), (rl.lane_t, rl.psi, "MRAC+RL")],
        )

    rows = [metric_row(mrac), metric_row(rl)]
    csv_path = args.out_dir / "summary_metrics.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print(f"[csv]  {csv_path}")

    print("\nSummary")
    print("-------")
    for row in rows:
        print(f"{row['run']}:")
        print(f"  duration_s:              {row['duration_s']:.3f}")
        print(f"  path_length_m:           {row['path_length_m']:.3f}")
        print(f"  mean_vx_body_mps:        {row['mean_vx_body_mps']:.3f}")
        print(f"  max_vx_body_mps:         {row['max_vx_body_mps']:.3f}")
        print(f"  mean_abs_joy_turn:       {row['mean_abs_joy_turn']:.3f}")
        print(f"  mean_abs_steering_rate:  {row['mean_abs_steering_rate']:.3f}")
        print(f"  rms_lane_ye_proxy:       {row['rms_lane_ye_proxy']:.4f}")
        print(f"  rms_lane_psi_proxy:      {row['rms_lane_psi_proxy']:.4f}")
        print(f"  lane_available_percent:  {row['lane_available_percent']:.1f}%")

    print(f"\noutputs saved in: {args.out_dir}")


if __name__ == "__main__":
    main()

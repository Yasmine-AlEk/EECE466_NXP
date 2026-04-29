#!/usr/bin/env python3
"""
Grouped MRAC log plotter.

Input:
  ROS output log containing lines like:
    [INFO] [time] [line_follower]: [MRAC scaffold] key=value, key=value, ...

Output:
  logs/mrac_tests/plots/<log_stem>/
    - grouped PNG plots
    - parsed_samples.csv
    - summary_metrics.csv
    - reason_counts.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt


MARKER = "[MRAC scaffold]"


PLOT_GROUPS = [
    (
        "speed_tracking",
        "Outer speed tracking",
        ["speed_cmd", "vx_recon", "outer_vx_ref", "outer_vx_m"],
        "speed / command",
    ),
    (
        "outer_speed_error",
        "Outer speed error and MRAC activity",
        ["outer_ex", "outer_phi_norm", "outer_mrac_applied", "outer_mrac_update"],
        "value",
    ),
    (
        "outer_pwm_compare",
        "Outer PWM: baseline vs MRAC",
        ["outer_pwm_baseline", "outer_pwm_raw", "outer_pwm_sat", "outer_pwm_final"],
        "PWM command",
    ),
    (
        "outer_adaptive_parameters",
        "Outer MRAC parameter evolution",
        ["outer_k_r_hat", "outer_k_x_hat", "outer_theta_norm", "outer_b_hat"],
        "parameter value",
    ),
    (
        "inner_yaw_tracking",
        "Inner yaw-rate tracking",
        ["inner_uc", "inner_xm_r", "r_recon", "inner_e_r"],
        "rad/s",
    ),
    (
        "inner_lateral_error",
        "Inner lateral/yaw errors",
        ["inner_e_vy", "inner_e_r", "inner_phi_norm"],
        "value",
    ),
    (
        "inner_steering_compare",
        "Inner steering: baseline vs MRAC",
        [
            "inner_delta_baseline_deg",
            "inner_delta_raw_deg",
            "inner_delta_sat_deg",
            "inner_delta_final_deg",
        ],
        "steering angle [deg]",
    ),
    (
        "inner_turn_command",
        "Final steering command sent to rover",
        ["inner_turn_final", "turn_cmd"],
        "normalized turn command",
    ),
    (
        "inner_adaptive_parameters",
        "Inner steering MRAC parameter evolution",
        [
            "inner_theta_vy_hat",
            "inner_theta_r_hat",
            "inner_theta_uc_hat",
            "inner_theta_norm",
        ],
        "parameter value",
    ),
    (
        "inner_mrac_flags",
        "Inner MRAC validity / update / applied flags",
        ["inner_mrac_valid", "inner_mrac_update", "inner_mrac_applied"],
        "False=0, True=1",
    ),
    (
        "tv_force_command",
        "Torque-vectoring force command",
        [
            "inner_tv_dfx_raw_n",
            "inner_tv_dfx_sat_n",
            "inner_tv_dfx_final_n",
            "inner_tv_yaw_moment_nm",
        ],
        "N or N*m",
    ),
    (
        "tv_adaptive_parameters",
        "TV MRAC parameter evolution",
        [
            "inner_tv_theta_vy_hat",
            "inner_tv_theta_r_hat",
            "inner_tv_theta_uc_hat",
            "inner_tv_theta_norm",
        ],
        "parameter value",
    ),
    (
        "tv_flags",
        "TV validity / update / applied flags",
        ["inner_tv_valid", "inner_tv_update", "inner_tv_applied"],
        "False=0, True=1",
    ),
    (
        "rls_stiffness_raw_vs_used",
        "Cornering stiffness: raw RLS vs controller-used",
        [
            "c_alpha_f_hat",
            "c_alpha_r_hat",
            "c_alpha_f_used",
            "c_alpha_r_used",
        ],
        "N/rad",
    ),
    (
        "rls_activity",
        "RLS estimator activity",
        [
            "rls_phi_norm",
            "rls_phi_ready",
            "rls_est_valid",
            "rls_est_update",
            "rls_update_count",
        ],
        "value",
    ),
    (
        "phase7_force_allocation",
        "Phase 7 wheel-force allocation",
        [
            "phase7_f_long_cmd_n",
            "phase7_dfx_cmd_n",
            "phase7_fx_left_n",
            "phase7_fx_right_n",
            "phase7_re_f_long_n",
            "phase7_re_dfx_n",
        ],
        "force [N]",
    ),
    (
        "phase7_recomposition_errors",
        "Phase 7 recomposition errors",
        ["phase7_f_long_err_n", "phase7_dfx_err_n"],
        "force error [N]",
    ),
    (
        "phase7_pwm_outputs",
        "Phase 7 PWM outputs",
        [
            "phase7_pwm_left",
            "phase7_pwm_right",
            "phase7_pwm_common",
            "phase7_pwm_diff",
        ],
        "PWM command",
    ),
    (
        "phase7_saturation_flags",
        "Phase 7 saturation flags",
        ["phase7_force_sat", "phase7_dfx_sat"],
        "False=0, True=1",
    ),
]


REASON_KEYS = [
    "outer_mrac_reason",
    "inner_mrac_reason",
    "inner_tv_reason",
    "rls_phi_reason",
    "rls_est_reason",
    "rls_used_reason",
    "batt_reason",
]

FLAG_KEYS = [
    "outer_mrac_valid",
    "outer_mrac_update",
    "outer_mrac_applied",
    "inner_mrac_valid",
    "inner_mrac_update",
    "inner_mrac_applied",
    "inner_tv_valid",
    "inner_tv_update",
    "inner_tv_applied",
    "rls_out_valid",
    "rls_phi_ready",
    "rls_est_valid",
    "rls_est_update",
    "rls_used_ready",
    "rls_used_frozen",
    "batt_gain_valid",
    "batt_gain_update",
    "phase7_force_sat",
    "phase7_dfx_sat",
]


def parse_numeric(raw: Optional[str]) -> Optional[float]:
    if raw is None:
        return None

    value = raw.strip()

    if value == "True":
        return 1.0
    if value == "False":
        return 0.0

    if value in ("None", "nan", "NaN", ""):
        return None

    if value.endswith("deg"):
        value = value[:-3]

    for suffix in ("rad/s", "m/s", "N*m", "Nm", "N"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]

    try:
        out = float(value)
    except ValueError:
        return None

    if not math.isfinite(out):
        return None

    return out


def parse_bool(raw: Optional[str]) -> Optional[bool]:
    if raw == "True":
        return True
    if raw == "False":
        return False
    return None


def extract_timestamp(line: str) -> Optional[float]:
    match = re.search(r"\[INFO\]\s+\[([0-9.]+)\]", line)
    if match:
        return float(match.group(1))
    return None


def parse_row(line: str, fallback_index: int) -> Optional[Dict[str, str]]:
    if MARKER not in line:
        return None

    payload = line.split(MARKER, 1)[1].strip()
    row: Dict[str, str] = {}

    timestamp = extract_timestamp(line)
    if timestamp is not None:
        row["_t_abs"] = f"{timestamp:.9f}"
    else:
        row["_t_abs"] = str(fallback_index)

    for part in payload.split(","):
        part = part.strip()
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        row[key.strip()] = value.strip()

    return row


def read_rows(path: Path) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []

    for i, line in enumerate(path.read_text(errors="ignore").splitlines()):
        row = parse_row(line, i)
        if row is not None:
            rows.append(row)

    if not rows:
        raise SystemExit(f"No {MARKER!r} rows found in {path}")

    t0 = parse_numeric(rows[0].get("_t_abs")) or 0.0
    for i, row in enumerate(rows):
        t_abs = parse_numeric(row.get("_t_abs"))
        if t_abs is None:
            row["_t"] = str(float(i))
        else:
            row["_t"] = f"{t_abs - t0:.9f}"

    return rows


def latest_log() -> Path:
    log_dir = Path("logs/mrac_tests")
    patterns = [
        "mrac_*.log",
        "inner_mrac_*.log",
        "phase7_*.log",
        "*.log",
    ]

    logs: List[Path] = []
    for pattern in patterns:
        logs.extend(log_dir.glob(pattern))

    logs = sorted(set(logs), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise SystemExit("No log files found under logs/mrac_tests.")

    return logs[-1]


def numeric_series(rows: Sequence[Dict[str, str]], key: str) -> Tuple[List[float], List[float]]:
    x_values: List[float] = []
    y_values: List[float] = []

    for row in rows:
        y = parse_numeric(row.get(key))
        x = parse_numeric(row.get("_t"))

        if x is None or y is None:
            continue

        x_values.append(x)
        y_values.append(y)

    return x_values, y_values


def thin_series(x_values: List[float], y_values: List[float], max_points: int) -> Tuple[List[float], List[float]]:
    if max_points <= 0 or len(x_values) <= max_points:
        return x_values, y_values

    step = max(1, len(x_values) // max_points)
    return x_values[::step], y_values[::step]


def plot_group(
    rows: Sequence[Dict[str, str]],
    output_dir: Path,
    index: int,
    name: str,
    title: str,
    keys: Sequence[str],
    ylabel: str,
    dpi: int,
    max_points: int,
) -> Optional[Path]:
    plotted = 0

    plt.figure(figsize=(10.5, 5.8))

    for key in keys:
        x_values, y_values = numeric_series(rows, key)
        if not y_values:
            continue

        x_values, y_values = thin_series(x_values, y_values, max_points)
        plt.plot(x_values, y_values, linewidth=1.4, label=key)
        plotted += 1

    if plotted == 0:
        plt.close()
        return None

    plt.title(title)
    plt.xlabel("time from first MRAC scaffold sample [s]")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend(loc="best")
    plt.tight_layout()

    out_path = output_dir / f"{index:02d}_{name}.png"
    plt.savefig(out_path, dpi=dpi)
    plt.close()

    return out_path


def all_keys(rows: Sequence[Dict[str, str]]) -> List[str]:
    keys = set()
    for row in rows:
        keys.update(row.keys())

    ordered = ["_t", "_t_abs"]
    remaining = sorted(k for k in keys if k not in ordered)
    return ordered + remaining


def write_parsed_csv(rows: Sequence[Dict[str, str]], output_dir: Path) -> Path:
    keys = all_keys(rows)
    out_path = output_dir / "parsed_samples.csv"

    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    return out_path


def mean(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    return sum(values) / len(values)


def max_abs(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    return max(abs(x) for x in values)


def numeric_values(rows: Sequence[Dict[str, str]], key: str) -> List[float]:
    out: List[float] = []
    for row in rows:
        value = parse_numeric(row.get(key))
        if value is not None:
            out.append(value)
    return out


def percent_true(rows: Sequence[Dict[str, str]], key: str) -> Optional[float]:
    seen = 0
    true_count = 0

    for row in rows:
        value = parse_bool(row.get(key))
        if value is None:
            continue
        seen += 1
        if value:
            true_count += 1

    if seen == 0:
        return None

    return 100.0 * true_count / seen


def diff_values(rows: Sequence[Dict[str, str]], a_key: str, b_key: str) -> List[float]:
    out: List[float] = []

    for row in rows:
        a = parse_numeric(row.get(a_key))
        b = parse_numeric(row.get(b_key))
        if a is None or b is None:
            continue
        out.append(a - b)

    return out


def add_metric(metrics: List[Tuple[str, str]], name: str, value: object) -> None:
    if value is None:
        return
    if isinstance(value, float):
        metrics.append((name, f"{value:.6g}"))
    else:
        metrics.append((name, str(value)))


def write_summary_csv(rows: Sequence[Dict[str, str]], output_dir: Path) -> Path:
    metrics: List[Tuple[str, str]] = []

    duration_values = numeric_values(rows, "_t")
    add_metric(metrics, "sample_count", len(rows))
    if duration_values:
        add_metric(metrics, "duration_s", duration_values[-1] - duration_values[0])

    for key in FLAG_KEYS:
        pct = percent_true(rows, key)
        if pct is not None:
            add_metric(metrics, f"{key}_percent_true", pct)

    comparisons = [
        ("outer_pwm_final_minus_baseline", "outer_pwm_final", "outer_pwm_baseline"),
        ("outer_pwm_raw_minus_baseline", "outer_pwm_raw", "outer_pwm_baseline"),
        ("inner_delta_final_minus_baseline_deg", "inner_delta_final_deg", "inner_delta_baseline_deg"),
        ("inner_delta_raw_minus_baseline_deg", "inner_delta_raw_deg", "inner_delta_baseline_deg"),
        ("inner_delta_final_minus_raw_deg", "inner_delta_final_deg", "inner_delta_raw_deg"),
        ("phase7_f_long_recomposition_error_n", "phase7_re_f_long_n", "phase7_f_long_cmd_n"),
        ("phase7_dfx_recomposition_error_n", "phase7_re_dfx_n", "phase7_dfx_cmd_n"),
    ]

    for name, a_key, b_key in comparisons:
        values = diff_values(rows, a_key, b_key)
        add_metric(metrics, f"{name}_count", len(values))
        add_metric(metrics, f"{name}_mean", mean(values))
        add_metric(metrics, f"{name}_max_abs", max_abs(values))

    for key in all_keys(rows):
        values = numeric_values(rows, key)
        if not values:
            continue

        add_metric(metrics, f"{key}_count", len(values))
        add_metric(metrics, f"{key}_min", min(values))
        add_metric(metrics, f"{key}_max", max(values))
        add_metric(metrics, f"{key}_mean", mean(values))
        add_metric(metrics, f"{key}_last", values[-1])

    out_path = output_dir / "summary_metrics.csv"
    with out_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value"])
        writer.writerows(metrics)

    return out_path


def write_reason_counts_csv(rows: Sequence[Dict[str, str]], output_dir: Path) -> Path:
    out_path = output_dir / "reason_counts.csv"

    with out_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["reason_key", "reason", "count"])

        for key in REASON_KEYS:
            counts = Counter(row.get(key, "MISSING") for row in rows)
            for reason, count in counts.most_common():
                writer.writerow([key, reason, count])

    return out_path


def print_quick_summary(rows: Sequence[Dict[str, str]], output_dir: Path) -> None:
    print()
    print("Quick summary")
    print("-------------")
    print(f"samples: {len(rows)}")

    duration_values = numeric_values(rows, "_t")
    if duration_values:
        print(f"duration_s: {duration_values[-1] - duration_values[0]:.3f}")

    for key in [
        "outer_mrac_applied",
        "outer_mrac_update",
        "inner_mrac_applied",
        "inner_mrac_update",
        "inner_tv_applied",
        "inner_tv_update",
        "rls_est_update",
        "phase7_force_sat",
        "phase7_dfx_sat",
    ]:
        pct = percent_true(rows, key)
        if pct is not None:
            print(f"{key}: {pct:.1f}% true")

    correction_checks = [
        ("outer pwm final-baseline", diff_values(rows, "outer_pwm_final", "outer_pwm_baseline")),
        ("inner delta final-baseline [deg]", diff_values(rows, "inner_delta_final_deg", "inner_delta_baseline_deg")),
        ("phase7 F_long recomposition err [N]", numeric_values(rows, "phase7_f_long_err_n")),
        ("phase7 dFx recomposition err [N]", numeric_values(rows, "phase7_dfx_err_n")),
    ]

    for label, values in correction_checks:
        if values:
            print(f"{label}: mean={mean(values):.4g}, max_abs={max_abs(values):.4g}")

    print()
    print(f"outputs saved in: {output_dir}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Create grouped validation plots from [MRAC scaffold] logs.")
    parser.add_argument(
        "log_file",
        nargs="?",
        help="Path to a log file. If omitted, the newest logs/mrac_tests/*.log is used.",
    )
    parser.add_argument(
        "--out-dir",
        default="logs/mrac_tests/plots",
        help="Base output directory for plots and CSV files.",
    )
    parser.add_argument("--dpi", type=int, default=160, help="PNG resolution.")
    parser.add_argument(
        "--max-points",
        type=int,
        default=6000,
        help="Maximum points per line on each plot. Use 0 for no thinning.",
    )
    parser.add_argument("--show", action="store_true", help="Open plot windows after saving.")
    parser.add_argument("--list-keys", action="store_true", help="Only print keys found in the log, then exit.")

    args = parser.parse_args()

    log_path = Path(args.log_file) if args.log_file else latest_log()
    if not log_path.exists():
        raise SystemExit(f"Log file not found: {log_path}")

    rows = read_rows(log_path)

    if args.list_keys:
        print("Keys found:")
        for key in all_keys(rows):
            print(f"  {key}")
        return 0

    output_dir = Path(args.out_dir) / log_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"log file: {log_path}")
    print(f"parsed MRAC scaffold rows: {len(rows)}")
    print(f"output directory: {output_dir}")

    for index, (name, title, keys, ylabel) in enumerate(PLOT_GROUPS, start=1):
        out_path = plot_group(
            rows=rows,
            output_dir=output_dir,
            index=index,
            name=name,
            title=title,
            keys=keys,
            ylabel=ylabel,
            dpi=args.dpi,
            max_points=args.max_points,
        )
        if out_path is None:
            print(f"[skip] {name}: no matching numeric fields")
        else:
            print(f"[plot] {out_path}")

    parsed_csv = write_parsed_csv(rows, output_dir)
    summary_csv = write_summary_csv(rows, output_dir)
    reasons_csv = write_reason_counts_csv(rows, output_dir)

    print(f"[csv]  {parsed_csv}")
    print(f"[csv]  {summary_csv}")
    print(f"[csv]  {reasons_csv}")

    print_quick_summary(rows, output_dir)

    if args.show:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

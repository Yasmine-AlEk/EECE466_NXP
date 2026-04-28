#!/usr/bin/env python3

from pathlib import Path
import argparse
import re
import math
import matplotlib.pyplot as plt


IMPORTANT_KEYS = [
    # main measured / reconstructed states
    "speed_cmd",
    "vx_recon",
    "r_recon",
    "delta_f_est_deg",

    # RLS tire estimator
    "rls_phi_norm",
    "rls_update_count",
    "c_alpha_f_hat",
    "c_alpha_r_hat",
    "rls_p_trace",
    "c_alpha_f_used",
    "c_alpha_r_used",

    # outer longitudinal MRAC
    "outer_vx_ref",
    "outer_vx_m",
    "outer_ex",
    "outer_phi_norm",
    "outer_k_r_hat",
    "outer_k_x_hat",
    "outer_k_r_dot",
    "outer_k_x_dot",
    "outer_theta_norm",
    "outer_pwm_baseline",
    "outer_pwm_raw",
    "outer_pwm_sat",
    "outer_pwm_final",
    "outer_b_hat",

    # inner lateral/yaw MRAC
    "inner_uc",
    "inner_phi_norm",
    "inner_e_vy",
    "inner_e_r",
    "inner_theta_vy_hat",
    "inner_theta_r_hat",
    "inner_theta_uc_hat",
    "inner_theta_norm",
    "inner_delta_baseline_deg",
    "inner_delta_raw_deg",
    "inner_delta_sat_deg",
    "inner_delta_final_deg",
    "inner_turn_final",
]


BOOLEAN_KEYS = [
    "rls_out_valid",
    "rls_phi_ready",
    "rls_est_valid",
    "rls_est_update",
    "rls_used_ready",
    "rls_used_frozen",
    "batt_gain_valid",
    "batt_gain_update",
    "outer_mrac_valid",
    "outer_mrac_update",
    "outer_mrac_applied",
    "inner_mrac_valid",
    "inner_mrac_update",
    "inner_mrac_applied",
]


def parse_float_value(raw: str):
    """
    Converts values like:
      0.550
      -0.04
      1.26deg
      True / False
    into floats.
    """
    raw = raw.strip()

    if raw == "True":
        return 1.0
    if raw == "False":
        return 0.0

    raw = raw.replace("deg", "")

    try:
        return float(raw)
    except ValueError:
        return None


def extract_timestamp(line: str):
    """
    Extracts ROS log timestamp from:
    [INFO] [1777385256.413505021] ...
    """
    m = re.search(r"\[INFO\]\s+\[([0-9.]+)\]", line)
    if not m:
        return None
    return float(m.group(1))


def get_key_value(line: str, key: str):
    m = re.search(rf"{re.escape(key)}=([^,\s]+)", line)
    if not m:
        return None
    return parse_float_value(m.group(1))


def find_latest_log():
    log_dir = Path("logs/mrac_tests")
    patterns = [
        "inner_mrac_apply_*.log",
        "inner_mrac_blend_*.log",
        "*.log",
    ]

    logs = []
    for pattern in patterns:
        logs.extend(log_dir.glob(pattern))

    logs = sorted(set(logs), key=lambda p: p.stat().st_mtime)

    if not logs:
        raise SystemExit("No log files found inside logs/mrac_tests.")

    return logs[-1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "log_file",
        nargs="?",
        help="Path to MRAC log file. If omitted, latest log in logs/mrac_tests is used.",
    )
    parser.add_argument(
        "--include-booleans",
        action="store_true",
        help="Also plot True/False flags as 1/0.",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save PNG plots into plots/mrac_states instead of only opening windows.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open plot windows. Useful with --save.",
    )

    args = parser.parse_args()

    path = Path(args.log_file) if args.log_file else find_latest_log()

    if not path.exists():
        raise SystemExit(f"Log file not found: {path}")

    text = path.read_text(errors="ignore")
    lines = [ln for ln in text.splitlines() if "[MRAC scaffold]" in ln]

    if not lines:
        raise SystemExit(f"No '[MRAC scaffold]' lines found in {path}")

    times = []
    for i, line in enumerate(lines):
        timestamp = extract_timestamp(line)
        if timestamp is None:
            times.append(float(i))
        else:
            times.append(timestamp)

    t0 = times[0]
    x = [t - t0 for t in times]

    keys = list(IMPORTANT_KEYS)
    if args.include_booleans:
        keys += BOOLEAN_KEYS

    plot_dir = Path("plots/mrac_states")
    if args.save:
        plot_dir.mkdir(parents=True, exist_ok=True)

    print(f"Log file: {path}")
    print(f"MRAC scaffold lines: {len(lines)}")
    print(f"Plotting {len(keys)} variables...")

    plotted = 0

    for key in keys:
        y = []
        x_valid = []

        for xi, line in zip(x, lines):
            value = get_key_value(line, key)
            if value is None:
                continue

            if not math.isfinite(value):
                continue

            x_valid.append(xi)
            y.append(value)

        if not y:
            print(f"Skipping {key}: no numeric values found.")
            continue

        plotted += 1

        plt.figure(key, figsize=(9, 4.8))
        plt.plot(x_valid, y, marker="o", linewidth=1.2, markersize=3)
        plt.title(key)
        plt.xlabel("time from start [s]")
        plt.ylabel(key)
        plt.grid(True)
        plt.tight_layout()

        if args.save:
            out = plot_dir / f"{key}.png"
            plt.savefig(out, dpi=160)
            print(f"Saved {out}")

    print(f"Created {plotted} plot windows.")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Estimate practical MRAC tuning/calibration hints from parsed MRAC logs.

Input:
  logs/mrac_tests/plots/<run>/parsed_samples.csv

Output:
  logs/mrac_tests/plots/<run>/calibration_recommendations.txt

This tool does not modify the controller. It gives recommended values to
copy into mrac_config.py after checking plots.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import List, Optional, Tuple


def latest_parsed_csv() -> Path:
    candidates = sorted(
        Path("logs/mrac_tests/plots").glob("*/parsed_samples.csv"),
        key=lambda p: p.stat().st_mtime,
    )
    if not candidates:
        raise SystemExit("No parsed_samples.csv found under logs/mrac_tests/plots/*/")
    return candidates[-1]


def to_float(x) -> Optional[float]:
    if x is None:
        return None
    s = str(x).strip()
    if s in ("", "None", "nan", "NaN"):
        return None
    if s == "True":
        return 1.0
    if s == "False":
        return 0.0
    if s.endswith("deg"):
        s = s[:-3]
    try:
        y = float(s)
    except ValueError:
        return None
    if not math.isfinite(y):
        return None
    return y


def read_rows(path: Path) -> List[dict]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def vals(rows: List[dict], key: str) -> List[float]:
    out = []
    for r in rows:
        v = to_float(r.get(key))
        if v is not None:
            out.append(v)
    return out


def median(xs: List[float]) -> Optional[float]:
    if not xs:
        return None
    xs = sorted(xs)
    n = len(xs)
    if n % 2:
        return xs[n // 2]
    return 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def linreg_1d(x: List[float], y: List[float]) -> Optional[Tuple[float, float]]:
    if len(x) < 3 or len(x) != len(y):
        return None
    mx = sum(x) / len(x)
    my = sum(y) / len(y)
    den = sum((xi - mx) ** 2 for xi in x)
    if abs(den) < 1.0e-12:
        return None
    slope = sum((xi - mx) * (yi - my) for xi, yi in zip(x, y)) / den
    intercept = my - slope * mx
    return slope, intercept


def linreg_2d(x1: List[float], x2: List[float], y: List[float]) -> Optional[Tuple[float, float]]:
    if len(y) < 5 or len(x1) != len(y) or len(x2) != len(y):
        return None

    # Fit y = a*x1 + b*x2 using normal equations.
    s11 = sum(v * v for v in x1)
    s22 = sum(v * v for v in x2)
    s12 = sum(a * b for a, b in zip(x1, x2))
    sy1 = sum(a * b for a, b in zip(x1, y))
    sy2 = sum(a * b for a, b in zip(x2, y))

    det = s11 * s22 - s12 * s12
    if abs(det) < 1.0e-12:
        return None

    a = (sy1 * s22 - sy2 * s12) / det
    b = (s11 * sy2 - s12 * sy1) / det
    return a, b


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("parsed_csv", nargs="?", help="Path to parsed_samples.csv")
    args = parser.parse_args()

    path = Path(args.parsed_csv) if args.parsed_csv else latest_parsed_csv()
    rows = read_rows(path)

    out_path = path.parent / "calibration_recommendations.txt"

    # Steering: compare MRAC final to baseline.
    b_deg = []
    f_deg = []
    uc = []
    r_meas = []

    for row in rows:
        b = to_float(row.get("inner_delta_baseline_deg"))
        f = to_float(row.get("inner_delta_final_deg"))
        u = to_float(row.get("inner_uc"))
        rr = to_float(row.get("r_recon"))

        if b is not None and f is not None and abs(b) > 1.0:
            b_deg.append(b)
            f_deg.append(f)
        if u is not None and rr is not None and abs(u) > 0.01:
            uc.append(u)
            r_meas.append(rr)

    steering_fit = linreg_1d(b_deg, f_deg)
    yaw_fit = linreg_1d(uc, r_meas)

    # Longitudinal: fit a_long = b*pwm - k_v*vx.
    x_pwm = []
    x_vx_neg = []
    y_acc = []

    for row in rows:
        a = to_float(row.get("a_long_filt"))
        pwm = to_float(row.get("outer_pwm_final"))
        vx = to_float(row.get("vx_recon"))
        if a is None or pwm is None or vx is None:
            continue
        if abs(pwm) < 0.05:
            continue
        x_pwm.append(pwm)
        x_vx_neg.append(-vx)
        y_acc.append(a)

    long_fit = linreg_2d(x_pwm, x_vx_neg, y_acc)

    vx_turn_samples = []
    for row in rows:
        vx = to_float(row.get("vx_recon"))
        b = to_float(row.get("inner_delta_baseline_deg"))
        if vx is not None and b is not None and abs(b) > 2.0 and vx > 0.05:
            vx_turn_samples.append(vx)

    turn_vx_med = median(vx_turn_samples)

    lines = []
    lines.append(f"input_csv: {path}")
    lines.append(f"sample_count: {len(rows)}")
    lines.append("")

    lines.append("Inner steering calibration")
    lines.append("--------------------------")
    if steering_fit is not None:
        slope, intercept = steering_fit
        lines.append(f"fit: inner_delta_final_deg ≈ {slope:.4f} * baseline_delta_deg + {intercept:.4f}")
        lines.append("interpretation: slope > 1 means MRAC steering is stronger than baseline.")
    else:
        lines.append("not enough steering data for final-vs-baseline fit")

    if turn_vx_med is not None:
        lines.append(f"median vx during |baseline steering| > 2 deg: {turn_vx_med:.4f} m/s")
        lines.append(f"candidate INNER_MRAC_NOMINAL_VX_MS: {max(0.20, min(0.45, turn_vx_med)):.4f}")

    lines.append("")

    lines.append("Yaw reference calibration")
    lines.append("-------------------------")
    if yaw_fit is not None:
        slope, intercept = yaw_fit
        lines.append(f"fit: r_recon ≈ {slope:.4f} * inner_uc + {intercept:.4f}")
        lines.append("interpretation: slope far from 1 means inner_uc scaling is not calibrated.")
    else:
        lines.append("not enough yaw data for r_recon-vs-inner_uc fit")

    lines.append("")

    lines.append("Outer longitudinal calibration")
    lines.append("------------------------------")
    if long_fit is not None:
        b_pwm, k_v = long_fit
        lines.append(f"fit: a_long_filt ≈ {b_pwm:.4f} * outer_pwm_final - {k_v:.4f} * vx_recon")
        lines.append(f"candidate OUTER_LONGITUDINAL_K_V_S: {max(0.0, k_v):.4f}")
        lines.append(f"candidate battery/input gain b estimate: {b_pwm:.4f} m/s^2 per PWM")
    else:
        lines.append("not enough longitudinal data for a_long = b*pwm - k_v*vx fit")

    lines.append("")

    lines.append("Suggested next config block, after checking plots")
    lines.append("------------------------------------------------")
    if turn_vx_med is not None:
        v_nom = max(0.20, min(0.45, turn_vx_med))
        lines.append(f"INNER_MRAC_NOMINAL_VX_MS = {v_nom:.4f}")
        lines.append("INNER_MRAC_THETA_UC_INITIAL = KINEMATIC_WHEELBASE_M / max(INNER_MRAC_NOMINAL_VX_MS, 1.0e-6)")
    if long_fit is not None:
        _, k_v = long_fit
        lines.append(f"OUTER_LONGITUDINAL_K_V_S = {max(0.0, k_v):.4f}")

    out_path.write_text("\n".join(lines) + "\n")
    print(out_path)
    print()
    print(out_path.read_text())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

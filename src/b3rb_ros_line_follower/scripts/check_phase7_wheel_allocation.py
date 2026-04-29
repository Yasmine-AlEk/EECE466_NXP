#!/usr/bin/env python3

from pathlib import Path
import re
import sys
from collections import Counter


def get_value(key, line):
    m = re.search(rf"{re.escape(key)}=([^,\s]+)", line)
    return m.group(1) if m else None


def get_float(key, line):
    value = get_value(key, line)

    if value is None:
        return None

    try:
        return float(value)
    except ValueError:
        return None


def percent(n, total):
    return 100.0 * n / total if total else 0.0


def main():
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        candidates = sorted(
            Path("logs/mrac_tests").glob("*.log"),
            key=lambda p: p.stat().st_mtime,
        )

        if not candidates:
            raise SystemExit("No logs found in logs/mrac_tests. Pass a log path manually.")

        path = candidates[-1]

    text = path.read_text(errors="ignore")
    lines = [line for line in text.splitlines() if "[MRAC scaffold]" in line]

    if not lines:
        raise SystemExit(f"No MRAC scaffold lines found in {path}")

    mode_counts = Counter(get_value("phase7_mode", line) for line in lines)
    outer_source_counts = Counter(get_value("phase7_outer_pwm_source", line) for line in lines)
    dfx_source_counts = Counter(get_value("phase7_dfx_source", line) for line in lines)

    f_long_errs = [
        abs(v)
        for line in lines
        if (v := get_float("phase7_f_long_err_n", line)) is not None
    ]

    dfx_errs = [
        abs(v)
        for line in lines
        if (v := get_float("phase7_dfx_err_n", line)) is not None
    ]

    wheel_common = [
        v
        for line in lines
        if (v := get_float("phase7_pwm_common", line)) is not None
    ]

    outer_pwm = [
        v
        for line in lines
        if (v := get_float("phase7_outer_pwm_cmd", line)) is not None
    ]

    phase7_samples = sum(count for key, count in mode_counts.items() if key not in (None, "None"))

    print(f"Log file: {path}")
    print(f"Total MRAC scaffold samples: {len(lines)}")
    print()
    print("=== Phase 7 actuation source ===")
    print(
        f"Phase 7 allocation samples: "
        f"{phase7_samples} / {len(lines)} = {percent(phase7_samples, len(lines)):.1f}%"
    )
    print(f"phase7_mode: {dict(mode_counts)}")
    print(f"outer PWM source: {dict(outer_source_counts)}")
    print(f"dfx source: {dict(dfx_source_counts)}")
    print()
    print("=== Recomposition validation ===")
    print(f"max |F_long reconstruction error|: {max(f_long_errs) if f_long_errs else None}")
    print(f"max |Delta_Fx reconstruction error|: {max(dfx_errs) if dfx_errs else None}")
    print()
    print("=== Command ranges ===")

    if wheel_common:
        print(
            "phase7_pwm_common: "
            f"min={min(wheel_common):.4f}, "
            f"max={max(wheel_common):.4f}, "
            f"last={wheel_common[-1]:.4f}"
        )

    if outer_pwm:
        print(
            "phase7_outer_pwm_cmd: "
            f"min={min(outer_pwm):.4f}, "
            f"max={max(outer_pwm):.4f}, "
            f"last={outer_pwm[-1]:.4f}"
        )


if __name__ == "__main__":
    main()

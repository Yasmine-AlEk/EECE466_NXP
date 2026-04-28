#!/usr/bin/env python3

import argparse
import math
import sys
from pathlib import Path


MARKER = "[MRAC scaffold]"


def parse_bool(value):
    if value is None:
        return None
    if value == "True":
        return True
    if value == "False":
        return False
    return None


def parse_float(value):
    if value is None:
        return None

    value = value.strip()

    if value in ("None", "nan", "NaN"):
        return None

    if value.endswith("deg"):
        value = value[:-3]

    try:
        x = float(value)
    except ValueError:
        return None

    if not math.isfinite(x):
        return None

    return x


def parse_row(line):
    if MARKER not in line:
        return None

    payload = line.split(MARKER, 1)[1].strip()
    row = {}

    for part in payload.split(","):
        part = part.strip()
        if "=" not in part:
            continue

        key, value = part.split("=", 1)
        row[key.strip()] = value.strip()

    return row


def numbers(rows, key):
    out = []
    for row in rows:
        x = parse_float(row.get(key))
        if x is not None:
            out.append(x)
    return out


def fmt(x, nd=3):
    if x is None:
        return "None"
    return f"{x:.{nd}f}"


def span(values):
    if not values:
        return None, None, None
    return min(values), max(values), max(values) - min(values)


def print_section(title):
    print()
    print("=" * len(title))
    print(title)
    print("=" * len(title))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_file")
    parser.add_argument("--min-used-ready-samples", type=int, default=20)
    parser.add_argument("--used-max-range", type=float, default=0.60)
    parser.add_argument("--used-max-drift", type=float, default=0.50)
    parser.add_argument("--batt-min", type=float, default=0.60)
    parser.add_argument("--batt-max", type=float, default=1.60)
    parser.add_argument("--batt-max-range", type=float, default=0.45)
    parser.add_argument("--require-both-turns", action="store_true")
    args = parser.parse_args()

    path = Path(args.log_file)
    if not path.exists():
        print(f"ERROR: log file not found: {path}")
        return 2

    rows = []
    for line in path.read_text(errors="ignore").splitlines():
        row = parse_row(line)
        if row is not None:
            rows.append(row)

    hard_fail = False

    def ok(msg):
        print(f"✅ {msg}")

    def warn(msg):
        print(f"⚠️  {msg}")

    def fail(msg):
        nonlocal hard_fail
        hard_fail = True
        print(f"❌ {msg}")

    print_section("Log summary")
    print(f"Parsed MRAC scaffold rows: {len(rows)}")

    if len(rows) == 0:
        fail("No [MRAC scaffold] rows were found.")
        return 1

    # ------------------------------------------------------------
    # 1. Baseline PID untouched
    # ------------------------------------------------------------
    print_section("1. Baseline PID untouched")

    applied_keys = [
        "mrac_applied",
        "inner_mrac_applied",
        "outer_mrac_applied",
        "mrac_inner_applied",
        "mrac_outer_applied",
    ]

    applied_true = []
    for i, row in enumerate(rows):
        for key in applied_keys:
            if parse_bool(row.get(key)) is True:
                applied_true.append((i, key, row.get(key)))

    if applied_true:
        fail(f"MRAC appears to have been applied in {len(applied_true)} rows.")
    else:
        ok("No MRAC-applied flags were found as True. Baseline appears untouched from logs.")

    # ------------------------------------------------------------
    # 2. Used cornering stiffness stability
    # ------------------------------------------------------------
    print_section("2. c_alpha_f_used / c_alpha_r_used stability")

    used_rows = [
        row for row in rows
        if parse_bool(row.get("rls_used_ready")) is True
        and parse_float(row.get("c_alpha_f_used")) is not None
        and parse_float(row.get("c_alpha_r_used")) is not None
    ]

    print(f"Rows with rls_used_ready=True: {len(used_rows)}")

    if len(used_rows) < args.min_used_ready_samples:
        fail(
            "Not enough ready used-stiffness samples. "
            f"Need at least {args.min_used_ready_samples}, got {len(used_rows)}."
        )
    else:
        caf_used = numbers(used_rows, "c_alpha_f_used")
        car_used = numbers(used_rows, "c_alpha_r_used")

        caf_min, caf_max, caf_range = span(caf_used)
        car_min, car_max, car_range = span(car_used)

        caf_drift = abs(caf_used[-1] - caf_used[0])
        car_drift = abs(car_used[-1] - car_used[0])

        print(
            f"c_alpha_f_used: start={fmt(caf_used[0])}, end={fmt(caf_used[-1])}, "
            f"min={fmt(caf_min)}, max={fmt(caf_max)}, range={fmt(caf_range)}, drift={fmt(caf_drift)}"
        )
        print(
            f"c_alpha_r_used: start={fmt(car_used[0])}, end={fmt(car_used[-1])}, "
            f"min={fmt(car_min)}, max={fmt(car_max)}, range={fmt(car_range)}, drift={fmt(car_drift)}"
        )

        if caf_range <= args.used_max_range and car_range <= args.used_max_range:
            ok("Used cornering stiffness range is bounded.")
        else:
            fail("Used cornering stiffness range is too large.")

        if caf_drift <= args.used_max_drift and car_drift <= args.used_max_drift:
            ok("Used cornering stiffness drift is acceptable.")
        else:
            fail("Used cornering stiffness drift is too large.")

    # ------------------------------------------------------------
    # 3. Left/right turn sign behavior
    # ------------------------------------------------------------
    print_section("3. Left/right turn sign behavior")

    turn_rows = []
    same_sign = 0
    opposite_sign = 0
    positive_turns = 0
    negative_turns = 0

    for row in rows:
        r = parse_float(row.get("r_recon"))
        delta_deg = parse_float(row.get("delta_f_est_deg"))
        phi_ready = parse_bool(row.get("rls_phi_ready"))

        if r is None or delta_deg is None:
            continue

        if phi_ready is not True:
            continue

        if abs(r) < 0.03 or abs(delta_deg) < 0.50:
            continue

        turn_rows.append(row)

        if r > 0:
            positive_turns += 1
        elif r < 0:
            negative_turns += 1

        if r * delta_deg > 0:
            same_sign += 1
        elif r * delta_deg < 0:
            opposite_sign += 1

    total_sign = same_sign + opposite_sign
    same_sign_ratio = same_sign / total_sign if total_sign > 0 else None

    print(f"Sign-check turn samples: {len(turn_rows)}")
    print(f"Positive-r turn samples: {positive_turns}")
    print(f"Negative-r turn samples: {negative_turns}")

    if total_sign > 0:
        print(
            f"r_recon and delta_f_est_deg same-sign ratio: "
            f"{100.0 * same_sign_ratio:.1f}%"
        )

    if positive_turns > 0 and negative_turns > 0:
        ok("Both turn directions appeared in the log.")
    else:
        msg = "Only one turn direction was strongly represented."
        if args.require_both_turns:
            fail(msg)
        else:
            warn(msg + " Run a test with both left and right turns before MRAC.")

    if total_sign > 0 and same_sign_ratio is not None and same_sign_ratio >= 0.70:
        ok("Yaw-rate and steering sign behavior looks mostly consistent.")
    elif total_sign > 0:
        warn("Yaw-rate and steering signs are mixed. Check left/right sign convention before MRAC.")
    else:
        warn("Not enough strong turn samples for sign validation.")

    # ------------------------------------------------------------
    # 4. Battery estimator stability
    # ------------------------------------------------------------
    print_section("4. Battery estimator stability")

    batt_values = numbers(rows, "batt_b_hat")

    if not batt_values:
        fail("No batt_b_hat values found.")
    else:
        batt_min, batt_max, batt_range = span(batt_values)

        print(
            f"batt_b_hat: start={fmt(batt_values[0])}, end={fmt(batt_values[-1])}, "
            f"min={fmt(batt_min)}, max={fmt(batt_max)}, range={fmt(batt_range)}"
        )

        if batt_min >= args.batt_min and batt_max <= args.batt_max:
            ok("Battery gain estimate stayed positive and bounded.")
        else:
            fail("Battery gain estimate went outside safe bounds.")

        if batt_range <= args.batt_max_range:
            ok("Battery gain estimate range is acceptable.")
        else:
            fail("Battery gain estimate range is too large.")

    bad_batt_turn_updates = []
    for i, row in enumerate(rows):
        batt_update = parse_bool(row.get("batt_gain_update"))
        batt_reason = row.get("batt_reason", "")

        if batt_update is True and batt_reason.startswith("turning"):
            bad_batt_turn_updates.append((i, batt_reason))

    if bad_batt_turn_updates:
        fail(
            "Battery estimator updated during turning segments "
            f"({len(bad_batt_turn_updates)} rows)."
        )
    else:
        ok("Battery estimator did not update when batt_reason was turning_*.")

    # ------------------------------------------------------------
    # 5. RLS-used freeze behavior
    # ------------------------------------------------------------
    print_section("5. RLS-used freeze behavior")

    used_fields_present = any("rls_used_ready" in row for row in rows)

    if not used_fields_present:
        fail("Task 5.6 used-estimate fields are missing from debug output.")
    else:
        ok("Task 5.6 used-estimate fields are present.")

    bad_not_ready_unfrozen = []
    bad_no_update_unfrozen = []

    for i, row in enumerate(rows):
        used_ready = parse_bool(row.get("rls_used_ready"))
        used_frozen = parse_bool(row.get("rls_used_frozen"))
        est_update = parse_bool(row.get("rls_est_update"))

        if used_ready is False and used_frozen is False:
            bad_not_ready_unfrozen.append(i)

        if est_update is False and used_frozen is False:
            bad_no_update_unfrozen.append(i)

    if bad_not_ready_unfrozen:
        fail("Used estimate was not ready but not frozen.")
    else:
        ok("Used estimate stays frozen before readiness.")

    if bad_no_update_unfrozen:
        warn(
            "Some rows show rls_est_update=False while rls_used_frozen=False. "
            "This may be okay if log sampling skipped an update instant, but inspect it."
        )
    else:
        ok("Used estimate freezes on no-update rows.")

    # ------------------------------------------------------------
    # 6. Raw RLS update sanity
    # ------------------------------------------------------------
    print_section("6. Raw RLS update sanity")

    update_counts = numbers(rows, "rls_update_count")
    p_traces = numbers(rows, "rls_p_trace")
    caf_hat = numbers(rows, "c_alpha_f_hat")
    car_hat = numbers(rows, "c_alpha_r_hat")

    if update_counts:
        print(f"rls_update_count: start={fmt(update_counts[0], 0)}, end={fmt(update_counts[-1], 0)}")
        if update_counts[-1] > update_counts[0]:
            ok("RLS update count increased during the simulation.")
        else:
            warn("RLS update count did not increase. Run longer or check gating.")
    else:
        fail("No rls_update_count values found.")

    if p_traces:
        p_min, p_max, p_range = span(p_traces)
        print(f"rls_p_trace: min={fmt(p_min)}, max={fmt(p_max)}, range={fmt(p_range)}")
        if p_min > 0.0 and p_max < 100.0:
            ok("RLS covariance trace stayed positive and bounded.")
        else:
            fail("RLS covariance trace is suspicious.")

    if caf_hat and car_hat:
        caf_hat_min, caf_hat_max, _ = span(caf_hat)
        car_hat_min, car_hat_max, _ = span(car_hat)
        print(f"c_alpha_f_hat raw range: {fmt(caf_hat_min)} to {fmt(caf_hat_max)}")
        print(f"c_alpha_r_hat raw range: {fmt(car_hat_min)} to {fmt(car_hat_max)}")

        if caf_hat_min > 0.0 and car_hat_min > 0.0:
            ok("Raw RLS estimates stayed positive.")
        else:
            fail("Raw RLS estimates became non-positive.")

    print_section("Final result")

    if hard_fail:
        print("❌ Validation failed. Do not move to MRAC yet.")
        return 1

    print("✅ Validation passed with no hard failures.")
    print("MRAC should still be introduced only in shadow mode first.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

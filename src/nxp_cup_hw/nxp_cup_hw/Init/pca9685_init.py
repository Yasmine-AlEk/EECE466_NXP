#!/usr/bin/env python3
"""
PCA9685 Initialization Script
- Calibrates Avian Smart 60A ESCs on channels 2 and 3
- Centers servo on channel 1 to 90 degrees
- Holds all outputs so devices stay silent and armed

Designed to run at boot via systemd.

Usage:
    python3 pca9685_init.py                  # calibrate ESCs + center servo
    python3 pca9685_init.py --skip-calib     # skip ESC calibration, just arm + center
    python3 pca9685_init.py --check          # print all channel values and exit

Requirements:
    pip3 install smbus2 --break-system-packages

Systemd install:
    sudo cp pca9685_init.py /usr/local/bin/pca9685_init.py
    sudo chmod +x /usr/local/bin/pca9685_init.py
    sudo cp pca9685.service /etc/systemd/system/
    sudo systemctl enable pca9685
    sudo systemctl start pca9685
"""

import smbus2
import time
import argparse
import sys
# inside nxp_bldc_node.py and nxp_servo_node.py
from nxp_cup_hw.Init.pca9685_init import PCA9685
# ── Config ────────────────────────────────────────────────────────────────────
I2C_BUS       = 5
PCA_ADDR      = 0x40
PWM_FREQ_HZ   = 50

SERVO_CHANNEL = 1
SERVO_MIN_US  = 500
SERVO_MAX_US  = 2500
SERVO_CENTER  = 90      # degrees

ESC_CHANNELS  = [2, 3]
ESC_MAX_US    = 2000
ESC_MIN_US    = 1000

# ── Registers ─────────────────────────────────────────────────────────────────
REG_MODE1     = 0x00
REG_PRESCALE  = 0xFE
REG_LED0_ON_L = 0x06

# ── PCA helpers ───────────────────────────────────────────────────────────────
def pca_init(bus, addr):
    bus.write_byte_data(addr, REG_MODE1, 0x00)
    time.sleep(0.01)


def set_pwm_freq(bus, addr, freq_hz):
    prescale = round(25_000_000 / (4096 * freq_hz)) - 1
    old      = bus.read_byte_data(addr, REG_MODE1)
    bus.write_byte_data(addr, REG_MODE1, (old & 0x7F) | 0x10)
    bus.write_byte_data(addr, REG_PRESCALE, prescale)
    bus.write_byte_data(addr, REG_MODE1, old)
    time.sleep(0.005)
    bus.write_byte_data(addr, REG_MODE1, old | 0x80)
    print(f"  PWM frequency: {freq_hz} Hz (prescale={prescale})")


def set_us(bus, addr, channel, pulse_us):
    off = int((pulse_us / (1_000_000 / PWM_FREQ_HZ)) * 4096)
    off = max(0, min(4095, off))
    reg = REG_LED0_ON_L + 4 * channel
    bus.write_byte_data(addr, reg + 0, 0x00)
    bus.write_byte_data(addr, reg + 1, 0x00)
    bus.write_byte_data(addr, reg + 2, off & 0xFF)
    bus.write_byte_data(addr, reg + 3, (off >> 8) & 0x0F)
    return off


def disable_channel(bus, addr, channel):
    reg = REG_LED0_ON_L + 4 * channel
    bus.write_byte_data(addr, reg + 0, 0x00)
    bus.write_byte_data(addr, reg + 1, 0x00)
    bus.write_byte_data(addr, reg + 2, 0x00)
    bus.write_byte_data(addr, reg + 3, 0x10)


def read_us(bus, addr, channel):
    reg   = REG_LED0_ON_L + 4 * channel
    off_l = bus.read_byte_data(addr, reg + 2)
    off_h = bus.read_byte_data(addr, reg + 3)
    tick  = ((off_h & 0x0F) << 8) | off_l
    return (tick / 4096) * (1_000_000 / PWM_FREQ_HZ)


def deg_to_us(deg):
    return SERVO_MIN_US + (deg / 180.0) * (SERVO_MAX_US - SERVO_MIN_US)


# ── ESC calibration ───────────────────────────────────────────────────────────
def calibrate_escs(bus):
    """
    Avian Smart 60A calibration per Spektrum official manual:
      1. Full throttle signal present BEFORE battery connect
      2. Connect battery -> 3 ascending tones -> 2 short tones (high accepted)
      3. Drop to min within 5 seconds
      4. Cell count tones -> 1 long tone = done
      5. Hold min = armed and silent
    """
    print("\n" + "=" * 55)
    print("  Avian Smart 60A ESC Calibration")
    print("=" * 55)
    print("  ESC battery must NOT be connected yet.")
    input("\n  Press Enter when ready...\n")

    # Step 1 - full throttle + verify
    print("  Step 1/3: Full throttle on ESC channels...")
    for ch in ESC_CHANNELS:
        tick = set_us(bus, PCA_ADDR, ch, ESC_MAX_US)
        actual = read_us(bus, PCA_ADDR, ch)
        print(f"    CH{ch}: {ESC_MAX_US}us -> tick {tick} -> readback {actual:.0f}us")
        if actual < 1900:
            print(f"    ERROR: CH{ch} readback too low ({actual:.0f}us). Aborting.")
            return False

    print("\n  --> Connect ESC battery NOW <--")
    print("  Waiting for 3 ascending tones + 2 short tones...")
    for i in range(6, 0, -1):
        print(f"    {i}s", flush=True)
        time.sleep(8)

    # Step 2 - min throttle
    print("\n  Step 2/3: Min throttle...")
    for ch in ESC_CHANNELS:
        set_us(bus, PCA_ADDR, ch, ESC_MIN_US)
        print(f"    CH{ch}: {ESC_MIN_US}us")

    print("  Waiting for cell-count tones + 1 long tone...")
    for i in range(6, 0, -1):
        print(f"    {i}s", flush=True)
        time.sleep(1)

    # Step 3 - arm hold
    print("\n  Step 3/3: Arming (holding min 3s)...")
    time.sleep(3)

    print("\n  ESCs calibrated and armed.")
    print("=" * 55)
    return True


# ── Arm without calibration ───────────────────────────────────────────────────
def arm_escs(bus):
    """Send min throttle to arm ESCs without full calibration."""
    print("  Arming ESCs with min throttle (1000us)...")
    for ch in ESC_CHANNELS:
        set_us(bus, PCA_ADDR, ch, ESC_MIN_US)
        print(f"    CH{ch}: {ESC_MIN_US}us")
    time.sleep(2)
    print("  ESCs armed.")


# ── Servo center ──────────────────────────────────────────────────────────────
def center_servo(bus):
    pulse = deg_to_us(SERVO_CENTER)
    tick  = set_us(bus, PCA_ADDR, SERVO_CHANNEL, pulse)
    print(f"  Servo CH{SERVO_CHANNEL}: {SERVO_CENTER}deg = {pulse:.1f}us -> tick {tick}")


# ── Channel check ─────────────────────────────────────────────────────────────
def check_channels(bus):
    print(f"\n  {'CH':<5} {'us':>10}  status")
    print("  " + "-" * 28)
    for ch in range(16):
        reg   = REG_LED0_ON_L + 4 * ch
        off_h = bus.read_byte_data(PCA_ADDR, reg + 3)
        if off_h & 0x10:
            print(f"  CH{ch:<3}  {'---':>10}  DISABLED")
        else:
            us = read_us(bus, PCA_ADDR, ch)
            print(f"  CH{ch:<3}  {us:>10.1f}  ACTIVE")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="PCA9685 boot initialization")
    parser.add_argument("--skip-calib", action="store_true",
                        help="Skip ESC calibration, just arm and center servo")
    parser.add_argument("--check",      action="store_true",
                        help="Print all channel values and exit")
    args = parser.parse_args()

    try:
        print(f"\nPCA9685 init — i2c-{I2C_BUS} @ 0x{PCA_ADDR:02X}")
        bus = smbus2.SMBus(I2C_BUS)
        pca_init(bus, PCA_ADDR)
        set_pwm_freq(bus, PCA_ADDR, PWM_FREQ_HZ)

        if args.check:
            check_channels(bus)
            bus.close()
            return

        # ESC init
        if args.skip_calib:
            print("\n[ESC] Skipping calibration, arming directly...")
            arm_escs(bus)
        else:
            print("\n[ESC] Starting calibration...")
            if not calibrate_escs(bus):
                print("ERROR: ESC calibration failed.")
                bus.close()
                sys.exit(1)

        # Servo center
        print("\n[Servo] Centering to 90 degrees...")
        center_servo(bus)

        # Summary
        print("\n[Done] All outputs active:")
        check_channels(bus)

        bus.close()
        print("Done. PCA9685 holding outputs independently.")

    except PermissionError:
        print("ERROR: Permission denied. Run: sudo chmod 666 /dev/i2c-5")
        sys.exit(1)
    except OSError as e:
        print(f"ERROR: I2C error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
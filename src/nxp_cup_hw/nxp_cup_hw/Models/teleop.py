#!/usr/bin/env python3
import sys, os, tty, termios, select, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# ── Parameters ────────────────────────────────────────────────────────────────
LINEAR_ACCEL  = 0.10   # m/s per key press  (above 0.05 deadzone)
LINEAR_MAX    = 2.0    # m/s
LINEAR_DECAY  = 0.0    # disabled — commands latch until key pressed

ANGULAR_STEP  = 0.10   # rad/s per key press
ANGULAR_MAX   = 1.5    # rad/s
ANGULAR_DECAY = 0.0    # disabled

LOOP_HZ       = 20
# ─────────────────────────────────────────────────────────────────────────────

ESC        = b'\x1b'
ARROW_UP   = b'\x1b[A'
ARROW_DOWN = b'\x1b[B'
ARROW_RIGHT= b'\x1b[C'
ARROW_LEFT = b'\x1b[D'

def clamp(val, limit): return max(-limit, min(limit, val))

def read_key():
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    ch = os.read(sys.stdin.fileno(), 1)
    if ch == ESC and select.select([sys.stdin], [], [], 0.05)[0]:
        ch += os.read(sys.stdin.fileno(), 2)
    return ch

def main():
    if not sys.stdin.isatty():
        print("[teleop] No TTY. Run with: ros2 run nxp_cup_hw teleop", flush=True)
        sys.exit(1)

    rclpy.init()
    node = Node('nxp_teleop')
    pub  = node.create_publisher(TwistStamped, '/nxp_cup/cmd', 10)

    v = 0.0;  omega = 0.0;  dt = 1.0 / LOOP_HZ
    fd = sys.stdin.fileno();  old_attr = termios.tcgetattr(fd)

    print("\n  NXP Cup Teleop  →  /nxp_cup/cmd")
    print("  up/W accel   down/S brake   left/A steer-L   right/D steer-R   SPACE stop   Q quit\n")

    try:
        tty.setraw(fd)
        while True:
            key = read_key()
            if key is not None:
                if   key in (b'q', b'Q', ESC, b'\x03'):  break
                elif key == b' ':                          v = 0.0;  omega = 0.0
                elif key in (ARROW_UP,    b'w', b'W'):    v     = clamp(v     + LINEAR_ACCEL, LINEAR_MAX)
                elif key in (ARROW_DOWN,  b's', b'S'):    v     = clamp(v     - LINEAR_ACCEL, LINEAR_MAX)
                elif key in (ARROW_LEFT,  b'a', b'A'):    omega = clamp(omega + ANGULAR_STEP, ANGULAR_MAX)
                elif key in (ARROW_RIGHT, b'd', b'D'):    omega = clamp(omega - ANGULAR_STEP, ANGULAR_MAX)

            msg = TwistStamped()
            msg.header.stamp    = node.get_clock().now().to_msg()
            msg.twist.linear.x  = v
            msg.twist.angular.z = omega
            pub.publish(msg)

            sys.stdout.write(f"\r  v={v:+6.3f} m/s   omega={omega:+6.3f} rad/s")
            sys.stdout.flush()

            time.sleep(dt)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        rclpy.shutdown()
        print("\n  Teleop stopped.")

if __name__ == "__main__":
    main()
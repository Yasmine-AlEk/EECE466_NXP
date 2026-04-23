from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Vision ────────────────────────────────────────────────────────────
        Node(
            package="nxp_cup_hw",
            executable="nxp_track_vision",
            name="nxp_track_vision",
            output="screen",
        ),

        # ── Encoders  (PCF8574 @ 0x20, P0=left P1=right, 10 counts/rev) ──────
        # Node(
        #     package="nxp_cup_hw",
        #     executable="nxp_encoder_node",
        #     name="nxp_encoder_node",
        #     output="screen",
        #     parameters=[{
        #         "i2c_bus":        5,
        #         "i2c_addr":       0x20,
        #         "left_pin":       0,
        #         "right_pin":      1,
        #         "counts_per_rev": 10,
        #         "wheel_radius_m": 0.033,
        #         "wheel_base_m":   0.165,
        #         "poll_hz":        500,
        #     }],
        # ),

        # ── IMU  (MPU6050 @ 0x68) ─────────────────────────────────────────────
        Node(
            package="nxp_cup_hw",
            executable="nxp_imu_node",
            name="nxp_imu_node",
            output="screen",
            parameters=[{
                "i2c_bus":     5,
                "i2c_addr":    0x68,
                "accel_range": 2,
                "gyro_range":  250,
                "publish_hz":  100,
                "calibrate":   True,
            }],
        ),

        # ── Servo  (PCA9685 @ 0x40, channel 0) ───────────────────────────────
        Node(
            package="nxp_cup_hw",
            executable="nxp_servo_node",
            name="nxp_servo_node",
            output="screen",
            parameters=[{
                "i2c_bus":      5,
                "i2c_addr":     0x40,
                "channel":      0,
                "pwm_freq_hz":  50,
                "pulse_min_us": 1000,
                "pulse_mid_us": 1500,
                "pulse_max_us": 2000,
                "rate_limit":   5.0,
                "timeout_sec":  0.5,
            }],
        ),

        # ── BLDCs  (PCA9685 @ 0x40, ch1=left ch2=right) ──────────────────────
        Node(
            package="nxp_cup_hw",
            executable="nxp_bldc_node",
            name="nxp_bldc_node",
            output="screen",
            parameters=[{
                "i2c_bus":        5,
                "i2c_addr":       0x40,
                "channel_left":   1,
                "channel_right":  2,
                "pwm_freq_hz":    50,
                "pulse_stop_us":  1500,
                "pulse_min_us":   1000,
                "pulse_max_us":   2000,
                "max_speed":      0.7,
                "ramp_rate":      2.0,
                "timeout_sec":    0.5,
                "arm_on_startup": True,
                "differential_k": 0.0,
            }],
        ),

        # ── Stream viewer ────────────────────────────────────────────────────────
        Node(
            package="nxp_cup_hw",
            executable="vision_stream",
            name="vision_stream",
            output="screen",

        ),

        # ── Bicycle model ─────────────────────────────────────────────────────
        Node(
            package="nxp_cup_hw",
            executable="nxp_bicycle_model_node",
            name="nxp_bicycle_model_node",
            output="screen",
            parameters=[{
                "wheelbase_m":   0.165,
                "max_steer_deg": 30.0,
                "max_speed_ms":  2.0,
                "timeout_sec":   0.5,
            }],
        ),

    ])
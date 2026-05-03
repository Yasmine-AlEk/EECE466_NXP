from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Camera driver ─────────────────────────────────────────────────────
        Node(
            package="nxp_cup_hw",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{
                "device":       "/dev/video3",
                "width":        640,
                "height":       480,
                "fps":          30,
                "jpeg_quality": 80,
            }],
        ),

        # ── Vision (nxp_cup_vision package) ──────────────────────────────────
        # Publishes /edge_vectors (EdgeVectors) + /nxp_cup/debug_image directly
        Node(
            package="nxp_cup_vision",
            executable="nxp_track_vision",
            name="nxp_track_vision",
            output="screen",
            parameters=[{'debug': True}],
        ),

        # ── Vision stream dashboard  (http://<navqplus-ip>:8081) ─────────────
        # Node(
        #     package="nxp_cup_hw",
        #     executable="vision_stream",
        #     name="vision_stream",
        #     output="screen",
        # ),

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
                "v_deadzone_ms": 0.01,
            }],
        ),

        # ── PCA9685 (motors + servo) ───────────────────────────────────────────
        Node(
            package="nxp_cup_hw",
            executable="pca9685_node",
            output="screen",
            parameters=[{'min_speed_only': False}],
        ),

        # ── Encoder odometry ──────────────────────────────────────────────────
        Node(
            package="nxp_cup_hw",
            executable="pcf8574ap_node",
            output="screen",
        ),

        # ── IMU ───────────────────────────────────────────────────────────────
        # Commented out — OSError on i2c-5, re-enable when IMU is connected
        # Node(
        #     package="nxp_cup_hw",
        #     executable="mpu6050_node",
        #     output="screen",
        # ),

        # ── Odometry fusion ───────────────────────────────────────────────────
        Node(
            package="nxp_cup_hw",
            executable="odom_fusion_node",
            output="screen",
            parameters=[{"use_imu": False}],
        ),
    ])
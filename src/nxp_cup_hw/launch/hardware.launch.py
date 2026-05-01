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
                "v_deadzone_ms": 0.01,
            }],
        ),

        Node(
            package="nxp_cup_hw",
            executable="pca9685_node", 
            output="screen"),

        Node(
            package="nxp_cup_hw",
            executable="pcf8574ap_node", 
            output="screen"),

    ])
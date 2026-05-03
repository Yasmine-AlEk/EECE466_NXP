from setuptools import find_packages, setup
import os
from glob import glob

package_name = "nxp_cup_hw"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "nxp_cup_hw": ["I2C/motor_calibration.csv"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="NXP Cup hardware interface nodes",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nxp_bldc_node          = nxp_cup_hw.Actuators.nxp_bldc_node:main",
            "nxp_servo_node         = nxp_cup_hw.Actuators.nxp_servo_node:main",
            "nxp_encoder_node       = nxp_cup_hw.Sensors.nxp_encoder_node:main",
            "nxp_imu_node           = nxp_cup_hw.Sensors.nxp_imu_node:main",
            "nxp_track_vision       = nxp_cup_hw.Vision.vision_chain:main",
            "nxp_cam_init           = nxp_cup_hw.Vision.vision_basic:main",
            "nxp_bicycle_model_node = nxp_cup_hw.Models.bicycle:main",
            "vision_stream          = nxp_cup_hw.Vision.vision_stream:main",
            "teleop                 = nxp_cup_hw.Models.teleop:main",
            "pca9685_node           = nxp_cup_hw.I2C.pca9685_node:main",
            "pcf8574ap_node         = nxp_cup_hw.I2C.pcf8574ap_node:main",
            "mpu6050_node           = nxp_cup_hw.I2C.mpu6050_node:main",
            "odom_fusion_node       = nxp_cup_hw.Models.odom_fusion_node:main",
            "lane_to_edge_vectors   = nxp_cup_hw.Vision.lane_to_edge_vectors:main",
        ],
    },
)
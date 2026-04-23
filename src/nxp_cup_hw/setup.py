from setuptools import find_packages, setup
import os
from glob import glob

package_name = "nxp_cup_hw"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
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
        ],
    },
)
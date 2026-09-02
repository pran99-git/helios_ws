import os
from glob import glob

from setuptools import find_packages, setup

package_name = "low_level_control_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pran",
    maintainer_email="upadhyp1@uci.edu",
    description="Low-level control: RoboClaw hardware driver + joystick teleop (Lynxmotion A4WD3)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_joy_node = teleop.teleop_joy_node:main",
            "roboclaw_driver_node = roboclaw.roboclaw_driver_node:main",
            "wheel_monitor = calibration.wheel_monitor:main",
            "calibrate_qpps = calibration.calibrate_qpps:main",
        ],
    },
)

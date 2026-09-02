import os
from glob import glob

from setuptools import find_packages, setup

package_name = "custom_covariance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pran",
    maintainer_email="pranavupadhyay1202@gmail.com",
    description="Workspace-owned sensor covariance corrections for the ZED.",
    license="MIT",
    entry_points={
        "console_scripts": [
            (
                "zed_odom_covariance_node = "
                "custom_covariance.zed_odom_covariance_node:main"
            ),
        ],
    },
)

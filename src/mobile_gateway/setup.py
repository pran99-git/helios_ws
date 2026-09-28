from setuptools import find_packages, setup

setup(
    name="mobile_gateway",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mobile_gateway"]),
        ("share/mobile_gateway", ["package.xml", "README.md", "OPERATIONS.md", "PRANAV_SETUP.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pran",
    maintainer_email="upadhyp1@uci.edu",
    description="Authenticated read-only mobile telemetry gateway for Helios",
    license="Apache-2.0",
    entry_points={"console_scripts": ["mobile_gateway = mobile_gateway.node:main"]},
)

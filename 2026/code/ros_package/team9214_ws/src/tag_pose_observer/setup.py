from setuptools import setup
import os
from glob import glob

package_name = "tag_pose_observer"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        # ament resource index
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),

        # package.xml
        ("share/" + package_name, ["package.xml"]),

        # launch files
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),

        # config files
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "pyyaml",
    ],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Compute map-frame robot pose from AprilTag detections using known tag map and TF.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tag_pose_observer = tag_pose_observer.observer_node:main",
        ],
    },
)

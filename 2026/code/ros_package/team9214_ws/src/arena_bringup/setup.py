from glob import glob
import os

from setuptools import setup

package_name = "arena_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robots",
    maintainer_email="sidenoteemail@gmail.com",
    description="Launch orchestration for arena localization and navigation bringup.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_nt4_bridge = arena_bringup.cmd_vel_nt4_bridge:main",
            "autonomy_mode_manager = arena_bringup.autonomy_mode_manager:main",
            "nt4_mode_bridge = arena_bringup.nt4_mode_bridge:main",
            "sim_inputs = arena_bringup.sim_inputs:main",
            "tag_test_monitor = arena_bringup.tag_test_monitor:main",
        ]
    },
)

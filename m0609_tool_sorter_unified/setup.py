import os
from glob import glob

from setuptools import find_packages, setup


package_name = "m0609_tool_sorter_unified"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "models"),
            [path for path in glob("models/*") if os.path.isfile(path)],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="jwanryu",
    maintainer_email="jeowryu@gmail.com",
    description="Unified M0609 RGB-D transform and tool sorting pipeline",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = m0609_tool_sorter_unified.perception_node:main",
            "task_manager = m0609_tool_sorter_unified.task_manager:main",
            "dashboard = m0609_tool_sorter_unified.dashboard:main",
            "tcp_pose_tf_broadcaster = "
            "m0609_tool_sorter_unified.transform.tcp_pose_tf_broadcaster:main",
            "hand_eye_tf_broadcaster = "
            "m0609_tool_sorter_unified.transform.hand_eye_tf_broadcaster:main",
        ],
    },
)

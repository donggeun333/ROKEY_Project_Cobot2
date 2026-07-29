import os
from glob import glob

from setuptools import find_packages, setup


package_name = "tool_sorter_handover"

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
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="jwanryu",
    maintainer_email="jeowryu@gmail.com",
    description="Keyword-requested tool handover for the M0609 cell",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "handover_task_manager = "
            "tool_sorter_handover.handover_task_manager:main",
            "handover_dashboard = "
            "tool_sorter_handover.handover_dashboard:main",
        ],
    },
)

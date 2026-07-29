from setuptools import find_packages, setup
from glob import glob
import os


package_name = "inspection_3d"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        (os.path.join("share", package_name, "resource"), glob("resource/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dg",
    maintainer_email="donggeun3237@gmail.com",
    description="Capture and merge RGB-D point clouds in base_link using TF.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "comparison_node = inspection_3d.comparison_node:main",
            "pipeline_node = inspection_3d.pipeline_node:main",
        ],
    },
)

import os
from glob import glob
from setuptools import setup

package_name = "final_iteration"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rocotics",
    maintainer_email="598062@stud.hvl.no",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "final_iteration = final_iteration.final_iteration:main",
            "wall_follower = final_iteration.wall_follower:main",
            "aruco_detector = final_iteration.aruco_detector:main",
        ],
    },
)

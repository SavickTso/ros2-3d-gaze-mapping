import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gaze_track"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config/camera_parameters"),
            glob(os.path.join("config/camera_parameters", "*.*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            ["config/result.rviz"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="savicktso@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gaze_2d = gaze_track.gaze_2d:main",
            "gaze_3d = gaze_track.gaze_3d:main",
            "gaze_3d_display = gaze_track.gaze_3d_display:main",
            # "zed_depth_completion = gaze_track.:main",
        ],
    },
)

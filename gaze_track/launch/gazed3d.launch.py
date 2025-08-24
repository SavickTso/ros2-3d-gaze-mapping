import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    enable_display = LaunchConfiguration("enable_display", default="false")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("zed_wrapper"),
                                "launch",
                                "zedm.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(enable_display),
                arguments=[
                    "--display-config",
                    os.path.join(
                        get_package_share_directory("gaze_track"), "config/result.rviz"
                    ),
                ],
            ),
            Node(
                package="gaze_track",
                executable="gaze_3d_display",
                name="gaze_3d_display",
                condition=IfCondition(enable_display),
            ),
            Node(
                package="gaze_track",
                executable="gaze_2d",
                name="gaze_2d",
            ),
            Node(
                package="gaze_track",
                executable="gaze_3d",
                name="gaze_3d",
                condition=UnlessCondition(enable_display),
            ),
        ]
    )

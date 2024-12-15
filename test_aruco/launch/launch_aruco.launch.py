from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    namespace_launch_arg = DeclareLaunchArgument("namespace", default_value="")
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([])

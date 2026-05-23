import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("joint_motor_test")
    default_cfg = os.path.join(pkg_share, "config", "joint_motor_test.yaml")

    config_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_cfg,
        description="Path to joint_motor_test yaml config",
    )

    node = Node(
        package="joint_motor_test",
        executable="joint_motor_test_runner",
        name="joint_motor_test_runner",
        output="screen",
        parameters=[{"config_path": LaunchConfiguration("config_path")}],
    )

    return LaunchDescription([
        config_arg,
        node,
    ])

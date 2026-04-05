import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    test_pkg_share = get_package_share_directory("joint_motor_test")
    mujoco_pkg_share = get_package_share_directory("mujoco_sim2sim")

    default_test_cfg = os.path.join(test_pkg_share, "config", "joint_motor_test.yaml")
    mujoco_launch = os.path.join(mujoco_pkg_share, "launch", "sim2sim_mujoco.launch.py")

    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Absolute path to MuJoCo xml/mjb model",
    )
    test_config_arg = DeclareLaunchArgument(
        "test_config_path",
        default_value=default_test_cfg,
        description="Path to joint_motor_test yaml config",
    )
    fixed_base_arg = DeclareLaunchArgument(
        "fixed_base",
        default_value="true",
        description="Lock base pose in sim2sim for trajectory tracking tests",
    )
    fixed_base_height_arg = DeclareLaunchArgument(
        "fixed_base_height",
        default_value="-1.0",
        description="Optional fixed base height override for sim2sim",
    )
    enable_viewer_arg = DeclareLaunchArgument(
        "enable_viewer",
        default_value="false",
        description="Enable MuJoCo viewer window",
    )
    viewer_fps_arg = DeclareLaunchArgument(
        "viewer_fps",
        default_value="60.0",
        description="MuJoCo viewer render rate",
    )
    viewer_width_arg = DeclareLaunchArgument(
        "viewer_width",
        default_value="1280",
        description="MuJoCo viewer width",
    )
    viewer_height_arg = DeclareLaunchArgument(
        "viewer_height",
        default_value="720",
        description="MuJoCo viewer height",
    )
    viewer_title_arg = DeclareLaunchArgument(
        "viewer_title",
        default_value="MuJoCo Sim2Sim Viewer",
        description="MuJoCo viewer title",
    )

    sim2sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mujoco_launch),
        launch_arguments={
            "model_path": LaunchConfiguration("model_path"),
            "start_rl_controller": "false",
            "fixed_base": LaunchConfiguration("fixed_base"),
            "fixed_base_height": LaunchConfiguration("fixed_base_height"),
            "enable_viewer": LaunchConfiguration("enable_viewer"),
            "viewer_fps": LaunchConfiguration("viewer_fps"),
            "viewer_width": LaunchConfiguration("viewer_width"),
            "viewer_height": LaunchConfiguration("viewer_height"),
            "viewer_title": LaunchConfiguration("viewer_title"),
        }.items(),
    )

    test_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(test_pkg_share, "launch", "joint_motor_test.launch.py")),
        launch_arguments={
            "config_path": LaunchConfiguration("test_config_path"),
        }.items(),
    )

    return LaunchDescription([
        model_path_arg,
        test_config_arg,
        fixed_base_arg,
        fixed_base_height_arg,
        enable_viewer_arg,
        viewer_fps_arg,
        viewer_width_arg,
        viewer_height_arg,
        viewer_title_arg,
        sim2sim_include,
        test_node,
    ])

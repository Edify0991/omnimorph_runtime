import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_sim2sim")
    default_bridge_cfg = os.path.join(pkg_share, "config", "mujoco_sim2sim.yaml")

    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Absolute path to MuJoCo XML/MJB model file.",
    )
    bridge_cfg_arg = DeclareLaunchArgument(
        "bridge_config",
        default_value=default_bridge_cfg,
        description="Parameter yaml for mujoco_sim_bridge node.",
    )
    control_hz_arg = DeclareLaunchArgument(
        "control_hz",
        default_value="100.0",
        description="Control frequency for the MuJoCo bridge.",
    )
    start_controller_arg = DeclareLaunchArgument(
        "start_rl_controller",
        default_value="true",
        description="Whether to launch rl_master/RL_controller together.",
    )

    rl_controller = Node(
        package="rl_master",
        executable="RL_controller",
        name="rl_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rl_controller")),
    )

    mujoco_bridge = Node(
        package="mujoco_sim2sim",
        executable="mujoco_sim_bridge",
        name="mujoco_sim_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("bridge_config"),
            {
                "model_path": LaunchConfiguration("model_path"),
                "control_hz": ParameterValue(LaunchConfiguration("control_hz"), value_type=float),
            },
        ],
    )

    return LaunchDescription(
        [
            model_path_arg,
            bridge_cfg_arg,
            control_hz_arg,
            start_controller_arg,
            rl_controller,
            mujoco_bridge,
        ]
    )

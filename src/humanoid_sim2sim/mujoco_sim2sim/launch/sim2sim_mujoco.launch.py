import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="cpp",
        description="sim2sim backend: cpp or python_interactive",
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
    pause_no_cmd_arg = DeclareLaunchArgument(
        "pause_when_no_command",
        default_value="false",
        description="Pause stepping when no fresh command is available.",
    )
    fixed_base_arg = DeclareLaunchArgument(
        "fixed_base",
        default_value="false",
        description="Whether to lock robot base pose during sim2sim.",
    )
    fixed_base_height_arg = DeclareLaunchArgument(
        "fixed_base_height",
        default_value="-1.0",
        description="Override locked base height (meters). Negative keeps model initial value.",
    )
    actuator_mode_arg = DeclareLaunchArgument(
        "actuator_control_mode",
        default_value="auto",
        description="MuJoCo actuator control mode: auto/torque/position.",
    )
    enable_viewer_arg = DeclareLaunchArgument(
        "enable_viewer",
        default_value="false",
        description="Enable MuJoCo GLFW visualization window.",
    )
    viewer_fps_arg = DeclareLaunchArgument(
        "viewer_fps",
        default_value="60.0",
        description="Viewer render rate in Hz.",
    )
    viewer_width_arg = DeclareLaunchArgument(
        "viewer_width",
        default_value="1280",
        description="Viewer window width in pixels.",
    )
    viewer_height_arg = DeclareLaunchArgument(
        "viewer_height",
        default_value="720",
        description="Viewer window height in pixels.",
    )
    viewer_title_arg = DeclareLaunchArgument(
        "viewer_title",
        default_value="MuJoCo Sim2Sim Viewer",
        description="Viewer window title.",
    )
    show_left_ui_arg = DeclareLaunchArgument(
        "show_left_ui",
        default_value="true",
        description="Python interactive backend only: show left UI panel.",
    )
    show_right_ui_arg = DeclareLaunchArgument(
        "show_right_ui",
        default_value="true",
        description="Python interactive backend only: show right UI panel.",
    )
    start_controller_arg = DeclareLaunchArgument(
        "start_rl_controller",
        default_value="false",
        description="Whether to launch rl_master/RL_controller together.",
    )
    controller_rt_enabled_arg = DeclareLaunchArgument(
        "controller_rt_enabled",
        default_value="",
        description="Optional override for RL_MASTER_CONTROLLER_RT_ENABLED env (empty keeps yaml/default).",
    )
    controller_rt_lock_memory_arg = DeclareLaunchArgument(
        "controller_rt_lock_memory",
        default_value="",
        description="Optional override for RL_MASTER_CONTROLLER_RT_LOCK_MEMORY env.",
    )
    controller_rt_set_affinity_arg = DeclareLaunchArgument(
        "controller_rt_set_affinity",
        default_value="",
        description="Optional override for RL_MASTER_CONTROLLER_RT_SET_AFFINITY env.",
    )
    controller_rt_cpu_id_arg = DeclareLaunchArgument(
        "controller_rt_cpu_id",
        default_value="",
        description="Optional override for RL_MASTER_CONTROLLER_RT_CPU_ID env.",
    )
    controller_rt_use_fifo_arg = DeclareLaunchArgument(
        "controller_rt_use_fifo",
        default_value="",
        description="Optional override for RL_MASTER_CONTROLLER_RT_USE_FIFO env.",
    )
    controller_rt_fifo_priority_arg = DeclareLaunchArgument(
        "controller_rt_fifo_priority",
        default_value="",
        description="Optional override for RL_MASTER_CONTROLLER_RT_FIFO_PRIORITY env.",
    )

    rl_controller = Node(
        package="rl_master",
        executable="RL_controller",
        name="rl_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rl_controller")),
        additional_env={
            "RL_MASTER_CONTROLLER_RT_ENABLED": LaunchConfiguration("controller_rt_enabled"),
            "RL_MASTER_CONTROLLER_RT_LOCK_MEMORY": LaunchConfiguration("controller_rt_lock_memory"),
            "RL_MASTER_CONTROLLER_RT_SET_AFFINITY": LaunchConfiguration("controller_rt_set_affinity"),
            "RL_MASTER_CONTROLLER_RT_CPU_ID": LaunchConfiguration("controller_rt_cpu_id"),
            "RL_MASTER_CONTROLLER_RT_USE_FIFO": LaunchConfiguration("controller_rt_use_fifo"),
            "RL_MASTER_CONTROLLER_RT_FIFO_PRIORITY": LaunchConfiguration("controller_rt_fifo_priority"),
        },
    )

    common_bridge_parameters = [
        LaunchConfiguration("bridge_config"),
        {
            "model_path": LaunchConfiguration("model_path"),
            "control_hz": ParameterValue(LaunchConfiguration("control_hz"), value_type=float),
            "pause_when_no_command": ParameterValue(LaunchConfiguration("pause_when_no_command"), value_type=bool),
            "fix_base": ParameterValue(LaunchConfiguration("fixed_base"), value_type=bool),
            "fixed_base_height": ParameterValue(LaunchConfiguration("fixed_base_height"), value_type=float),
            "actuator_control_mode": LaunchConfiguration("actuator_control_mode"),
            "enable_viewer": ParameterValue(LaunchConfiguration("enable_viewer"), value_type=bool),
            "viewer_fps": ParameterValue(LaunchConfiguration("viewer_fps"), value_type=float),
            "viewer_width": ParameterValue(LaunchConfiguration("viewer_width"), value_type=int),
            "viewer_height": ParameterValue(LaunchConfiguration("viewer_height"), value_type=int),
            "viewer_title": LaunchConfiguration("viewer_title"),
        },
    ]

    cpp_bridge = Node(
        package="mujoco_sim2sim",
        executable="mujoco_sim_bridge",
        name="mujoco_sim_bridge",
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("backend"), "' == 'cpp'"])),
        parameters=common_bridge_parameters,
    )

    python_interactive_bridge = Node(
        package="mujoco_sim2sim",
        executable="mujoco_sim_interactive_backend.py",
        name="mujoco_sim_interactive_backend",
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("backend"), "' == 'python_interactive'"])),
        parameters=[
            *common_bridge_parameters,
            {
                "show_left_ui": ParameterValue(LaunchConfiguration("show_left_ui"), value_type=bool),
                "show_right_ui": ParameterValue(LaunchConfiguration("show_right_ui"), value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            model_path_arg,
            backend_arg,
            bridge_cfg_arg,
            control_hz_arg,
            pause_no_cmd_arg,
            fixed_base_arg,
            fixed_base_height_arg,
            actuator_mode_arg,
            enable_viewer_arg,
            viewer_fps_arg,
            viewer_width_arg,
            viewer_height_arg,
            viewer_title_arg,
            show_left_ui_arg,
            show_right_ui_arg,
            start_controller_arg,
            controller_rt_enabled_arg,
            controller_rt_lock_memory_arg,
            controller_rt_set_affinity_arg,
            controller_rt_cpu_id_arg,
            controller_rt_use_fifo_arg,
            controller_rt_fifo_priority_arg,
            rl_controller,
            cpp_bridge,
            python_interactive_bridge,
        ]
    )

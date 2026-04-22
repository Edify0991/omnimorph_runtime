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
        description="sim2sim backend: cpp | python_frontend | python_interactive",
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
    mode_id_arg = DeclareLaunchArgument(
        "mode_id",
        default_value="0",
        description="Startup deploy mode id used by the fused runtime.",
    )
    pause_no_cmd_arg = DeclareLaunchArgument(
        "pause_when_no_command",
        default_value="false",
        description="Pause stepping when controller output is inactive.",
    )
    no_cmd_behavior_arg = DeclareLaunchArgument(
        "no_command_behavior",
        default_value="hold_position",
        description="Behavior when command stream is stale: hold_position / hold_last / zero_torque.",
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
    viewer_inspector_hz_arg = DeclareLaunchArgument(
        "viewer_inspector_hz",
        default_value="10.0",
        description="Publish rate for Python viewer inspector telemetry.",
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
    enable_state_telemetry_arg = DeclareLaunchArgument(
        "enable_state_telemetry",
        default_value="true",
        description="Enable low-frequency /humanoid/rl/state telemetry publishing.",
    )
    state_telemetry_hz_arg = DeclareLaunchArgument(
        "state_telemetry_hz",
        default_value="50.0",
        description="Publish rate for /humanoid/rl/state telemetry.",
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
    common_bridge_parameters = [
        LaunchConfiguration("bridge_config"),
        {
            "model_path": LaunchConfiguration("model_path"),
            "control_hz": ParameterValue(LaunchConfiguration("control_hz"), value_type=float),
            "startup_mode_id": ParameterValue(LaunchConfiguration("mode_id"), value_type=int),
            "pause_when_no_command": ParameterValue(LaunchConfiguration("pause_when_no_command"), value_type=bool),
            "no_command_behavior": LaunchConfiguration("no_command_behavior"),
            "fix_base": ParameterValue(LaunchConfiguration("fixed_base"), value_type=bool),
            "fixed_base_height": ParameterValue(LaunchConfiguration("fixed_base_height"), value_type=float),
            "actuator_control_mode": LaunchConfiguration("actuator_control_mode"),
            "enable_viewer": ParameterValue(LaunchConfiguration("enable_viewer"), value_type=bool),
            "viewer_fps": ParameterValue(LaunchConfiguration("viewer_fps"), value_type=float),
            "viewer_inspector_hz": ParameterValue(LaunchConfiguration("viewer_inspector_hz"), value_type=float),
            "viewer_width": ParameterValue(LaunchConfiguration("viewer_width"), value_type=int),
            "viewer_height": ParameterValue(LaunchConfiguration("viewer_height"), value_type=int),
            "viewer_title": LaunchConfiguration("viewer_title"),
            "enable_state_telemetry": ParameterValue(LaunchConfiguration("enable_state_telemetry"), value_type=bool),
            "state_telemetry_hz": ParameterValue(LaunchConfiguration("state_telemetry_hz"), value_type=float),
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

    python_frontend_bridge = Node(
        package="mujoco_sim2sim",
        executable="mujoco_sim_bridge",
        name="mujoco_sim_bridge",
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("backend"), "' == 'python_frontend'"])),
        parameters=[
            *common_bridge_parameters,
            {
                "enable_viewer": False,
                "enable_python_viewer_stream": True,
                "enable_python_viewer_inspector": True,
            },
        ],
    )

    python_frontend_viewer = Node(
        package="mujoco_sim2sim",
        executable="mujoco_sim_viewer_frontend.py",
        name="mujoco_sim_viewer_frontend",
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("backend"), "' == 'python_frontend'"])),
        parameters=[
            {
                "model_path": LaunchConfiguration("model_path"),
                "enable_viewer": ParameterValue(LaunchConfiguration("enable_viewer"), value_type=bool),
                "viewer_fps": ParameterValue(LaunchConfiguration("viewer_fps"), value_type=float),
                "viewer_title": LaunchConfiguration("viewer_title"),
                "show_left_ui": ParameterValue(LaunchConfiguration("show_left_ui"), value_type=bool),
                "show_right_ui": ParameterValue(LaunchConfiguration("show_right_ui"), value_type=bool),
            },
        ],
    )

    python_interactive_bridge = Node(
        package="mujoco_sim2sim",
        executable="mujoco_sim_interactive_backend.py",
        # Keep node name aligned with yaml root key `mujoco_sim_bridge`
        # so both backends consume the same parameter block.
        name="mujoco_sim_bridge",
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
            mode_id_arg,
            pause_no_cmd_arg,
            no_cmd_behavior_arg,
            fixed_base_arg,
            fixed_base_height_arg,
            actuator_mode_arg,
            enable_viewer_arg,
            viewer_fps_arg,
            viewer_inspector_hz_arg,
            viewer_width_arg,
            viewer_height_arg,
            viewer_title_arg,
            enable_state_telemetry_arg,
            state_telemetry_hz_arg,
            show_left_ui_arg,
            show_right_ui_arg,
            cpp_bridge,
            python_frontend_bridge,
            python_frontend_viewer,
            python_interactive_bridge,
        ]
    )

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
        description="sim2sim backend: cpp | python_frontend",
    )
    bridge_cfg_arg = DeclareLaunchArgument(
        "bridge_config",
        default_value=default_bridge_cfg,
        description="Parameter yaml for mujoco_sim_bridge node.",
    )
    rl_cfg_path_arg = DeclareLaunchArgument(
        "rl_cfg_path",
        default_value="",
        description="Optional absolute path to rl_master root config yaml used by the fused runtime.",
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
    enable_fixed_base_zeroing_arg = DeclareLaunchArgument(
        "enable_fixed_base_zeroing",
        default_value="true",
        description="Lock base in air during startup/zeroing and required sim2sim re-zeroing paths.",
    )
    enable_fixed_base_hold_after_zeroing_arg = DeclareLaunchArgument(
        "enable_fixed_base_hold_after_zeroing",
        default_value="true",
        description="Keep base locked after zeroing while waiting for explicit start.",
    )
    enable_release_before_running_arg = DeclareLaunchArgument(
        "enable_release_before_running",
        default_value="true",
        description="Release fixed base before entering physical running.",
    )
    post_release_settle_ticks_arg = DeclareLaunchArgument(
        "post_release_settle_ticks",
        default_value="20",
        description="Number of control ticks to stay in hold after base release before allowing running.",
    )
    enable_prepose_snap_arg = DeclareLaunchArgument(
        "enable_prepose_snap",
        default_value="false",
        description="Snap controlled joints to prepose_joint_q before fixed-base zeroing.",
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
    enable_python_viewer_stream_arg = DeclareLaunchArgument(
        "enable_python_viewer_stream",
        default_value="false",
        description="Enable Python viewer frame stream topic for external frontend/inspection.",
    )
    enable_python_viewer_inspector_arg = DeclareLaunchArgument(
        "enable_python_viewer_inspector",
        default_value="false",
        description="Enable Python viewer inspector telemetry topic.",
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
    enable_com_support_visualization_arg = DeclareLaunchArgument(
        "enable_com_support_visualization",
        default_value="false",
        description="Overlay Pinocchio COM, COM projection, and support polygon in the native MuJoCo viewer/video.",
    )
    com_support_pinocchio_urdf_path_arg = DeclareLaunchArgument(
        "com_support_pinocchio_urdf_path",
        default_value="",
        description="Optional Pinocchio URDF override for COM overlay. Empty uses active profile pinocchio_urdf_path.",
    )
    support_foot_half_length_arg = DeclareLaunchArgument(
        "support_foot_half_length",
        default_value="0.11",
        description="Half length of each foot support rectangle in meters.",
    )
    support_foot_half_width_arg = DeclareLaunchArgument(
        "support_foot_half_width",
        default_value="0.055",
        description="Half width of each foot support rectangle in meters.",
    )
    support_contact_height_threshold_arg = DeclareLaunchArgument(
        "support_contact_height_threshold",
        default_value="0.05",
        description="Foot sites within this height of the lowest foot site are included in support polygon.",
    )
    cop_marker_radius_arg = DeclareLaunchArgument(
        "cop_marker_radius",
        default_value="0.025",
        description="Radius of the COP marker in the native MuJoCo viewer/video.",
    )
    enable_video_recording_arg = DeclareLaunchArgument(
        "enable_video_recording",
        default_value="false",
        description="Record native C++ MuJoCo MP4 video using physical simulation time.",
    )
    video_output_dir_arg = DeclareLaunchArgument(
        "video_output_dir",
        default_value="/tmp/omnimorph_sim2sim_videos",
        description="Fixed directory for native C++ sim2sim MP4 recordings.",
    )
    video_output_name_arg = DeclareLaunchArgument(
        "video_output_name",
        default_value="",
        description="Optional MP4 file name inside video_output_dir. Empty generates a timestamped name.",
    )
    video_fps_arg = DeclareLaunchArgument(
        "video_fps",
        default_value="60.0",
        description="Native C++ video frame rate in physical simulation seconds.",
    )
    video_width_arg = DeclareLaunchArgument(
        "video_width",
        default_value="1280",
        description="Native C++ video width in pixels.",
    )
    video_height_arg = DeclareLaunchArgument(
        "video_height",
        default_value="720",
        description="Native C++ video height in pixels.",
    )
    enable_state_telemetry_arg = DeclareLaunchArgument(
        "enable_state_telemetry",
        default_value="true",
        description="Enable low-frequency /omnimorph/rl/state telemetry publishing.",
    )
    state_telemetry_hz_arg = DeclareLaunchArgument(
        "state_telemetry_hz",
        default_value="50.0",
        description="Publish rate for /omnimorph/rl/state telemetry.",
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
            "rl_cfg_path": LaunchConfiguration("rl_cfg_path"),
            "control_hz": ParameterValue(LaunchConfiguration("control_hz"), value_type=float),
            "startup_mode_id": ParameterValue(LaunchConfiguration("mode_id"), value_type=int),
            "pause_when_no_command": ParameterValue(LaunchConfiguration("pause_when_no_command"), value_type=bool),
            "no_command_behavior": LaunchConfiguration("no_command_behavior"),
            "fix_base": ParameterValue(LaunchConfiguration("fixed_base"), value_type=bool),
            "fixed_base_height": ParameterValue(LaunchConfiguration("fixed_base_height"), value_type=float),
            "enable_fixed_base_zeroing": ParameterValue(LaunchConfiguration("enable_fixed_base_zeroing"), value_type=bool),
            "enable_fixed_base_hold_after_zeroing": ParameterValue(LaunchConfiguration("enable_fixed_base_hold_after_zeroing"), value_type=bool),
            "enable_release_before_running": ParameterValue(LaunchConfiguration("enable_release_before_running"), value_type=bool),
            "post_release_settle_ticks": ParameterValue(LaunchConfiguration("post_release_settle_ticks"), value_type=int),
            "enable_prepose_snap": ParameterValue(LaunchConfiguration("enable_prepose_snap"), value_type=bool),
            "actuator_control_mode": LaunchConfiguration("actuator_control_mode"),
            "enable_viewer": ParameterValue(LaunchConfiguration("enable_viewer"), value_type=bool),
            "enable_python_viewer_stream": ParameterValue(LaunchConfiguration("enable_python_viewer_stream"), value_type=bool),
            "enable_python_viewer_inspector": ParameterValue(LaunchConfiguration("enable_python_viewer_inspector"), value_type=bool),
            "viewer_fps": ParameterValue(LaunchConfiguration("viewer_fps"), value_type=float),
            "viewer_inspector_hz": ParameterValue(LaunchConfiguration("viewer_inspector_hz"), value_type=float),
            "viewer_width": ParameterValue(LaunchConfiguration("viewer_width"), value_type=int),
            "viewer_height": ParameterValue(LaunchConfiguration("viewer_height"), value_type=int),
            "viewer_title": LaunchConfiguration("viewer_title"),
            "enable_com_support_visualization": ParameterValue(
                LaunchConfiguration("enable_com_support_visualization"),
                value_type=bool,
            ),
            "com_support_pinocchio_urdf_path": LaunchConfiguration("com_support_pinocchio_urdf_path"),
            "support_foot_half_length": ParameterValue(LaunchConfiguration("support_foot_half_length"), value_type=float),
            "support_foot_half_width": ParameterValue(LaunchConfiguration("support_foot_half_width"), value_type=float),
            "support_contact_height_threshold": ParameterValue(
                LaunchConfiguration("support_contact_height_threshold"),
                value_type=float,
            ),
            "cop_marker_radius": ParameterValue(LaunchConfiguration("cop_marker_radius"), value_type=float),
            "enable_video_recording": ParameterValue(LaunchConfiguration("enable_video_recording"), value_type=bool),
            "video_output_dir": LaunchConfiguration("video_output_dir"),
            "video_output_name": LaunchConfiguration("video_output_name"),
            "video_fps": ParameterValue(LaunchConfiguration("video_fps"), value_type=float),
            "video_width": ParameterValue(LaunchConfiguration("video_width"), value_type=int),
            "video_height": ParameterValue(LaunchConfiguration("video_height"), value_type=int),
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

    return LaunchDescription(
        [
            model_path_arg,
            backend_arg,
            bridge_cfg_arg,
            rl_cfg_path_arg,
            control_hz_arg,
            mode_id_arg,
            pause_no_cmd_arg,
            no_cmd_behavior_arg,
            fixed_base_arg,
            fixed_base_height_arg,
            enable_fixed_base_zeroing_arg,
            enable_fixed_base_hold_after_zeroing_arg,
            enable_release_before_running_arg,
            post_release_settle_ticks_arg,
            enable_prepose_snap_arg,
            actuator_mode_arg,
            enable_viewer_arg,
            enable_python_viewer_stream_arg,
            enable_python_viewer_inspector_arg,
            viewer_fps_arg,
            viewer_inspector_hz_arg,
            viewer_width_arg,
            viewer_height_arg,
            viewer_title_arg,
            enable_com_support_visualization_arg,
            com_support_pinocchio_urdf_path_arg,
            support_foot_half_length_arg,
            support_foot_half_width_arg,
            support_contact_height_threshold_arg,
            cop_marker_radius_arg,
            enable_video_recording_arg,
            video_output_dir_arg,
            video_output_name_arg,
            video_fps_arg,
            video_width_arg,
            video_height_arg,
            enable_state_telemetry_arg,
            state_telemetry_hz_arg,
            show_left_ui_arg,
            show_right_ui_arg,
            cpp_bridge,
            python_frontend_bridge,
            python_frontend_viewer,
        ]
    )

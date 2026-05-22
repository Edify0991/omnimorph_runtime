# Operator GUI Architecture

This note captures the proposed GUI direction for the fused JC01 deploy runtime.
It is intentionally biased toward keeping the real-time controller isolated from
desktop rendering, plotting, and sensor display work.

## Current Runtime Integration Points

The active deploy interface already has a clean operator boundary:

- `/omnimorph/rl/mode_control`: `std_msgs/Int32`, reliable operator lifecycle and mode words.
- `/omnimorph/rl/teleop`: `geometry_msgs/Twist`, best-effort velocity/yaw command.
- `/omnimorph/rl/state`: `std_msgs/Float32MultiArray`, best-effort protocol-v2 robot telemetry.
- `/imu/yesense`: `sensor_msgs/Imu`, consumed by `RL_solver` on the real robot path.

The GUI should stay outside `RL_solver` and communicate only through these
topics or future non-real-time services. This preserves the fused runtime's
single-process control path and avoids GUI stalls affecting motor shared-memory
I/O, policy inference, or command writeback.

## Technology Survey

### Qt / PyQt / PySide

Qt remains the best primary shell for a local operator console: mature widgets,
tables, charts, OpenGL surfaces, camera widgets, and deployable desktop
packaging. Qt for Python is the official Python binding stack for Qt and targets
cross-platform desktop/mobile applications. Source:
https://www.qt.io/qt-for-python

For this repository, the first GUI is implemented with PyQt5 because it is
available in the current environment and maps well to ROS 2 Python nodes. For a
long-lived product UI, PySide6/Qt6 is a good migration target because it is the
official binding and unlocks newer Qt 6 modules.

### RQt

RQt is a ROS GUI plugin framework. ROS 2 documentation highlights that it can
host dockable tools, run plugins as standalone windows, and supports Python or
C++ plugins. Source:
https://docs.ros.org/en/humble/Concepts/Intermediate/About-RQt.html

RQt is useful for small reusable debug plugins. It is less ideal as the main
operator console because custom safety flows, teach-style layout, camera
composition, and 3D rendering often outgrow the generic RQt shell.

### RViz2 Panel

RViz2 supports custom Qt panels that can subscribe and publish ROS topics.
Source:
https://docs.ros.org/en/ros2_documentation/rolling/Tutorials/Intermediate/RViz/RViz-Custom-Panel/RViz-Custom-Panel.html

RViz2 is the right companion for TF, URDF, markers, robot model, point clouds,
maps, and camera overlays. It should not be the only operator UI unless the
interface is mostly 3D visualization. A good split is: custom Qt console for
operations, RViz2/Foxglove for heavy ROS visualization.

### Foxglove

Foxglove Bridge supports live ROS 2 visualization through a high-performance C++
WebSocket bridge and can open MCAP recordings. Sources:
https://docs.foxglove.dev/docs/visualization/connecting/live/ros-foxglove-bridge
https://docs.foxglove.dev/docs/getting-started/frameworks/ros2

Foxglove is excellent for remote debugging, logs, camera/point cloud panels, and
team-shared layouts. It is a strong secondary tool, especially because this
runtime already records MCAP logs. It can also be extended with custom panels:
https://docs.foxglove.dev/docs/extensions

### PlotJuggler

PlotJuggler is a purpose-built time-series viewer with ROS/ROS 2 plugins,
live topic subscription, bag loading, layouts, transforms, and OpenGL plotting.
Sources:
https://github.com/facontidavide/PlotJuggler
https://docs.ros.org/en/ros2_packages/humble/api/plotjuggler_ros/index.html

Use it for high-density signal analysis instead of trying to make the operator
console handle every diagnostic plot.

### Open3D / Rerun

Open3D has point cloud and mesh visualization tools and Python APIs for point
cloud rendering. Source:
https://www.open3d.org/docs/release/tutorial/visualization/visualization.html

Rerun provides multimodal robotics visualization and SDKs for time-aware data,
including 3D point clouds in C++/Python APIs. Sources:
https://docs.rerun.io/dev/getting-started/
https://ref.rerun.io/docs/cpp/stable/structrerun_1_1archetypes_1_1Points3D.html

For future 3DGS or dense environment visualization, avoid embedding all
rendering directly into the core operator console at first. Prefer a separate
viewer process or embedded viewport fed by a stable scene stream.

## Recommendation

Use a layered GUI stack:

1. Primary operator console: standalone Qt process.
   - Start/stop/zero/e-stop and mode switching.
   - Teleop command publishing.
   - Joint state table and simple low-latency plots.
   - Small robot-state mirror and camera preview region.

2. Heavy visualization companions:
   - RViz2 for URDF/TF/PointCloud2/Image/markers.
   - Foxglove for remote live visualization and MCAP replay.
   - PlotJuggler for detailed time-series analysis.

3. Future 3D environment rendering:
   - Publish robot pose, joint state, camera, and LiDAR as standard ROS topics.
   - Add an optional 3D viewer bridge that consumes `sensor_msgs/PointCloud2`,
     camera topics, and robot state.
   - Keep 3DGS rendering out of `RL_solver`; run it as a GPU-sidecar process.

## Implemented v0

`scripts/omnimorph_ops_gui.py` provides the initial standalone Qt console:

- Publishes `/omnimorph/rl/mode_control`.
- Publishes `/omnimorph/rl/teleop`.
- Subscribes to `/omnimorph/rl/state`.
- Decodes protocol-v2 dynamic joint telemetry.
- Loads `robot_global_joint_order` and `deploy_mode_profiles` from `rl_cfg_jc01.yaml`.
- Shows mode buttons, teleop controls, joint table, simple rolling plot, and a
  lightweight robot twin panel.

## Current Layout Meaning

The GUI is currently split into three vertical areas:

1. Operator controls on the left.
2. State tables and plots in the middle.
3. A right-side twin panel that mirrors the robot pose in a lightweight 2D form.

That right-side panel is not yet a full environment-aware digital twin. It is a
compact placeholder for the display stack that will later hold a real 3D scene.

## How To Place The Robot In The Environment

To show the robot "inside" the sensed environment in a true digital-twin sense,
the GUI needs a shared world frame and a scene renderer:

1. Choose one canonical world frame.
   - Base pose from the runtime becomes `world -> base`.
   - Joint states drive the robot URDF or mesh skeleton in that same frame.

2. Stream environment geometry in the same frame.
   - LiDAR point clouds.
   - Occupancy / voxel / mesh maps.
   - Optional 3DGS scene assets.

3. Render both robot and environment in one scene graph.
   - Robot root comes from base position and quaternion.
   - Articulated links come from joint states.
   - Camera images become an inset overlay, not the main scene.

4. Keep frame alignment explicit.
   - Use TF-like transforms or a small scene-transform table.
   - If camera extrinsics are known, place the camera frustum in the same world.
   - If LiDAR extrinsics are known, fuse the cloud into the same world before drawing.

5. Treat 3DGS as a viewer, not a control dependency.
   - The controller should never wait on rendering.
   - The renderer can subscribe to robot state + sensor topics and redraw asynchronously.

For this repo, the most natural follow-up is to add a dedicated scene widget or
an external viewer process that consumes:

- robot base pose
- joint state
- camera image
- LiDAR / point cloud
- optional scene reconstruction / 3DGS stream

Run after building and sourcing the workspace:

```bash
ros2 run rl_master omnimorph_ops_gui.py
```

Or from source:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/omnimorph_rl_controller/rl_master/scripts/omnimorph_ops_gui.py
```

## Next Interfaces To Add

The GUI becomes much more useful if the runtime publishes a richer status topic
or service set in addition to raw robot state:

- `runtime_status`: lifecycle state, active mode id, pending mode id, policy tag,
  control-loop frequency, policy frequency, stale-input flags, last error.
- `mode_profiles`: resolved mode profile list after config load.
- `joint_limits`: names, soft limits, torque limits, kp/kd, and enabled groups.
- `operator_ack`: command acceptance/rejection with sequence number and reason.

Recommended ROS shapes:

- Keep high-rate telemetry as best-effort topics.
- Use reliable topics or services for command acknowledgements and config
  introspection.
- Use lifecycle confirmations for zero/e-stop/start actions.

## Safety Notes

- GUI commands must remain supervisory; the low-level safety state machine stays
  in the controller.
- E-stop and zero commands should require confirmation or a hardware safety
  channel, depending on deployment setting.
- Direct joint debugging should be gated by a separate runtime mode and joint
  limit checks before any per-joint command stream is accepted.
- Camera, LiDAR, and 3DGS streams should be rate-limited or decimated before
  entering the Qt process to avoid starving operator controls.

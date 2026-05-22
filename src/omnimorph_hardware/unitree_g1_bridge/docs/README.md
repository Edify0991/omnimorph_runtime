# Unitree G1 Bridge

This package is an optional sim2real edge bridge. It keeps `RL_solver` robot-agnostic by using the shared-memory motor contract as the upper interface, then publishes/subscribes Unitree's low-level ROS 2 messages as the lower interface.

The bridge follows the official Unitree G1 low-level ROS 2 example:

- subscribe `unitree_hg/msg/LowState` from `lowstate`
- publish `unitree_hg/msg/LowCmd` to `/lowcmd`
- use 29 G1 motor slots in Unitree's official order
- call the official `common/motor_crc_hg.h` helper when that header is available

Build this package after sourcing the official Unitree ROS 2 workspace. If `unitree_hg` is not available, the package is skipped so the rest of OmniMorph still builds.

```bash
source /opt/ros/humble/setup.bash
source <unitree_ros2_ws>/install/setup.bash
source ./script/dev_env.sh
colcon build --packages-select unitree_g1_bridge
```

Runtime:

```bash
source ./script/dev_env.sh
./script/start_unitree_g1_bridge.sh
```

Useful parameters:

```bash
ros2 run unitree_g1_bridge unitree_g1_bridge --ros-args \
  -p lowstate_topic:=lowstate \
  -p lowcmd_topic:=/lowcmd \
  -p control_hz:=500.0
```

# Kinematics Adapter Structure

The runtime supports multiple robot morphologies through a narrow adapter boundary:

```text
robot_identity.kinematics_adapter
  -> RobotKinematicsAdapter factory
  -> robot-specific joint/motor mapping implementation
```

The public runtime-facing types live in:

- `include/rl_master/kinematics/joint_data.h`
- `include/rl_master/kinematics/robot_kinematics_adapter.h`

Robot-specific implementations live below `kinematics/<robot>/`. For example, JC01's coupled ankle/knee implementation is isolated in:

```text
kinematics/jc01/
  ankle_kinematics.cpp
  knee_kinematics.cpp
  kin_conv.cpp

include/rl_master/kinematics/jc01/
  ankle_kinematics.h
  knee_kinematics.h
  kin_conv.h
```

CMake keeps these functions internal to this repository by building non-exported static libraries:

```cmake
rl_master_jc01_kinematics
rl_master_robot_kinematics
```

`RL_solver` and `test_kine` link these private targets. The old standalone `KinConv` install target is intentionally removed; external code should select behavior through `robot_identity.kinematics_adapter` instead of linking JC01-specific conversion functions directly.

When adding another robot:

1. Add `robot_identity.id` and `robot_identity.kinematics_adapter` to `rl_cfg_<robot>.yaml`.
2. Add a folder such as `kinematics/unitree_g1/`, `kinematics/jc05/`, or `kinematics/wheel_legged_x/` only if direct joint/motor mapping is insufficient.
3. Register the adapter id in `createRobotKinematicsAdapter`.
4. Keep vendor hardware communication out of the kinematics adapter; put it in `src/omnimorph_hardware/<vendor_or_robot>_bridge`.

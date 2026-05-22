# External Hardware Bridge Contract

`RL_solver` keeps the policy/runtime side robot-agnostic by using:

- `robot_identity.kinematics_adapter` for joint-space to motor-space conversion
- `robot_identity.motor_io_backend` for the solver-to-hardware motor IO boundary
- `robot_identity.external_hardware_bridge` as a descriptive label for the bottom vendor runtime/driver

The bottom hardware layer is intentionally robot-specific. For JC01 this is a
local driver process connected through shared memory. For Unitree G1 this is
Unitree's official onboard runtime exposed through DDS topics, while `RL_solver`
publishes/subscribes those topics directly in-process.

Current backend identifiers:

- `shm`: shared-memory motor IO for JC01 and the future JC05 bridge
- `unitree_g1_dds`: in-process Unitree G1 low-level DDS motor IO

Current startup mapping:

| Robot | `motor_io_backend` | Bottom driver/runtime | Startup |
| --- | --- | --- | --- |
| JC01 | `shm` | local JC01 driver process | `sudo ./script/start_driver_jc01.sh` |
| JC05 placeholder | `shm` | future JC05 vendor bridge process | TBD |
| Unitree G1 | `unitree_g1_dds` | official Unitree runtime publishing `lowstate` and consuming `/lowcmd` | verify `lowstate`, then start `RL_solver` |

For Unitree, keep the official Unitree ROS 2 workspace as the bottom communication layer. Source it before building/running this repository so `unitree_hg` messages and the optional `common/motor_crc_hg.h` CRC helper are available.

When adding a new robot:

1. Add `robot_identity` to `rl_cfg_<robot>.yaml`.
2. Add or select a `RobotKinematicsAdapter`.
3. Add or select a `MotorIoBackend`.
4. Publish motor state and consume motor command in `robot_global_motor_order`.

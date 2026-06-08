# External Hardware Bridge Contract

`RL_solver` keeps the policy/runtime side robot-agnostic by using:

- `robot_identity.kinematics_adapter` for joint-space to motor-space conversion
- `robot_identity.unitree_transport` as the Unitree G1 transport selector (`sdk2` / `ros2`)
- `robot_identity.motor_io_backend` for the solver-to-hardware motor IO boundary
- `robot_identity.external_hardware_bridge` as a descriptive label for the bottom vendor runtime/driver

The bottom hardware layer is intentionally robot-specific. For JC01 this is a
local driver process connected through shared memory. For Unitree G1 this is
Unitree's official onboard runtime exposed through DDS topics, while `RL_solver`
publishes/subscribes those topics directly in-process.

Current backend identifiers:

- `shm`: shared-memory motor IO for JC01 and the future JC05 bridge
- `unitree_g1_dds`: in-process Unitree G1 low-level DDS motor IO
- `unitree_g1_sdk2`: in-process Unitree G1 SDK2 motor IO

Current startup mapping:

| Robot | Selector | `motor_io_backend` | Bottom driver/runtime | Startup |
| --- | --- | --- | --- | --- |
| JC01 | n/a | `shm` | local JC01 driver process | `sudo ./script/start_driver_jc01.sh` |
| JC05 placeholder | n/a | `shm` | future JC05 vendor bridge process | TBD |
| Unitree G1 | `unitree_transport: ros2` | `unitree_g1_dds` | official Unitree ROS 2 runtime publishing `lowstate` and consuming `/lowcmd` | verify `lowstate`, then start `RL_solver` |
| Unitree G1 | `unitree_transport: sdk2` | `unitree_g1_sdk2` | official Unitree SDK2 runtime using `rt/lowstate` and `rt/lowcmd` | verify SDK2 channel connectivity, then start `RL_solver` |

For Unitree G1, choose the transport in
`config/rl_cfg_unitree_g1.yaml` with `robot_identity.unitree_transport`.
The G1 profiles set `source_contract.imu_input.follow_unitree_transport: true`,
so the IMU lowstate source follows the same selector.

For `unitree_transport: ros2`, source the official Unitree ROS 2 workspace
before building/running this repository so `unitree_hg` messages and the
optional `common/motor_crc_hg.h` CRC helper are available. For
`unitree_transport: sdk2`, set `UNITREE_SDK2_ROOT` or keep the SDK2 checkout at
`/home/edify/Code/unitree_sdk2` before building.

When adding a new robot:

1. Add `robot_identity` to `rl_cfg_<robot>.yaml`.
2. Add or select a `RobotKinematicsAdapter`.
3. Add or select a `MotorIoBackend`.
4. Publish motor state and consume motor command in `robot_global_motor_order`.

# External Hardware Bridge Contract

`RL_solver` keeps the policy/runtime side robot-agnostic by using:

- `robot_identity.kinematics_adapter` for joint-space to motor-space conversion
- `robot_identity.external_hardware_bridge` for the vendor-specific transport process

The bridge is intentionally outside the policy runtime. A vendor bridge, for
example a Unitree SDK deploy process, should translate between the vendor SDK
state/command packets and this repository's shared motor interface or DDS
contract, while leaving policy observation/action code unchanged.

Current bridge identifiers:

- `jc01_shm_bridge`: existing JC01 shared-memory motor bridge
- `unitree_sdk_bridge`: optional Unitree G1 bridge package under `src/omnimorph_hardware/unitree_g1_bridge`
- `jc05_vendor_bridge`: placeholder for the future JC05 quadruped bridge

Current startup mapping:

| Robot | Upper runtime | Lower communication | Startup |
| --- | --- | --- | --- |
| JC01 | `RL_solver` shared-memory motor target/feedback | local JC01 driver | `sudo ./script/start_driver_jc01.sh` |
| Unitree G1 | same shared-memory motor target/feedback | official Unitree ROS 2 low-level topics `lowstate` and `/lowcmd` | `./script/start_unitree_g1_bridge.sh` |

For Unitree, keep the official Unitree ROS 2 workspace as the bottom communication layer. Source it before building/running this repository so `unitree_hg` messages and the optional `common/motor_crc_hg.h` CRC helper are available.

When adding a new robot:

1. Add `robot_identity` to `rl_cfg_<robot>.yaml`.
2. Add or select a `RobotKinematicsAdapter`.
3. Keep vendor communication in a dedicated bridge process/module.
4. Publish motor state and consume motor command in `robot_global_motor_order`.

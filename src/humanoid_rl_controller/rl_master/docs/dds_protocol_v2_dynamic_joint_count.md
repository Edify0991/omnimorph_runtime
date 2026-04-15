# DDS Protocol v2 Dynamic Joint Count

The active runtime now uses a single DDS payload format for robot command/state transport:

- protocol version: `2`
- dynamic `joint_count` carried in the payload header
- no legacy fixed-width 12-dof decode path in the active encoder/decoder

## Policy Command Payload

Header:

- `[0]`: `kProtocolV2Magic`
- `[1]`: protocol version `2`
- `[2]`: payload type `kProtocolV2PayloadPolicyCommand`
- `[3]`: `joint_count`
- `[4]`: `open_rl`
- `[5]`: sequence
- `[6]`: stamp seconds

Body:

- from `[7]`: `joint_count` groups of `q, dq, tau`

In code:

- encode: [`dds_protocol.cpp`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/dds_protocol.cpp:103)
- decode: [`dds_protocol.cpp`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/dds_protocol.cpp:130)

## Robot State Payload

Header:

- `[0]`: `kProtocolV2Magic`
- `[1]`: protocol version `2`
- `[2]`: payload type `kProtocolV2PayloadRobotState`
- `[3]`: `joint_count`

Body:

- `joint_count` groups of `q, dq, tau`
- base angular velocity `(3)`
- base quaternion `xyzw (4)`
- base rpy `(3)`

In code:

- encode: [`dds_protocol.cpp`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/dds_protocol.cpp:186)
- decode: [`dds_protocol.cpp`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/dds_protocol.cpp:219)

## Runtime Meaning

`joint_count` is the active vector width carried between producer and consumer for that message.

- In sim2sim fused runtime, MuJoCo bridge publishes the configured runtime joint set.
- In sim2real fused runtime, solver publishes the global runtime joint layout, then maps the physically installed hardware joints into that layout.
- Consumers must not assume `12`, lower-body-only ordering, or any compile-time joint width.

## Compatibility Boundary

The repository still keeps some legacy code paths and docs under `legacy/` for historical split-runtime comparison, but the active `dds_protocol.cpp` implementation no longer accepts old v1 fixed-width payloads.

That means:

- all active producers should use `encodePolicyCommand(...)` / `encodeRobotState(...)`
- all active consumers should use `decodePolicyCommand(...)` / `decodeRobotState(...)`
- any remaining external tool that still emits old 12-dof flat arrays must be upgraded before use with the current fused runtime

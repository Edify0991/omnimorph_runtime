# DDS Protocol v2 (Dynamic `joint_count`) Skeleton

This repository now supports dual decode paths:
- v1 legacy fixed width (`kLegJointCount = 12`)
- v2 dynamic width (`joint_count` carried in payload header)

## Policy Command Payload

### v1 (legacy)
- Flat array length: `kJointCmdValueCount`
- Layout:
  - `[0 ... 3*N-1]`: `q, dq, tau` interleaved (`N=12`)
  - `[kJointStateValueCount]`: `open_rl`
  - `[kJointCmdSeqIndex]`: sequence
  - `[kJointCmdStampIndex]`: stamp sec

### v2 (dynamic)
- Header:
  - `[0]`: `kProtocolV2Magic`
  - `[1]`: protocol version `2`
  - `[2]`: payload type `kProtocolV2PayloadPolicyCommand`
  - `[3]`: `joint_count`
  - `[4]`: `open_rl`
  - `[5]`: sequence
  - `[6]`: stamp sec
- Body:
  - From `[7]`: `joint_count` groups of `q, dq, tau`

## Robot State Payload

### v1 (legacy)
- Flat array length: `kRobotStateValueCount`
- Layout:
  - joint `q,dq,tau` for 12 joints
  - base ang vel (3), base quat xyzw (4), base rpy (3)

### v2 (dynamic)
- Header:
  - `[0]`: `kProtocolV2Magic`
  - `[1]`: protocol version `2`
  - `[2]`: payload type `kProtocolV2PayloadRobotState`
  - `[3]`: `joint_count`
- Body:
  - `joint_count` groups of `q, dq, tau`
  - base ang vel (3), base quat xyzw (4), base rpy (3)

## Backward Compatibility

- Decoder first checks v2 magic/version/payload header.
- If absent, decoder falls back to v1 legacy parser.
- For v2 decode:
  - dynamic vectors are filled (`*_full`)
  - legacy 12-dim arrays are also populated by truncation/zero-pad
- For v1 decode:
  - legacy arrays are parsed as before
  - dynamic vectors are mirrored from legacy arrays

## How to emit v2 from current code

`encodePolicyCommand(...)` / `encodeRobotState(...)` emits v2 when at least one is true:
- `protocol_version >= 2`
- `active_joint_count != 12`
- dynamic vectors (`*_full`) are non-empty

Otherwise it emits legacy v1 payload.

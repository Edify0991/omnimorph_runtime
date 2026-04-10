# Lower-Body to Full-Body Extension Guide

This codebase currently treats `kLegJointCount` as the fixed transport width for:
- DDS command/state payload (`rl_protocol.h`, `dds_protocol.cpp`)
- Controller and solver joint arrays (`robot_types.h`)
- Canonical joint name order (`RL_controller.cpp`, `robot_state.cpp`)

That means lower-body policies (`action_dim < kLegJointCount`) are supported, but true full-body (`joint_count > kLegJointCount`) requires protocol upgrade.

## Current Safe Usage

- Keep `kLegJointCount = 12` for DDS compatibility.
- Use profile-level `action_dim`, `motor_N`, `action_joint_order`, `obs_joint_order` to run different lower-body policies.
- Use `deploy_mode_profiles` to switch policy families by `mode_id` without code branch duplication.

## Required Changes for Full-Body (>12)

1. Introduce protocol max and runtime active joint count.
   - Add `kMaxJointCount` (transport capacity) and `active_joint_count`.
   - Carry `active_joint_count` in command/state message header.

2. Update DDS encoder/decoder.
   - `encodePolicyCommand/decodePolicyCommand`
   - `encodeRobotState/decodeRobotState`
   - Keep backward compatibility path for legacy 12-dof message layout.

3. Replace fixed-size arrays in `RobotStateData`/`RobotCommandData`.
   - Move from `std::array<float, kLegJointCount>` to dynamic vectors or fixed max + valid length.

4. Make canonical joint order profile-driven.
   - Move hardcoded 12-joint order into config (`control_joint_order`).
   - Build mapping tables from profile config instead of compile-time array.

5. Split transport width and control width.
   - Solver/controller should iterate `active_joint_count` for logic.
   - Transport can still publish fixed max width for compatibility.

6. Add validator checks.
   - `validate_deploy_config.py` should verify:
     - `action_dim <= active_joint_count`
     - `motor_N <= active_joint_count`
     - joint order names are unique and complete for configured policy.

## Recommended Migration Path

1. Keep runtime behavior unchanged for 12-dof.
2. Add new payload version with `active_joint_count`.
3. Gate full-body with explicit config flag (e.g. `protocol_version: 2`).
4. Remove legacy path after all producer/consumer nodes are upgraded.

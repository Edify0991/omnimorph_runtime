# Unitree G1 Integration

The default Unitree G1 sim2real path is now in-process inside `RL_solver`.
`rl_cfg_unitree_g1.yaml` selects:

```yaml
robot_identity:
  motor_io_backend: unitree_g1_dds
```

That backend publishes/subscribes Unitree's low-level ROS 2 messages directly
from the fused solver process:

- subscribe `unitree_hg/msg/LowState` from `lowstate`
- publish `unitree_hg/msg/LowCmd` to `/lowcmd`
- use 29 G1 motor slots in Unitree's official order
- call the official `common/motor_crc_hg.h` helper when that header is available

The `unitree_g1_bridge` package in this folder is kept only as a standalone
diagnostic/legacy edge bridge. Normal G1 deployment does not start
`./script/start_unitree_g1_bridge.sh`.

The official low-level command interface does not expose this repository's
legacy `CSP/CST/R1` run-mode names directly. It exposes one motor command record
per joint:

```text
mode, q, dq, tau, kp, kd
```

This bridge maps repository modes to Unitree fields as follows:

| Repository run mode | Unitree command fields | Intended use |
| --- | --- | --- |
| `r1` | `q`, `dq`, `kp`, `kd`, `tau` | mixed position/velocity PD plus feed-forward torque |
| `csp` | `q`, `dq`, `kp`, `kd`, `tau=0` from the solver path | position tracking |
| `cst` | `tau` only, with `q=dq=kp=kd=0` | pure torque test path only |

For the current G1 BeyondMimic profile, use `r1`. The policy produces target
joint positions, and `RL_controller` computes a PD torque from `target_q`,
current `q/dq`, `kps`, and `kds`; the hardware command should therefore carry
the same `q/kp/kd` context plus torque feed-forward, not a repository-side pure
torque-only `cst` command.

The configured 29DOF hardware order matches Unitree's official `G1JointIndex`:

```text
0  left_hip_pitch_joint
1  left_hip_roll_joint
2  left_hip_yaw_joint
3  left_knee_joint
4  left_ankle_pitch_joint
5  left_ankle_roll_joint
6  right_hip_pitch_joint
7  right_hip_roll_joint
8  right_hip_yaw_joint
9  right_knee_joint
10 right_ankle_pitch_joint
11 right_ankle_roll_joint
12 waist_yaw_joint
13 waist_roll_joint
14 waist_pitch_joint
15 left_shoulder_pitch_joint
16 left_shoulder_roll_joint
17 left_shoulder_yaw_joint
18 left_elbow_joint
19 left_wrist_roll_joint
20 left_wrist_pitch_joint
21 left_wrist_yaw_joint
22 right_shoulder_pitch_joint
23 right_shoulder_roll_joint
24 right_shoulder_yaw_joint
25 right_elbow_joint
26 right_wrist_roll_joint
27 right_wrist_pitch_joint
28 right_wrist_yaw_joint
```

## Ankle PR / AB Semantics

G1 has a parallel ankle mechanism, but Unitree's low-level message can expose
two different command coordinates:

- `mode_pr:=0`: PR mode. `motor_cmd[4/5]` and `motor_cmd[10/11]` are pitch/roll
  virtual ankle joints. This matches `left_ankle_pitch_joint`,
  `left_ankle_roll_joint`, `right_ankle_pitch_joint`, and
  `right_ankle_roll_joint` in `rl_cfg_unitree_g1.yaml`, so the current direct
  kinematics adapter is correct.
- `mode_pr:=1`: AB mode. The same slots are interpreted as the parallel
  actuator coordinates `ankle_A/ankle_B`. A pitch/roll policy must not be sent
  directly in this mode. Add a Unitree-G1 ankle adapter that converts
  pitch/roll commands and feedback to/from A/B before enabling AB mode.

In other words, the official lower layer handles the PR virtual-joint
representation only when you explicitly keep the robot in PR mode. It does not
magically infer your policy's joint convention after you switch to AB mode.

Build `rl_master` after sourcing the official Unitree ROS 2 workspace. If
`unitree_hg` is not available, `rl_master` still builds, but selecting
`motor_io_backend: unitree_g1_dds` will fail at runtime with a clear error.

```bash
source /opt/ros/humble/setup.bash
source <unitree_ros2_ws>/install/setup.bash
source ./script/dev_env.sh
colcon build --packages-select rl_master
```

## Real-Robot Startup Flow

Unitree's official ROS 2 path does not normally require you to start a separate
user-space daemon that converts `lowcmd` into motor packets. Unitree SDK2 and
the robot's own runtime communicate through DDS; after the network and
CycloneDDS environment are configured correctly, ROS 2 nodes can directly see
robot topics such as `lowstate` and publish `LowCmd`.

On an offboard PC, the official setup is:

```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
ros2 topic echo lowstate --once
```

On the robot onboard computer, use the same official Unitree ROS 2 environment
for the local network interface. In both cases, verify `lowstate` before
starting OmniMorph. If `lowstate` is absent, fix the Unitree network/DDS setup
first; the OmniMorph bridge cannot create the robot-side DDS participant.

Before taking low-level control, release the active Unitree motion service
(`sport_mode`, `ai_sport`, `advanced_sport`, etc.) using Unitree's
`MotionSwitchClient` flow, as shown in the official G1 low-level examples.
The safe sequence is:

1. Power on G1 and enter the supported developer/low-level workflow described
   by Unitree.
2. Source Unitree ROS 2 and verify `lowstate`.
3. Release active motion services with Unitree's `MotionSwitchClient` or an
   official example that performs this step.
4. Start `RL_solver` with `rl_cfg_unitree_g1.yaml` selected by the runtime
   environment/config setup.
5. Send the OmniMorph mode-control word to start the policy.

Runtime:

```bash
source ./script/dev_env.sh
./script/start_rl_solver.sh --ros-args -p startup_mode_id:=0
```

Useful parameters:

```bash
./script/start_rl_solver.sh --ros-args \
  -p startup_mode_id:=0 \
  -p unitree_lowstate_topic:=lowstate \
  -p unitree_lowcmd_topic:=/lowcmd \
  -p unitree_mode_pr:=0
```

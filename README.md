# OmniMorph Runtime

<p align="center">
  <img src="./assets/readme/OmniMorph Runtime.png" alt="OmniMorph Runtime illustration" width="100%" />
</p>

<p align="center">
  Unified policy deployment runtime for humanoid, wheel-legged, and quadruped robots.
</p>

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-22314A?style=flat-square" />
  <img alt="ONNX Runtime" src="https://img.shields.io/badge/ONNX-Runtime-EA7A33?style=flat-square" />
  <img alt="MuJoCo" src="https://img.shields.io/badge/MuJoCo-Sim2Sim-4B9ECF?style=flat-square" />
  <img alt="Policies" src="https://img.shields.io/badge/Policies-RL%20%7C%20Imitation%20%7C%20Generative-4FAF7A?style=flat-square" />
</p>

OmniMorph Runtime is the deployment shell around the actual controller/runtime
packages already in this repository. The goal is simple: keep one explicit,
inspectable runtime that can serve different robot bodies, different
observation contracts, and different policy families without cloning the whole
stack every time.

It is already being used for:

- `Jingchu01` humanoid policies
- `Unitree G1` humanoid policies
- MuJoCo `sim2sim` validation
- real-robot `sim2real` execution
- RL / AMP / BeyondMimic style policies
- future diffusion or flow-matching style generative policies

## Required Environment Variables

The YAML configs use environment-variable placeholders for robot assets. `script/dev_env.sh`
does not guess these paths for you anymore, because those directories are machine-specific.
Set them explicitly before startup, and preserve them with `sudo -E` when a command needs root.

ROS 2/DDS traffic is also intentionally domain-gated. Pick one non-zero
`ROS_DOMAIN_ID` per robot or test setup before sourcing `script/dev_env.sh`;
domain `0` is blocked by the project scripts because it is the ROS 2 default
and can collide with other machines on the same LAN.

JC01:

```bash
export OMNIMORPH_RUNTIME_ROOT=/abs/path/to/omnimorph_runtime
export ROS_DOMAIN_ID=73
export ROBOT_ASSETS_DIR=/abs/path/to/jc01-model
export MUJOCO_MODEL_PATH="${ROBOT_ASSETS_DIR}/scene_jingchu01.xml"
```

Unitree G1:

```bash
export OMNIMORPH_RUNTIME_ROOT=/abs/path/to/omnimorph_runtime
export ROS_DOMAIN_ID=83
export G1_PINOCCHIO_URDF=/abs/path/to/g1_29dof.urdf
export G1_SCENE_XML=/abs/path/to/scene_29dof.xml
export MUJOCO_MODEL_PATH="${G1_SCENE_XML}"
```

These variables are consumed here:

- `src/omnimorph_rl_controller/rl_master/config/rl_cfg_jc01.yaml`: `${ROBOT_ASSETS_DIR}`
- `src/omnimorph_rl_controller/rl_master/config/rl_cfg_unitree_g1.yaml`: `${G1_PINOCCHIO_URDF}`, `${G1_SCENE_XML}`
- sim2sim CLI examples below: `${MUJOCO_MODEL_PATH}`

If you want this to persist across sessions, add those `export` lines to `~/.bashrc` or a machine-local startup file.

Before bringing up hardware, you can check whether the chosen domain already
sees ROS nodes:

```bash
./script/check_ros_domain.sh --domain "${ROS_DOMAIN_ID}" --strict
```

Startup scripts run the same scan in warning mode by default. Set
`OMNIMORPH_ROS_DOMAIN_SCAN=off` only after manually verifying the domain is
expected to be shared.

## Low-Memory Remote Build

On small embedded machines or SSH sessions that disconnect during compilation,
use the low-memory build helper. It limits parallelism, reduces compiler debug
metadata, uses lower optimization during compilation, and skips optional
developer targets by default.

```bash
tmux new -s omnimorph-build
./script/build_low_memory.sh rl_master mujoco_sim2sim
```

By default this also disables the optional Unitree SDK2 backend and `test_kine`
tool to reduce compile/link pressure. Re-enable them only when needed:

```bash
OMNIMORPH_LOW_MEMORY_DISABLE_UNITREE_SDK2=0 ./script/build_low_memory.sh rl_master
OMNIMORPH_LOW_MEMORY_BUILD_TEST_TOOLS=1 ./script/build_low_memory.sh rl_master
```

## Terminal Workflow

### Real robot

Terminal 1:

```bash
source ./script/dev_env.sh
sudo -E ./script/start_driver_jc01.sh  # JC01 only
```

For Unitree G1, Terminal 1 is the official Unitree low-level runtime/DDS check
instead of a repository driver:

```bash
source ~/unitree_ros2/setup.sh
ros2 topic echo lowstate --once
```

Terminal 2:

```bash
source ./script/dev_env.sh
sudo -E ./script/start_imu_yesense.sh
```

Terminal 3:

```bash
source ./script/dev_env.sh
sudo -E ./script/start_jc01_policy.sh --mode-id 0
# Unitree G1:
sudo -E ./script/start_unitree_g1_policy.sh --mode-id 0
```

Terminal 4:

```bash
source ./script/dev_env.sh
sudo -E ./script/start_joylaunch.sh
```

Start policy manually:

```bash
./script/publish_mode_control.sh start --mode-id 0
# Equivalent raw ROS 2 publish:
# ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
```


> Note: real-robot driver startup is robot-specific. JC01 uses the local shared-memory driver script. Unitree G1 selects the low-level transport in `config/rl_cfg_unitree_g1.yaml` with `robot_identity.unitree_transport: sdk2` or `ros2`. The same selector switches both motor IO and Unitree lowstate IMU input.
### MuJoCo sim2sim (Python viewer frontend)

Terminal 1:

```bash
ros2 run mujoco_sim2sim mujoco_sim_bridge --ros-args \
  --params-file /path/to/omnimorph_runtime/src/omnimorph_sim2sim/mujoco_sim2sim/config/unitree_g1_beyondmimic_sim2sim.yaml \
  -p rl_cfg_path:=/path/to/omnimorph_runtime/src/omnimorph_rl_controller/rl_master/config/rl_cfg_unitree_g1.yaml \
  -p startup_mode_id:=1 \
  -p enable_viewer:=false \
  -p enable_python_viewer_stream:=true \
  -p enable_python_viewer_inspector:=true
```

Terminal 2:

```bash
source ./script/dev_env.sh
ros2 run mujoco_sim2sim mujoco_sim_viewer_frontend.py --ros-args \
  -p model_path:="${MUJOCO_MODEL_PATH}" \
  -p enable_viewer:=true \
  -p viewer_fps:=500.0
```

Start policy manually:

```bash
./script/publish_mode_control.sh start --mode-id 0
```

## What Lives Here

- `src/omnimorph_rl_controller/rl_master`: fused runtime, policy adapter, observation builder, deploy mode/profile registry
- `src/omnimorph_sim2sim/mujoco_sim2sim`: fused MuJoCo backend and Python viewer frontend path
- `src/omnimorph_rl_controller/joint_motor_test`: standalone joint/motor trajectory verification tooling
- `script/`: low-level helpers for environment setup, operator tools, IMU, and mode control

## Runtime Principles

- One runtime core, many robots
- One mode/profile system, many policy contracts
- Keep joint-order, observation-order, and reference-order rules explicit
- Keep sim2sim and sim2real as close as possible at runtime boundaries
- Remove split-runtime and hidden legacy paths when they stop paying for themselves

## Common Runtime Commands

| Purpose | Command |
| --- | --- |
| Start JC01 policy runtime | `sudo -E ./script/start_jc01_policy.sh --mode-id <N>` |
| Start Unitree G1 policy runtime | `sudo -E ./script/start_unitree_g1_policy.sh --mode-id <N>` |
| Start solver directly | `./script/start_rl_solver.sh --rl-cfg <rl_cfg.yaml> --mode-id <N>` |
| Start JC01 driver | `sudo ./script/start_driver_jc01.sh` |
| Check Unitree G1 lowstate | `source ~/unitree_ros2/setup.sh && ros2 topic echo lowstate --once` |
| Start MuJoCo Python frontend | `ros2 run mujoco_sim2sim mujoco_sim_viewer_frontend.py --ros-args -p model_path:="${MUJOCO_MODEL_PATH}" -p enable_viewer:=true -p viewer_fps:=500.0` |
| Start MuJoCo fused backend (optional) | `./script/sim2sim_runtime.sh --model-path "${MUJOCO_MODEL_PATH}" --mode-id <N>` |
| Start policy | `./script/publish_mode_control.sh start --mode-id <N>` |
| Switch mode only | `./script/publish_mode_control.sh switch --mode-id <N>` |
| Stop policy | `./script/publish_mode_control.sh stop` |

## Documentation

- Runtime/controller docs:
  [src/omnimorph_rl_controller/rl_master/docs/README.md](./src/omnimorph_rl_controller/rl_master/docs/README.md)
- Sim2sim docs:
  [src/omnimorph_sim2sim/mujoco_sim2sim/docs/README.md](./src/omnimorph_sim2sim/mujoco_sim2sim/docs/README.md)
- Script usage:
  [script/README.md](./script/README.md)

## License

This project is licensed under the MIT License.
See the [LICENSE](./LICENSE) file for the full text.

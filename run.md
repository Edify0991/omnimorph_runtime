# OmniMorph Runtime

JC01:

```bash
export ROBOT_ASSETS_DIR=/home/nvidia/Documents/jc_deploy/jc01-model
```

Unitree G1:

```bash
export OMNIMORPH_RUNTIME_ROOT=/abs/path/to/omnimorph_runtime
export G1_PINOCCHIO_URDF=/abs/path/to/g1_29dof.urdf
export G1_SCENE_XML=/abs/path/to/scene_29dof.xml
export MUJOCO_MODEL_PATH="${G1_SCENE_XML}"
```

These variables are consumed here:

- `src/omnimorph_rl_controller/rl_master/config/rl_cfg_jc01.yaml`: `${ROBOT_ASSETS_DIR}`
- `src/omnimorph_rl_controller/rl_master/config/rl_cfg_unitree_g1.yaml`: `${G1_PINOCCHIO_URDF}`, `${G1_SCENE_XML}`
- sim2sim CLI examples below: `${MUJOCO_MODEL_PATH}`

If you want this to persist across sessions, add those `export` lines to `~/.bashrc` or a machine-local startup file.

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
sudo -E ./script/sim2real_runtime.sh --mode-id 0
# Unitree G1:
sudo -E ./script/start_rl_solver.sh \
  --rl-cfg src/omnimorph_rl_controller/rl_master/config/rl_cfg_jc01.yaml \
  --mode-id 0
```

Terminal 4:

```bash
source ./script/dev_env.sh
sudo -E ./script/start_joylaunch.sh
```

Start policy manually:

```bash
ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
```


> Note: real-robot driver startup is robot-specific. JC01 uses the local shared-memory driver script. Unitree G1 uses the official Unitree runtime as the low-level driver; `RL_solver` connects to `lowstate` and `/lowcmd` in-process through `motor_io_backend: unitree_g1_dds`.
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
ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
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
| Start solver | `./script/start_rl_solver.sh --ros-args -p startup_mode_id:=<N>` |
| Start JC01 driver | `sudo ./script/start_driver_jc01.sh` |
| Check Unitree G1 lowstate | `source ~/unitree_ros2/setup.sh && ros2 topic echo lowstate --once` |
| Start MuJoCo Python frontend | `ros2 run mujoco_sim2sim mujoco_sim_viewer_frontend.py --ros-args -p model_path:="${MUJOCO_MODEL_PATH}" -p enable_viewer:=true -p viewer_fps:=500.0` |
| Start MuJoCo fused backend (optional) | `./script/sim2sim_runtime.sh --model-path "${MUJOCO_MODEL_PATH}" --mode-id <N>` |
| Start policy | `ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 1000 + N}"` |
| Switch mode only | `ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 2000 + N}"` |
| Stop policy | `ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 11}"` |

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

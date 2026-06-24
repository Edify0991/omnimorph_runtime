# Architecture-Adaptive Build Guide

This project now includes an architecture-adaptive CMake template for `rl_master` and `mujoco_sim2sim`.

Scope:
- Keeps your existing DDS topics/protocol unchanged.
- Keeps your deploy lifecycle/state machine unchanged.
- Only upgrades build-system portability and architecture-aware optimization flags.
- Adds configurable realtime thread pinning/scheduling for `RL_controller` and `RL_solver`.

## 1) What Changed

### `rl_master/CMakeLists.txt`
- Auto-detects host architecture:
  - `aarch64|arm64` -> ARM profile
  - `x86_64|amd64` -> x86_64 profile
  - fallback -> generic
- Adds tuning options:
  - `RL_MASTER_ENABLE_ARCH_TUNING` (default `ON`)
  - `RL_MASTER_ENABLE_NATIVE_TUNING` (default `OFF`)
  - `RL_MASTER_ARM_BASELINE` (default `armv8-a`)
  - `RL_MASTER_LOW_MEMORY_BUILD` (default `OFF`)
  - `RL_MASTER_ENABLE_UNITREE_SDK2` (default `ON`)
  - `RL_MASTER_BUILD_TEST_TOOLS` (default `ON`)
- Replaces hardcoded ONNX Runtime path with portable detection:
  - cache/env: `ONNXRUNTIME_ROOT` (or env `ONNXRUNTIME_DIR`)
  - auto-search in `/usr/local` and `/usr`
- Keeps legacy executable targets and source organization intact.

### `mujoco_sim2sim/CMakeLists.txt`
- Adds the same architecture-aware tuning knobs:
  - `MUJOCO_SIM2SIM_ENABLE_ARCH_TUNING` (default `ON`)
  - `MUJOCO_SIM2SIM_ENABLE_NATIVE_TUNING` (default `OFF`)
  - `MUJOCO_SIM2SIM_ARM_BASELINE` (default `armv8-a`)
  - `MUJOCO_SIM2SIM_LOW_MEMORY_BUILD` (default `OFF`)

## 2) Build Examples

### Portable default build (recommended first)
```bash
colcon build --symlink-install --packages-up-to rl_master mujoco_sim2sim
```

### ARM board (portable ARM baseline, no machine lock-in)
```bash
colcon build --symlink-install \
  --packages-up-to rl_master mujoco_sim2sim \
  --cmake-args \
    -DRL_MASTER_ENABLE_ARCH_TUNING=ON \
    -DRL_MASTER_ENABLE_NATIVE_TUNING=OFF \
    -DRL_MASTER_ARM_BASELINE=armv8-a \
    -DMUJOCO_SIM2SIM_ENABLE_ARCH_TUNING=ON \
    -DMUJOCO_SIM2SIM_ENABLE_NATIVE_TUNING=OFF \
    -DMUJOCO_SIM2SIM_ARM_BASELINE=armv8-a
```

### Single-machine peak tuning (rebuild when moving to a different CPU)
```bash
colcon build --symlink-install \
  --packages-up-to rl_master mujoco_sim2sim \
  --cmake-args \
    -DRL_MASTER_ENABLE_NATIVE_TUNING=ON \
    -DMUJOCO_SIM2SIM_ENABLE_NATIVE_TUNING=ON
```

### Low-memory remote build
Use this on embedded boards or SSH sessions where `colcon build` often stalls
or disconnects under memory pressure:

```bash
tmux new -s omnimorph-build
./script/build_low_memory.sh rl_master mujoco_sim2sim
```

The helper sets `CMAKE_BUILD_PARALLEL_LEVEL=1`, `MAKEFLAGS=-j1 -l1`, enables
`RL_MASTER_LOW_MEMORY_BUILD` and `MUJOCO_SIM2SIM_LOW_MEMORY_BUILD`, disables
native tuning, and skips optional `rl_master` SDK2/test-tool targets by default.

Equivalent manual form:

```bash
colcon build --parallel-workers 1 \
  --event-handlers console_direct+ \
  --packages-select rl_master mujoco_sim2sim \
  --cmake-args \
    --no-warn-unused-cli \
    -Wno-dev \
    -DCMAKE_BUILD_TYPE=Release \
    -DRL_MASTER_LOW_MEMORY_BUILD=ON \
    -DMUJOCO_SIM2SIM_LOW_MEMORY_BUILD=ON \
    -DRL_MASTER_ENABLE_NATIVE_TUNING=OFF \
    -DMUJOCO_SIM2SIM_ENABLE_NATIVE_TUNING=OFF \
    -DRL_MASTER_ENABLE_UNITREE_SDK2=OFF \
    -DRL_MASTER_BUILD_TEST_TOOLS=OFF
```

If the SDK2 backend or developer tools are required:

```bash
OMNIMORPH_LOW_MEMORY_DISABLE_UNITREE_SDK2=0 ./script/build_low_memory.sh rl_master
OMNIMORPH_LOW_MEMORY_BUILD_TEST_TOOLS=1 ./script/build_low_memory.sh rl_master
```

## 3) ONNX Runtime Path

If ONNX Runtime is not found automatically, set:
```bash
export ONNXRUNTIME_ROOT=/path/to/onnxruntime
```

Expected layout:
- `${ONNXRUNTIME_ROOT}/include/.../onnxruntime_cxx_api.h`
- `${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so`

Then rebuild `rl_master`.

## 4) Notes for ARM Optimization

- Start with:
  - `onnx_intra_threads: 1`
  - `onnx_inter_threads: 1`
  in `config/rl_cfg_jc01.yaml`, then tune per CPU load.
- If you deploy on heterogeneous ARM cores (big.LITTLE), keep your real-time pinning policy aligned with big cores.
- Keep `native_tuning=OFF` for release artifacts shared across multiple ARM devices.

## 5) Realtime Thread Pinning (YAML)

Two levels are supported:

1. Per-profile defaults (`sim2real.realtime`, `stand_sim2real.realtime`)
2. Process-level override (`runtime_process.controller`, `runtime_process.solver`)

Process-level override has higher priority and is recommended for big.LITTLE tuning.

Example:

```yaml
runtime_process:
  controller:
    enabled: true
    lock_memory: true
    set_affinity: true
    cpu_id: 3
    use_fifo: true
    fifo_priority: 90
  solver:
    enabled: true
    lock_memory: true
    set_affinity: true
    cpu_id: 2
    use_fifo: true
    fifo_priority: 90
```

## 6) Realtime Thread Pinning (Launch / Env Override)

Environment variables can override YAML at runtime:

- Controller prefix: `RL_MASTER_CONTROLLER_RT_`
- Solver prefix: `RL_MASTER_SOLVER_RT_`
- Keys: `ENABLED`, `LOCK_MEMORY`, `SET_AFFINITY`, `CPU_ID`, `USE_FIFO`, `FIFO_PRIORITY`

Examples:

```bash
export RL_MASTER_SOLVER_RT_CPU_ID=4
export RL_MASTER_SOLVER_RT_FIFO_PRIORITY=92
```

The supported sim2sim launch no longer starts a standalone controller process.

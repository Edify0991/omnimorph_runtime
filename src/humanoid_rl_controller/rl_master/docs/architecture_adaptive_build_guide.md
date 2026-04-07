# Architecture-Adaptive Build Guide

This project now includes an architecture-adaptive CMake template for `rl_master` and `mujoco_sim2sim`.

Scope:
- Keeps your existing DDS topics/protocol unchanged.
- Keeps your deploy lifecycle/state machine unchanged.
- Only upgrades build-system portability and architecture-aware optimization flags.

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
- Replaces hardcoded ONNX Runtime path with portable detection:
  - cache/env: `ONNXRUNTIME_ROOT` (or env `ONNXRUNTIME_DIR`)
  - auto-search in `/usr/local` and `/usr`
- Keeps legacy executable targets and source organization intact.

### `mujoco_sim2sim/CMakeLists.txt`
- Adds the same architecture-aware tuning knobs:
  - `MUJOCO_SIM2SIM_ENABLE_ARCH_TUNING` (default `ON`)
  - `MUJOCO_SIM2SIM_ENABLE_NATIVE_TUNING` (default `OFF`)
  - `MUJOCO_SIM2SIM_ARM_BASELINE` (default `armv8-a`)

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
  in `config/rl_cfg.yaml`, then tune per CPU load.
- If you deploy on heterogeneous ARM cores (big.LITTLE), keep your real-time pinning policy aligned with big cores.
- Keep `native_tuning=OFF` for release artifacts shared across multiple ARM devices.

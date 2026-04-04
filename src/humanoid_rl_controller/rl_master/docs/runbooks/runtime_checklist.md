# Runtime Checklist And Data Logging Runbook

## 1. Engineering Conventions

### 1.1 Module Layout

The deploy stack is now grouped by functional ownership:

- `include/rl_master/runtime/`: runtime/system helpers
- `include/rl_master/filters/`: generic filters
- `include/rl_master/logging/`: structured logging interfaces
- `include/rl_master/solver/`: solver interfaces (DDS bridge integration + SHM motor I/O abstraction)
- `runtime/`: implementations of runtime helpers
- `filters/`: implementations of filter helpers
- `logging/`: implementations of structured logging
- `solver/`: solver implementations (`robot_solver.cpp`, `motor_shm_io.cpp`)
- `tools/analysis/`: post-run analysis tools

### 1.2 Naming Conventions

- Type/class names: `PascalCase` (for example `StructuredLogger`)
- Function names: `camelCase` (for example `setRealtimePriority`)
- Variables: `snake_case` for local/global C++ data and config keys
- Constants: `kUpperCamelCase` for compile-time constants

### 1.3 Declaration vs Definition

- Public interfaces are declared in headers under `include/rl_master/...`
- Implementations are defined in matching `.cpp` modules
- Avoid defining new utility classes directly inside large runtime files

## 2. Build Checklist

From workspace root:

```bash
colcon build --packages-select SharedMemory imu_communication_yesense rl_master
```

If first-time host setup:

```bash
sudo apt update
sudo apt install -y python3 python3-pip python3-evdev python3-requests
```

## 3. Bringup Checklist

Recommended launch order:

1. Motor driver stack (external repository / external process)
2. IMU node
3. `RL_solver`
4. `RL_controller`
5. `joyLaunch.py`
6. DDS self-check

Example:

```bash
cd script
sudo ./driver.sh
sudo ./imu.sh
sudo ./solver.sh
sudo ./controller.sh
sudo python3 joyLaunch.py
sudo ./dds_selfcheck.sh
```

## 4. Data Logging Format (Structured)

When `save_data_flag: true`, both solver and controller output:

- `<session_base>_solver_metadata.json`
- `<session_base>_solver_records.jsonl`
- `<session_base>_controller_metadata.json`
- `<session_base>_controller_records.jsonl`

`<session_base>` is generated from config `data_path`.

### 4.1 Metadata File

JSON object includes:

- `schema_version`
- `created_time_unix_sec`
- `string_fields` (policy path/name/family, manifest path, module)
- `numeric_fields` (obs/action dims, control hz, watchdog timeout)
- `vector_fields` (kps/kds/tau limits)
- `string_list_fields` (joint order mappings, sub-model names/paths)

### 4.2 Records File (JSONL)

Each line is one JSON object:

- `record_type`
- `monotonic_time_sec`
- `scalars` (frame index, mode, state flags)
- `vectors` (joint states, targets, observations, actions)

Solver records use `record_type=solver_loop`.
Controller records use `record_type=controller_step`.

## 5. Analysis Workflow

Primary tool:

- `tools/analysis/analyze_structured_logs.py`

Example summary + CSV export:

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/analyze_structured_logs.py \
  --records /path/to/session_solver_records.jsonl \
  --vector-field motor_state_tau \
  --csv-out /tmp/motor_state_tau.csv
```

Plot selected indices:

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/analyze_structured_logs.py \
  --records /path/to/session_solver_records.jsonl \
  --vector-field motor_state_tau \
  --plot --plot-indices 3,9
```

Compatibility wrapper (legacy name):

```bash
python3 src/humanoid_rl_controller/rl_master/data_process.py \
  --records /path/to/session_solver_records.jsonl \
  --vector-field motor_state_tau
```

## 6. Validation Checklist After Changes

- `ros2 topic echo /imu/yesense --once`
- `ros2 topic echo /humanoid/rl/state --once`
- `ros2 topic echo /humanoid/rl/command --once`
- `script/dds_selfcheck.sh`
- Confirm structured output files exist in `rl_master/data/`

## 7. Operational Notes

- Motor closed loop remains shared-memory based in `RL_solver`:
  - `getMotorState()`
  - `sendMotorCmd()`
- Upper-level transport remains DDS-based.
- Controller mode switching is profile-driven:
  - Configure `deploy_mode_profiles` in `config/rl_cfg.yaml`.
  - Runtime control word channel supports `mode_id`, `1000+mode_id`, `2000+mode_id`.
- To compare different models/parameters reliably, use metadata files as the source of truth for run context.

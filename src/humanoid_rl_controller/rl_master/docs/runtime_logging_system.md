# Runtime Logging System

## 1. Current Active Logging Path

The active runtime logging path is now:

```text
fused runtime main loop -> RuntimeRecorder -> single .mcap file
```

The active runtime logger produces exactly one primary file per run:

- `<session>.mcap`

Only the `.mcap` runtime log is produced for new runs.

## 2. Why MCAP Is the Primary Runtime Format

The runtime logger is optimized for:

- stable single-file recording
- explicit time ordering
- topic/channel-style separation for events and tick data
- compatibility with robotics timeline inspection workflows
- later export into analysis-friendly table formats

In this repository, MCAP is the runtime recording format, while CSV / NPZ / Parquet are offline export formats.

Current implementation note:

- the in-repo writer now writes chunked MCAP
- `logging.writer.compression` supports `none / zstd / lz4`
- compression is applied on chunk payloads, not just stored in metadata
- the runtime analysis tools in `tools/analysis/` understand this chunked format directly

## 3. Active Channels

Current active channels are:

- `runtime/config`
- `runtime/event`
- `runtime/tick`
- `runtime/source/base_imu` when base IMU source-sample logging is enabled
- `runtime/source/policy_action`
- `runtime/source/external/<name>` when external observation source-sample logging is enabled
- `runtime/amp` when AMP score logging is enabled
- `runtime/reference_motion` when reference-motion logging is enabled
- `runtime/external_obs/<name>` when external observation logging is enabled

`runtime/tick` is the main per-control-step record. It aggregates both policy-side and execution-side data into one aligned payload.

`runtime/source/...` channels are different from `runtime/tick`:

- `runtime/tick` is control-loop aligned
- `runtime/source/...` preserves raw source arrival cadence
- if IMU is slower than proprioception, that difference is visible in the log as different message timestamps and message counts

## 4. Main Logged Content

Typical `runtime/tick` payload includes:

- frame and monotonic time
- active mode id and deploy lifecycle state
- teleop command (`vx`, `vy`, `dyaw`)
- `joint_q / joint_dq / joint_tau`
- `joint_target_q / joint_target_tau`
- `joint_cmd_*`
- `joint_state_*`
- `motor_cmd_*`
- `motor_state_*`
- `motor_cmd_mode`
- `observation`
- `policy_action`
- optional AMP score
- optional named features / external observations

Sparse transitions are emitted on `runtime/event`, for example:

- `solver_initialized`
- `controller_initialized`
- `mode_switch`
- `lifecycle_transition`
- `solver_mode_config_switched`
- `runtime_log_queue_drop`

## 5. Logging Configuration

The active logging config is top-level in `rl_cfg.yaml`:

```yaml
logging:
  enabled: true
  backend: mcap
  output_dir: data/runtime_logs
  session_name_policy: timestamp_policy
  writer:
    queue_capacity: 256
    flush_period_ms: 250
    compression: zstd
    chunk_size_kb: 1024
  tick:
    enabled: true
    decimation: 1
    include_observation: true
    include_policy_action: true
    include_motor_io: true
    include_joint_targets: true
    include_external_observations: false
  events:
    enabled: true
  reference_motion:
    enabled: false
  amp:
    enabled: false
  source_samples:
    enabled: true
    include_base_imu: true
    include_external_observations: false
  export:
    default_format: none
```

`save_data_flag` has been removed. Runtime logging is controlled only by top-level `logging.enabled`.

## 6. Analysis Workflow

Analyze a runtime MCAP log:

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/analyze_runtime_log.py \
  series \
  --mcap /abs/path/to/session.mcap \
  --topic runtime/tick \
  --field motor_state_tau
```

Analyze timing / jitter / hold-alignment:

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/analyze_runtime_log.py \
  timing \
  --mcap /abs/path/to/session.mcap \
  --topics runtime/tick runtime/source/base_imu runtime/source/policy_action \
  --reference-topic runtime/tick
```

Export runtime tick data to CSV:

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/export_runtime_log.py \
  --mcap /abs/path/to/session.mcap \
  --topic runtime/tick \
  --format csv \
  --output /tmp/runtime_tick.csv
```

Export another channel to NPZ:

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/export_runtime_log.py \
  --mcap /abs/path/to/session.mcap \
  --topic runtime/amp \
  --format npz \
  --output /tmp/runtime_amp.npz
```

Parquet export is supported when `pyarrow` is installed.

## 7. Different-Rate Sensors

This runtime log intentionally separates:

- aligned control-step data: `runtime/tick`
- raw asynchronous sensor cadence: `runtime/source/...`

That means a slower IMU and a faster proprioceptive stream are not forced into the same apparent frequency in the stored log:

- proprioception still appears every control tick inside `runtime/tick`
- IMU can appear at its own callback cadence in `runtime/source/base_imu`
- offline analysis can choose either:
  - resample source channels onto control ticks
  - or analyze true source latency / jitter directly from raw source timestamps

## 8. Active Tooling

Current active path:

- `.mcap` runtime file
- `analyze_runtime_log.py`
- `export_runtime_log.py`

The old JSONL structured logger and its helper scripts have been removed from the active codebase.

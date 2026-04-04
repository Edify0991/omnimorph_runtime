# Observation Pipeline Diagram

## 1. End-to-End Flow

```mermaid
flowchart TD
    A[observation_manifest_*.yaml] --> B[ObservationManifest::loadFromYAML]
    B --> C[ObservationBuilder ctor]
    C --> D[registry(): term -> lambda provider]
    D --> E[resolved_terms_ + offset]

    F[RobotState + Cmd + last_action + phase_t] --> G[RL_controller::buildObservationFeatureContext]
    H[ReferenceMotionProvider] --> G
    I[ExternalObservationProvider] --> G

    E --> J[ObservationBuilder::build]
    G --> J
    F --> J

    J --> K[obs vector]
    K --> L[obs_deque stack]
    L --> M[stacked_obs_buffer]
    M --> N[OnnxPolicyRunner::forward]
```

## 2. `registry()` Role

- `registry()` is a static provider table: `term_name -> ObservationProvider`.
- Each `ObservationProvider` contains:
  - `gather`: one lambda to append this term's values into output `obs`.
  - `default_dim`: default dimension for this term.
  - `supports_count`: whether manifest `count` override is allowed.
  - `supports_components`: whether manifest `components` is allowed.
- Build stage:
  - constructor resolves manifest terms with `registry()`, then stores them into `resolved_terms_`.
- Runtime stage:
  - `ObservationBuilder::build()` iterates `resolved_terms_` and runs each `gather(...)`.

Key code:
- `ObservationBuilder::registry()` in `observation_builder.cpp`
- `ObservationBuilder::build()` in `observation_builder.cpp`

## 3. Term-to-Lambda Map

- `phase`: sinusoidal gait phase (`sin/cos`)
- `command`: `[vx, vy, dyaw]` (or component subset)
- `joint_pos`: mapped joint position residuals `(q - default_angle)`
- `joint_vel`: mapped joint velocities
- `last_action`: previous action
- `base_ang_vel`: IMU angular velocity
- `base_rpy`: base orientation in RPY
- `reference_motion`: feature from `feature_context.named_features["reference_motion"]`
- `external_sensor`: feature from `feature_context.named_features[source]`
- `feature`: generic named feature slot from `feature_context`

## 4. AMP Layout (Current `observation_manifest_amp.yaml`)

Total dim: **47**

| Index range | Dim | Term |
|---|---:|---|
| 0..1 | 2 | phase |
| 2..4 | 3 | command(vx, vy, dyaw) |
| 5..16 | 12 | joint_pos |
| 17..28 | 12 | joint_vel |
| 29..40 | 12 | last_action |
| 41..43 | 3 | base_ang_vel |
| 44..46 | 3 | base_rpy |

## 5. BeyondMimic Layout (Current `observation_manifest_beyondmimic.yaml`)

Total dim: **71** (= AMP 47 + reference_motion 24)

| Index range | Dim | Term |
|---|---:|---|
| 0..1 | 2 | phase |
| 2..4 | 3 | command(vx, vy, dyaw) |
| 5..16 | 12 | joint_pos |
| 17..28 | 12 | joint_vel |
| 29..40 | 12 | last_action |
| 41..43 | 3 | base_ang_vel |
| 44..46 | 3 | base_rpy |
| 47..70 | 24 | reference_motion |

## 6. External Sensor Append Example

If manifest appends:

```yaml
- name: external_sensor
  source: vision_latent
  count: 64
```

then layout becomes:

- `vision_latent`: next 64 dims appended after existing terms.
- Missing sensor data is zero-padded by `ExternalObservationProvider::collect`.

## 7. What `obs_index_map` Actually Affects

- `obs_index_map` only affects mapped joint terms (`joint_pos`, `joint_vel`).
- Non-joint terms (`phase`, `command`, `base_*`, `reference_motion`, `external_sensor`) are not reordered by this map.

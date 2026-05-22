# RL Master Docs Index

This folder contains the active deployment/runtime documentation for the fused
`RL_solver` + `RL_controller` stack.

## Core Guides

- Real-robot deployment:
  [dds_sim2real_deploy_guide.md](./dds_sim2real_deploy_guide.md)
- Policy deployment and config workflow:
  [policy_deploy_and_config_guide.md](./policy_deploy_and_config_guide.md)
- Runtime checklist:
  [runbooks/runtime_checklist.md](./runbooks/runtime_checklist.md)

## Architecture

- Runtime architecture overview:
  [policy_runtime_architecture.md](./policy_runtime_architecture.md)
- End-to-end function chain:
  [runtime_end_to_end_function_flow.md](./runtime_end_to_end_function_flow.md)
- Operator GUI architecture:
  [operator_gui_architecture.md](./operator_gui_architecture.md)
- Topic matrix:
  [topic_interface_matrix.md](./topic_interface_matrix.md)

## Observation And Policy I/O

- Observation pipeline:
  [observation_pipeline_diagram.md](./observation_pipeline_diagram.md)
- Observation order contract:
  [deploy_observation_order_contract_guide.md](./deploy_observation_order_contract_guide.md)
- Runtime logging:
  [runtime_logging_system.md](./runtime_logging_system.md)

## Sensor / Estimation / Multimodal

- Base velocity Kalman filter:
  [base_velocity_kalman_filter_guide.md](./base_velocity_kalman_filter_guide.md)
- Realsense camera pipeline:
  [realsense_camera_pipeline.md](./realsense_camera_pipeline.md)

## Protocol / Runtime Contracts

- DDS protocol v2 notes:
  [dds_protocol_v2_dynamic_joint_count.md](./dds_protocol_v2_dynamic_joint_count.md)

## Robot / Policy Adaptation

- Full-body extension:
  [full_body_extension_guide.md](./full_body_extension_guide.md)
- BeyondMimic sim2real adaptation:
  [beyondmimic_sim2real_adaptation.md](./beyondmimic_sim2real_adaptation.md)

## Maintainer Notes

- Prefer linking new docs from this index instead of duplicating startup
  instructions in multiple places.

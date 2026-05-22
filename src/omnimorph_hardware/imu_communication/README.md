# Yesense IMU Communication

This package contains the ROS 2 driver wrapper and decode utilities used by the
real-robot deploy path to ingest Yesense IMU data.

## Role In This Repository

- package: `imu_communication_yesense`
- primary executable: `imu_communication_yesense`
- standard launcher script:
  [`script/start_imu_yesense.sh`](${OMNIMORPH_RUNTIME_ROOT}/script/start_imu_yesense.sh)

The node publishes IMU samples that are consumed by the fused real-robot
runtime (`RL_solver`) through DDS.

## Standard Usage

From the workspace root:

```bash
sudo ./script/start_imu_yesense.sh
```

If you need to skip the serial reset step or pass ROS arguments:

```bash
sudo ./script/start_imu_yesense.sh --no-serial-reset -- --ros-args -r __node:=imu_yesense
```

## Notes

- For full real-robot startup order, see:
  [`dds_sim2real_deploy_guide.md`](${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_rl_controller/rl_master/docs/dds_sim2real_deploy_guide.md)

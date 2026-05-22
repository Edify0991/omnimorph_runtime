# Sim2Sim Runtime Environment Notes

This note collects the main runtime-environment pitfalls discovered while bringing up MuJoCo sim2sim for `engineai_walk`, along with the exact commands to check and fix them.

## 1. Recommended Clean Runtime Environment

Before running any ROS2 sim2sim executable, use a clean shell and source ROS/workspace in this order:

```bash
source /opt/ros/humble/setup.bash
source ${OMNIMORPH_RUNTIME_ROOT}/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/roslog
mkdir -p /tmp/roslog
```

If you suspect Conda has polluted the shell, start from:

```bash
conda deactivate
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE _CE_CONDA _CE_M
```

Then source ROS/workspace again.

## 2. FastDDS / RMW Selection

For this repository's supported sim2sim path, ROS2 topics still carry:

- `/omnimorph/rl/mode_control`
- `/omnimorph/rl/teleop`
- `/omnimorph/rl/state`
- optional Python viewer topics

To avoid mixed middleware between terminals, set:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Usually no extra FastDDS XML profile is needed for single-machine local sim2sim.

## 3. ROS Log Directory Failures

Inside sandboxed or read-only environments, ROS logging can fail early with file open errors under `~/.ros/log`.

Use:

```bash
export ROS_LOG_DIR=/tmp/roslog
mkdir -p /tmp/roslog
```

This avoids unrelated startup failures while debugging the actual runtime.

## 4. Conda `libstdc++` Hijacking

If sim2sim fails with errors such as:

```text
.../anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found
```

then Conda's `libstdc++.so.6` was loaded instead of the system one required by ROS Humble.

Check the available symbol versions:

```bash
strings /home/edify/Software/anaconda3/lib/libstdc++.so.6 | grep GLIBCXX_3.4.30
strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX_3.4.30
```

If Conda lacks the required symbol, do not run ROS2/C++ executables from that Conda shell.

As a temporary one-shot workaround:

```bash
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u PYTHONPATH \
bash -lc '
source /opt/ros/humble/setup.bash
source ${OMNIMORPH_RUNTIME_ROOT}/install/setup.bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/roslog
mkdir -p /tmp/roslog
ros2 run mujoco_sim2sim mujoco_sim_bridge --help
'
```

## 5. ONNX Runtime Mismatch

The repository must use the same ONNX Runtime family at build time and runtime.

The intended configuration is:

- headers: `/opt/ros/humble/include/onnxruntime`
- library: `/opt/ros/humble/lib/libonnxruntime.so`

### 5.1 Check build-time selection

```bash
grep -E 'ONNXRUNTIME_(INCLUDE_DIR|LIBRARY|ROOT)' ${OMNIMORPH_RUNTIME_ROOT}/build/rl_master/CMakeCache.txt
```

Expected result:

```text
ONNXRUNTIME_INCLUDE_DIR:PATH=/opt/ros/humble/include/onnxruntime
ONNXRUNTIME_LIBRARY:FILEPATH=/opt/ros/humble/lib/libonnxruntime.so
```

### 5.2 Check runtime selection

```bash
source /opt/ros/humble/setup.bash
source ${OMNIMORPH_RUNTIME_ROOT}/install/setup.bash

ldd ${OMNIMORPH_RUNTIME_ROOT}/install/rl_master/lib/rl_master/librl_master_runtime.so | grep onnxruntime
ldd ${OMNIMORPH_RUNTIME_ROOT}/install/mujoco_sim2sim/lib/mujoco_sim2sim/mujoco_sim_bridge | grep onnxruntime
```

Expected result:

```text
/opt/ros/humble/lib/libonnxruntime.so.1
```

### 5.3 Confirm library version tags

```bash
readelf --version-info /opt/ros/humble/lib/libonnxruntime.so | grep VERS_
readelf --version-info /usr/local/lib/libonnxruntime.so.1 | grep VERS_
```

In the observed environment:

- `/opt/ros/humble` exported `VERS_1.20.0`
- `/usr/local/lib` exported `VERS_1.24.4`

If runtime accidentally loads `/usr/local/lib/libonnxruntime.so.1`, you can get crashes or ABI-looking metadata failures.

## 6. RPATH Fix Already Applied

This repository now writes explicit ONNX Runtime RPATH/RUNPATH for:

- `rl_master_runtime`
- `RL_solver`
- `mujoco_sim_bridge`

The RPATH is derived from the resolved `ONNXRUNTIME_LIBRARY`, not only from `ONNXRUNTIME_ROOT`.

Rebuild with:

```bash
source /opt/ros/humble/setup.bash
export ONNXRUNTIME_ROOT=/opt/ros/humble
cd ${OMNIMORPH_RUNTIME_ROOT}
colcon build --packages-select rl_master mujoco_sim2sim --cmake-clean-cache
```

Verify RPATH:

```bash
readelf -d ${OMNIMORPH_RUNTIME_ROOT}/install/rl_master/lib/rl_master/librl_master_runtime.so | grep -E 'RPATH|RUNPATH'
readelf -d ${OMNIMORPH_RUNTIME_ROOT}/install/rl_master/lib/rl_master/RL_solver | grep -E 'RPATH|RUNPATH'
readelf -d ${OMNIMORPH_RUNTIME_ROOT}/install/mujoco_sim2sim/lib/mujoco_sim2sim/mujoco_sim_bridge | grep -E 'RPATH|RUNPATH'
```

Expected to include:

```text
/opt/ros/humble/lib
```

## 7. Config / Model Contract Mismatches

Not every startup error is an environment issue. Two common configuration mismatches:

### 7.1 Legs preset vs fullbody XML

- `jc01_legs_engineai_walk_sim2sim.yaml` is for the 12-DOF legs-only XML
- `jc01_fullbody_engineai_walk_sim2sim.yaml` is for the 28-joint fullbody XML

Do not mix them.

### 7.2 Fixed-base features require a base free joint

If `enable_fixed_base_zeroing`, `enable_fixed_base_hold_after_zeroing`, or `enable_release_before_running` is on, the XML must expose a free joint under the base body.

Current JC01 XML expectation:

- base body: `Body`
- free joint: unnamed `<freejoint/>` is acceptable

## 8. Recommended Launch Templates

### 8.1 JC01 legs-only headless

```bash
source /opt/ros/humble/setup.bash
source ${OMNIMORPH_RUNTIME_ROOT}/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/roslog
mkdir -p /tmp/roslog

ros2 run mujoco_sim2sim mujoco_sim_bridge --ros-args \
  --params-file ${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_sim2sim/mujoco_sim2sim/config/jc01_legs_engineai_walk_sim2sim.yaml \
  -p model_path:="/home/edify/Code/jingchu01/jingchu01_legs.xml" \
  -p startup_mode_id:=0 \
  -p enable_viewer:=false
```

### 8.2 JC01 fullbody headless

```bash
source /opt/ros/humble/setup.bash
source ${OMNIMORPH_RUNTIME_ROOT}/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/roslog
mkdir -p /tmp/roslog

ros2 run mujoco_sim2sim mujoco_sim_bridge --ros-args \
  --params-file ${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_sim2sim/mujoco_sim2sim/config/jc01_fullbody_engineai_walk_sim2sim.yaml \
  -p model_path:="/home/edify/Code/jingchu01/JC01-7DOF-URDF/JC01-URDF-18所/jingchu01_fullbody_standard.xml" \
  -p startup_mode_id:=0 \
  -p enable_viewer:=false
```

## 9. Quick Triage Checklist

When sim2sim fails at startup, check in this order:

1. Did you source ROS and workspace in a clean shell?
2. Is Conda inactive?
3. Is `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` set in all terminals?
4. Is `ROS_LOG_DIR` writable?
5. Does `ldd ... | grep onnxruntime` point to `/opt/ros/humble/lib/libonnxruntime.so.1`?
6. Are you using the correct YAML for the chosen XML?
7. Does the XML expose a base free joint when fixed-base safety is enabled?

# AMP Sim2Sim Mode-0 Tick Call Flow

This document traces the current code path for the AMP full-body sim2sim case
starting from:

```bash
ros2 run mujoco_sim2sim mujoco_sim_bridge --ros-args \
  --params-file ${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_sim2sim/mujoco_sim2sim/config/jc01_amp_full_body_sim2sim.yaml \
  -p rl_cfg_path:=${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_rl_controller/rl_master/config/rl_cfg_amp_mjlab_jc01_full_body.yaml \
  -p startup_mode_id:=0 \
  -p enable_viewer:=false \
  -p enable_python_viewer_stream:=true \
  -p enable_python_viewer_inspector:=true \
  -p fixed_base_height:=0.872
```

Note:

1. The actual executable name in the current repo is `mujoco_sim_bridge`.
2. The CLI override `-p fixed_base_height:=0.872` replaces the YAML file's
   `fixed_base_height: 0.92`.
3. `startup_mode_id:=0` resolves to `amp_mjlab_jc01_full_body` from
   [rl_cfg_amp_mjlab_jc01_full_body.yaml](../../../omnimorph_rl_controller/rl_master/config/rl_cfg_amp_mjlab_jc01_full_body.yaml).

## Effective runtime assumptions

For the current AMP config/profile, the relevant settings are:

1. `startup_mode_id = 0`
2. `startup_completion_action = hold`
3. `zeroing_duration_s = 2.0`
4. `enable_fixed_base_zeroing = true`
5. `enable_fixed_base_hold_after_zeroing = true`
6. `enable_release_before_running = true`
7. `post_release_settle_ticks = 0`
8. `sim_sync_running_start_to_reference = false`
9. `control_hz = 500`, `sim_dt = 0.001`, so `substeps_per_control = 2`
10. `RL_control_f = 50`
11. `prefill_observation_history_on_running_start = true`
12. `enable_time_step_input = false`, `time_step_start = 0`

## Important reality check

Your sentence "执行一个 tick 后就已经回到零位了，然后立马 publish running 信号" needs one correction:

1. In the current code, the very first `RL_controller::step()` call always
   enters `DeployLifecycleState::kZeroing`.
2. That same call cannot also finish startup zeroing, because
   `DeployStateMachine::startZeroing()` sets `zeroing_start_time_s_ = now_s`
   immediately before the zeroing branch computes:

   ```cpp
   alpha = (now_s - zeroing_start_time_s_) / duration
   ```

   so on that first `update()` call `alpha == 0`.
3. Therefore, after the first bridge tick, the robot may already be physically
   near the zero pose, but the lifecycle state is still `ZEROING`, not `HOLD`.
4. If you publish `running` immediately after that first tick, the next tick
   still will not execute the AMP policy. The bridge rewrites that start request
   to `STOP_POLICY` while zeroing is still active.

Because of that, this document gives two flows:

1. The strict real flow for "first tick, then immediate running publish".
2. The first later tick that actually enters `RUNNING` and executes AMP policy.

## 1. Process startup call order

From process entry to bridge-ready state, the call order is:

1. `main()` in [main.cpp](../src/main.cpp)
2. `mujoco_sim2sim::createMujocoSimBridgeNode()` in [mujoco_sim_bridge_core.cpp](../src/mujoco_sim_bridge_core.cpp)
3. `MujocoSimBridge::MujocoSimBridge()` in [mujoco_sim_bridge_core.cpp](../src/mujoco_sim_bridge_core.cpp)
4. `rclcpp::Node("mujoco_sim_bridge")`
5. `declare_parameter("rl_cfg_path", ...)`
6. `trimCopy(...)`
7. `rl_master::ModeProfileRegistry::loadFromYaml(...)`
8. `ModeProfileRegistry::loadInternal(...)`
9. `loadDeployModeProfilesFromYAML(...)`
10. `loadRobotGlobalJointOrderFromYAML(...)`
11. `loadJointGroupsFromYAML(...)`
12. `MujocoSimBridge::loadParameters()`
13. `MujocoSimBridge::loadModel()`
14. `mj_loadXML(...)` or `mj_loadModel(...)`
15. `mj_makeData(...)`
16. `mj_forward(model_, data_)`
17. `MujocoSimBridge::resolveModelMappings()`
18. `mj_name2id(...)` for each controlled joint / actuator / base body / free joint
19. `controller_runtime_.setModeProfileRegistry(mode_registry_)`
20. `IntegratedControllerRuntime::initialize(startup_mode_id_)`
21. `RL_controller::create(mode_registry_)`
22. `RL_controller::RL_controller_Init(startup_mode_id)`
23. `RL_controller::initModeProfiles()`
24. `RL_controller::loadModeProfileSpecsFromYaml()`
25. `mode_registry_->specs()`
26. `mode_registry_->cfgForSection(...)`
27. `mode_registry_->layoutForSection(...)`
28. `ObservationManifest::loadFromYAML(...)`
29. `RL_controller::initPolicyGroup(...)`
30. `OnnxPolicyAdapter::init()`
31. `OnnxPolicyRunner::init()`
32. `Ort::Session(...)`
33. `OnnxPolicyRunner::reset()`
34. `RL_controller::initReferenceMotionProvider(...)`
35. `robot->initialize_buffers(...)`
36. `RL_controller::refreshPolicyMode(initial_mode_id, false)`
37. `RL_controller::syncActiveProfileToRobotState()`
38. `RL_controller::handlePolicySwitch()`
39. `RL_controller::resetPolicyScheduler()`
40. `MujocoSimBridge::initializeState()`
41. `MujocoSimBridge::resolvePerJointControlConfig(controller_runtime_.activeModeId())`
42. `MujocoSimBridge::refreshPositionActuatorTuning(false)`
43. `MujocoSimBridge::captureBaseLockPoseFromModel()`
44. `MujocoSimBridge::activateDynamicBaseLock(BaseLockReason::kStartupZeroing, enable_prepose_snap_)`
45. `MujocoSimBridge::initRuntimeRecorder()`
46. `mode_command_cache_.store(kCtrlWordSetModeBase + startup_mode_id_)`
47. `MujocoSimBridge::initializeViewer()`
48. `MujocoSimBridge::setupRosInterfaces()`
49. `create_wall_timer(..., controlLoopTick)`
50. `MujocoSimBridge::startInputExecutor()`
51. `create_subscription(... teleopCallback)`
52. `create_subscription(... modeControlCallback)`
53. `input_executor_->spin()` in the IO thread
54. `MujocoSimBridge::startStateTelemetry()`
55. `MujocoSimBridge::startViewerTelemetry()`
56. `rclcpp::spin(node)`

At this point:

1. The controller active mode is already mode 0.
2. The bridge-side `mode_command_cache_` is initialized to `2000 + 0 = 2000`.
3. Dynamic base lock is already active because `enable_fixed_base_zeroing=true`.

## 2. Tick A: the first bridge control tick after startup

This is the exact function order inside the first `controlLoopTick()`:

1. `MujocoSimBridge::controlLoopTick()`
2. `mode_command_cache_.load()` returns `2000`
3. `MujocoSimBridge::prepareModeControlWordForTick(2000)`
4. `DeployStateMachine::decodeControlWord(2000, startup_mode_id_)`
5. `prepareModeControlWordForTick()` returns `2000`
   Reason: `controller_state_initialized_ == false`, so no lifecycle gating is applied yet.
6. `MujocoSimBridge::buildRobotState()`
7. `controller_runtime_.runtimeCfg()`
8. `quatXyzwToRpy(...)`
9. `MujocoSimBridge::latestTeleopCommand()`
10. `IntegratedControllerRuntime::step(state, teleop_command, 2000)`
11. `RL_controller::step(state, command, 2000, phase_t)`
12. `RL_controller::updateStateFromIO(state)`
13. `RL_controller::updateCommandFromIO(command)`
14. `deploy_state_machine_initialized_ == false`, so:
15. `RL_controller::handlePolicySwitch()`
16. `deploy_state_machine_.configure(activePolicyCfg())`
17. `RL_controller::activeZeroPose()`
18. `deploy_state_machine_.initialize(robot->joint_q, activeZeroPose(), active_mode_id_)`
19. `rl_master::monotonicTimeSec()`
20. `RL_controller::sanitizeRuntimeModeCommand(2000)`
21. `deploy_state_machine_.update(2000, now_s, robot->joint_q, robot->joint_dq)`
22. `DeployStateMachine::decodeControlWord(2000, pending_locomotion_mode_)`
23. `DeployStateMachine::startZeroing(now_s, current_q, DeployLifecycleState::kHold)`
24. Zeroing branch executes:
25. `alpha = 0`
26. `output.enable_policy = false`
27. `output.enable_command_stream = true`
28. `RL_controller::refreshPolicyMode(deploy_output.locomotion_mode, false)`
29. `RL_controller::handlePolicySwitch()`
   This second `handlePolicySwitch()` is effectively a no-op here because the
   active mode is still 0 and `last_active_mode_id_` is already 0.
30. `phase_origin_t_` is initialized
31. `last_deploy_state_` becomes `ZEROING`
32. Command-stream branch executes:
33. `robot->joint_target_q.assign(robot->default_angle.begin(), robot->default_angle.end())`
34. copy `deploy_output.target_q` into `robot->joint_target_q`
35. `RL_controller::get_joint_target_torque(robot->joint_target_q)`
36. `robot->open_rl = kOpenRlCommandStream`
37. build `rl_master::RobotCommandData out_cmd`
38. populate `latest_log_snapshot_`
39. return to `IntegratedControllerRuntime::step(...)`
40. return to `MujocoSimBridge::controlLoopTick()`
41. `controller_runtime_.controller().latestLogSnapshot()`
42. `MujocoSimBridge::emitDerivedRuntimeEvents(controller_snapshot)`
43. `rl_master::resolveCommandRuntimeMode(true, command.open_rl)`
44. `controller_state == ZEROING`, so the bridge keeps/enforces startup base lock
45. `MujocoSimBridge::maybeApplyRunningStartReferenceSync(controller_snapshot)`
46. returns immediately because `sim_sync_running_start_to_reference == false`
47. `MujocoSimBridge::enforceBaseLock()`
48. `MujocoSimBridge::updateControlInput(command, control_active, now)`
49. `controller_runtime_.runtimeCfg()`
50. `MujocoSimBridge::resolvePerJointControlConfig(controller_runtime_.activeModeId())`
51. `MujocoSimBridge::refreshPositionActuatorTuning(control_active)`
52. `rl_master::resolveCommandRuntimeMode(true, command.open_rl)`
53. For each joint, command-stream path runs with non-policy semantics
54. For the current AMP XML path, the backend is torque-actuated, so hold/stream
   PD torque is written into `data_->ctrl[actuator_id]`
55. physics step loop:
56. `MujocoSimBridge::enforceBaseLock()`
57. `mj_step(model_, data_)`
58. `MujocoSimBridge::enforceBaseLock()`
59. repeat substep loop twice because `substeps_per_control_ == 2`
60. `mj_forward(model_, data_)`
61. `MujocoSimBridge::buildRobotState()` for `post_state`
62. `MujocoSimBridge::updateMirroredState(post_state)`
63. `MujocoSimBridge::emitBaseImuSourceSample(post_state, ...)`
64. `MujocoSimBridge::logLoopData(...)`
65. `MujocoSimBridge::updateViewerFrameMirror()`
66. `MujocoSimBridge::updateViewerInspectorMirror(...)`
67. `MujocoSimBridge::renderViewerFrame()`
68. save:
69. `last_controller_mode_id_ = 0`
70. `last_controller_deploy_state_ = ZEROING`
71. `controller_state_initialized_ = true`

Result of Tick A:

1. The robot may already be physically very close to zero pose.
2. The lifecycle is still `ZEROING`.
3. No AMP policy inference has run yet.

## 3. Immediate running publish after Tick A

If you immediately publish a running signal after Tick A, the call order is:

1. ROS2 delivers `/omnimorph/rl/mode_control` to the IO executor thread
2. `MujocoSimBridge::modeControlCallback(msg)`
3. `DeployStateMachine::isValidControlWord(msg->data)`
4. `mode_command_cache_.store(msg->data)`

For mode 0, both of the following are accepted:

1. `10`
   Meaning: generic `start policy` while keeping the current pending locomotion mode.
2. `1000`
   Meaning: `start mode 0`.

Under the current startup case, both end up being blocked on the next tick
because the controller is still in `ZEROING`.

## 4. Tick B: the next tick immediately after that running publish

This is the strict real path for your exact scenario.

1. `MujocoSimBridge::controlLoopTick()`
2. `mode_command_cache_.load()` returns `10` or `1000`
3. `MujocoSimBridge::prepareModeControlWordForTick(raw_start)`
4. `DeployStateMachine::decodeControlWord(raw_start, last_controller_mode_id_)`
5. `controller_state_initialized_ == true`
6. `current_state_is_zeroing == true`
7. `dynamic_base_lock_active_ == true`
8. branch hits:

   ```cpp
   if (decoded.request_start &&
       dynamic_base_lock_active_ &&
       current_state_is_zeroing)
   {
       return rl_master::kCtrlWordStopPolicy;
   }
   ```

9. so `effective_mode_control_word = 11`
10. `MujocoSimBridge::buildRobotState()`
11. `MujocoSimBridge::latestTeleopCommand()`
12. `IntegratedControllerRuntime::step(state, teleop_command, 11)`
13. `RL_controller::step(...)`
14. `RL_controller::updateStateFromIO(...)`
15. `RL_controller::updateCommandFromIO(...)`
16. `RL_controller::sanitizeRuntimeModeCommand(11)`
17. `deploy_state_machine_.update(11, now_s, robot->joint_q, robot->joint_dq)`
18. `DeployStateMachine::decodeControlWord(11, pending_locomotion_mode_)`
19. because state is already `ZEROING`, this only changes `post_zeroing_state_`
   toward `HOLD`
20. zeroing branch executes again
21. unless `alpha >= 1.0` and position/velocity tolerances are satisfied, the
   output is still:
22. `enable_policy = false`
23. `enable_command_stream = true`
24. bridge returns to command-stream control again
25. `MujocoSimBridge::updateControlInput(...)`
26. `mj_step(...)`
27. `mj_forward(...)`
28. logging / telemetry / mirrored-state updates run as usual

Result of Tick B:

1. This tick still does not run the AMP policy.
2. It is still a zeroing/command-stream tick, not a running/policy tick.

## 5. Tick C: the first later tick that really enters RUNNING and executes AMP policy

This is the first tick that actually matters for AMP inference.

Preconditions:

1. Zeroing has already completed on a previous tick.
2. `last_controller_deploy_state_ == HOLD`
3. `last_controller_mode_id_ == 0`
4. `last_completed_zeroing_mode_id_ == 0`
5. `enable_fixed_base_hold_after_zeroing == true`, so dynamic base lock reason
   is `kPreRunHold`
6. The running signal is present in `mode_command_cache_`
7. `post_release_settle_ticks == 0`

Call order:

1. `MujocoSimBridge::controlLoopTick()`
2. `mode_command_cache_.load()` returns `10` or `1000`
3. `MujocoSimBridge::prepareModeControlWordForTick(raw_start)`
4. `DeployStateMachine::decodeControlWord(raw_start, last_controller_mode_id_)`
5. `current_state_is_hold == true`
6. `active_mode_needs_zeroing == false`
7. `enable_release_before_running_ == true`
8. `dynamic_base_lock_reason_ == BaseLockReason::kPreRunHold`
9. `MujocoSimBridge::deactivateDynamicBaseLock("pre-running release")`
10. `release_settle_ticks_remaining_ = 0`
11. because `post_release_settle_ticks == 0`, `prepareModeControlWordForTick()`
    returns the raw start request instead of converting it to stop
12. `MujocoSimBridge::buildRobotState()`
13. `MujocoSimBridge::latestTeleopCommand()`
14. `IntegratedControllerRuntime::step(state, teleop_command, raw_start)`
15. `RL_controller::step(...)`
16. `RL_controller::updateStateFromIO(...)`
17. `RL_controller::updateCommandFromIO(...)`
18. `RL_controller::sanitizeRuntimeModeCommand(raw_start)`
19. `deploy_state_machine_.update(raw_start, now_s, robot->joint_q, robot->joint_dq)`
20. `DeployStateMachine::decodeControlWord(raw_start, pending_locomotion_mode_)`
21. hold-state start branch runs:
22. `active_locomotion_mode_ = 0`
23. `state_ = RUNNING`
24. output becomes:
25. `enable_policy = true`
26. `enable_command_stream = true`
27. `RL_controller::refreshPolicyMode(0, false)`
28. `RL_controller::handlePolicySwitch()`
   This is effectively a no-op if mode 0 did not change.
29. `entered_running = true`
30. `resetPolicyScheduler()`
31. `local_phase_t = 0`
32. policy branch begins:
33. `policy_hz = activePolicyCfg().RL_control_f = 50`
34. `policy_schedule_initialized_ == false`
35. `should_run_policy = true`
36. `RL_controller::warmStartPolicyState(local_phase_t)`
   For the current AMP profile, `policy_startup_warmup_steps` is not configured,
   so this returns immediately.
37. `latest_policy_extra_outputs_.empty() == true`
38. `RL_controller::prefetchCurrentPolicyReferenceOutputs()`
   For AMP mode 0 this usually returns quickly unless reference-motion extra
   outputs are configured.
39. `RL_controller::get_robot_observation(local_phase_t)`
40. `RL_controller::buildObservationFeatureContext(active_cfg, local_phase_t)`
41. `external_observation_provider_.collect(...)`
42. `activeObservationBuilder().build(...)`
43. because `prefill_observation_history_on_running_start == true`:
44. `obs_deque.clear()`
45. push the same `current_obs` frame `obs_stack_N` times
46. `RL_controller::run_policy()`
47. flatten `obs_deque` into `stacked_obs_buffer_`
48. `RL_controller::runPolicyGroup(...)`
49. `activePolicyGroup().strategy->execute(...)`
50. `OnnxPolicyAdapter::infer(request)`
51. `OnnxPolicyRunner::forward(...)`
52. `OnnxPolicyRunner::runSelectedOutputs(...)`
53. for each input binding:
54. `OnnxPolicyRunner::resolveInputData(...)`
55. `Ort::Value::CreateTensor<float>(...)`
56. `session_->Run(...)`
57. flatten action tensor and extra-output tensors
58. `enable_time_step_input == false`, so `time_step_` is not consumed and is
   not incremented by this policy call
59. return action to `run_policy()`
60. `run_policy()` applies `clip_actions`
61. `run_policy()` applies `action_filter`
62. `RL_controller::get_joint_target_q(policy_action)`
63. `RL_controller::get_joint_target_torque(robot->joint_target_q)`
64. `robot->open_rl = kOpenRlPolicyEnabled`
65. build `RobotCommandData out_cmd`
66. populate `latest_log_snapshot_`, including:
67. `policy_ran_this_tick = true`
68. `policy_step_index = 1`
69. `observation`
70. `policy_action`
71. return to `IntegratedControllerRuntime::step(...)`
72. return to `MujocoSimBridge::controlLoopTick()`
73. `controller_runtime_.controller().latestLogSnapshot()`
74. `MujocoSimBridge::emitDerivedRuntimeEvents(controller_snapshot)`
75. `rl_master::resolveCommandRuntimeMode(true, command.open_rl)`
76. transition bookkeeping sees `HOLD -> RUNNING`
77. `running_start_reference_sync_pending_ = sim_sync_running_start_to_reference_`
78. because `sim_sync_running_start_to_reference == false`, that value becomes `false`
79. `MujocoSimBridge::maybeApplyRunningStartReferenceSync(controller_snapshot)`
80. returns immediately without modifying `qpos/qvel`
81. `MujocoSimBridge::enforceBaseLock()`
   Now it is effectively inactive because pre-running release already removed
   the dynamic base lock.
82. `MujocoSimBridge::updateControlInput(command, control_active=true, now)`
83. `controller_runtime_.runtimeCfg()`
84. `MujocoSimBridge::resolvePerJointControlConfig(controller_runtime_.activeModeId())`
85. `MujocoSimBridge::refreshPositionActuatorTuning(true)`
86. `rl_master::resolveCommandRuntimeMode(true, command.open_rl)`
87. policy branch per joint begins
88. for each policy-controlled joint:
89. current profile uses `installed_joint_run_modes: cst`
90. current AMP sim2sim XML path resolves torque actuators
91. therefore `updateControlInput()` uses:
92. `q_des = commandQAt(i)`
93. `tau_cmd = commandTauAt(i)`
94. `data_->ctrl[actuator_id] = tau_cmd`
95. `applied_tau_[i] = tau_cmd`
96. for non-policy joints, hold-target / hold-torque path is used
97. physics step loop:
98. `mj_step(model_, data_)` twice
99. `mj_forward(model_, data_)`
100. `MujocoSimBridge::buildRobotState()` for `post_state`
101. `MujocoSimBridge::updateMirroredState(post_state)`
102. `MujocoSimBridge::emitBaseImuSourceSample(post_state, ...)`
103. `MujocoSimBridge::logLoopData(...)`
104. `MujocoSimBridge::updateViewerFrameMirror()`
105. `MujocoSimBridge::updateViewerInspectorMirror(...)`
106. `MujocoSimBridge::renderViewerFrame()`
107. save:
108. `last_controller_mode_id_ = 0`
109. `last_controller_deploy_state_ = RUNNING`
110. `controller_state_initialized_ = true`

Result of Tick C:

1. This is the first tick that truly performs AMP policy inference.
2. Observation is built from the pre-action robot state of this tick.
3. The produced action is converted into `joint_target_q`, then into
   `joint_target_tau`.
4. Under the current AMP mode-0 configuration, torque-actuator + `cst` runtime
   mode means the bridge writes the controller-computed torque directly into
   `data_->ctrl`.

## 6. Short answer

If you literally do:

1. start bridge
2. let exactly one bridge tick happen
3. immediately publish running
4. let one more bridge tick happen

then the second tick still does not run AMP policy. It is still inside the
`ZEROING` gate.

The first tick that really executes AMP policy is the later tick where all of
these are simultaneously true:

1. `DeployStateMachine` has already completed zeroing and reached `HOLD`
2. pre-run base release has happened
3. the start control word is still present
4. `RL_controller::step()` enters the `deploy_output.enable_policy` branch

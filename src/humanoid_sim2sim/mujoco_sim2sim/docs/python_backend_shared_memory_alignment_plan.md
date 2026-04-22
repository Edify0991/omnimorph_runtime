# Python Backend Shared-Memory Alignment Plan

## Current Situation

### Fused C++ sim2sim

```text
mujoco_sim_bridge (C++)
  |- MuJoCo physics
  |- IntegratedControllerRuntime
  |- ONNX inference
```

### Historical Python interactive sim2sim

```text
RL_controller (legacy standalone process) <-> DDS <-> python_interactive backend
```

That historical path has now been removed. The rest of this document is kept only as design history for why the repo moved toward the fused backend plus viewer-frontend architecture.

## Why It Cannot Be "Just Switched" To SHM Immediately

The current Python backend does all of these inside Python:

- owns MuJoCo model/data
- runs the friendly Python viewer
- receives policy commands over DDS
- publishes robot state over DDS

To align it with the real runtime using shared memory, we need to decide who owns physics and who owns inference.

## Two Practical Directions

### Option A: Keep physics in Python, replace DDS with SHM

Topology:

```text
RL_controller legacy process <-> SHM <-> python_interactive backend
```

Pros:

- preserves current Python viewer and physics ownership
- smaller change than full viewer split

Cons:

- still split runtime
- still not the same as real fused `RL_solver`
- requires defining a stable SHM protocol shared by C++ and Python

### Option B: Keep physics + inference in fused C++, Python is viewer-only

Topology:

```text
mujoco_sim_bridge (C++ fused runtime) <-> SHM <-> python GUI frontend
```

Pros:

- closest to real deployment logic
- one control owner
- Python keeps the nicer GUI

Cons:

- larger refactor
- Python frontend must stop owning MuJoCo stepping and become a visualization / inspection client

## Recommendation

If the goal is "as close to real deploy logic as possible", Option B is the right direction.

That means the next serious refactor would be:

1. keep fused C++ backend as the single control owner
2. add a sim2sim SHM block for state snapshots and optional UI commands
3. convert Python interactive backend into a GUI client instead of the control owner

## What Was Done In That Round

- the old Python split backend was still present at that time
- the fused C++ backend was already the standard path
- standalone controller glue had been isolated under `rl_master/legacy/`

So the repo is now in a cleaner state for doing Option B next, if you want me to continue.

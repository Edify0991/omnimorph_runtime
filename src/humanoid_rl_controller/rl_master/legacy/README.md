# Legacy Standalone Controller Path

This directory contains the old standalone-controller glue code:

```text
RL_controller (process A) <-> DDS <-> simulator/solver (process B)
```

Files here are kept only for:

- Python interactive MuJoCo backend compatibility
- isolated debugging of the pre-fused runtime topology

They are not used by the standard fused deploy path anymore.

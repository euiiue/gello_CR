# Phase 2.1 - GELLO device boundary

## Purpose

Introduce the production-facing GELLO master adapter without changing the
currently validated legacy executable.

This is intentionally a two-step migration:

1. add and test the new adapter;
2. only after offline tests pass, bridge the legacy runtime to this adapter.

The old `teleop_runtime.GelloController` remains untouched in this commit so the
real-robot baseline stays directly runnable.

## GELLO data semantics

The nested legacy `DynamixelRobot` returns seven values:

- channels 1..6: arm joints in radians;
- channel 7: gripper position normalized to `[0, 1]`.

V2 therefore publishes:

```text
MasterSnapshot
  joints.positions_rad = J1..J6
  auxiliary["gripper"] = normalized channel 7
```

Do not represent channel 7 as a radian robot joint.

## Path handling

`GelloDevice` contains no `/home/ace/...` path.  `software_root` is supplied by
configuration as a transitional compatibility option.  A later packaging phase
will install/vendor the GELLO dependency normally and remove this transitional
path injection.

## Acceptance gate

This phase passes when:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src \
  <python3.12> -m pytest tests/v2 -q
```

passes and the legacy production files have not changed.

No real robot should move during this phase.

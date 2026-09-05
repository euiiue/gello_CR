# Phase 2.4a - Full CR3A lifecycle device

This phase adds `Cr3aDevice`, which owns the NRC connection lifecycle while
delegating the proven controller state machine and motion commands to
`NrcRobotSession`.

## Layering

```text
Cr3aDevice
  ├─ NRC SDK load/injection
  ├─ connect 6001
  ├─ connect 7000
  ├─ partial-connect rollback
  ├─ callback registration / strong reference
  ├─ SI-unit RobotSnapshot boundary
  ├─ rad → degree command conversion
  ├─ safe stop on close
  └─ disconnect 6001 / 7000
          ↓
NrcRobotSession
  ├─ serialized NRC calls
  ├─ servo state machine
  ├─ ServoJ / MoveJ / MoveL
  ├─ FK / IK
  └─ warning code 4098 state
```

## Unit contract

The V2 application must not inherit NRC's mixed native units.

At the `Cr3aDevice` boundary:

```text
NRC joints degrees        → RobotSnapshot positions_rad
NRC TCP xyz millimetres   → TcpPose xyz_m
NRC TCP ABC radians       → TcpPose rpy_rad

V2 6D joint target radians
        ↓
degrees + external-axis 0
        ↓
NRC native 7D command
```

The `.session` property remains temporarily available so the legacy
`TeleopEngine` can continue to use native NRC semantics during staged migration.

## Safety

`connect()` never powers on the robot and never starts ServoJ.

`close()` makes a best-effort `stop_motion()` before closing both NRC
connections.

Callback registration failure remains non-fatal, matching the current UI
behavior.

`TEST_INEXBOT.py` is not modified in Phase 2.4a.

No real robot motion is required for this phase.

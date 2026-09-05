# GELLO CR Teleoperation V2 Architecture

## Scope

V2 is a staged replacement for the monolithic operator application. During the
migration the current `TEST_INEXBOT.py`, `teleop_runtime.py` and
`lerobot_recorder.py` remain the executable baseline.

Validated production path:

```text
GELLO J1..J6
    -> relative joint mapping
    -> CR3A J1..J6
    -> NRC :7000 ServoJ

GELLO J7
    -> O6 open/close
```

WEIXUE and Inverse3 remain future `MasterDevice` adapters. Future slave robots
implement `RobotDevice` adapters. UI and workflow do not fork per device.

## Frozen data contract

UI/architecture migration must not change:

- observation.state: 18D
- action: 12D
- RGB: base, wrist, base ROI
- 224x224
- 20 Hz

Any change is a separate dataset schema migration.

## Dependency direction

```text
PySide6 UI
   |
Application service + workflow state machine
   |
Control / Recording / Diagnostics
   |
Hardware contracts
   |
GELLO / WEIXUE / Inverse3 / NRC robot / O6 / RealSense
```

UI must never call NRC, Dynamixel, LinkerHand or RealSense SDK objects directly.

## Operator workflow

```text
OFFLINE
 -> CONNECTED
 -> ROBOT_ENABLED
 -> TELEOP_RUNNING
 -> RECORDING
```

Episode keys:

- F8: start
- F9: success
- Shift+F9: failure
- F10: discard

Saving/discarding does not auto-start the next episode. Fault/E-stop reset never
auto-resumes ServoJ.

## Main UI

Only daily collection functions belong on the main page:

- device health
- wrist/base/ROI video
- connect all
- robot clear-error/power-on
- start/stop teleop
- episode start/success/failure/discard
- software E-stop
- FPS/tracking/time
- event log

Calibration, serial paths, offsets, ServoJ tuning and diagnostics belong under
Advanced/Diagnostics. Historical MoveJ/MoveL/manual slider/test pages are not
migrated.

## Migration order

1. Freeze baseline.
2. Add V2 contracts/state machine/data-contract tests.
3. Extract GELLO adapter.
4. Extract O6 adapter.
5. Extract NRC robot adapter.
6. Extract RealSense adapter.
7. Extract mapping/safety control core.
8. Move recorder without schema changes.
9. Add application service.
10. Rebuild PySide6 UI.
11. Pass hardware gates.
12. Delete legacy UI/runtime code.

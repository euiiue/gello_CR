# Phase 2.3a - NRC control-session boundary

This phase extracts the proven NRC command/servo logic into:

```text
src/gello_cr/devices/nrc_robot.py
```

The new class is deliberately named `NrcRobotSession`, not `Cr3aDevice`.

## Why "session" first?

The legacy `NrcRobotAdapter` receives:

```text
api
command_fd  # 6001
servo_fd    # 7000
robot_num
```

The sockets have already been created elsewhere. Therefore this class is not
yet a complete `RobotDevice` lifecycle owner.

The target layering is:

```text
Cr3aDevice                    (later)
  SDK load + connect/close
        ↓
NrcRobotSession               (this phase)
  serialized NRC access
  servo state machine
  ServoJ / MoveJ / MoveL
  FK / IK
  stop handling
```

This separation keeps the validated connection path unchanged while extracting
the motion-control subsystem.

## Preserved safety behavior

- both 6001 and 7000 connection-status checks;
- alarm clear → power-off stabilization → ready → run mode → power-on;
- NRC error-code descriptions;
- stable non-moving wait before mode transitions;
- ServoJ enable/open/stop sequence;
- seven-value NRC command shape;
- MoveJ rate limiting;
- controller warning code 4098 handling;
- stop-without-power-off when available;
- existing near-zero ±360° feedback cleanup.

`teleop_runtime.py` is not modified in Phase 2.3a.

No robot motion is required for this phase.

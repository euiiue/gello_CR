# Phase 3.1b - Bridge TeleopEngine joint mode to RelativeJointMapper

Phase 3.1a characterized the existing GELLO J1-J6 relative mapping as a pure
control primitive.

Phase 3.1b replaces only the duplicated mapping math inside
`TeleopEngine._gello_follow_loop()`.

## Before

```text
leader
  ↓
joint_scale * atan2(sin(delta), cos(delta))
  ↓
zero locked joints
  ↓
slave startup pose + rad2deg(relative)
```

and the leader-speed safety path repeated the wrapped-delta formula.

## After

```text
RelativeJointMapper
  ├─ target_deg(...)
  └─ leader_delta_rad(...)
```

## Intentionally unchanged

The following remain in `TeleopEngine` for later safety/control phases:

- GELLO feedback freshness checks;
- O6 feedback/fault checks;
- startup settle behavior;
- CR3A command-step limit;
- leader-speed limit and consecutive violation count;
- CR3A tracking-error limit;
- MoveJ short-segment behavior;
- ServoJ/MoveJ dispatch;
- NRC busy handling;
- telemetry;
- J7 -> O6 behavior;
- stop/fault handling.

This keeps Phase 3.1b behavior-equivalent and low-risk.

No hardware motion is required for this phase.

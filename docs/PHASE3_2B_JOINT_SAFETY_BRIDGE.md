# Phase 3.2b - Bridge TeleopEngine joint safety to V2 primitives

Phase 3.2a characterized the validated GELLO joint-mode safety math in
`src/gello_cr/control/joint_safety.py`.

Phase 3.2b replaces only the matching safety calculations inside
`TeleopEngine._gello_follow_loop()`.

## Replaced

```text
NumPy command-step math
    -> max_command_step_violation()

NumPy speed_counts state
    -> LeaderSpeedViolationCounter

NumPy tracking-error math
    -> max_tracking_error_violation()
```

The startup-settle reset maps from `speed_counts.fill(0)` to
`speed_guard.reset()`.

## Preserved

The runtime still owns and live-refreshes all four safety settings. GELLO/O6
freshness checks, RelativeJointMapper, the low-latency MoveJ full desired-error
guard, MoveJ short-segment policy, duplicate deadband, NRC busy handling,
ServoJ/MoveJ dispatch, telemetry, J7 -> O6, and stop/fault behavior are not
changed.

No hardware movement is required for this phase.

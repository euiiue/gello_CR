# Phase 3.2a - Pure joint-safety primitives

This phase extracts the safety math from the validated GELLO joint loop without
modifying `teleop_runtime.py`.

## Current legacy behavior being characterized

### Command-step limit

```text
abs(deg2rad(target - last_target))
```

The largest joint is compared with:

```text
safety_max_command_step_rad
```

using strict `>`.

### Tracking-error limit

```text
abs(deg2rad(last_target - actual_feedback))
```

The largest joint is compared with:

```text
safety_max_tracking_error_rad
```

again using strict `>`.

### Leader-speed limit

The mapper already produces scaled absolute wrapped leader deltas.  The runtime
then computes:

```text
period = max(0.001, master_timestamp - previous_timestamp)
speed = leader_delta_rad / period
```

Each joint has an independent consecutive over-limit counter:

```text
speed > limit  -> count + 1
safe sample    -> count = 0
```

A stop is triggered when the largest count reaches:

```text
safety_speed_violation_cycles
```

## New module

```text
src/gello_cr/control/joint_safety.py
```

It contains:

- `command_step_rad`
- `tracking_error_rad`
- `max_command_step_violation`
- `max_tracking_error_violation`
- `LeaderSpeedViolationCounter`

No hardware, Qt, threads, timing loop, or SDK calls are introduced.

## Not changed in Phase 3.2a

`teleop_runtime.py` remains byte-for-byte unchanged.

Phase 3.2b will bridge the legacy loop to these primitives while preserving
error messages, live-config refresh, ServoJ/MoveJ dispatch and stop behavior.

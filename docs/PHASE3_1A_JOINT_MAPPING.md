# Phase 3.1a - Pure GELLO → CR3A relative joint mapper

This phase extracts the mathematically pure part of the validated GELLO joint teleoperation path.

Current runtime equation:

```text
relative = joint_scale * atan2(sin(leader - leader_origin), cos(leader - leader_origin))
locked joints: relative[joint] = 0
target_deg = slave_origin_deg + rad2deg(relative)
```

The leader-speed path uses the same wrapped angular difference with absolute value and the same joint scale. Locked joints are removed from that speed delta.

New V2 boundary:

```text
src/gello_cr/control/joint_mapping.py
RelativeJointMapper
  ├─ relative_rad()
  ├─ target_deg()
  └─ leader_delta_rad()
```

The mapper has no NRC/GELLO/O6 I/O, threads, timers, Qt, or mutable runtime state. It preserves the startup-pose-relative convention and shortest-path wrapping across ±π.

Phase 3.1a does not modify `teleop_runtime.py`. Phase 3.1b will bridge only the duplicated mapping expressions while leaving safety thresholds, ServoJ dispatch, tracking checks and O6 behavior unchanged.

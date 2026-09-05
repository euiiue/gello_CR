# Phase 2.1b - Legacy GELLO compatibility bridge

The legacy `teleop_runtime.GelloController` no longer owns direct
`DynamixelRobot` import, connection lifecycle, feedback thread/event/lock, or
raw seven-channel feedback validation. Those responsibilities now live in
`src/gello_cr/devices/gello.py`.

The old public `GelloController` and `GelloFeedback` surface is intentionally
kept temporarily so the existing Qt UI and teleoperation loops do not need to
change in the same commit.

New V2 code must use `MasterSnapshot.joints` and `auxiliary["gripper"]`, not the
legacy seven-value `joints_rad` compatibility shape.

No hardware movement is required for this phase.

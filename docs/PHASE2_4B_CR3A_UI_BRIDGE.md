# Phase 2.4b - Route legacy Qt NRC lifecycle through Cr3aDevice

This phase is the first direct change to `TEST_INEXBOT.py`.

`RobotCONNECT()` no longer calls NRC `connect_robot`, callback registration, or
disconnect rollback directly. It creates a `Cr3aDevice`, calls `connect()`, and
temporarily exposes `device.session` and the two fds as compatibility aliases
for the old `TeleopEngine` and historical diagnostics.

`closeEvent()` calls `Cr3aDevice.close()` instead of disconnecting NRC fds
itself.

Historical manual diagnostic methods still contain direct `aa.*` read/motion
calls; they are intentionally outside this migration step and will be moved to
diagnostics or removed after the new application service/UI exists.

`RobotCONNECT()` still does not power on the robot and does not start ServoJ.

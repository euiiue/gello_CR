# Phase 2.3b - Legacy NRC compatibility bridge

`teleop_runtime.NrcRobotAdapter` is now only the legacy class name for
`src/gello_cr/devices/nrc_robot.NrcRobotSession`.

The following implementation has moved out of `teleop_runtime.py`:

- serialized NRC access via the per-session lock;
- 6001/7000 connection-status checks;
- servo-state and run-mode state machines;
- clear-error / power-on / power-off sequencing;
- running-state and stable-stop checks;
- joint/TCP position reads;
- ServoJ open/send/stop;
- MoveJ / MoveL;
- FK / IK;
- controller warning code 4098 handling;
- stop-motion logic.

`NrcServoTransition` is also imported from the V2 NRC module rather than being
defined twice.

## Deliberately not migrated yet

This phase does **not** move:

- NRC SDK loading;
- 6001 socket creation/destruction;
- 7000 socket creation/destruction;
- NRC error/warning callback registration.

Those belong to the later full `Cr3aDevice` lifecycle adapter.

No real robot motion is required for this phase.

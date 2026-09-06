# Phase 6.6 - Explicit preparation workflow

Phase 6.6 adds auxiliary-device and camera preparation to the new PySide6
operator flow without expanding `WorkflowState`.

## New Commands

```text
PREPARE_DEVICES
START_CAMERAS
STOP_CAMERAS
```

All are submitted through the same asynchronous `CommandPort` and dispatched
through `ApplicationService`.

`PREPARE_DEVICES` is allowed only after CR3A connection (`CONNECTED`) or while
the robot is already enabled. It calls the existing
`TeleopEngine.connect_devices()` path, which connects the configured master
(GELLO in the validated path) and O6 but does not power or move CR3A.

`START_CAMERAS` is an explicit self-transition in non-safety states.

`STOP_CAMERAS` is a workflow self-transition even in FAULT/ESTOP so cameras can
be shut down safely, but the concrete handler refuses the command while a
LeRobot Episode is active or has pending buffered frames.

## Readiness is not WorkflowState

The main workflow remains:

```text
OFFLINE
CONNECTED
ROBOT_ENABLED
TELEOP_RUNNING
RECORDING
FAULT
ESTOP
```

Auxiliary readiness is represented separately by `OperatorReadiness`:

```text
master_connected
o6_connected
cameras_running
camera_frames_ready
camera_error
```

This prevents state explosion such as
`CONNECTED_CAMERAS_RUNNING_GELLO_READY`.

## Final UI gates

The new window applies two layers:

```text
Application workflow permission
AND
readiness permission
```

Important gates:

```text
POWER_ON
= CONNECTED + master/O6 ready

START_TELEOP
= ROBOT_ENABLED + master/O6 ready

START_EPISODE
= TELEOP_RUNNING + fresh camera frames
```

These are presentation gates only. Existing TeleopEngine and Recorder runtime
checks remain authoritative.

## Startup remains inert

Launching `python -m gello_cr.ui` still does not automatically:

- connect CR3A;
- connect GELLO/O6;
- start cameras;
- power on;
- start ServoJ;
- start an Episode.

The intended sequence is now visible in the new UI:

```text
1. Connect CR3A
2. Connect GELLO + O6
3. Power on CR3A
4. Start teleoperation

Cameras are started explicitly before recording.
```

Phase 6.7 is the first hardware-preparation gate: install/use PySide6, launch
the new UI, validate OFFLINE startup, then test connect-only operations before
any ServoJ motion.

# Phase 5.3 - Legacy Qt application bridge

This phase installs `ApplicationService + RuntimeCommandBindings` inside the
existing PyQt5 application without redesigning the UI.

Routed now:
- successful CR3A connection confirms CONNECTED;
- successful NRC power-on confirms ROBOT_ENABLED;
- start/stop follow use START_TELEOP / STOP_TELEOP;
- Episode start/save/discard use application commands;
- software emergency stop uses EMERGENCY_STOP.

The validated legacy CR3A connect/power operations still perform the actual
hardware work. Application state advances only after those operations succeed.

A faulted/E-stopped Episode may now be resolved with SAVE_FAILURE or DISCARD
while remaining in FAULT/ESTOP. SAVE_SUCCESS remains forbidden. Reset still
returns only to ROBOT_ENABLED after an interruption during TELEOP/RECORDING.

Still deferred:
- CR3A disconnect/power-off/reset migration;
- replay/preset/manual diagnostic commands;
- asynchronous TeleopEngine fault synchronization back into ApplicationService.

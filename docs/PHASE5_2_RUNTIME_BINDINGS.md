# Phase 5.2 - Runtime command bindings

Phase 5.2 connects the new `ApplicationService` to runtime operations that
already have stable APIs.

Direct bindings:

- START_TELEOP -> `TeleopEngine.start_follow()`
- STOP_TELEOP -> `TeleopEngine.stop_follow(reason)`
- START_EPISODE -> `LeRobotEpisodeRecorder.start_episode(...)`
- SAVE_SUCCESS / SAVE_FAILURE -> `save_episode(...)`
- DISCARD_EPISODE -> `discard_episode()`
- EMERGENCY_STOP -> `TeleopEngine.emergency_stop(reason)`
- REPORT_FAULT -> safe-stop through `TeleopEngine.emergency_stop(...)`

CR3A lifecycle is not falsely moved in this phase. The legacy Qt application
still owns creation/disconnection of the CR3A device and currently invokes NRC
power-on directly. Therefore these commands use explicit injected callbacks:

- CONNECT
- DISCONNECT
- POWER_ON
- POWER_OFF
- RESET_FAULT
- RESET_ESTOP

If a lifecycle callback is not configured, the command fails before the
ApplicationService state transition.

This is deliberate: a missing reset callback must not allow the application
state to leave ESTOP/FAULT while the underlying legacy runtime remains faulted.

Phase 5.3 will install these bindings inside the legacy Qt application and
route the first workflow buttons/shortcuts through `ApplicationService`.

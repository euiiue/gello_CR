# Phase 5.4 - Runtime fault synchronization

Phase 5.4 removes the remaining runtime/application fault-state gap.

The existing Qt refresh loop already polls TeleopEngine every 100 ms. It now
checks the authoritative runtime state:

`TeleopEngine.state == "fault"`

It does not infer workflow faults from arbitrary `error` log messages.

When a runtime fault is observed, Qt calls:

`ApplicationService.report_external_fault(...)`

This changes the application workflow state without invoking the REPORT_FAULT
handler again. TeleopEngine has already executed its runtime-side safe stop.

ESTOP has priority. If ApplicationService is already ESTOP, an observed runtime
fault does not replace ESTOP with FAULT.

There is no automatic recovery. If TeleopEngine later becomes idle,
ApplicationService remains FAULT/ESTOP until an explicit reset succeeds.
Recovery after TELEOP/RECORDING remains ROBOT_ENABLED, never TELEOP_RUNNING.

No robot motion parameters or hardware commands are added in this phase.

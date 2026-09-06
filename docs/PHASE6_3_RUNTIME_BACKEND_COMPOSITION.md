# Phase 6.3 - Runtime backend composition boundary

Phase 6.3 connects the new application/UI stack to existing runtime objects
without constructing or connecting hardware inside the PySide6 window.

## OperatorBackend

`OperatorBackend.compose(...)` accepts already-created:

- TeleopEngine-compatible runtime;
- LeRobotEpisodeRecorder-compatible recorder;
- RuntimeLifecycleCallbacks.

It creates:

```text
ApplicationService
    + RuntimeCommandBindings
    + RuntimeFaultSnapshotBridge
    + OperatorUiPresenter
    + AsyncApplicationCommandPort
```

Construction is side-effect free with respect to hardware:

- no CR3A connect;
- no GELLO/O6 connect;
- no RealSense start;
- no servo power-on;
- no teleoperation start;
- no Episode start.

Application state starts at `OFFLINE`.

## Why hardware construction is not in this phase

The legacy `_setup_roarm_teleop()` currently mixes configuration, runtime
object construction, camera caches, recorder sample-provider callbacks, logging
and Qt widgets. Copying that function into the new UI would recreate the
monolithic coupling Phase 1-5 removed.

Phase 6.4 will extract a concrete hardware/runtime factory from those remaining
legacy construction responsibilities.

## Runtime fault synchronization

The old Qt window used `_sync_application_runtime_state()` every 100 ms.
The new backend now carries that behavior independently through
`RuntimeFaultSnapshotBridge`:

```text
TeleopEngine state=fault
    -> ApplicationService.report_external_fault()
    -> FAULT

Application already ESTOP
    + runtime fault
    -> remains ESTOP
```

A later runtime return to idle never auto-resets Application state.

## Ownership

`OperatorBackend.close()` closes only command/presentation plumbing. It does
not implicitly power off, disconnect, finalize recorder data or shut down
hardware. Those actions must remain explicit workflow/lifecycle decisions.

No real robot movement is part of Phase 6.3 tests.

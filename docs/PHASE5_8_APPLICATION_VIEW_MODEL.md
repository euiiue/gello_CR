# Phase 5.8 - Application view model and Phase 5 completion

Phase 5.8 adds the presentation boundary that Phase 6 can reuse:

`src/gello_cr/app/view_model.py`

The view model is frozen/read-only and combines:

- Application workflow state and recovery state;
- Application error;
- Teleop runtime state/error;
- LeRobot session / Episode state;
- buffered/saved Episode counts;
- recorder quality `needs_review`;
- workflow button policy;
- a presentation headline and severity.

The builder performs no hardware I/O and dispatches no command.

## Legacy Qt bridge

`refresh_teleop_ui()` now builds one `ApplicationViewModel` from the three
already-existing snapshots:

```text
ApplicationService.snapshot()
TeleopEngine.snapshot()
LeRobotEpisodeRecorder.snapshot()
        ↓
build_application_view_model(...)
```

The workflow gates consume `view_model.policy` rather than recalculating the
policy from Qt-local variables.

The primary teleop status label consumes `view_model.headline`. Lower-level
device/runtime detail labels remain unchanged.

## Phase 5 complete

At this point the application layer provides:

- command dispatch;
- workflow state machine;
- runtime command bindings;
- external-fault synchronization;
- application events;
- thread-safe event buffering;
- UI workflow policy;
- lifecycle closure;
- read-only presentation view model.

Phase 6 can therefore build a new PySide6 UI against the application boundary
instead of directly against TeleopEngine, LeRobotEpisodeRecorder, NRC, GELLO,
O6 or camera SDK objects.

No hardware command, motion limit, recorder schema or sampling behavior changes
in Phase 5.8.

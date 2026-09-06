# Phase 5.1 - Application service foundation

Phase 5 starts the UI/application separation.

The repository previously had a workflow state machine but no `app/` package.
Phase 5.1 adds:

```text
src/gello_cr/app/
├── __init__.py
├── events.py
└── service.py
```

## Command direction

```text
Qt / future PySide6
      |
      | Command
      v
ApplicationService
      |
      | injected handler
      v
Teleop / Recorder / Robot / Hand / Camera
```

## Event direction

```text
ApplicationService
      |
      | AppEvent
      +------> UI
      +------> log
      +------> diagnostics
```

The application layer has no Qt/PySide import.

## State-transition ordering

For normal commands:

```text
validate -> handler succeeds -> apply state transition
```

This means a failed power-on, start-recording or save operation does not make
the UI claim that the operation succeeded.

For `REPORT_FAULT` and `EMERGENCY_STOP`:

```text
validate -> enter FAULT/ESTOP -> run best-effort handler
```

If the hardware stop handler itself fails, the application remains in the safe
FAULT/ESTOP state.

Reset handlers run before RESET transition.  Combined with the existing
WorkflowStateMachine recovery rule, resetting an ESTOP/FAULT that happened
during TELEOP/RECORDING returns only to ROBOT_ENABLED and never automatically
resumes ServoJ.

Phase 5.1 intentionally does not modify the legacy Qt UI yet.  Phase 5.2 will
bind existing TeleopEngine/Recorder operations to these application commands.

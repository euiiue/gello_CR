# Phase 6.2 - Asynchronous command dispatch and safety preemption

Phase 6.2 provides the production command-port execution model for the PySide6
UI.

## Two command lanes

`AsyncApplicationCommandPort` has two daemon workers:

```text
Normal lane:
CONNECT / POWER_ON / START_TELEOP / recorder commands / reset / ...
    -> FIFO single worker

Safety lane:
EMERGENCY_STOP / REPORT_FAULT
    -> independent worker
```

Submitting from the GUI is non-blocking. A software E-stop therefore does not
wait behind a slow dataset save or lifecycle operation in the normal queue.

## ApplicationService lock change

Previously ApplicationService executed every handler while holding its RLock.
That made a second safety worker ineffective.

Phase 6.2 changes normal dispatch to:

```text
lock: validate + capture handler
unlock
run normal handler
lock: commit transition only if state is unchanged
```

Normal commands remain mutually serialized through a separate normal-dispatch
lock, including commands still issued by the legacy Qt bridge.

Safety commands:

```text
lock: pretransition to FAULT/ESTOP
unlock
run best-effort safety handler
```

## Late completion and safety reassert

Opaque vendor calls cannot always be cancelled. If a normal handler finishes
after FAULT/ESTOP has already taken priority:

1. its normal workflow transition is suppressed;
2. `command_superseded` is emitted;
3. the active safety handler is reasserted once;
4. `CommandSuperseded` is raised to the async-port error sink.

Software ESTOP retains its existing meaning: stop commanded motion. It is not a
replacement for the cabinet physical emergency stop and does not automatically
power the robot off.

## Window boundary

`OperatorMainWindow` still submits only `CommandRequest` to `CommandPort`.
It never imports or calls ApplicationService directly. On close it closes the
port and presenter.

No real robot movement is part of Phase 6.2 unit testing.

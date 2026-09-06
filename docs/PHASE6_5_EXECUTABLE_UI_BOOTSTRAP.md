# Phase 6.5 - Executable PySide6 bootstrap and camera bridge

Phase 6.5 creates the first executable entry point for the new UI:

```bash
PYTHONPATH=src python -m gello_cr.ui --check-only
PYTHONPATH=src python -m gello_cr.ui
```

## Startup safety

Both paths construct the object graph without opening hardware.

Normal GUI startup guarantees:

```text
ApplicationService = OFFLINE
CameraPollingService = stopped
CR3A = not connected
GELLO/O6 = not connected
robot servo = untouched
TeleopEngine follow = not started
```

`--check-only` does not import PySide6 and is intended as a safe composition
smoke test.

## CameraPollingService

`RealSenseRgbDevice` already owns the SDK polling thread for each physical
camera. The new service adds only the cross-camera bridge:

```text
wrist RealSense.latest() ──────────────┐
                                       ├─ RecordingSampleSource
base RealSense.latest() ──┬────────────┘
                          └─ crop_normalized_roi() → ROI RGB
```

Calling `start()` explicitly connects the two cameras. If the second camera
fails, the first is rolled back and both sample-source streams are cleared.

Calling `stop()` is idempotent and closes both cameras.

## OperatorApplication

`build_operator_application()` composes:

```text
ConcreteRuntimeFactory
        ↓
ConcreteRuntime
        ↓
OperatorBackend
        +
CameraPollingService
```

No hardware operation occurs during composition.

## Scope

Phase 6.5 intentionally does not add a "start cameras" or "connect GELLO/O6"
button yet. Those controls need explicit workflow policy and shutdown semantics,
which are the Phase 6.6 hardware-preparation gate.

`TEST_INEXBOT.py` remains the validated real-hardware entry point.

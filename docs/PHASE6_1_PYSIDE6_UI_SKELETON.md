# Phase 6.1 - PySide6 operator UI skeleton

Phase 6 starts a new UI under `src/gello_cr/ui/`. The existing PyQt5
`TEST_INEXBOT.py` remains intact and remains the validated operational UI.

## Boundary

The new window is allowed to consume only:

- `ApplicationViewModel`;
- `AppEvent`;
- `CommandRequest` through `CommandPort`;
- `OperatorUiPresenter`.

It must not import NRC, TeleopEngine, LeRobot recorder implementation, GELLO,
O6, RealSense or vendor SDK modules.

## Presenter

`OperatorUiPresenter` is framework independent. It combines:

```text
ApplicationService.snapshot()
runtime snapshot provider
recorder snapshot provider
        ↓
ApplicationViewModel
```

and drains application events through the existing bounded
`ApplicationEventBuffer`.

It dispatches no command.

## CommandPort

The PySide6 window creates immutable `CommandRequest` objects and submits them
to `CommandPort`.

Phase 6.1 intentionally does not define the production threading/execution
strategy. That is Phase 6.2, because robot lifecycle commands must remain off
the Qt GUI thread and software E-stop ordering must be handled deliberately.

## STOP_EPISODE contract

The old UI could directly call `LeRobotEpisodeRecorder.stop_episode()`. The new
UI may not touch Recorder directly, so Phase 6.1 adds:

```text
Command.STOP_EPISODE
RECORDING -> RECORDING
```

The runtime binding calls `recorder.stop_episode()`. Saving or discarding still
performs the later transition out of RECORDING.

## Preview

`python -m gello_cr.ui.preview` is a no-hardware UI preview. It only logs
CommandRequest objects and never creates NRC/GELLO/O6/camera objects.

The preview requires PySide6 to be installed. The normal V2 unit suite does not
require PySide6 and therefore remains runnable in the baseline Python 3.12
environment.

## Phase 6.1 is not a hardware gate

Do not use this preview to validate robot movement. The old validated PyQt5
entrypoint remains the operational path until later Phase 6 hardware gates.

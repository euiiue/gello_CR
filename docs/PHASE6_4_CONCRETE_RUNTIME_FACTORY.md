# Phase 6.4 - Concrete runtime factory

Phase 6.4 extracts the remaining runtime construction responsibilities from the
legacy PyQt5 window without changing the validated operational entrypoint yet.

## New bootstrap package

```text
src/gello_cr/bootstrap/
├── runtime_factory.py
└── sample_source.py
```

`ConcreteRuntimeFactory.build()` creates the current production object graph:

- TeleopConfigStore;
- RoArm / Inverse3 / GELLO controller objects;
- selected master controller;
- O6 controller;
- TeleopEngine;
- thread-safe RecordingSampleSource;
- LeRobotEpisodeRecorder;
- wrist/base RealSenseRgbDevice objects;
- explicit CR3A lifecycle callbacks.

Construction performs no hardware I/O. In particular it does not call
`connect()`, `power_on()`, `start_follow()`, camera start or `start_episode()`.

## CR3A lifecycle

`Cr3aLifecycle.connect()` is the first point that constructs and opens a
Cr3aDevice. Successful connection attaches its NrcRobotSession to TeleopEngine.
Power/reset/disconnect remain explicit ApplicationService lifecycle callbacks.

Reset verifies servo state 3 before calling `TeleopEngine.acknowledge_fault()`;
it never starts follow automatically.

## Recorder sample source

The old `_lerobot_sample_provider()` lived on `MyMainForm` because camera frame
caches also lived on the Qt object. `RecordingSampleSource` now owns that
thread-safe frame boundary independently of Qt and preserves the existing
quality fields, freshness checks and three-image recording contract.

Phase 6.5 can therefore add the concrete camera polling/service and executable
PySide6 bootstrap without copying legacy Qt state into the new window.

This phase does not replace `TEST_INEXBOT.py` as the hardware-validated entry
point and does not perform real robot motion tests.

# Phase 4.5 - LeRobot worker dataset service extraction

Phase 4.5 moves the worker-side LeRobotDataset state machine from the root
`lerobot_recorder.py` into:

`src/gello_cr/recording/worker_service.py`

The moved service owns:

- `init`
- `add_frame`
- `save_episode`
- `clear_episode`
- `finalize`
- worker socket request loop
- best-effort finalization on worker exit

The root function is now intentionally tiny:

```python
def _worker_main(fd: int) -> int:
    return _run_worker_service(fd)
```

## Preserved behavior

- LeRobot imports remain lazy and occur only in the dedicated worker process.
- H.264 encoder settings are unchanged.
- `robot_type="dobot_cr5_o6"` is unchanged.
- 224x224 validation is unchanged.
- 18D state / 12D action are unchanged.
- raw image order remains base / wrist / ROI.
- collection sidecar is still written before `dataset.save_episode()`.
- unsaved frames still block finalize.
- worker errors still return `{ok: false, error: ...}`.
- root CLI remains `lerobot_recorder.py --worker-fd <fd>`.

No robot, camera, or real LeRobot worker is needed for the tests in this phase.

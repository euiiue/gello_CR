# Phase 4.4 - LeRobot worker client extraction

Phase 4.4 moves `_WorkerClient` out of the root legacy recorder into:

`src/gello_cr/recording/worker_client.py`

The moved client still owns:

- `socket.socketpair()`
- worker subprocess launch
- inherited `LD_LIBRARY_PATH`
- serialized request locking
- stderr collection
- request timeout handling
- communication-error latch
- wait -> terminate -> kill shutdown sequence

## Worker script path

Before extraction, `_WorkerClient` lived in `lerobot_recorder.py`, so
`Path(__file__).resolve()` naturally pointed to the root worker script.

After extraction that would point to `worker_client.py`, which would be wrong.
The new client therefore receives an explicit `worker_script`, and the root
recorder passes its own path:

```python
WorkerClient(
    self.worker_python,
    worker_script=Path(__file__).resolve(),
)
```

The actual subprocess target remains:

```text
<LeRobot Python> <repo>/lerobot_recorder.py --worker-fd <fd>
```

## Compatibility

The root recorder preserves the old names:

- `_WorkerClient` -> `WorkerClient`
- `LeRobotRecorderError` -> `WorkerClientError`

Episode lifecycle, worker main, protocol, sample cadence, schema, frame
preparation and dataset output do not change.

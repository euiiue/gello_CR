# Phase 4.6 - Episode recorder migration and Phase 4 completion

Phase 4.6 moves `LeRobotEpisodeRecorder` from the root legacy module into:

`src/gello_cr/recording/episode_recorder.py`

After this phase, the recording package owns the complete recording stack:

```text
episode_recorder.py
  -> frame.py
  -> worker_client.py
  -> protocol.py
  -> worker_service.py
  -> schema.py
```

The root `lerobot_recorder.py` remains as a compatibility facade and the
subprocess CLI entrypoint.

Moving the class changes the meaning of `__file__`. The moved recorder
therefore stores an explicit worker script path; existing callers do not need
to provide it, and the default still resolves to `<repo>/lerobot_recorder.py`.

Frozen behavior remains: 20 Hz, 18D state, 12D action, 3 RGB, 224x224, quality
accounting, Episode lifecycle, worker timeouts, protocol and dataset schema.

This is the final structural extraction planned for Phase 4.

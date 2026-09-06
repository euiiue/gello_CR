# Phase 4.2 - Recording frame preparation

Phase 4.2 moves frame-preparation logic out of the root legacy recorder.

New module:

`src/gello_cr/recording/frame.py`

It now owns:

- OpenPI-style aspect-preserving RGB resize with black padding;
- conversion of state/action values to float lists;
- fixed raw RGB byte order;
- construction of the existing worker `add_frame` request.

Frozen raw image order:

1. base full RGB
2. wrist full RGB
3. base ROI RGB

The following are intentionally unchanged:

- synchronous Unix socket framing;
- worker subprocess;
- request timeouts;
- 20 Hz sampling loop;
- episode start/stop/save/discard;
- three image streams;
- state/action schema;
- on-disk LeRobot feature names.

`lerobot_recorder.resize_rgb_for_openpi` remains import-compatible because the
root module imports the moved function from V2.

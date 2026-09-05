# Phase 2.5a - RealSense RGB device boundary

This phase adds a thread-owned RealSense RGB device adapter without modifying
the legacy Qt camera functions yet.

## Current physical camera contract

The existing application configures two D435 RGB streams:

```text
wrist D435: 640x480 BGR8 @ 30 FPS
base  D435: 640x480 BGR8 @ 30 FPS
```

Training records RGB only.

The base ROI is **not** a third camera. It remains an image-processing output
derived from the base RGB frame.

## New V2 boundary

```text
RealSenseRgbDevice
  ├─ serial selection
  ├─ pyrealsense2 pipeline/config
  ├─ RGB stream setup
  ├─ non-blocking poll thread
  ├─ BGR -> RGB conversion
  ├─ monotonic receive timestamp
  └─ thread-safe CameraSnapshot
```

`CameraSnapshot` and a generic `CameraDevice` protocol are added to
`core/contracts.py`.

## Deliberately not migrated yet

- `TEST_INEXBOT.py` still owns its legacy D435 pipelines in Phase 2.5a.
- base ROI cropping remains in the old UI until the pure ROI transform is
  extracted.
- no dataset key/schema is changed.

No RealSense hardware is required for this phase.

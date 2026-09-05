# Phase 2.5b - Route legacy Qt cameras through RealSenseRgbDevice

This phase removes direct `pyrealsense2` ownership from `TEST_INEXBOT.py`.

## Before

```text
Qt window
  ├─ rs.pipeline/config for wrist
  ├─ rs.pipeline/config for base
  ├─ poll_for_frames()
  ├─ BGR -> RGB
  └─ pipeline.stop()
```

## After

```text
RealSenseRgbDevice (wrist)
RealSenseRgbDevice (base)
        ↓
CameraSnapshot
        ↓
legacy Qt render/cache compatibility
```

The Qt timers remain temporarily. They only copy/render the latest
`CameraSnapshot`; the actual SDK polling thread lives inside the V2 device.

The existing recorder cache fields remain unchanged:

```text
_wrist_rgb_frame
_wrist_rgb_timestamp
_base_rgb_frame
_base_roi_rgb_frame
_base_rgb_timestamp
```

Therefore this phase does not alter the LeRobot sample provider or dataset
schema.

Base ROI cropping also remains in `update_frame_D435_2()` for one more phase.
Phase 2.5c will extract it as a pure transform.

No robot motion is involved. A later hardware gate will verify both camera
serial numbers and live frame freshness.

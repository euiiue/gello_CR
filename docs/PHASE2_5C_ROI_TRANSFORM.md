# Phase 2.5c - Extract base ROI as a pure image transform

The base ROI is derived from the base RealSense RGB image. It is not a physical
camera device.

This phase moves ROI pixel-bound calculation and cropping out of the legacy Qt
window into:

```text
src/gello_cr/recording/image_processing.py
```

## Preserved behavior

For the current production configuration:

```text
source image: 640x480
base_roi_norm: [0.37, 0.56, 0.55, 0.79]
```

the old and new implementation both produce:

```text
pixel bounds: (237, 269) -> (352, 379)
crop size:    115 x 110
```

The transform intentionally preserves the legacy:

- `round(normalized * size)` behavior;
- coordinate clamping;
- minimum one-pixel ROI behavior.

No ROI geometry is changed in this refactor.

## Layering after this phase

```text
RealSenseRgbDevice
        ↓ full base RGB
crop_normalized_roi()
        ↓
base ROI RGB
        ↓
legacy cache / preview / recorder
```

Qt still draws the diagnostic rectangle and status text, because those are
presentation concerns.

The LeRobot recorder cache names and dataset schema remain unchanged.

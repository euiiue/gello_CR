# Phase 4.1 - Recording schema consolidation

This phase starts the recorder migration without changing recording timing,
socket transport, worker ownership, video encoding, or episode lifecycle.

The recorder no longer keeps a second independent copy of the 18D state / 12D
action field names and dimensions.  It now delegates validation and LeRobot
feature construction to `src/gello_cr/recording/schema.py`, which imports the
frozen definitions from `gello_cr.data_contract`.

Historical serialized image feature names are intentionally unchanged:

- `observation.images.base_0_rgb` = base full RGB
- `observation.images.left_wrist_0_rgb` = wrist full RGB
- `observation.images.right_wrist_0_rgb` = base ROI RGB

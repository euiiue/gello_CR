# Phase 2.2a - O6 device boundary

This phase introduces `O6Device` under `src/gello_cr/devices/o6.py` without
modifying the legacy `teleop_runtime.O6Controller`.

The adapter preserves the proven single-owner worker-thread model and keeps all
LinkerHand SDK access below the device boundary.

## Preserved behavior

- six 0..255 position channels;
- six fault-code channels;
- speed/torque profile writes;
- target de-duplication;
- 0.2 s position polling by default;
- 0.8 s fault polling by default;
- hold-current behavior;
- wait-until-position behavior;
- single-motor recovery safety policy;
- recovery refuses fault codes other than 0 and 1.

The position/fault pair is exposed as a typed `HandSnapshot`.

## Migration strategy

This is step 1 of 2:

1. add/test `O6Device`;
2. bridge the legacy `O6Controller` to `O6Device` in Phase 2.2b.

No real robot or hand movement is required for this phase.

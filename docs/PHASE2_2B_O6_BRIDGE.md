# Phase 2.2b - Legacy O6 compatibility bridge

`teleop_runtime.O6Controller` is now a compatibility facade over
`src/gello_cr/devices/o6.py`.

The legacy runtime no longer owns LinkerHand SDK import, O6 worker-thread I/O,
position/fault polling, profile/target dispatch, or motor-recovery logic.

The public O6 API used by `TeleopEngine` remains unchanged.

One legacy characterization test directly accesses `_thread`,
`_recovery_requests`, and `_process_recovery`; these are temporarily proxied to
`O6Device` until the old test suite itself is migrated.

No real O6 motion is required for this phase.

# Phase 5.6 - Application-state workflow UI policy

Phase 5.6 makes ApplicationService state the primary gate for workflow controls
in the legacy Qt UI.

The policy is reusable and UI-framework independent:

`src/gello_cr/app/workflow_ui.py`

## Gate composition

This phase deliberately does not replace device-level readiness checks.

```text
final enabled
=
legacy device/busy/freshness condition
AND
application workflow policy
```

Therefore migration cannot make a hardware control more permissive than the
validated legacy UI.

## Application-gated controls

- CR3A connect buttons
- CR3A power-on buttons
- start teleop buttons
- software E-stop buttons
- Episode start / stop / save / discard

The generic "stop current action" buttons remain under legacy runtime gating
because they also stop preset/replay/high-priority legacy actions that are not
yet represented in WorkflowStateMachine.

## Episode save-then-next compatibility

A stopped Episode with pending frames keeps Application state `RECORDING`.
The existing UI supports pressing Start to save that previous Episode and
immediately begin the next one. The policy therefore allows Start when:

```text
state == RECORDING
and episode_pending
and not episode_active
```

The actual operation remains:

```text
SAVE_SUCCESS / SAVE_FAILURE
-> TELEOP_RUNNING
-> START_EPISODE
-> RECORDING
```

## Fault/ESTOP

Pending data in FAULT/ESTOP may be saved only as FAILURE or discarded.
SAVE_SUCCESS and new Episode start remain disabled. FAULT may still escalate to
ESTOP; ESTOP itself does not expose another software E-stop action.

No hardware commands, motion limits, recorder schema, or sampling behavior are
changed.

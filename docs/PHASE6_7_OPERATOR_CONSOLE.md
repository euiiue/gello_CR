# Phase 6.7 - Operator console visual consolidation

Phase 6.7 turns the new PySide6 skeleton into the first practical collection
console without changing robot-control semantics.

## Three RGB previews

The UI now renders:

- Base RGB;
- Wrist RGB;
- Base ROI.

The window never accesses physical camera objects. `RecordingSampleSource`
provides detached preview snapshots through:

```text
RecordingSampleSource.preview_snapshot()
        ↓
OperatorUiPresenter
        ↓
UiFrame.preview
        ↓
OperatorMainWindow
```

The same source continues to feed LeRobot recording. No duplicate camera
pipeline/cache is created.

## Operator layout

The window now uses:

```text
workflow headline              [large software ESTOP]
CR3A | GELLO/Master | O6 | Cameras status cards
primary four-step workflow
camera controls / recovery controls
Base RGB | Wrist RGB | Base ROI
LeRobot Episode
compact bounded event log
```

The ESTOP control is deliberately separated from normal workflow buttons.

## Event log

The log is limited to 400 blocks and approximately 145 px height so live camera
views remain the dominant operational information.

## Episode status

Before any dataset session exists, the UI displays `未启动数据集` instead of
showing a quality-review flag with no active collection.

## Safety

This phase does not change:

- ApplicationService safety preemption;
- CR3A lifecycle;
- ServoJ parameters or loop;
- GELLO/O6 control mapping;
- recorder schema;
- camera start/stop commands.

Launching the UI still performs no hardware auto-connect.

After Phase 6.7 software regression, the next work should be hardware gates in
this order:

1. camera-only;
2. CR3A connect-only;
3. GELLO/O6 connect-only;
4. power-on;
5. ServoJ follow last.

# Phase 5.5 - Application event bridge to Qt

Phase 5.5 connects `ApplicationService` events to the legacy Qt UI without
allowing worker threads to touch Qt widgets.

## Thread-safe direction

```text
ApplicationService
    |
    | AppEvent (possibly worker thread)
    v
ApplicationEventBuffer
    |
    | drain() every 100 ms
    v
Qt GUI thread
    +-> file log
    +-> teleop log
    +-> status bar
```

`ApplicationEventBuffer` is UI-independent and bounded. The legacy Qt bridge
uses a capacity of 256. If producers outrun the GUI, oldest application events
are discarded instead of allowing unbounded memory growth, and the GUI reports
the drop count.

`teleop_engine.events` remains for lower-level runtime/device detail messages
that have not migrated into the application event model.

No hardware command, motion parameter, dataset schema, or sampling behavior is
changed in this phase.

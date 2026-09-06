
"""Framework-independent presenter for the operator UI.

The presenter owns no device and dispatches no command.  It only combines
already-safe snapshots into ApplicationViewModel and transfers AppEvent objects
from ApplicationService to a GUI polling loop.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from gello_cr.app import (
    AppEvent,
    ApplicationEventBuffer,
    ApplicationService,
    ApplicationViewModel,
    build_application_view_model,
)

SnapshotSource = Callable[[], Mapping[str, Any]]


@dataclass(frozen=True, slots=True)
class UiFrame:
    view_model: ApplicationViewModel
    events: tuple[AppEvent, ...]


class OperatorUiPresenter:
    """Read-only presentation adapter used by Qt/PySide UI layers."""

    def __init__(
        self,
        service: ApplicationService,
        *,
        runtime_snapshot: SnapshotSource,
        recorder_snapshot: SnapshotSource,
        event_capacity: int = 256,
    ) -> None:
        self._service = service
        self._runtime_snapshot = runtime_snapshot
        self._recorder_snapshot = recorder_snapshot
        self._events = ApplicationEventBuffer(capacity=event_capacity)
        self._unsubscribe = service.subscribe(self._events.push)
        self._closed = False

    @property
    def closed(self) -> bool:
        return self._closed

    def poll(self) -> UiFrame:
        if self._closed:
            raise RuntimeError("operator UI presenter is closed")

        runtime = dict(self._runtime_snapshot())
        recorder = dict(self._recorder_snapshot())
        view_model = build_application_view_model(
            self._service.snapshot(),
            runtime,
            recorder,
        )
        return UiFrame(
            view_model=view_model,
            events=self._events.drain(),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._unsubscribe()

"""Application command/event/state orchestration.

This module is intentionally UI-framework agnostic.  Qt/PySide should issue
commands to ApplicationService and subscribe to AppEvent objects instead of
directly calling robot/teleop/recorder SDK objects.
"""

from __future__ import annotations

import threading
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from gello_cr.core.state_machine import (
    Command,
    InvalidTransition,
    WorkflowState,
    WorkflowStateMachine,
)

from .events import AppEvent, EventLevel

CommandHandler = Callable[[Mapping[str, Any]], Any]
EventSubscriber = Callable[[AppEvent], None]

# Safety commands enter their safe state before any best-effort hardware
# handler.  A handler failure must never put the application back into an
# active motion state.
_PRETRANSITION_COMMANDS = frozenset(
    {
        Command.REPORT_FAULT,
        Command.EMERGENCY_STOP,
    }
)


@dataclass(frozen=True, slots=True)
class CommandResult:
    command: Command
    previous_state: WorkflowState
    state: WorkflowState
    value: Any = None


@dataclass(frozen=True, slots=True)
class ApplicationSnapshot:
    state: WorkflowState
    recovery_state: WorkflowState
    event_sequence: int
    last_error: str


class ApplicationService:
    """Single application-layer command dispatcher.

    Normal command:
        validate -> handler -> state transition

    REPORT_FAULT / EMERGENCY_STOP:
        validate -> safe state transition -> best-effort handler

    This ordering gives two useful guarantees:
    - failed start/save/reset operations do not falsely advance UI state;
    - a failed emergency-stop handler cannot restore an active state.
    """

    def __init__(
        self,
        *,
        state_machine: WorkflowStateMachine | None = None,
        handlers: Mapping[Command, CommandHandler] | None = None,
    ) -> None:
        self._state_machine = state_machine or WorkflowStateMachine()
        self._handlers: dict[Command, CommandHandler] = dict(handlers or {})
        self._subscribers: list[EventSubscriber] = []
        self._lock = threading.RLock()
        self._event_sequence = 0
        self._last_error = ""

    @property
    def state(self) -> WorkflowState:
        with self._lock:
            return self._state_machine.state

    def snapshot(self) -> ApplicationSnapshot:
        with self._lock:
            return ApplicationSnapshot(
                state=self._state_machine.state,
                recovery_state=self._state_machine.recovery_state,
                event_sequence=self._event_sequence,
                last_error=self._last_error,
            )

    def can(self, command: Command) -> bool:
        with self._lock:
            return self._state_machine.can(command)

    def set_handler(
        self,
        command: Command,
        handler: CommandHandler | None,
    ) -> None:
        with self._lock:
            if handler is None:
                self._handlers.pop(command, None)
            else:
                self._handlers[command] = handler

    def subscribe(
        self,
        subscriber: EventSubscriber,
    ) -> Callable[[], None]:
        with self._lock:
            self._subscribers.append(subscriber)

        def unsubscribe() -> None:
            with self._lock:
                try:
                    self._subscribers.remove(subscriber)
                except ValueError:
                    pass

        return unsubscribe

    def dispatch(
        self,
        command: Command,
        payload: Mapping[str, Any] | None = None,
    ) -> CommandResult:
        data = MappingProxyType(dict(payload or {}))

        with self._lock:
            previous_state = self._state_machine.state
            if not self._state_machine.can(command):
                message = (
                    f"{command.name} is not allowed from "
                    f"{previous_state.name}"
                )
                self._emit_locked(
                    kind="command_rejected",
                    level=EventLevel.WARNING,
                    message=message,
                    command=command,
                    details=data,
                )
                raise InvalidTransition(message)

            handler = self._handlers.get(command)

            if command in _PRETRANSITION_COMMANDS:
                new_state = self._state_machine.apply(command)
                self._emit_locked(
                    kind="state_changed",
                    level=EventLevel.WARNING,
                    message=(
                        f"{previous_state.name} -> {new_state.name} "
                        f"by {command.name}"
                    ),
                    command=command,
                    details=data,
                )

                try:
                    value = handler(data) if handler is not None else None
                except Exception as exc:
                    self._last_error = f"{type(exc).__name__}: {exc}"
                    self._emit_locked(
                        kind="command_failed",
                        level=EventLevel.ERROR,
                        message=self._last_error,
                        command=command,
                        details=data,
                    )
                    raise

                self._last_error = ""
                self._emit_locked(
                    kind="command_completed",
                    level=EventLevel.INFO,
                    message=f"{command.name} completed",
                    command=command,
                    details=data,
                )
                return CommandResult(
                    command=command,
                    previous_state=previous_state,
                    state=new_state,
                    value=value,
                )

            try:
                value = handler(data) if handler is not None else None
            except Exception as exc:
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._emit_locked(
                    kind="command_failed",
                    level=EventLevel.ERROR,
                    message=self._last_error,
                    command=command,
                    details=data,
                )
                raise

            new_state = self._state_machine.apply(command)
            self._last_error = ""
            self._emit_locked(
                kind="state_changed",
                level=EventLevel.INFO,
                message=(
                    f"{previous_state.name} -> {new_state.name} "
                    f"by {command.name}"
                ),
                command=command,
                details=data,
            )
            self._emit_locked(
                kind="command_completed",
                level=EventLevel.INFO,
                message=f"{command.name} completed",
                command=command,
                details=data,
            )
            return CommandResult(
                command=command,
                previous_state=previous_state,
                state=new_state,
                value=value,
            )

    def _emit_locked(
        self,
        *,
        kind: str,
        level: EventLevel,
        message: str,
        command: Command | None,
        details: Mapping[str, Any],
    ) -> None:
        self._event_sequence += 1
        event = AppEvent(
            sequence=self._event_sequence,
            kind=kind,
            level=level,
            state=self._state_machine.state,
            message=message,
            command=command,
            details=details,
        )
        subscribers = tuple(self._subscribers)

        # Subscribers are observers.  One broken UI/log subscriber must not
        # corrupt command/state processing or block the other observers.
        for subscriber in subscribers:
            try:
                subscriber(event)
            except Exception:
                continue

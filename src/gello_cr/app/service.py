
"""Application command/event/state orchestration.

Normal command handlers execute outside the state lock. This allows
REPORT_FAULT / EMERGENCY_STOP to preempt a long-running lifecycle/recording
handler while preserving one-at-a-time normal command execution.
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

_PRETRANSITION_COMMANDS = frozenset(
    {
        Command.REPORT_FAULT,
        Command.EMERGENCY_STOP,
    }
)


class CommandSuperseded(RuntimeError):
    """A normal handler finished after a safety transition had taken priority."""


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

    Normal commands are serialized by ``_normal_dispatch_lock`` but their
    handlers run without holding ``_lock``. Safety commands therefore remain
    able to pretransition the workflow while an opaque lifecycle/recording
    operation is still in progress.

    If a normal handler later returns after FAULT/ESTOP took priority, its
    original workflow transition is suppressed and the active safety handler is
    reasserted once.
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
        self._normal_dispatch_lock = threading.Lock()
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

        if command in _PRETRANSITION_COMMANDS:
            return self._dispatch_safety(command, data)

        with self._normal_dispatch_lock:
            return self._dispatch_normal(command, data)

    def _dispatch_safety(
        self,
        command: Command,
        data: Mapping[str, Any],
    ) -> CommandResult:
        with self._lock:
            previous_state = self._state_machine.state
            self._validate_locked(command, data)
            handler = self._handlers.get(command)
            new_state = self._state_machine.apply(command)
            self._last_error = ""
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
            with self._lock:
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._emit_locked(
                    kind="command_failed",
                    level=EventLevel.ERROR,
                    message=self._last_error,
                    command=command,
                    details=data,
                )
            raise

        with self._lock:
            current_state = self._state_machine.state
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
                state=current_state,
                value=value,
            )

    def _dispatch_normal(
        self,
        command: Command,
        data: Mapping[str, Any],
    ) -> CommandResult:
        with self._lock:
            previous_state = self._state_machine.state
            self._validate_locked(command, data)
            handler = self._handlers.get(command)

        try:
            value = handler(data) if handler is not None else None
        except Exception as exc:
            with self._lock:
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._emit_locked(
                    kind="command_failed",
                    level=EventLevel.ERROR,
                    message=self._last_error,
                    command=command,
                    details=data,
                )
            raise

        safety_reassert: tuple[
            Command,
            CommandHandler,
            Mapping[str, Any],
        ] | None = None

        with self._lock:
            current_state = self._state_machine.state
            if current_state is not previous_state:
                message = (
                    f"{command.name} handler completed after workflow changed "
                    f"{previous_state.name} -> {current_state.name}; "
                    "normal transition suppressed"
                )
                self._emit_locked(
                    kind="command_superseded",
                    level=EventLevel.WARNING,
                    message=message,
                    command=command,
                    details=data,
                )
                safety_reassert = self._safety_reassert_locked(
                    current_state,
                    superseded_command=command,
                )
            else:
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

        if safety_reassert is not None:
            safety_command, safety_handler, safety_payload = safety_reassert
            try:
                safety_handler(safety_payload)
            except Exception as exc:
                with self._lock:
                    self._last_error = f"{type(exc).__name__}: {exc}"
                    self._emit_locked(
                        kind="safety_reassert_failed",
                        level=EventLevel.ERROR,
                        message=self._last_error,
                        command=safety_command,
                        details=safety_payload,
                    )
            else:
                with self._lock:
                    self._emit_locked(
                        kind="safety_reasserted",
                        level=EventLevel.WARNING,
                        message=(
                            f"{safety_command.name} reasserted after "
                            f"{command.name} completed late"
                        ),
                        command=safety_command,
                        details=safety_payload,
                    )

        raise CommandSuperseded(
            f"{command.name} was superseded by {current_state.name}"
        )

    def _validate_locked(
        self,
        command: Command,
        data: Mapping[str, Any],
    ) -> None:
        if self._state_machine.can(command):
            return

        previous_state = self._state_machine.state
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

    def _safety_reassert_locked(
        self,
        state: WorkflowState,
        *,
        superseded_command: Command,
    ) -> tuple[Command, CommandHandler, Mapping[str, Any]] | None:
        if state is WorkflowState.ESTOP:
            command = Command.EMERGENCY_STOP
        elif state is WorkflowState.FAULT:
            command = Command.REPORT_FAULT
        else:
            return None

        handler = self._handlers.get(command)
        if handler is None:
            return None

        payload = MappingProxyType(
            {
                "reason": (
                    f"reassert {state.name} after late "
                    f"{superseded_command.name} completion"
                ),
                "superseded_command": superseded_command.name,
            }
        )
        return command, handler, payload

    def report_external_fault(
        self,
        reason: str,
        details: Mapping[str, Any] | None = None,
    ) -> WorkflowState:
        """Synchronize a runtime fault that has already performed safe-stop."""

        message = str(reason).strip() or "runtime fault"
        data = MappingProxyType(dict(details or {}))

        with self._lock:
            current = self._state_machine.state

            if current is WorkflowState.ESTOP:
                if self._last_error != message:
                    self._last_error = message
                    self._emit_locked(
                        kind="external_fault_observed",
                        level=EventLevel.ERROR,
                        message=message,
                        command=Command.REPORT_FAULT,
                        details=data,
                    )
                return current

            if current is WorkflowState.FAULT:
                if self._last_error != message:
                    self._last_error = message
                    self._emit_locked(
                        kind="external_fault_observed",
                        level=EventLevel.ERROR,
                        message=message,
                        command=Command.REPORT_FAULT,
                        details=data,
                    )
                return current

            previous_state = current
            new_state = self._state_machine.apply(Command.REPORT_FAULT)
            self._last_error = message
            self._emit_locked(
                kind="state_changed",
                level=EventLevel.WARNING,
                message=(
                    f"{previous_state.name} -> {new_state.name} "
                    "by external runtime fault"
                ),
                command=Command.REPORT_FAULT,
                details=data,
            )
            self._emit_locked(
                kind="external_fault_observed",
                level=EventLevel.ERROR,
                message=message,
                command=Command.REPORT_FAULT,
                details=data,
            )
            return new_state

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

        for subscriber in subscribers:
            try:
                subscriber(event)
            except Exception:
                continue

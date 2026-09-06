
"""Asynchronous command-request boundary for the PySide6 operator UI."""

from __future__ import annotations

import queue
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Protocol

from gello_cr.app import ApplicationService
from gello_cr.core.state_machine import Command


def _empty_payload() -> Mapping[str, Any]:
    return MappingProxyType({})


@dataclass(frozen=True, slots=True)
class CommandRequest:
    command: Command
    payload: Mapping[str, Any] = field(default_factory=_empty_payload)

    @classmethod
    def create(
        cls,
        command: Command,
        payload: Mapping[str, Any] | None = None,
    ) -> "CommandRequest":
        return cls(
            command=command,
            payload=MappingProxyType(dict(payload or {})),
        )


class CommandPortError(RuntimeError):
    pass


class CommandPortClosed(CommandPortError):
    pass


class CommandQueueFull(CommandPortError):
    pass


class CommandPort(Protocol):
    def submit(self, request: CommandRequest) -> None:
        ...

    def close(self, timeout: float = 1.0) -> None:
        ...


class CallbackCommandPort:
    """Synchronous adapter used by the no-hardware preview/tests."""

    def __init__(self, callback: Callable[[CommandRequest], None]) -> None:
        self._callback = callback
        self._closed = False

    def submit(self, request: CommandRequest) -> None:
        if not isinstance(request, CommandRequest):
            raise TypeError("request must be CommandRequest")
        if self._closed:
            raise CommandPortClosed("command port is closed")
        self._callback(request)

    def close(self, timeout: float = 1.0) -> None:
        del timeout
        self._closed = True


CommandErrorCallback = Callable[[CommandRequest, Exception], None]

_SAFETY_COMMANDS = frozenset(
    {
        Command.EMERGENCY_STOP,
        Command.REPORT_FAULT,
    }
)


class AsyncApplicationCommandPort:
    """Two-lane asynchronous dispatcher.

    Normal commands are FIFO on one worker. Safety commands use a separate
    worker and therefore never wait behind CONNECT/POWER_ON/SAVE/etc.
    """

    def __init__(
        self,
        service: ApplicationService,
        *,
        normal_capacity: int = 32,
        safety_capacity: int = 8,
        error_callback: CommandErrorCallback | None = None,
    ) -> None:
        if int(normal_capacity) <= 0:
            raise ValueError("normal_capacity must be positive")
        if int(safety_capacity) <= 0:
            raise ValueError("safety_capacity must be positive")

        self._service = service
        self._normal_queue: queue.Queue[CommandRequest] = queue.Queue(
            maxsize=int(normal_capacity)
        )
        self._safety_queue: queue.Queue[CommandRequest] = queue.Queue(
            maxsize=int(safety_capacity)
        )
        self._error_callback = error_callback
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._closed = False

        self._normal_thread = threading.Thread(
            target=self._worker,
            args=(self._normal_queue,),
            name="ApplicationCommand-Normal",
            daemon=True,
        )
        self._safety_thread = threading.Thread(
            target=self._worker,
            args=(self._safety_queue,),
            name="ApplicationCommand-Safety",
            daemon=True,
        )
        self._normal_thread.start()
        self._safety_thread.start()

    @property
    def closed(self) -> bool:
        with self._lock:
            return self._closed

    def submit(self, request: CommandRequest) -> None:
        if not isinstance(request, CommandRequest):
            raise TypeError("request must be CommandRequest")

        with self._lock:
            if self._closed:
                raise CommandPortClosed("command port is closed")

            target = (
                self._safety_queue
                if request.command in _SAFETY_COMMANDS
                else self._normal_queue
            )
            try:
                target.put_nowait(request)
            except queue.Full as exc:
                lane = (
                    "safety"
                    if request.command in _SAFETY_COMMANDS
                    else "normal"
                )
                raise CommandQueueFull(
                    f"{lane} command queue is full"
                ) from exc

    def _worker(self, work_queue: queue.Queue[CommandRequest]) -> None:
        while not self._stop.is_set():
            try:
                request = work_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                if self._stop.is_set():
                    continue
                self._service.dispatch(
                    request.command,
                    request.payload,
                )
            except Exception as exc:
                callback = self._error_callback
                if callback is not None:
                    try:
                        callback(request, exc)
                    except Exception:
                        pass
            finally:
                work_queue.task_done()

    def close(self, timeout: float = 1.0) -> None:
        duration = max(0.0, float(timeout))
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._stop.set()

        self._discard_pending(self._normal_queue)
        self._discard_pending(self._safety_queue)

        deadline = time.monotonic() + duration
        for thread in (self._normal_thread, self._safety_thread):
            remaining = max(0.0, deadline - time.monotonic())
            thread.join(remaining)

    @staticmethod
    def _discard_pending(
        work_queue: queue.Queue[CommandRequest],
    ) -> None:
        while True:
            try:
                work_queue.get_nowait()
            except queue.Empty:
                return
            else:
                work_queue.task_done()

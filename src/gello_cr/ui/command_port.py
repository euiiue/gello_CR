
"""Command-request boundary for GUI code.

The PySide6 window submits requests to this port instead of calling
ApplicationService, TeleopEngine, Recorder, NRC, GELLO, O6 or camera objects
directly.  Phase 6.2 will provide the production asynchronous implementation.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Protocol

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


class CommandPort(Protocol):
    def submit(self, request: CommandRequest) -> None:
        ...


class CallbackCommandPort:
    """Small adapter useful for tests/preview and the Phase 6.2 bridge."""

    def __init__(self, callback: Callable[[CommandRequest], None]) -> None:
        self._callback = callback

    def submit(self, request: CommandRequest) -> None:
        if not isinstance(request, CommandRequest):
            raise TypeError("request must be CommandRequest")
        self._callback(request)

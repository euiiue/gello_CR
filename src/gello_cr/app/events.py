"""Application-level events independent of Qt/PySide."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Any, Mapping

from gello_cr.core.state_machine import Command, WorkflowState


class EventLevel(str, Enum):
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


@dataclass(frozen=True, slots=True)
class AppEvent:
    sequence: int
    kind: str
    level: EventLevel
    state: WorkflowState
    message: str
    command: Command | None = None
    details: Mapping[str, Any] = field(
        default_factory=lambda: MappingProxyType({})
    )

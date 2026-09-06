
"""Read-only device/camera readiness model for the new operator UI."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class OperatorReadiness:
    master_type: str = "unknown"
    master_connected: bool = False
    o6_connected: bool = False
    cameras_running: bool = False
    camera_frames_ready: bool = False
    camera_error: str = ""

    @property
    def devices_ready(self) -> bool:
        return self.master_connected and self.o6_connected

    @classmethod
    def from_mapping(
        cls,
        snapshot: Mapping[str, Any] | None,
    ) -> "OperatorReadiness":
        data = snapshot or {}
        return cls(
            master_type=str(
                data.get("master_type", "unknown")
            ).strip()
            or "unknown",
            master_connected=bool(
                data.get("master_connected", False)
            ),
            o6_connected=bool(data.get("o6_connected", False)),
            cameras_running=bool(
                data.get("cameras_running", False)
            ),
            camera_frames_ready=bool(
                data.get("camera_frames_ready", False)
            ),
            camera_error=str(
                data.get("camera_error", "") or ""
            ).strip(),
        )

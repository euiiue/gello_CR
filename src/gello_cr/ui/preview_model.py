
"""Framework-neutral RGB preview model for the operator UI."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class CameraPreview:
    base_rgb: Any = None
    wrist_rgb: Any = None
    roi_rgb: Any = None
    base_timestamp: float = 0.0
    wrist_timestamp: float = 0.0

    @property
    def has_base(self) -> bool:
        return self.base_rgb is not None

    @property
    def has_wrist(self) -> bool:
        return self.wrist_rgb is not None

    @property
    def has_roi(self) -> bool:
        return self.roi_rgb is not None

    @classmethod
    def from_mapping(
        cls,
        snapshot: Mapping[str, Any] | None,
    ) -> "CameraPreview":
        data = snapshot or {}
        return cls(
            base_rgb=data.get("base_rgb"),
            wrist_rgb=data.get("wrist_rgb"),
            roi_rgb=data.get("roi_rgb"),
            base_timestamp=float(
                data.get("base_timestamp", 0.0) or 0.0
            ),
            wrist_timestamp=float(
                data.get("wrist_timestamp", 0.0) or 0.0
            ),
        )

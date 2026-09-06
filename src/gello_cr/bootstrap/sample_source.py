
"""Thread-safe LeRobot sample source independent of any GUI framework."""

from __future__ import annotations

import threading
import time
from collections.abc import Mapping
from typing import Any


class RecordingSampleSource:
    """Combine TeleopEngine dataset state with the latest three RGB streams.

    The source owns only references to the latest immutable/replaced frame
    objects.  Camera devices are updated explicitly by a later camera service;
    constructing this object performs no hardware I/O.
    """

    def __init__(
        self,
        teleop_engine: Any,
        dataset_config: Mapping[str, Any],
    ) -> None:
        self._teleop_engine = teleop_engine
        self._dataset_config = dataset_config
        self._lock = threading.RLock()
        self._wrist_rgb = None
        self._wrist_timestamp = 0.0
        self._base_rgb = None
        self._base_roi_rgb = None
        self._base_timestamp = 0.0

    def update_wrist(self, image_rgb: Any, timestamp: float) -> None:
        with self._lock:
            self._wrist_rgb = image_rgb
            self._wrist_timestamp = float(timestamp)

    def update_base(
        self,
        image_rgb: Any,
        roi_rgb: Any,
        timestamp: float,
    ) -> None:
        with self._lock:
            self._base_rgb = image_rgb
            self._base_roi_rgb = roi_rgb
            self._base_timestamp = float(timestamp)

    def clear_wrist(self) -> None:
        with self._lock:
            self._wrist_rgb = None
            self._wrist_timestamp = 0.0

    def clear_base(self) -> None:
        with self._lock:
            self._base_rgb = None
            self._base_roi_rgb = None
            self._base_timestamp = 0.0

    def snapshot_ready(self) -> bool:
        cfg = self._dataset_config
        max_age = float(cfg['camera_max_age_s'])
        max_skew = float(cfg['camera_max_skew_s'])
        now = time.monotonic()
        with self._lock:
            return (
                self._wrist_rgb is not None
                and self._base_rgb is not None
                and self._base_roi_rgb is not None
                and now - self._wrist_timestamp <= max_age
                and now - self._base_timestamp <= max_age
                and abs(self._wrist_timestamp - self._base_timestamp) <= max_skew
            )

    def __call__(self) -> dict[str, Any]:
        cfg = self._dataset_config
        sample = self._teleop_engine.dataset_sample()

        with self._lock:
            wrist = self._wrist_rgb
            wrist_timestamp = self._wrist_timestamp
            base = self._base_rgb
            roi = self._base_roi_rgb
            base_timestamp = self._base_timestamp

        if wrist is None:
            raise RuntimeError('腕部 D435 尚无 RGB 图像，请先启动 wrist camera')
        if base is None or roi is None:
            raise RuntimeError('基座 D435 尚无完整图/ROI 图像，请先启动 base camera')

        now = time.monotonic()
        max_age = float(cfg['camera_max_age_s'])
        max_skew = float(cfg['camera_max_skew_s'])
        wrist_age = now - wrist_timestamp
        base_age = now - base_timestamp
        skew = abs(wrist_timestamp - base_timestamp)

        quality = sample.setdefault('quality', {})
        quality.update(
            {
                'camera_skew_s': skew,
                'wrist_age_s': wrist_age,
                'base_age_s': base_age,
                'wrist_timestamp': wrist_timestamp,
                'base_timestamp': base_timestamp,
                'wrist_age_exceeded': float(wrist_age > max_age),
                'base_age_exceeded': float(base_age > max_age),
                'camera_skew_exceeded': float(skew > max_skew),
            }
        )
        sample['image_base_rgb'] = base
        sample['image_wrist_rgb'] = wrist
        sample['image_roi_rgb'] = roi
        return sample

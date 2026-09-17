"""Thread-safe LeRobot sample source independent of any GUI framework."""

from __future__ import annotations

import threading
import time
from collections.abc import Mapping
from typing import Any

from gello_cr.devices.camera_streams import CameraStreamConfig, camera_stream_metadata
from gello_cr.recording.schema import SAMPLE_IMAGE_KEYS


def _copy_image(image: Any) -> Any:
    if image is None:
        return None
    copier = getattr(image, "copy", None)
    return copier() if callable(copier) else image


class RecordingSampleSource:
    """One image cache supplies both the UI and the LeRobot worker."""

    def __init__(
        self,
        teleop_engine: Any,
        dataset_config: Mapping[str, Any],
        *,
        streams: tuple[CameraStreamConfig, ...] = (),
    ) -> None:
        self._teleop_engine = teleop_engine
        self._dataset_config = dataset_config
        self._streams = streams
        self._metadata = camera_stream_metadata(streams) if streams else []
        self._names = tuple(s.name for s in streams) if streams else ("基座", "腕部", "第三路")
        self._lock = threading.RLock()
        self._images = [None, None, None]
        self._timestamps = [0.0, 0.0, 0.0]

    def update_stream(self, index: int, image_rgb: Any, timestamp: float) -> None:
        with self._lock:
            self._images[index] = image_rgb
            self._timestamps[index] = float(timestamp)

    def clear_streams(self) -> None:
        with self._lock:
            self._images = [None, None, None]
            self._timestamps = [0.0, 0.0, 0.0]

    def update_wrist(self, image_rgb: Any, timestamp: float) -> None:
        self.update_stream(1, image_rgb, timestamp)

    def update_base(self, image_rgb: Any, roi_rgb: Any, timestamp: float) -> None:
        # Compatibility with the original base RGB + base ROI producer.
        with self._lock:
            self.update_stream(0, image_rgb, timestamp)
            self.update_stream(2, roi_rgb, timestamp)

    def clear_wrist(self) -> None:
        self.update_stream(1, None, 0.0)

    def clear_base(self) -> None:
        self.update_base(None, None, 0.0)

    def preview_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "base_rgb": _copy_image(self._images[0]),
                "wrist_rgb": _copy_image(self._images[1]),
                "roi_rgb": _copy_image(self._images[2]),
                "base_timestamp": self._timestamps[0],
                "wrist_timestamp": self._timestamps[1],
                "roi_timestamp": self._timestamps[2],
                "stream_names": tuple(s.name for s in self._streams)
                or ("Base RGB", "Wrist RGB", "Base ROI"),
                "stream_descriptions": tuple(s.description for s in self._streams),
            }

    def snapshot_ready(self) -> bool:
        return not self.readiness_error()

    def readiness_error(self) -> str:
        cfg = self._dataset_config
        max_age = float(cfg["camera_max_age_s"])
        max_skew = float(cfg["camera_max_skew_s"])
        now = time.monotonic()
        with self._lock:
            for index in (1, 0, 2):
                name = self._names[index]
                if self._images[index] is None:
                    return f"{name}尚无 RGB 画面，请启动相机"
                age = now - self._timestamps[index]
                if not 0 <= age <= max_age:
                    reason = "帧过期" if age > max_age else "帧时间异常"
                    return (
                        f"{name}相机{reason}（时龄 {age:.2f}s，阈值 {max_age:.2f}s）；"
                        "请停止后重新启动相机"
                    )
            skew = max(self._timestamps) - min(self._timestamps)
            if skew > max_skew:
                return (
                    f"相机画面不同步（相差 {skew:.2f}s，阈值 {max_skew:.2f}s）；"
                    "持续异常时请停止后重新启动相机"
                )
        return ""

    def __call__(self) -> dict[str, Any]:
        cfg = self._dataset_config
        sample = self._teleop_engine.dataset_sample()
        with self._lock:
            images = tuple(self._images)
            timestamps = tuple(self._timestamps)
        for index in (1, 0, 2):
            if images[index] is None:
                raise RuntimeError(f"{self._names[index]}尚无 RGB 图像，请先启动相机")
        now = time.monotonic()
        max_age = float(cfg["camera_max_age_s"])
        skew = max(timestamps) - min(timestamps)
        quality = sample.setdefault("quality", {})
        quality.update(
            camera_skew_s=skew, camera_skew_exceeded=float(skew > float(cfg["camera_max_skew_s"]))
        )
        for name, timestamp in zip(("base", "wrist", "roi"), timestamps, strict=True):
            age = now - timestamp
            quality.update(
                {
                    f"{name}_timestamp": timestamp,
                    f"{name}_age_s": age,
                    f"{name}_age_exceeded": float(not 0 <= age <= max_age),
                }
            )
        sample.update(zip(SAMPLE_IMAGE_KEYS, images, strict=True))
        if self._streams:
            sample["camera_streams"] = self._metadata
        return sample

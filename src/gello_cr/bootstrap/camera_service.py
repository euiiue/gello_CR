
"""Explicit two-camera polling bridge for the recording sample source.

RealSenseRgbDevice already owns one SDK polling thread per physical camera.
This service owns only the cross-camera presentation/data bridge:

- explicit start/stop lifecycle;
- read latest wrist/base snapshots;
- derive the normalized base ROI;
- publish all three RGB streams into RecordingSampleSource.

Construction performs no camera I/O.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Sequence

from gello_cr.recording.image_processing import crop_normalized_roi


@dataclass(frozen=True, slots=True)
class CameraServiceSnapshot:
    running: bool
    wrist_connected: bool
    base_connected: bool
    wrist_timestamp: float
    base_timestamp: float
    error: str


class CameraPollingService:
    """Bridge two physical RGB cameras into one RecordingSampleSource."""

    def __init__(
        self,
        *,
        wrist_camera: Any,
        base_camera: Any,
        sample_source: Any,
        base_roi_norm: Sequence[float],
        poll_hz: float = 30.0,
    ) -> None:
        rate = float(poll_hz)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("poll_hz must be positive and finite")

        roi = tuple(float(value) for value in base_roi_norm)
        if len(roi) != 4 or any(not math.isfinite(value) for value in roi):
            raise ValueError("base_roi_norm must contain four finite values")

        self._wrist_camera = wrist_camera
        self._base_camera = base_camera
        self._sample_source = sample_source
        self._base_roi_norm = roi
        self._period_s = 1.0 / rate

        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._error = ""
        self._wrist_timestamp = 0.0
        self._base_timestamp = 0.0

    @property
    def running(self) -> bool:
        thread = self._thread
        return bool(
            thread is not None
            and thread.is_alive()
            and not self.error
        )

    @property
    def error(self) -> str:
        with self._lock:
            return self._error

    def snapshot(self) -> CameraServiceSnapshot:
        with self._lock:
            error = self._error
            wrist_timestamp = self._wrist_timestamp
            base_timestamp = self._base_timestamp
        return CameraServiceSnapshot(
            running=self.running,
            wrist_connected=bool(
                getattr(self._wrist_camera, "connected", False)
            ),
            base_connected=bool(
                getattr(self._base_camera, "connected", False)
            ),
            wrist_timestamp=float(wrist_timestamp),
            base_timestamp=float(base_timestamp),
            error=error,
        )

    def start(self, timeout: float = 5.0) -> CameraServiceSnapshot:
        if self.running:
            return self.snapshot()

        if not math.isfinite(float(timeout)) or float(timeout) <= 0:
            raise ValueError("camera timeout must be positive and finite")

        self.stop()
        self._stop.clear()
        with self._lock:
            self._error = ""
            self._wrist_timestamp = 0.0
            self._base_timestamp = 0.0

        try:
            wrist = self._wrist_camera.connect(timeout=float(timeout))
            self._sample_source.update_wrist(
                wrist.image_rgb,
                wrist.timestamp,
            )

            base = self._base_camera.connect(timeout=float(timeout))
            roi = crop_normalized_roi(
                base.image_rgb,
                self._base_roi_norm,
            )
            self._sample_source.update_base(
                base.image_rgb,
                roi.image_rgb,
                base.timestamp,
            )
        except Exception as exc:
            self._rollback_start()
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
            raise

        with self._lock:
            self._wrist_timestamp = float(wrist.timestamp)
            self._base_timestamp = float(base.timestamp)

        self._thread = threading.Thread(
            target=self._run,
            name="RecordingCameraBridge",
            daemon=True,
        )
        self._thread.start()
        return self.snapshot()

    def stop(self) -> None:
        self._stop.set()

        thread = self._thread
        if (
            thread is not None
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=2.0)
        errors = []
        if thread is not None and thread.is_alive():
            errors.append(TimeoutError("Camera bridge thread did not stop"))
        else:
            self._thread = None
        for camera in (self._wrist_camera, self._base_camera):
            try:
                camera.close()
            except Exception as exc:
                errors.append(exc)
        self._sample_source.clear_wrist()
        self._sample_source.clear_base()
        with self._lock:
            self._wrist_timestamp = 0.0
            self._base_timestamp = 0.0
        if errors:
            raise ExceptionGroup("Camera shutdown incomplete", errors)

    close = stop

    def _rollback_start(self) -> None:
        self._stop.set()
        for camera in (self._wrist_camera, self._base_camera):
            try:
                camera.close()
            except Exception:
                pass
        self._sample_source.clear_wrist()
        self._sample_source.clear_base()
        self._thread = None

    def _run(self) -> None:
        last_wrist = 0.0
        last_base = 0.0

        try:
            while not self._stop.is_set():
                wrist_error = str(
                    getattr(self._wrist_camera, "error", "") or ""
                )
                base_error = str(
                    getattr(self._base_camera, "error", "") or ""
                )
                if wrist_error:
                    raise RuntimeError(wrist_error)
                if base_error:
                    raise RuntimeError(base_error)

                wrist = self._wrist_camera.latest()
                if (
                    wrist is not None
                    and float(wrist.timestamp) != last_wrist
                ):
                    last_wrist = float(wrist.timestamp)
                    self._sample_source.update_wrist(
                        wrist.image_rgb,
                        last_wrist,
                    )
                    with self._lock:
                        self._wrist_timestamp = last_wrist

                base = self._base_camera.latest()
                if (
                    base is not None
                    and float(base.timestamp) != last_base
                ):
                    last_base = float(base.timestamp)
                    roi = crop_normalized_roi(
                        base.image_rgb,
                        self._base_roi_norm,
                    )
                    self._sample_source.update_base(
                        base.image_rgb,
                        roi.image_rgb,
                        last_base,
                    )
                    with self._lock:
                        self._base_timestamp = last_base

                self._stop.wait(self._period_s)

        except Exception as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
            self._stop.set()

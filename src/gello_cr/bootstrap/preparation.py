
"""Explicit preparation commands for auxiliary devices and cameras."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from gello_cr.app import ApplicationService
from gello_cr.core.state_machine import Command


class OperatorReadinessProvider:
    """Snapshot master/O6/camera readiness without performing hardware I/O."""

    def __init__(
        self,
        *,
        runtime: Any,
        cameras: Any,
    ) -> None:
        self._runtime = runtime
        self._cameras = cameras

    def snapshot(self) -> Mapping[str, Any]:
        master = self._runtime.master_controller
        o6 = self._runtime.o6_controller
        camera = self._cameras.snapshot()

        return {
            "master_type": str(
                getattr(master, "master_type", "unknown")
            ),
            "master_connected": bool(
                getattr(master, "connected", False)
            ),
            "o6_connected": bool(
                getattr(o6, "connected", False)
            ),
            "cameras_running": bool(camera.running),
            "camera_frames_ready": bool(
                self._runtime.sample_source.snapshot_ready()
            ),
            "camera_error": str(camera.error or ""),
        }


class OperatorPreparationBindings:
    """Install ApplicationService handlers for preparation commands."""

    def __init__(
        self,
        service: ApplicationService,
        *,
        teleop_engine: Any,
        recorder: Any,
        cameras: Any,
    ) -> None:
        self._service = service
        self._teleop_engine = teleop_engine
        self._recorder = recorder
        self._cameras = cameras

    def install(self) -> "OperatorPreparationBindings":
        self._service.set_handler(
            Command.PREPARE_DEVICES,
            self._prepare_devices,
        )
        self._service.set_handler(
            Command.START_CAMERAS,
            self._start_cameras,
        )
        self._service.set_handler(
            Command.STOP_CAMERAS,
            self._stop_cameras,
        )
        return self

    def _prepare_devices(
        self,
        _payload: Mapping[str, Any],
    ) -> Any:
        master = self._teleop_engine.roarm
        o6 = self._teleop_engine.o6
        if (
            bool(getattr(master, "connected", False))
            and bool(getattr(o6, "connected", False))
        ):
            return None
        return self._teleop_engine.connect_devices()

    def _start_cameras(
        self,
        payload: Mapping[str, Any],
    ) -> Any:
        timeout = float(payload.get("timeout", 5.0))
        return self._cameras.start(timeout=timeout)

    def _stop_cameras(
        self,
        _payload: Mapping[str, Any],
    ) -> None:
        dataset = self._recorder.snapshot()
        if bool(dataset.get("episode_active", False)):
            raise RuntimeError(
                "Episode 正在录制，必须先结束并保存/丢弃后再停止相机"
            )
        if int(dataset.get("buffered_frames", 0) or 0) > 0:
            raise RuntimeError(
                "Episode 存在待处理帧，必须先保存或丢弃后再停止相机"
            )
        self._cameras.stop()

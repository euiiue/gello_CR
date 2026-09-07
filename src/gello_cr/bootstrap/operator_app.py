
"""Top-level composition object for the new operator application."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from gello_cr.ui.backend import OperatorBackend

from .camera_service import CameraPollingService
from .preparation import (
    OperatorPreparationBindings,
    OperatorReadinessProvider,
)
from .runtime_factory import (
    ConcreteRuntime,
    ConcreteRuntimeFactory,
    RuntimeFactoryPaths,
)


@dataclass(slots=True)
class OperatorApplication:
    runtime: ConcreteRuntime
    backend: OperatorBackend
    cameras: CameraPollingService
    readiness: OperatorReadinessProvider
    preparation_bindings: OperatorPreparationBindings

    _closed_resources: set[str] = field(default_factory=set, init=False)

    def close(self, timeout: float = 1.0) -> None:
        dataset = self.runtime.recorder.snapshot()
        if dataset.get("episode_active") or dataset.get("buffered_frames", 0):
            raise RuntimeError("Episode 未处理：请先停止录制并明确保存或丢弃，再退出")
        if self.backend.command_port.pending_commands:
            raise RuntimeError("命令正在执行，请等待完成后退出；F12 软件停止仍可用")
        errors = []
        steps = [
            ("commands", lambda: self.backend.command_port.close(timeout=timeout)),
            ("runtime", self.runtime.close),
            ("cameras", self.cameras.close),
            ("recorder", self.runtime.recorder.close),
        ]
        for name, close in steps:
            if name in self._closed_resources:
                continue
            try:
                close()
            except Exception as exc:
                exc.add_note(f"shutdown resource: {name}")
                errors.append(exc)
            else:
                self._closed_resources.add(name)
        if errors:
            raise ExceptionGroup("Application shutdown incomplete", errors)
        self.backend.presenter.close()


def build_operator_application(
    repo_root: str | Path,
    *,
    factory: Any | None = None,
) -> OperatorApplication:
    paths = RuntimeFactoryPaths.from_repo(repo_root)
    runtime_factory = factory or ConcreteRuntimeFactory(paths)
    runtime = runtime_factory.build()

    dataset_cfg = runtime.store.data["dataset"]
    cameras = CameraPollingService(
        wrist_camera=runtime.wrist_camera,
        base_camera=runtime.base_camera,
        sample_source=runtime.sample_source,
        base_roi_norm=dataset_cfg["base_roi_norm"],
        poll_hz=30.0,
    )
    readiness = OperatorReadinessProvider(
        runtime=runtime,
        cameras=cameras,
    )

    preview_snapshot = getattr(
        runtime.sample_source,
        "preview_snapshot",
        None,
    )
    if preview_snapshot is None:
        preview_snapshot = lambda: {}

    backend = OperatorBackend.compose(
        teleop_engine=runtime.teleop_engine,
        recorder=runtime.recorder,
        lifecycle=runtime.lifecycle,
        readiness_snapshot=readiness.snapshot,
        preview_snapshot=preview_snapshot,
        episode_metadata={"config": runtime.store.data},
    )

    preparation_bindings = OperatorPreparationBindings(
        backend.service,
        teleop_engine=runtime.teleop_engine,
        recorder=runtime.recorder,
        cameras=cameras,
    ).install()

    return OperatorApplication(
        runtime=runtime,
        backend=backend,
        cameras=cameras,
        readiness=readiness,
        preparation_bindings=preparation_bindings,
    )


"""Top-level composition object for the new operator application."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from gello_cr.devices.camera_streams import resolve_camera_streams
from gello_cr.ui.backend import OperatorBackend

from .camera_service import CameraPollingService
from .dagger import DaggerBindings
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
    dagger: DaggerBindings | None = None

    _closed_resources: set[str] = field(default_factory=set, init=False)

    def close(self, timeout: float = 30.0, *, save_pending: bool = False) -> None:
        dataset = self.runtime.recorder.snapshot()
        if not save_pending and (dataset.get("episode_active") or dataset.get("buffered_frames", 0)):
            raise RuntimeError("Episode 未处理：请先停止录制并明确保存或丢弃，再退出")
        if "commands" not in self._closed_resources:
            self.backend.command_port.stop_accepting()
            self.runtime.teleop_engine.shutdown(close_devices=False)
            self.backend.command_port.close(timeout=timeout)
            self._closed_resources.add("commands")
        if save_pending:
            self.runtime.recorder.stop_episode()
            if self.runtime.recorder.snapshot().get("buffered_frames", 0):
                self.runtime.recorder.save_episode("failure", "退出程序时保留未完成 Episode")
        errors = []
        steps = [
            *([("dagger", self.dagger.close)] if self.dagger is not None else []),
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
    dagger_options: dict | None = None,
) -> OperatorApplication:
    paths = RuntimeFactoryPaths.from_repo(repo_root)
    runtime_factory = factory or ConcreteRuntimeFactory(paths)
    runtime = runtime_factory.build()

    dataset_cfg = runtime.store.data["dataset"]
    cameras = CameraPollingService(
        cameras=runtime.camera_devices,
        streams=resolve_camera_streams(dataset_cfg),
        sample_source=runtime.sample_source,
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

    dagger = DaggerBindings(
        runtime,
        backend.service,
        readiness,
        **{
            "openpi_root": os.environ.get("GELLO_CR_OPENPI_ROOT", str(paths.repo_root)),
            **(dagger_options or {}),
        },
    ).install()

    return OperatorApplication(
        runtime=runtime,
        backend=backend,
        cameras=cameras,
        readiness=readiness,
        preparation_bindings=preparation_bindings,
        dagger=dagger,
    )

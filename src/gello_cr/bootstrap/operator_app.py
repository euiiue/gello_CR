
"""Top-level composition object for the new operator application."""

from __future__ import annotations

from dataclasses import dataclass
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

    def close(self, timeout: float = 1.0) -> None:
        """Close UI/application plumbing and cameras; no implicit robot power cycle."""

        self.cameras.close()
        self.backend.close(timeout=timeout)


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

    backend = OperatorBackend.compose(
        teleop_engine=runtime.teleop_engine,
        recorder=runtime.recorder,
        lifecycle=runtime.lifecycle,
        readiness_snapshot=readiness.snapshot,
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

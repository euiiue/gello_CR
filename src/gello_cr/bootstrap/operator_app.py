
"""Top-level composition object for the new operator application.

Building the application creates runtime objects and application plumbing only.
It does not open robot/master/hand/camera hardware and does not start motion.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from gello_cr.ui.backend import OperatorBackend

from .camera_service import CameraPollingService
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

    def close(self, timeout: float = 1.0) -> None:
        """Close application plumbing and cameras; no implicit robot power cycle."""

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

    backend = OperatorBackend.compose(
        teleop_engine=runtime.teleop_engine,
        recorder=runtime.recorder,
        lifecycle=runtime.lifecycle,
    )

    dataset_cfg = runtime.store.data["dataset"]
    cameras = CameraPollingService(
        wrist_camera=runtime.wrist_camera,
        base_camera=runtime.base_camera,
        sample_source=runtime.sample_source,
        base_roi_norm=dataset_cfg["base_roi_norm"],
        poll_hz=30.0,
    )

    return OperatorApplication(
        runtime=runtime,
        backend=backend,
        cameras=cameras,
    )

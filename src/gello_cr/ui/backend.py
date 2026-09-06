
"""Composition boundary between the application layer and existing runtime objects.

Phase 6.3 deliberately accepts already-constructed runtime objects.  Constructing
an OperatorBackend does not connect CR3A, GELLO, O6, RealSense, power the robot,
start teleoperation, or create a LeRobot dataset session.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Protocol

from gello_cr.app import (
    ApplicationService,
    RuntimeCommandBindings,
    RuntimeLifecycleCallbacks,
)
from gello_cr.core.state_machine import WorkflowState

from .command_port import AsyncApplicationCommandPort
from .presenter import OperatorUiPresenter


class SnapshotRuntime(Protocol):
    def snapshot(self) -> Mapping[str, Any]:
        ...


class SnapshotRecorder(Protocol):
    def snapshot(self) -> Mapping[str, Any]:
        ...


class RuntimeFaultSnapshotBridge:
    """Read TeleopEngine snapshot and synchronize an already-handled fault."""

    def __init__(
        self,
        service: ApplicationService,
        runtime: SnapshotRuntime,
    ) -> None:
        self._service = service
        self._runtime = runtime

    def snapshot(self) -> Mapping[str, Any]:
        snapshot = dict(self._runtime.snapshot())
        runtime_state = str(snapshot.get("state", "")).strip().lower()

        if (
            runtime_state == "fault"
            and self._service.state is not WorkflowState.ESTOP
        ):
            reason = str(
                snapshot.get("last_error")
                or snapshot.get("error")
                or "TeleopEngine runtime fault"
            ).strip()
            self._service.report_external_fault(
                reason,
                {
                    "runtime": "TeleopEngine",
                    "runtime_state": runtime_state,
                },
            )

        return snapshot


@dataclass(slots=True)
class OperatorBackend:
    """Application/UI plumbing around existing runtime and recorder objects.

    Ownership is intentionally narrow:
    - owns ApplicationService, bindings, command port and presenter;
    - does NOT own or automatically close/power-off the injected hardware
      runtime/recorder;
    - close() only stops UI/application plumbing.
    """

    service: ApplicationService
    bindings: RuntimeCommandBindings
    command_port: AsyncApplicationCommandPort
    presenter: OperatorUiPresenter
    runtime_fault_bridge: RuntimeFaultSnapshotBridge
    teleop_engine: Any
    recorder: Any

    @classmethod
    def compose(
        cls,
        *,
        teleop_engine: Any,
        recorder: Any,
        lifecycle: RuntimeLifecycleCallbacks,
        event_capacity: int = 256,
        normal_command_capacity: int = 32,
        safety_command_capacity: int = 8,
    ) -> "OperatorBackend":
        if not hasattr(teleop_engine, "snapshot"):
            raise TypeError("teleop_engine must provide snapshot()")
        if not hasattr(recorder, "snapshot"):
            raise TypeError("recorder must provide snapshot()")

        service = ApplicationService()
        bindings = RuntimeCommandBindings(
            service,
            teleop_engine=teleop_engine,
            recorder=recorder,
            lifecycle=lifecycle,
        ).install()

        runtime_fault_bridge = RuntimeFaultSnapshotBridge(
            service,
            teleop_engine,
        )
        presenter = OperatorUiPresenter(
            service,
            runtime_snapshot=runtime_fault_bridge.snapshot,
            recorder_snapshot=recorder.snapshot,
            event_capacity=event_capacity,
        )
        command_port = AsyncApplicationCommandPort(
            service,
            normal_capacity=normal_command_capacity,
            safety_capacity=safety_command_capacity,
        )

        return cls(
            service=service,
            bindings=bindings,
            command_port=command_port,
            presenter=presenter,
            runtime_fault_bridge=runtime_fault_bridge,
            teleop_engine=teleop_engine,
            recorder=recorder,
        )

    def close(self, timeout: float = 1.0) -> None:
        """Close application/UI plumbing only; never power or move hardware."""

        self.command_port.close(timeout=timeout)
        self.presenter.close()

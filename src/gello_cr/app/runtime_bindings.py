"""Bind the V2 ApplicationService to the existing runtime objects.

This module is the transitional seam between the new UI-independent application
layer and the validated legacy TeleopEngine / LeRobotEpisodeRecorder runtime.

Robot lifecycle callbacks remain explicit because CR3A socket/device creation is
still owned by the legacy Qt application at this migration stage.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from gello_cr.core.state_machine import Command

from .service import ApplicationService

LifecycleCallback = Callable[[], Any]


@dataclass(frozen=True, slots=True)
class RuntimeLifecycleCallbacks:
    """Lifecycle operations still owned outside TeleopEngine.

    Missing callbacks are intentionally not treated as no-ops.  A command whose
    callback is absent fails before the WorkflowStateMachine advances.
    """

    connect: LifecycleCallback | None = None
    disconnect: LifecycleCallback | None = None
    power_on: LifecycleCallback | None = None
    power_off: LifecycleCallback | None = None
    reset_fault: LifecycleCallback | None = None
    reset_estop: LifecycleCallback | None = None


class RuntimeCommandBindings:
    """Register current runtime operations as ApplicationService handlers."""

    def __init__(
        self,
        service: ApplicationService,
        *,
        teleop_engine: Any,
        recorder: Any,
        lifecycle: RuntimeLifecycleCallbacks | None = None,
    ) -> None:
        self.service = service
        self.teleop_engine = teleop_engine
        self.recorder = recorder
        self.lifecycle = lifecycle or RuntimeLifecycleCallbacks()

    def install(self) -> "RuntimeCommandBindings":
        handlers = {
            Command.CONNECT: self._connect,
            Command.DISCONNECT: self._disconnect,
            Command.POWER_ON: self._power_on,
            Command.POWER_OFF: self._power_off,
            Command.START_TELEOP: self._start_teleop,
            Command.STOP_TELEOP: self._stop_teleop,
            Command.START_EPISODE: self._start_episode,
            Command.SAVE_SUCCESS: self._save_success,
            Command.SAVE_FAILURE: self._save_failure,
            Command.DISCARD_EPISODE: self._discard_episode,
            Command.REPORT_FAULT: self._report_fault,
            Command.RESET_FAULT: self._reset_fault,
            Command.EMERGENCY_STOP: self._emergency_stop,
            Command.RESET_ESTOP: self._reset_estop,
        }
        for command, handler in handlers.items():
            self.service.set_handler(command, handler)
        return self

    @staticmethod
    def _require_callback(
        callback: LifecycleCallback | None,
        command: Command,
    ) -> LifecycleCallback:
        if callback is None:
            raise RuntimeError(
                f"{command.name} runtime lifecycle callback is not configured"
            )
        return callback

    @staticmethod
    def _required_text(
        payload: Mapping[str, Any],
        key: str,
    ) -> str:
        value = str(payload.get(key, "")).strip()
        if not value:
            raise ValueError(f"{key} is required")
        return value

    def _connect(self, _payload: Mapping[str, Any]) -> Any:
        return self._require_callback(
            self.lifecycle.connect,
            Command.CONNECT,
        )()

    def _disconnect(self, _payload: Mapping[str, Any]) -> Any:
        return self._require_callback(
            self.lifecycle.disconnect,
            Command.DISCONNECT,
        )()

    def _power_on(self, _payload: Mapping[str, Any]) -> Any:
        return self._require_callback(
            self.lifecycle.power_on,
            Command.POWER_ON,
        )()

    def _power_off(self, _payload: Mapping[str, Any]) -> Any:
        return self._require_callback(
            self.lifecycle.power_off,
            Command.POWER_OFF,
        )()

    def _start_teleop(self, _payload: Mapping[str, Any]) -> Any:
        return self.teleop_engine.start_follow()

    def _stop_teleop(self, payload: Mapping[str, Any]) -> Any:
        reason = str(
            payload.get("reason", "application command")
        ).strip() or "application command"
        return self.teleop_engine.stop_follow(reason)

    def _start_episode(self, payload: Mapping[str, Any]) -> Any:
        task = self._required_text(payload, "task")
        base_root = self._required_text(payload, "base_root")
        metadata = payload.get("metadata")
        if metadata is not None and not isinstance(metadata, Mapping):
            raise TypeError("metadata must be a mapping")
        return self.recorder.start_episode(
            task,
            base_root,
            metadata=dict(metadata or {}),
        )

    def _save_success(self, payload: Mapping[str, Any]) -> Any:
        return self.recorder.save_episode(
            "success",
            str(payload.get("notes", "")),
        )

    def _save_failure(self, payload: Mapping[str, Any]) -> Any:
        return self.recorder.save_episode(
            "failure",
            str(payload.get("notes", "")),
        )

    def _discard_episode(self, _payload: Mapping[str, Any]) -> Any:
        return self.recorder.discard_episode()

    def _report_fault(self, payload: Mapping[str, Any]) -> Any:
        reason = str(
            payload.get("reason", "runtime fault")
        ).strip() or "runtime fault"
        return self.teleop_engine.emergency_stop(
            f"故障停止：{reason}"
        )

    def _emergency_stop(self, payload: Mapping[str, Any]) -> Any:
        reason = str(
            payload.get("reason", "软件紧急停止")
        ).strip() or "软件紧急停止"
        return self.teleop_engine.emergency_stop(reason)

    def _reset_fault(self, _payload: Mapping[str, Any]) -> Any:
        return self._require_callback(
            self.lifecycle.reset_fault,
            Command.RESET_FAULT,
        )()

    def _reset_estop(self, _payload: Mapping[str, Any]) -> Any:
        return self._require_callback(
            self.lifecycle.reset_estop,
            Command.RESET_ESTOP,
        )()

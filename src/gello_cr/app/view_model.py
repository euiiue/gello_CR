
"""Read-only application view model for Qt/PySide presentation.

The builder combines the workflow state, runtime snapshot and recorder snapshot
into one immutable object.  It performs no hardware I/O and dispatches no
commands.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from gello_cr.core.state_machine import WorkflowState

from .service import ApplicationSnapshot
from .workflow_ui import WorkflowUiPolicy, workflow_ui_policy


_WORKFLOW_LABELS = {
    WorkflowState.OFFLINE: "OFFLINE",
    WorkflowState.CONNECTED: "CONNECTED",
    WorkflowState.ROBOT_ENABLED: "ROBOT_ENABLED",
    WorkflowState.TELEOP_RUNNING: "TELEOP_RUNNING",
    WorkflowState.RECORDING: "RECORDING",
    WorkflowState.FAULT: "FAULT",
    WorkflowState.ESTOP: "ESTOP",
}


@dataclass(frozen=True, slots=True)
class ApplicationViewModel:
    workflow_state: WorkflowState
    recovery_state: WorkflowState
    workflow_label: str
    severity: str
    headline: str
    application_error: str
    runtime_state: str
    runtime_error: str
    recording_error: str
    dataset_session_active: bool
    episode_active: bool
    episode_pending: bool
    buffered_frames: int
    saved_episodes: int
    quality_needs_review: bool
    policy: WorkflowUiPolicy


def _text(value: Any) -> str:
    if value is None:
        return ""
    return str(value).strip()


def _runtime_error(snapshot: Mapping[str, Any]) -> str:
    return _text(
        snapshot.get("error")
        or snapshot.get("last_error")
        or ""
    )


def _headline(
    state: WorkflowState,
    *,
    episode_active: bool,
    episode_pending: bool,
    buffered_frames: int,
) -> str:
    if state is WorkflowState.ESTOP:
        return "ESTOP：等待人工复位"
    if state is WorkflowState.FAULT:
        return "FAULT：等待排故 / 复位"
    if state is WorkflowState.RECORDING:
        if episode_active:
            return f"RECORDING：Episode 录制中 · {buffered_frames} frames"
        if episode_pending:
            return (
                "RECORDING：Episode 已停止，等待保存 failure/success 或丢弃"
            )
        return "RECORDING：等待 Episode 状态收敛"
    if state is WorkflowState.TELEOP_RUNNING:
        return "TELEOP_RUNNING：主从跟随运行中"
    if state is WorkflowState.ROBOT_ENABLED:
        return "ROBOT_ENABLED：机器人已使能，跟随未启动"
    if state is WorkflowState.CONNECTED:
        return "CONNECTED：控制器已连接，机器人未使能"
    return "OFFLINE：机器人未连接"


def build_application_view_model(
    application: ApplicationSnapshot,
    runtime: Mapping[str, Any],
    recorder: Mapping[str, Any],
) -> ApplicationViewModel:
    state = application.state
    episode_active = bool(recorder.get("episode_active", False))
    buffered_frames = max(0, int(recorder.get("buffered_frames", 0) or 0))
    episode_pending = buffered_frames > 0 and not episode_active
    quality = recorder.get("quality") or {}
    if not isinstance(quality, Mapping):
        quality = {}

    application_error = _text(application.last_error)
    runtime_error = _runtime_error(runtime)
    recording_error = _text(recorder.get("error"))
    quality_needs_review = bool(quality.get("needs_review", False))

    if (
        state in (WorkflowState.FAULT, WorkflowState.ESTOP)
        or application_error
        or runtime_error
        or recording_error
    ):
        severity = "error"
    elif quality_needs_review or episode_pending:
        severity = "warning"
    else:
        severity = "info"

    return ApplicationViewModel(
        workflow_state=state,
        recovery_state=application.recovery_state,
        workflow_label=_WORKFLOW_LABELS[state],
        severity=severity,
        headline=_headline(
            state,
            episode_active=episode_active,
            episode_pending=episode_pending,
            buffered_frames=buffered_frames,
        ),
        application_error=application_error,
        runtime_state=_text(runtime.get("state")) or "unknown",
        runtime_error=runtime_error,
        recording_error=recording_error,
        dataset_session_active=bool(
            recorder.get("session_active", False)
        ),
        episode_active=episode_active,
        episode_pending=episode_pending,
        buffered_frames=buffered_frames,
        saved_episodes=max(
            0,
            int(recorder.get("saved_episodes", 0) or 0),
        ),
        quality_needs_review=quality_needs_review,
        policy=workflow_ui_policy(
            state,
            episode_active=episode_active,
            episode_pending=episode_pending,
        ),
    )

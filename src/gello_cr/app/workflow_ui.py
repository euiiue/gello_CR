
"""UI workflow policy derived from the application state.

Device readiness, background-operation locks and camera freshness remain
separate UI concerns. This module answers only whether the application
workflow permits an operator action in the current state.
"""

from __future__ import annotations

from dataclasses import dataclass

from gello_cr.core.state_machine import WorkflowState


@dataclass(frozen=True, slots=True)
class WorkflowUiPolicy:
    connect_robot: bool
    disconnect_robot: bool
    power_on: bool
    power_off: bool
    reset_fault: bool
    reset_estop: bool
    start_teleop: bool
    emergency_stop: bool
    start_episode: bool
    stop_episode: bool
    save_success: bool
    save_failure: bool
    discard_episode: bool


def workflow_ui_policy(
    state: WorkflowState,
    *,
    episode_active: bool,
    episode_pending: bool,
) -> WorkflowUiPolicy:
    """Return application-level enablement for workflow controls."""

    active = bool(episode_active)
    pending = bool(episode_pending)

    return WorkflowUiPolicy(
        connect_robot=state is WorkflowState.OFFLINE,
        disconnect_robot=state
        in (
            WorkflowState.CONNECTED,
            WorkflowState.ROBOT_ENABLED,
        ),
        power_on=state is WorkflowState.CONNECTED,
        power_off=state is WorkflowState.ROBOT_ENABLED,
        reset_fault=state is WorkflowState.FAULT,
        reset_estop=state is WorkflowState.ESTOP,
        start_teleop=state is WorkflowState.ROBOT_ENABLED,
        emergency_stop=state
        in (
            WorkflowState.CONNECTED,
            WorkflowState.ROBOT_ENABLED,
            WorkflowState.TELEOP_RUNNING,
            WorkflowState.RECORDING,
            WorkflowState.FAULT,
        ),
        start_episode=(
            state is WorkflowState.TELEOP_RUNNING
            or (
                state is WorkflowState.RECORDING
                and pending
                and not active
            )
        ),
        stop_episode=state is WorkflowState.RECORDING and active,
        save_success=(
            state is WorkflowState.RECORDING
            and pending
            and not active
        ),
        save_failure=(
            state
            in (
                WorkflowState.RECORDING,
                WorkflowState.FAULT,
                WorkflowState.ESTOP,
            )
            and pending
            and not active
        ),
        discard_episode=(
            state
            in (
                WorkflowState.RECORDING,
                WorkflowState.FAULT,
                WorkflowState.ESTOP,
            )
            and pending
            and not active
        ),
    )

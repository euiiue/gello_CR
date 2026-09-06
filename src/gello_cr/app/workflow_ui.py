
"""UI workflow policy derived from the application state.

Device readiness, background-operation locks and camera freshness remain
separate UI concerns.  This module answers only whether the application
workflow permits an operator action in the current state.
"""

from __future__ import annotations

from dataclasses import dataclass

from gello_cr.core.state_machine import WorkflowState


@dataclass(frozen=True, slots=True)
class WorkflowUiPolicy:
    connect_robot: bool
    power_on: bool
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
    """Return application-level enablement for workflow controls.

    `episode_pending` means a stopped Episode still has buffered frames awaiting
    save/discard.  The legacy UI intentionally supports pressing Start in that
    condition: it saves the previous Episode first and then starts the next one.
    """

    active = bool(episode_active)
    pending = bool(episode_pending)

    return WorkflowUiPolicy(
        connect_robot=state is WorkflowState.OFFLINE,
        power_on=state is WorkflowState.CONNECTED,
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


from __future__ import annotations

import pytest

from gello_cr.app.workflow_ui import workflow_ui_policy
from gello_cr.core.state_machine import WorkflowState


def test_offline_only_allows_connect_workflow_control() -> None:
    policy = workflow_ui_policy(
        WorkflowState.OFFLINE,
        episode_active=False,
        episode_pending=False,
    )

    assert policy.connect_robot
    assert not policy.power_on
    assert not policy.start_teleop
    assert not policy.start_episode
    assert not policy.emergency_stop


def test_connected_allows_power_and_estop() -> None:
    policy = workflow_ui_policy(
        WorkflowState.CONNECTED,
        episode_active=False,
        episode_pending=False,
    )

    assert policy.power_on
    assert policy.emergency_stop
    assert not policy.start_teleop


def test_robot_enabled_allows_start_teleop() -> None:
    policy = workflow_ui_policy(
        WorkflowState.ROBOT_ENABLED,
        episode_active=False,
        episode_pending=False,
    )

    assert policy.start_teleop
    assert policy.emergency_stop
    assert not policy.start_episode


def test_teleop_running_allows_episode_start() -> None:
    policy = workflow_ui_policy(
        WorkflowState.TELEOP_RUNNING,
        episode_active=False,
        episode_pending=False,
    )

    assert policy.start_episode
    assert policy.emergency_stop
    assert not policy.save_success
    assert not policy.discard_episode


def test_recording_active_only_allows_episode_stop() -> None:
    policy = workflow_ui_policy(
        WorkflowState.RECORDING,
        episode_active=True,
        episode_pending=False,
    )

    assert policy.stop_episode
    assert not policy.start_episode
    assert not policy.save_success
    assert not policy.save_failure
    assert not policy.discard_episode


def test_stopped_pending_recording_allows_save_discard_and_save_then_next() -> None:
    policy = workflow_ui_policy(
        WorkflowState.RECORDING,
        episode_active=False,
        episode_pending=True,
    )

    assert policy.start_episode
    assert policy.save_success
    assert policy.save_failure
    assert policy.discard_episode
    assert not policy.stop_episode


@pytest.mark.parametrize(
    "safe_state",
    [WorkflowState.FAULT, WorkflowState.ESTOP],
)
def test_safe_state_allows_only_failure_resolution_for_pending_episode(
    safe_state,
) -> None:
    policy = workflow_ui_policy(
        safe_state,
        episode_active=False,
        episode_pending=True,
    )

    assert not policy.start_episode
    assert not policy.save_success
    assert policy.save_failure
    assert policy.discard_episode


def test_fault_allows_escalation_to_estop_but_estop_does_not() -> None:
    fault = workflow_ui_policy(
        WorkflowState.FAULT,
        episode_active=False,
        episode_pending=False,
    )
    estop = workflow_ui_policy(
        WorkflowState.ESTOP,
        episode_active=False,
        episode_pending=False,
    )

    assert fault.emergency_stop
    assert not estop.emergency_stop


def test_fault_without_pending_episode_has_no_episode_resolution_controls() -> None:
    policy = workflow_ui_policy(
        WorkflowState.FAULT,
        episode_active=False,
        episode_pending=False,
    )

    assert not policy.save_failure
    assert not policy.discard_episode


from __future__ import annotations

from gello_cr.app.workflow_ui import workflow_ui_policy
from gello_cr.core.state_machine import WorkflowState


def _policy(state):
    return workflow_ui_policy(
        state,
        episode_active=False,
        episode_pending=False,
    )


def test_connected_exposes_power_on_and_disconnect() -> None:
    policy = _policy(WorkflowState.CONNECTED)

    assert policy.power_on
    assert policy.disconnect_robot
    assert not policy.power_off


def test_robot_enabled_exposes_power_off_and_disconnect() -> None:
    policy = _policy(WorkflowState.ROBOT_ENABLED)

    assert policy.power_off
    assert policy.disconnect_robot
    assert not policy.power_on


def test_fault_exposes_reset_fault_only() -> None:
    policy = _policy(WorkflowState.FAULT)

    assert policy.reset_fault
    assert not policy.reset_estop
    assert not policy.power_on
    assert not policy.power_off


def test_estop_exposes_reset_estop_only() -> None:
    policy = _policy(WorkflowState.ESTOP)

    assert policy.reset_estop
    assert not policy.reset_fault
    assert not policy.power_on
    assert not policy.power_off


def test_offline_does_not_expose_shutdown_or_reset_controls() -> None:
    policy = _policy(WorkflowState.OFFLINE)

    assert not policy.disconnect_robot
    assert not policy.power_off
    assert not policy.reset_fault
    assert not policy.reset_estop

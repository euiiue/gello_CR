import pytest

from gello_cr.core.state_machine import (
    Command,
    InvalidTransition,
    WorkflowState,
    WorkflowStateMachine,
)


def test_normal_collection_workflow() -> None:
    workflow = WorkflowStateMachine()
    assert workflow.apply(Command.CONNECT) is WorkflowState.CONNECTED
    assert workflow.apply(Command.POWER_ON) is WorkflowState.ROBOT_ENABLED
    assert workflow.apply(Command.START_TELEOP) is WorkflowState.TELEOP_RUNNING
    assert workflow.apply(Command.START_EPISODE) is WorkflowState.RECORDING
    assert workflow.apply(Command.SAVE_SUCCESS) is WorkflowState.TELEOP_RUNNING
    assert workflow.apply(Command.START_EPISODE) is WorkflowState.RECORDING
    assert workflow.apply(Command.SAVE_FAILURE) is WorkflowState.TELEOP_RUNNING
    assert workflow.apply(Command.START_EPISODE) is WorkflowState.RECORDING
    assert workflow.apply(Command.DISCARD_EPISODE) is WorkflowState.TELEOP_RUNNING


def test_recording_cannot_start_before_teleop() -> None:
    workflow = WorkflowStateMachine(WorkflowState.ROBOT_ENABLED)
    with pytest.raises(InvalidTransition):
        workflow.apply(Command.START_EPISODE)


def test_estop_from_recording_never_auto_resumes_motion() -> None:
    workflow = WorkflowStateMachine(WorkflowState.RECORDING)
    assert workflow.apply(Command.EMERGENCY_STOP) is WorkflowState.ESTOP
    assert workflow.apply(Command.RESET_ESTOP) is WorkflowState.ROBOT_ENABLED


def test_fault_from_teleop_requires_explicit_restart() -> None:
    workflow = WorkflowStateMachine(WorkflowState.TELEOP_RUNNING)
    assert workflow.apply(Command.REPORT_FAULT) is WorkflowState.FAULT
    assert workflow.apply(Command.RESET_FAULT) is WorkflowState.ROBOT_ENABLED
    assert workflow.apply(Command.START_TELEOP) is WorkflowState.TELEOP_RUNNING


def test_disconnect_is_rejected_while_recording() -> None:
    workflow = WorkflowStateMachine(WorkflowState.RECORDING)
    assert not workflow.can(Command.DISCONNECT)
    with pytest.raises(InvalidTransition):
        workflow.apply(Command.DISCONNECT)

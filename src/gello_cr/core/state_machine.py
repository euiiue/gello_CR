"""Single workflow state machine for the V2 operator application."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class WorkflowState(Enum):
    OFFLINE = auto()
    CONNECTED = auto()
    ROBOT_ENABLED = auto()
    TELEOP_RUNNING = auto()
    RECORDING = auto()
    FAULT = auto()
    ESTOP = auto()


class Command(Enum):
    CONNECT = auto()
    DISCONNECT = auto()
    POWER_ON = auto()
    POWER_OFF = auto()
    START_TELEOP = auto()
    STOP_TELEOP = auto()
    START_EPISODE = auto()
    STOP_EPISODE = auto()
    SAVE_SUCCESS = auto()
    SAVE_FAILURE = auto()
    DISCARD_EPISODE = auto()
    REPORT_FAULT = auto()
    RESET_FAULT = auto()
    EMERGENCY_STOP = auto()
    RESET_ESTOP = auto()


class InvalidTransition(RuntimeError):
    pass


_NORMAL_TRANSITIONS: dict[tuple[WorkflowState, Command], WorkflowState] = {
    (WorkflowState.OFFLINE, Command.CONNECT): WorkflowState.CONNECTED,
    (WorkflowState.CONNECTED, Command.DISCONNECT): WorkflowState.OFFLINE,
    (WorkflowState.CONNECTED, Command.POWER_ON): WorkflowState.ROBOT_ENABLED,
    (WorkflowState.ROBOT_ENABLED, Command.POWER_OFF): WorkflowState.CONNECTED,
    (WorkflowState.ROBOT_ENABLED, Command.DISCONNECT): WorkflowState.OFFLINE,
    (WorkflowState.ROBOT_ENABLED, Command.START_TELEOP): WorkflowState.TELEOP_RUNNING,
    (WorkflowState.TELEOP_RUNNING, Command.STOP_TELEOP): WorkflowState.ROBOT_ENABLED,
    (WorkflowState.TELEOP_RUNNING, Command.START_EPISODE): WorkflowState.RECORDING,
    (WorkflowState.RECORDING, Command.STOP_EPISODE): WorkflowState.RECORDING,
    (WorkflowState.RECORDING, Command.SAVE_SUCCESS): WorkflowState.TELEOP_RUNNING,
    (WorkflowState.RECORDING, Command.SAVE_FAILURE): WorkflowState.TELEOP_RUNNING,
    (WorkflowState.RECORDING, Command.DISCARD_EPISODE): WorkflowState.TELEOP_RUNNING,
}


@dataclass(slots=True)
class WorkflowStateMachine:
    state: WorkflowState = WorkflowState.OFFLINE
    recovery_state: WorkflowState = WorkflowState.OFFLINE

    def can(self, command: Command) -> bool:
        if command is Command.EMERGENCY_STOP:
            return self.state is not WorkflowState.ESTOP
        if command is Command.REPORT_FAULT:
            return self.state not in (WorkflowState.FAULT, WorkflowState.ESTOP)
        if command is Command.RESET_FAULT:
            return self.state is WorkflowState.FAULT
        if command is Command.RESET_ESTOP:
            return self.state is WorkflowState.ESTOP
        if (
            self.state in (WorkflowState.FAULT, WorkflowState.ESTOP)
            and command in (
                Command.SAVE_FAILURE,
                Command.DISCARD_EPISODE,
            )
        ):
            return True
        return (self.state, command) in _NORMAL_TRANSITIONS

    def apply(self, command: Command) -> WorkflowState:
        if not self.can(command):
            raise InvalidTransition(f"{command.name} is not allowed from {self.state.name}")

        if command is Command.EMERGENCY_STOP:
            self.recovery_state = self._safe_recovery_state(self.state)
            self.state = WorkflowState.ESTOP
            return self.state

        if command is Command.REPORT_FAULT:
            self.recovery_state = self._safe_recovery_state(self.state)
            self.state = WorkflowState.FAULT
            return self.state

        if command in (Command.RESET_FAULT, Command.RESET_ESTOP):
            self.state = self.recovery_state
            return self.state

        if (
            self.state in (WorkflowState.FAULT, WorkflowState.ESTOP)
            and command in (
                Command.SAVE_FAILURE,
                Command.DISCARD_EPISODE,
            )
        ):
            return self.state

        self.state = _NORMAL_TRANSITIONS[(self.state, command)]
        return self.state

    @staticmethod
    def _safe_recovery_state(previous: WorkflowState) -> WorkflowState:
        if previous in (WorkflowState.RECORDING, WorkflowState.TELEOP_RUNNING):
            return WorkflowState.ROBOT_ENABLED
        if previous is WorkflowState.ROBOT_ENABLED:
            return WorkflowState.ROBOT_ENABLED
        if previous is WorkflowState.CONNECTED:
            return WorkflowState.CONNECTED
        return WorkflowState.OFFLINE

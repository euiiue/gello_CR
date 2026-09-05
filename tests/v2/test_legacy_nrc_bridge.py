from __future__ import annotations

import teleop_runtime as legacy
from gello_cr.devices.nrc_robot import (
    NrcRobotSession,
    NrcServoTransition,
)


class FakeApi:
    def __init__(self) -> None:
        self.connection_status = {11: 0, 22: -2}

    def get_connection_status(self, fd: int) -> int:
        return self.connection_status[fd]


def test_legacy_nrc_adapter_is_v2_session() -> None:
    api = FakeApi()
    adapter = legacy.NrcRobotAdapter(
        api,
        command_fd=11,
        servo_fd=22,
        robot_num=1,
        motion_mode="servoj",
        movej_velocity=10.0,
        movej_acc=20.0,
        movej_dec=20.0,
        movej_period_s=0.1,
        movej_low_latency=False,
    )

    assert isinstance(adapter, NrcRobotSession)
    assert adapter.api is api
    assert adapter.command_fd == 11
    assert adapter.servo_fd == 22
    assert adapter.robot_num == 1
    assert adapter.motion_mode == "servoj"
    assert adapter.connection_statuses() == {
        "6001": 0,
        "7000": -2,
    }


def test_legacy_transition_name_points_to_v2_type() -> None:
    assert legacy.NrcServoTransition is NrcServoTransition

    transition = legacy.NrcServoTransition(
        initial_state=1,
        final_state=3,
        actions=("设置运行模式", "伺服上电"),
    )
    assert transition.initial_state == 1
    assert transition.final_state == 3
    assert transition.actions == (
        "设置运行模式",
        "伺服上电",
    )


def test_legacy_adapter_inherits_controller_warning_behavior() -> None:
    adapter = legacy.NrcRobotAdapter(
        FakeApi(),
        command_fd=11,
        servo_fd=22,
        robot_num=1,
    )

    assert adapter.controller_job_warning == ""
    adapter.record_controller_message(
        "warning",
        "controller job busy",
        4098,
    )
    assert "4098" in adapter.controller_job_warning
    assert "controller job busy" in adapter.controller_job_warning

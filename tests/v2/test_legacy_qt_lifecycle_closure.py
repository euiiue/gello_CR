
from __future__ import annotations

import ast
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2] / "TEST_INEXBOT.py"


def _source() -> str:
    return SOURCE.read_text(encoding="utf-8")


def _method(name: str) -> str:
    source = _source()
    module = ast.parse(source)
    for node in ast.walk(module):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            segment = ast.get_source_segment(source, node)
            assert segment is not None
            return segment
    raise AssertionError(f"method not found: {name}")


def test_all_remaining_lifecycle_callbacks_are_installed() -> None:
    source = _source()

    assert "disconnect=self._app_disconnect_robot" in source
    assert "power_off=self._app_power_off_robot" in source
    assert "reset_fault=self._app_reset_fault" in source
    assert "reset_estop=self._app_reset_estop" in source


def test_power_off_is_now_an_application_command() -> None:
    method = _method("RobotPowerOFF")

    assert "Command.POWER_OFF" in method
    assert "nrc_adapter.power_off" not in method


def test_clear_error_routes_fault_estop_and_connected_states() -> None:
    method = _method("RobotClearError")

    assert "Command.RESET_FAULT" in method
    assert "Command.RESET_ESTOP" in method
    assert "Command.POWER_ON" in method


def test_reset_callback_verifies_hardware_before_runtime_ack() -> None:
    method = _method("_app_reset_safe_state")

    power = method.index("adapter.power_on()")
    verify = method.index("state != 3")
    ack = method.index("self.teleop_engine.acknowledge_fault()")

    assert power < verify < ack
    assert "start_follow" not in method
    assert "send_servoj" not in method


def test_disconnect_detaches_runtime_then_closes_device() -> None:
    method = _method("_app_disconnect_robot")

    detach = method.index("self.teleop_engine.detach_robot(")
    close = method.index("device.close()")

    assert detach < close
    assert "self.robot1_connected = False" in method
    assert "self.nrc_adapter = None" in method


def test_explicit_disconnect_method_dispatches_application_command() -> None:
    method = _method("RobotDISCONNECT")

    assert "Command.DISCONNECT" in method


def test_legacy_robot_tab_buttons_are_application_gated() -> None:
    method = _method("_apply_application_workflow_gates")

    for name in (
        "pushButtonCONNECT",
        "pushButtonON",
        "pushButtonOFF",
        "pushButtonCLEARERROR",
        "pushButtonFollowStart",
        "pushButtonRobotStop",
    ):
        assert name in method

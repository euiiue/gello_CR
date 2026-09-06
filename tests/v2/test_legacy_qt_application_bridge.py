
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


def test_legacy_qt_imports_application_layer() -> None:
    source = _source()
    assert "ApplicationService" in source
    assert "RuntimeCommandBindings" in source
    assert "RuntimeLifecycleCallbacks" in source
    assert "Command, WorkflowState" in source


def test_legacy_qt_installs_runtime_bindings() -> None:
    source = _source()
    assert "self.app_service = ApplicationService()" in source
    assert "self.app_bindings = RuntimeCommandBindings(" in source
    assert "connect=self._app_require_robot_connected" in source
    assert "power_on=self._app_require_robot_enabled" in source


def test_robot_connect_confirms_application_connected_only_after_success() -> None:
    method = _method("RobotCONNECT")
    assert "self.robot1_connected = True" in method
    assert "self._app_confirm_connected()" in method
    assert method.index("self.robot1_connected = True") < method.index(
        "self._app_confirm_connected()"
    )


def test_power_on_confirms_application_state_after_nrc_success() -> None:
    method = _method("WorkflowPowerOn")
    assert "transition = self.nrc_adapter.power_on()" in method
    assert "self._app_confirm_power_on()" in method
    assert method.index("transition = self.nrc_adapter.power_on()") < method.index(
        "self._app_confirm_power_on()"
    )


def test_follow_start_and_stop_dispatch_commands() -> None:
    start = _method("TeleopFollowStart")
    stop = _method("TeleopFollowStop")
    assert "Command.START_TELEOP" in start
    assert "self.teleop_engine.start_follow" not in start
    assert "Command.STOP_TELEOP" in stop


def test_episode_start_save_discard_use_application_helpers() -> None:
    start = _method("LeRobotEpisodeStart")
    save = _method("LeRobotEpisodeSave")
    discard = _method("LeRobotEpisodeDiscard")
    assert "self._app_start_episode" in start
    assert "self._app_save_episode" in save
    assert "self._app_discard_episode" in discard


def test_software_estop_dispatches_application_command() -> None:
    method = _method("EmergencyStop")
    assert "Command.EMERGENCY_STOP" in method
    assert "self.teleop_engine.emergency_stop" not in method

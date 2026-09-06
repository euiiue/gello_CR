
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


def test_qt_imports_workflow_ui_policy() -> None:
    assert "workflow_ui_policy" in _source()


def test_qt_workflow_gate_uses_application_state() -> None:
    method = _method("_apply_application_workflow_gates")

    assert "self.app_service.state" in method
    assert "workflow_ui_policy(" in method


def test_qt_workflow_gate_is_additive_not_a_replacement_for_device_checks() -> None:
    method = _method("_gate_widget_enabled")

    assert "widget.isEnabled() and bool(allowed)" in method


def test_qt_gates_connect_power_start_and_estop_controls() -> None:
    method = _method("_apply_application_workflow_gates")

    for name in (
        "gello_page_connect_robot",
        "teleop_connect_robot_button",
        "gello_page_power_on",
        "teleop_power_on_button",
        "gello_page_start",
        "teleop_start_button",
        "gello_page_estop",
        "teleop_estop_button",
    ):
        assert name in method


def test_qt_gates_episode_controls_from_application_policy() -> None:
    method = _method("_apply_application_workflow_gates")

    assert "lerobot_start_button" in method
    assert "lerobot_stop_button" in method
    assert "lerobot_save_button" in method
    assert "lerobot_discard_button" in method
    assert "policy.start_episode" in method
    assert "policy.discard_episode" in method


def test_failure_outcome_uses_failure_save_policy() -> None:
    method = _method("_apply_application_workflow_gates")

    assert 'outcome == "failure"' in method
    assert "dataset[\"error\"]" in method
    assert "policy.save_failure" in method


def test_refresh_applies_application_gates_after_existing_device_logic() -> None:
    method = _method("refresh_teleop_ui")

    gate = method.index("self._apply_application_workflow_gates(dataset)")
    legacy_event_drain = method.index(
        "self.teleop_engine.events.get_nowait()"
    )
    assert gate < legacy_event_drain


def test_generic_stop_current_action_is_not_overrestricted_by_workflow_policy() -> None:
    method = _method("_apply_application_workflow_gates")

    # teleop_stop_button/gello_page_stop also stop replay/preset legacy actions,
    # so Phase 5.6 deliberately leaves them under their existing runtime gates.
    assert "teleop_stop_button" not in method
    assert "gello_page_stop" not in method

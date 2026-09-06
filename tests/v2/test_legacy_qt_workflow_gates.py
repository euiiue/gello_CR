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


def test_qt_imports_application_view_model_builder() -> None:
    source = _source()

    assert "build_application_view_model" in source

    # Phase 5.8 deliberately removed direct workflow-policy construction
    # from the legacy Qt layer. The policy is now part of ApplicationViewModel.
    app_import = source.split(
        "from gello_cr.app import (", 1
    )[1].split(")", 1)[0]

    assert "workflow_ui_policy" not in app_import


def test_qt_workflow_gate_uses_view_model_policy() -> None:
    method = _method("_apply_application_workflow_gates")

    assert "policy = view_model.policy" in method

    # Qt no longer recalculates workflow state itself.
    assert "self.app_service.state" not in method
    assert "workflow_ui_policy(" not in method


def test_qt_workflow_gate_remains_additive() -> None:
    method = _method("_gate_widget_enabled")

    # Application policy only narrows the already-computed legacy
    # device/busy/freshness gates.
    assert "widget.isEnabled() and bool(allowed)" in method


def test_qt_gates_connect_power_start_and_estop_controls() -> None:
    method = _method("_apply_application_workflow_gates")

    for name in (
        "gello_page_connect_robot",
        "teleop_connect_robot_button",
        "pushButtonCONNECT",
        "gello_page_power_on",
        "teleop_power_on_button",
        "pushButtonON",
        "gello_page_start",
        "teleop_start_button",
        "pushButtonFollowStart",
        "gello_page_estop",
        "teleop_estop_button",
        "pushButtonRobotStop",
    ):
        assert name in method


def test_qt_gates_episode_controls_from_view_model_policy() -> None:
    method = _method("_apply_application_workflow_gates")

    assert "lerobot_start_button" in method
    assert "lerobot_stop_button" in method
    assert "lerobot_save_button" in method
    assert "lerobot_discard_button" in method

    assert "policy.start_episode" in method
    assert "policy.stop_episode" in method
    assert "policy.save_success" in method
    assert "policy.save_failure" in method
    assert "policy.discard_episode" in method


def test_failure_outcome_uses_view_model_recording_error() -> None:
    method = _method("_apply_application_workflow_gates")

    assert 'outcome == "failure"' in method
    assert "view_model.recording_error" in method
    assert "policy.save_failure" in method

    # Recorder state is now normalized into ApplicationViewModel.
    assert 'dataset["error"]' not in method


def test_refresh_builds_view_model_before_applying_workflow_gates() -> None:
    method = _method("refresh_teleop_ui")

    build = method.index(
        "app_view_model = build_application_view_model("
    )

    gate = method.index(
        "self._apply_application_workflow_gates(app_view_model)"
    )

    legacy_event_drain = method.index(
        "self.teleop_engine.events.get_nowait()"
    )

    assert build < gate < legacy_event_drain


def test_generic_stop_current_action_is_not_overrestricted_by_policy() -> None:
    method = _method("_apply_application_workflow_gates")

    # These controls still also stop replay/preset legacy actions.
    # They must not be restricted solely by the application workflow.
    assert "teleop_stop_button" not in method
    assert "gello_page_stop" not in method

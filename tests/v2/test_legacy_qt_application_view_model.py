
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


def test_qt_imports_view_model_builder() -> None:
    source = _source()

    assert "build_application_view_model" in source
    assert "workflow_ui_policy" not in source.split(
        "from gello_cr.app import (", 1
    )[1].split(")", 1)[0]


def test_refresh_builds_one_application_view_model() -> None:
    method = _method("refresh_teleop_ui")

    assert "app_view_model = build_application_view_model(" in method
    assert "self.app_service.snapshot()" in method
    assert "snapshot," in method
    assert "dataset," in method
    assert "self._latest_application_view_model = app_view_model" in method


def test_workflow_gate_consumes_view_model_policy() -> None:
    method = _method("_apply_application_workflow_gates")

    assert "policy = view_model.policy" in method
    assert "workflow_ui_policy(" not in method
    assert "self.app_service.state" not in method


def test_save_gate_consumes_view_model_recording_error() -> None:
    method = _method("_apply_application_workflow_gates")

    assert "view_model.recording_error" in method
    assert 'dataset["error"]' not in method


def test_main_state_label_uses_application_headline() -> None:
    method = _method("refresh_teleop_ui")

    assert "self.teleop_state_label.setText(app_view_model.headline)" in method


def test_state_tooltip_exposes_recovery_and_runtime_without_hardware_reads() -> None:
    method = _method("refresh_teleop_ui")

    assert "app_view_model.recovery_state.name" in method
    assert "app_view_model.runtime_state" in method
    assert "servo_state()" not in method[
        method.index("app_view_model = build_application_view_model("):
        method.index("self.teleop_state_label.setToolTip(") + 500
    ]

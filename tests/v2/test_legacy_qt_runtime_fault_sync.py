
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


def test_qt_runtime_sync_uses_runtime_state() -> None:
    method = _method("_sync_application_runtime_state")

    assert 'runtime_state = self.teleop_engine.state' in method
    assert 'runtime_state != "fault"' in method
    assert "self.teleop_engine.last_error" in method
    assert "report_external_fault(" in method


def test_qt_runtime_sync_does_not_override_estop() -> None:
    method = _method("_sync_application_runtime_state")

    assert "WorkflowState.ESTOP" in method


def test_refresh_loop_calls_runtime_state_sync() -> None:
    method = _method("refresh_teleop_ui")

    assert "snapshot = self.teleop_engine.snapshot()" in method
    assert "self._sync_application_runtime_state()" in method


def test_runtime_sync_does_not_dispatch_report_fault_again() -> None:
    method = _method("_sync_application_runtime_state")

    assert "app_service.report_external_fault" in method
    assert "app_service.dispatch(Command.REPORT_FAULT" not in method

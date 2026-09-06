
from __future__ import annotations

import pytest


class _RuntimeStateHarness:
    """Minimal behavioral mirror used to document acknowledge semantics."""

    def __init__(self, state: str, error: str = "fault"):
        self._state = state
        self._last_error = error


def test_runtime_source_defines_acknowledge_fault() -> None:
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[2] / "teleop_runtime.py"
    ).read_text(encoding="utf-8")

    assert "def acknowledge_fault(self) -> None:" in source
    assert 'if self._state not in ("fault", "idle")' in source
    assert 'self._state = "idle"' in source
    assert 'self._last_error = ""' in source


def test_acknowledge_fault_source_contains_no_motion_call() -> None:
    import ast
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[2] / "teleop_runtime.py"
    ).read_text(encoding="utf-8")
    module = ast.parse(source)
    method = None
    for node in ast.walk(module):
        if isinstance(node, ast.FunctionDef) and node.name == "acknowledge_fault":
            method = ast.get_source_segment(source, node)
            break
    assert method is not None

    for forbidden in (
        "send_servoj",
        "send_movej",
        "open_servoj",
        "start_follow",
        "power_on",
    ):
        assert forbidden not in method


def test_acknowledge_fault_documents_no_auto_resume() -> None:
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[2] / "teleop_runtime.py"
    ).read_text(encoding="utf-8")

    assert "不会自动恢复主从跟随" in source


def test_acknowledge_fault_rejects_non_idle_active_states_by_source_contract() -> None:
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[2] / "teleop_runtime.py"
    ).read_text(encoding="utf-8")

    assert '("fault", "idle")' in source
    assert "不允许确认故障恢复" in source

from __future__ import annotations

import ast
from pathlib import Path

SOURCE_PATH = Path(__file__).resolve().parents[2] / "TEST_INEXBOT.py"


def _source() -> str:
    return SOURCE_PATH.read_text(encoding="utf-8")


def _module() -> ast.Module:
    return ast.parse(_source())


def _method(name: str) -> ast.FunctionDef:
    for node in ast.walk(_module()):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"method not found: {name}")


def _aa_calls(function: ast.FunctionDef) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(function):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if (
            isinstance(func, ast.Attribute)
            and isinstance(func.value, ast.Name)
            and func.value.id == "aa"
        ):
            names.add(func.attr)
    return names


def _called_names(function: ast.FunctionDef) -> set[str]:
    return {
        node.func.id
        for node in ast.walk(function)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }


def test_robot_connect_uses_cr3a_device_not_vendor_connect_calls() -> None:
    function = _method("RobotCONNECT")
    called = _called_names(function)

    assert "Cr3aConfig" in called
    assert "Cr3aDevice" in called

    vendor_calls = _aa_calls(function)
    assert "connect_robot" not in vendor_calls
    assert "disconnect_robot" not in vendor_calls


def test_robot_connect_keeps_legacy_session_alias_after_device_connect() -> None:
    function = _method("RobotCONNECT")
    source = ast.get_source_segment(_source(), function)
    assert source is not None

    assert "device.connect(timeout=10.0)" in source
    assert "session = device.session" in source
    assert "self.nrc_adapter = session" in source
    assert "self.socketFd = device.command_fd" in source
    assert "self.socketFd_7000 = device.servo_fd" in source


def test_close_event_uses_cr3a_device_close_not_vendor_disconnect() -> None:
    function = _method("closeEvent")
    source = ast.get_source_segment(_source(), function)
    assert source is not None

    assert "cr3a_device.close()" in source
    assert "self.cr3a_device = None" in source
    assert "disconnect_robot" not in _aa_calls(function)

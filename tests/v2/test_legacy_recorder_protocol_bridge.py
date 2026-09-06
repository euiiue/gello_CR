from __future__ import annotations

import ast
from pathlib import Path

SOURCE_PATH = Path(__file__).resolve().parents[2] / "lerobot_recorder.py"


def _source() -> str:
    return SOURCE_PATH.read_text(encoding="utf-8")


def _class_source(name: str) -> str:
    source = _source()
    module = ast.parse(source)
    for node in ast.walk(module):
        if isinstance(node, ast.ClassDef) and node.name == name:
            segment = ast.get_source_segment(source, node)
            assert segment is not None
            return segment
    raise AssertionError(f"class not found: {name}")


def test_root_recorder_imports_protocol_functions_as_legacy_aliases() -> None:
    source = _source()

    assert "read_exact as _read_exact" in source
    assert "receive_packet as _receive_packet" in source
    assert "send_packet as _send_packet" in source


def test_root_recorder_no_longer_defines_protocol_functions() -> None:
    source = _source()

    assert "def _read_exact(" not in source
    assert "def _send_packet(" not in source
    assert "def _receive_packet(" not in source


def test_worker_client_request_keeps_same_send_receive_calls() -> None:
    source = (
        Path(__file__).resolve().parents[2]
        / "src/gello_cr/recording/worker_client.py"
    ).read_text(encoding="utf-8")

    assert "send_packet(self._socket, payload, raw)" in source
    assert "response, _ = receive_packet(self._socket)" in source


def test_worker_client_preserves_timeout_and_late_response_safety_behavior() -> None:
    source = (
        Path(__file__).resolve().parents[2]
        / "src/gello_cr/recording/worker_client.py"
    ).read_text(encoding="utf-8")

    assert "self._socket.settimeout(timeout)" in source
    assert "self._socket.settimeout(previous_timeout)" in source
    assert "A late response must never be mistaken" in source
    assert "self._communication_error" in source

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


def test_root_recorder_imports_moved_worker_client_with_legacy_names() -> None:
    source = _source()

    assert "WorkerClient as _WorkerClient" in source
    assert "WorkerClientError as LeRobotRecorderError" in source


def test_root_recorder_no_longer_defines_worker_client_classes() -> None:
    source = _source()

    assert "class _WorkerClient:" not in source
    assert "class LeRobotRecorderError(RuntimeError):" not in source


def test_episode_recorder_passes_root_script_to_worker_client() -> None:
    source = _class_source("LeRobotEpisodeRecorder")

    assert "client = _WorkerClient(" in source
    assert "self.worker_python" in source
    assert "worker_script=Path(__file__).resolve()" in source


def test_root_worker_main_and_protocol_calls_remain_in_place() -> None:
    source = _source()

    assert "def _worker_main(fd: int)" in source
    assert "sock = socket.socket(fileno=fd)" in source
    assert "request, raw = _receive_packet(sock)" in source
    assert "_send_packet(" in source

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ROOT_SOURCE = ROOT / "lerobot_recorder.py"
EPISODE_SOURCE = ROOT / "src/gello_cr/recording/episode_recorder.py"


def _source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _class_source(path: Path, name: str) -> str:
    source = _source(path)
    module = ast.parse(source)
    for node in ast.walk(module):
        if isinstance(node, ast.ClassDef) and node.name == name:
            segment = ast.get_source_segment(source, node)
            assert segment is not None
            return segment
    raise AssertionError(f"class not found: {name}")


def _function_source(path: Path, name: str) -> str:
    source = _source(path)
    module = ast.parse(source)
    for node in ast.walk(module):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            segment = ast.get_source_segment(source, node)
            assert segment is not None
            return segment
    raise AssertionError(f"function not found: {name}")


def test_root_recorder_imports_moved_worker_client_with_legacy_names() -> None:
    source = _source(ROOT_SOURCE)
    assert "WorkerClient as _WorkerClient" in source
    assert "WorkerClientError as LeRobotRecorderError" in source


def test_root_recorder_no_longer_defines_worker_client_classes() -> None:
    source = _source(ROOT_SOURCE)
    assert "class _WorkerClient:" not in source
    assert "class LeRobotRecorderError(RuntimeError):" not in source


def test_episode_recorder_passes_root_script_to_worker_client() -> None:
    source = _class_source(EPISODE_SOURCE, "LeRobotEpisodeRecorder")
    assert "client = WorkerClient(" in source
    assert "self.worker_python" in source
    assert "worker_script=self.worker_script" in source


def test_root_worker_main_preserves_entrypoint_via_worker_service() -> None:
    source = _function_source(ROOT_SOURCE, "_worker_main")
    assert "return _run_worker_service(fd)" in source

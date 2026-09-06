from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ROOT_RECORDER = ROOT / "lerobot_recorder.py"
EPISODE_RECORDER = ROOT / "src/gello_cr/recording/episode_recorder.py"


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


def test_root_recorder_imports_episode_recorder_compatibility_name() -> None:
    source = _source(ROOT_RECORDER)
    assert (
        "from gello_cr.recording.episode_recorder import "
        "LeRobotEpisodeRecorder"
    ) in source


def test_root_recorder_no_longer_defines_episode_recorder_class() -> None:
    source = _source(ROOT_RECORDER)
    assert "class LeRobotEpisodeRecorder:" not in source


def test_episode_lifecycle_now_lives_in_v2_module() -> None:
    source = _class_source(EPISODE_RECORDER, "LeRobotEpisodeRecorder")
    for method in (
        "start_episode",
        "stop_episode",
        "save_episode",
        "discard_episode",
        "finalize",
        "preserve_failed_session",
        "snapshot",
    ):
        assert f"def {method}(" in source


def test_root_recorder_is_compatibility_facade_plus_worker_cli() -> None:
    source = _source(ROOT_RECORDER)
    assert "def _dataset_features(" in source
    assert "def _worker_main(fd: int) -> int:" in source
    assert "return _run_worker_service(fd)" in source
    assert "def main() -> int:" in source

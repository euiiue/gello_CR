from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ROOT_SOURCE = ROOT / "lerobot_recorder.py"
EPISODE_SOURCE = ROOT / "src/gello_cr/recording/episode_recorder.py"


def _source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_root_recorder_bootstraps_v2_recording_schema_for_worker_process() -> None:
    source = _source(ROOT_SOURCE)
    assert '_V2_SRC_DIR = Path(__file__).resolve().parent / "src"' in source
    assert "validate_recording_sample" in source
    assert "dataset_features as v2_dataset_features" in source


def test_episode_recorder_validation_delegates_to_v2_schema() -> None:
    source = _source(EPISODE_SOURCE)
    module = ast.parse(source)
    method = None
    for node in ast.walk(module):
        if isinstance(node, ast.FunctionDef) and node.name == "_validate_sample":
            method = ast.get_source_segment(source, node)
            break
    assert method is not None
    assert "validate_recording_sample(sample)" in method
    assert "len(state) != 18" not in method
    assert "len(action) != 12" not in method


def test_worker_feature_helper_delegates_to_v2_schema() -> None:
    source = _source(ROOT_SOURCE)
    assert "return v2_dataset_features(height, width)" in source


def test_recorder_no_longer_duplicates_state_action_field_names() -> None:
    source = _source(ROOT_SOURCE) + _source(EPISODE_SOURCE)
    assert "CR5_JOINT_STATE_NAMES =" not in source
    assert "CR5_TCP_STATE_NAMES =" not in source
    assert "O6_STATE_NAMES =" not in source
    assert "ACTION_NAMES =" not in source

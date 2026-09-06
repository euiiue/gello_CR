from __future__ import annotations

import ast
from pathlib import Path

SOURCE_PATH = Path(__file__).resolve().parents[2] / "lerobot_recorder.py"


def _source() -> str:
    return SOURCE_PATH.read_text(encoding="utf-8")


def _method_source(name: str) -> str:
    source = _source()
    module = ast.parse(source)
    for node in ast.walk(module):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            segment = ast.get_source_segment(source, node)
            assert segment is not None
            return segment
    raise AssertionError(f"method not found: {name}")


def test_root_recorder_imports_v2_frame_preparation() -> None:
    source = _source()

    assert "from gello_cr.recording.frame import (" in source
    assert "prepare_recording_frame" in source
    assert "resize_rgb_for_openpi" in source


def test_root_recorder_no_longer_defines_resize_function() -> None:
    source = _source()

    assert "def resize_rgb_for_openpi(" not in source


def test_add_sample_delegates_payload_and_raw_preparation() -> None:
    source = _method_source("_add_sample")

    assert "prepared = prepare_recording_frame(" in source
    assert "client.request(" in source
    assert "prepared.payload" in source
    assert "raw=prepared.raw" in source


def test_add_sample_no_longer_inlines_three_image_byte_join() -> None:
    source = _method_source("_add_sample")

    assert 'b"".join(image.tobytes(order="C") for image in images)' not in source
    assert '"image_base_rgb"' not in source
    assert '"image_wrist_rgb"' not in source
    assert '"image_roi_rgb"' not in source

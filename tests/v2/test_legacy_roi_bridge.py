from __future__ import annotations

import ast
from pathlib import Path

SOURCE_PATH = Path(__file__).resolve().parents[2] / "TEST_INEXBOT.py"


def _source() -> str:
    return SOURCE_PATH.read_text(encoding="utf-8")


def _module() -> ast.Module:
    return ast.parse(_source())


def _method_source(name: str) -> str:
    for node in ast.walk(_module()):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            segment = ast.get_source_segment(_source(), node)
            assert segment is not None
            return segment
    raise AssertionError(f"method not found: {name}")


def test_legacy_qt_no_longer_defines_roi_math_helper() -> None:
    source = _source()

    assert "def _base_roi_bounds" not in source
    assert (
        "from gello_cr.recording.image_processing import "
        "crop_normalized_roi"
    ) in source


def test_base_camera_update_uses_pure_roi_transform_and_keeps_overlay() -> None:
    source = _method_source("update_frame_D435_2")

    assert (
        "crop_normalized_roi(base_rgb, self.base_roi_norm)"
        in source
    )
    assert "roi_crop.bounds_px" in source
    assert "roi_crop.image_rgb" in source
    assert "cv2.rectangle" in source
    assert "self._base_roi_rgb_frame = roi_rgb" in source

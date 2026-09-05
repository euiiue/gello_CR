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


def _method_source(name: str) -> str:
    segment = ast.get_source_segment(_source(), _method(name))
    assert segment is not None
    return segment


def test_legacy_qt_no_longer_imports_or_owns_realsense_sdk_pipeline() -> None:
    source = _source()

    assert "import pyrealsense2" not in source
    assert "self.pipeline =" not in source
    assert "self.pipeline_D435_2 =" not in source
    assert "poll_for_frames" not in source


def test_camera_start_stop_routes_through_v2_devices() -> None:
    wrist = _method_source("D435_1_Start")
    wrist_stop = _method_source("D435_1_Stop")
    base = _method_source("D435_2_Start")
    base_stop = _method_source("D435_2_Stop")

    assert "self.wrist_camera_device.connect" in wrist
    assert "self.wrist_camera_device.close" in wrist_stop
    assert "self.base_camera_device.connect" in base
    assert "self.base_camera_device.close" in base_stop


def test_camera_update_reads_snapshots_and_preserves_recorder_caches() -> None:
    wrist = _method_source("update_frame_D435_1")
    base = _method_source("update_frame_D435_2")

    assert "self.wrist_camera_device.latest()" in wrist
    assert "self._wrist_rgb_frame" in wrist
    assert "self._wrist_rgb_timestamp" in wrist

    assert "self.base_camera_device.latest()" in base
    assert "self._base_rgb_frame" in base
    assert "self._base_roi_rgb_frame" in base
    assert "self._base_rgb_timestamp" in base


def test_close_event_closes_both_v2_camera_devices() -> None:
    source = _method_source("closeEvent")

    assert "self.wrist_camera_device" in source
    assert "self.base_camera_device" in source
    assert "device.close()" in source
    assert "pipeline.stop()" not in source

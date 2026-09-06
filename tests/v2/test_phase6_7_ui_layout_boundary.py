
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _source(path):
    return (ROOT / path).read_text(encoding="utf-8")


def test_main_window_has_three_preview_panels() -> None:
    source = _source("src/gello_cr/ui/main_window.py")

    assert '"Base RGB"' in source
    assert '"Wrist RGB"' in source
    assert '"Base ROI"' in source


def test_main_window_renders_preview_from_presenter_frame() -> None:
    source = _source("src/gello_cr/ui/main_window.py")

    assert "self.render_preview(frame.preview)" in source
    assert "preview.base_rgb" in source
    assert "preview.wrist_rgb" in source
    assert "preview.roi_rgb" in source


def test_main_window_does_not_read_physical_camera_objects() -> None:
    source = _source("src/gello_cr/ui/main_window.py")

    for forbidden in (
        "RealSenseRgbDevice",
        "CameraPollingService",
        "pyrealsense2",
        ".latest()",
        ".wrist_camera",
        ".base_camera",
    ):
        assert forbidden not in source


def test_estop_is_prominent_independent_header_control() -> None:
    source = _source("src/gello_cr/ui/main_window.py")

    assert 'setObjectName("estopButton")' in source
    assert "setMinimumSize(230, 64)" in source
    assert "Command.EMERGENCY_STOP" in source


def test_event_log_is_bounded_and_compact() -> None:
    source = _source("src/gello_cr/ui/main_window.py")

    assert "setMaximumHeight(145)" in source
    assert "setMaximumBlockCount(400)" in source


def test_operator_app_supplies_preview_source_without_starting_camera() -> None:
    source = _source(
        "src/gello_cr/bootstrap/operator_app.py"
    )

    # The concrete RecordingSampleSource exposes preview_snapshot(), while
    # alternate/test sample sources are allowed to fall back to an empty
    # preview.  Either way the preview source is injected through the
    # presentation boundary and camera hardware is never auto-started.
    assert "preview_snapshot = getattr(" in source
    assert '"preview_snapshot"' in source
    assert "preview_snapshot=preview_snapshot" in source
    assert ".cameras.start(" not in source

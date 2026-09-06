
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _source(path):
    return (ROOT / path).read_text(encoding="utf-8")


def test_main_window_never_calls_camera_or_runtime_objects() -> None:
    source = _source("src/gello_cr/ui/main_window.py")

    for forbidden in (
        "CameraPollingService",
        "TeleopEngine",
        "connect_devices()",
        ".cameras.start(",
        ".wrist_camera",
        ".base_camera",
    ):
        assert forbidden not in source


def test_main_window_submits_preparation_commands() -> None:
    source = _source("src/gello_cr/ui/main_window.py")

    assert "Command.PREPARE_DEVICES" in source
    assert "Command.START_CAMERAS" in source
    assert "Command.STOP_CAMERAS" in source


def test_readiness_only_narrows_primary_workflow_buttons() -> None:
    source = _source("src/gello_cr/ui/main_window.py")

    assert "and readiness.devices_ready" in source
    assert "and readiness.camera_frames_ready" in source


def test_operator_app_does_not_start_devices_or_cameras() -> None:
    source = _source(
        "src/gello_cr/bootstrap/operator_app.py"
    )

    assert ".cameras.start(" not in source
    assert ".connect_devices(" not in source
    assert ".power_on(" not in source
    assert ".start_follow(" not in source


def test_preparation_handlers_live_outside_ui_layer() -> None:
    source = _source(
        "src/gello_cr/bootstrap/preparation.py"
    )

    assert "self._teleop_engine.connect_devices()" in source
    assert "self._cameras.start(" in source
    assert "self._cameras.stop()" in source
    assert "PySide6" not in source
    assert "PyQt5" not in source

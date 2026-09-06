
from __future__ import annotations

import ast
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2] / "TEST_INEXBOT.py"


def _source() -> str:
    return SOURCE.read_text(encoding="utf-8")


def _method(name: str) -> str:
    source = _source()
    module = ast.parse(source)
    for node in ast.walk(module):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            segment = ast.get_source_segment(source, node)
            assert segment is not None
            return segment
    raise AssertionError(f"method not found: {name}")


def test_qt_imports_application_event_buffer() -> None:
    source = _source()
    assert "ApplicationEventBuffer" in source


def test_qt_subscriber_only_pushes_to_buffer() -> None:
    source = _source()

    assert "self.app_event_buffer = ApplicationEventBuffer(capacity=256)" in source
    assert "self.app_service.subscribe(" in source
    assert "self.app_event_buffer.push" in source


def test_qt_drain_formats_app_event_on_gui_thread() -> None:
    method = _method("_drain_application_events")

    assert "self.app_event_buffer.drain()" in method
    assert "event.level.value" in method
    assert "event.kind" in method
    assert "event.state.name" in method
    assert "self.teleop_log.appendPlainText(" in method
    assert "self.statusBar().showMessage(" in method


def test_refresh_drains_after_runtime_fault_sync() -> None:
    method = _method("refresh_teleop_ui")

    sync = method.index("self._sync_application_runtime_state()")
    drain = method.index("self._drain_application_events()")
    assert sync < drain


def test_drain_reports_bounded_buffer_overflow() -> None:
    method = _method("_drain_application_events")

    assert "dropped_count" in method
    assert "_app_event_dropped_reported" in method
    assert "事件缓冲已丢弃" in method


def test_application_subscriber_does_not_directly_touch_qt_widgets() -> None:
    source = _source()

    subscription_start = source.index("self._app_event_unsubscribe =")
    subscription_end = source.index("self._build_teleop_tab()", subscription_start)
    block = source[subscription_start:subscription_end]

    assert "self.app_event_buffer.push" in block
    assert "teleop_log" not in block
    assert "statusBar" not in block

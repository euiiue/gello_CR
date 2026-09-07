"""Qt behavior tests; offscreen window, injected backend, no device constructors."""
import os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import pytest
pytest.importorskip('PySide6')
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QMessageBox

from gello_cr.app import ApplicationService
from gello_cr.core.state_machine import Command, WorkflowState, WorkflowStateMachine
from gello_cr.ui.command_port import CallbackCommandPort
from gello_cr.ui.main_window import OperatorMainWindow
from gello_cr.ui.presenter import OperatorUiPresenter


@pytest.fixture
def window():
    app = QApplication.instance() or QApplication([])
    recorder = {'episode_active': False, 'buffered_frames': 0}
    service = ApplicationService()
    requests = []
    presenter = OperatorUiPresenter(service, runtime_snapshot=lambda: {},
        recorder_snapshot=lambda: recorder)
    widget = OperatorMainWindow(presenter, CallbackCommandPort(requests.append))
    widget.show()
    app.processEvents()
    yield widget, recorder, requests, app
    recorder.update(episode_active=False, buffered_frames=0)
    if not presenter.closed:
        widget.close()
    app.processEvents()


@pytest.mark.parametrize('active,frames', [(True,0),(False,3)])
def test_episode_exit_is_blocked_and_does_not_submit_save_or_discard(window, monkeypatch, active, frames):
    widget, recorder, requests, app = window
    warnings = []
    monkeypatch.setattr(QMessageBox, 'warning', lambda *args: warnings.append(args[2]))
    recorder.update(episode_active=active, buffered_frames=frames)
    assert not widget.close()
    assert widget.isVisible() and warnings and requests == []
    assert not widget._presenter.closed


def test_f12_submits_software_estop_through_command_port(window):
    widget, recorder, requests, app = window
    widget.activateWindow()
    widget.setFocus()
    app.processEvents()
    QTest.keyClick(widget, Qt.Key_F12)
    app.processEvents()
    assert [r.command for r in requests] == [Command.EMERGENCY_STOP]


def test_offline_disables_motion_and_episode_controls(window):
    widget, _, requests, _ = window
    assert widget.connect_button.isEnabled()
    assert not widget.power_on_button.isEnabled()
    assert not widget.start_teleop_button.isEnabled()
    assert not widget.episode_start_button.isEnabled()
    assert requests == []

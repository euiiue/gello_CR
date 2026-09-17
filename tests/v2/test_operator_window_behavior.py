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
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: (warnings.append(args[2]), QMessageBox.Cancel)[1])
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


def test_third_preview_refreshes_without_a_new_base_or_wrist_frame(window, monkeypatch):
    from gello_cr.ui.preview_model import CameraPreview

    widget, _, _, _ = window
    rendered = []
    monkeypatch.setattr(widget, '_set_preview_image', lambda label, image, text: rendered.append(image))
    widget.render_preview(CameraPreview(base_timestamp=1, wrist_timestamp=1, roi_timestamp=1))
    rendered.clear()
    image = object()
    widget.render_preview(CameraPreview(
        base_timestamp=1, wrist_timestamp=1, roi_timestamp=2, roi_rgb=image,
        stream_names=('Front', 'Wrist', 'Side camera'),
        stream_descriptions=('a', 'b', 'c'),
    ))
    assert rendered[-1] is image
    assert widget.preview_titles[2].text() == 'Side camera'
    assert widget.preview_titles[2].toolTip() == 'c'


def test_dataset_root_is_submitted_and_locked_only_for_unresolved_episode(window):
    widget, recorder, requests, app = window
    service = widget._presenter._service
    service.dispatch(Command.CONNECT)
    service.dispatch(Command.POWER_ON)
    service.dispatch(Command.START_TELEOP)
    widget._presenter._readiness_snapshot = lambda: {
        'master_connected': True, 'o6_connected': True,
        'cameras_running': True, 'camera_frames_ready': True,
    }
    widget.root_edit.setText('/tmp/new collection folder')
    widget.task_edit.setText('test task')
    widget.refresh_from_presenter()
    assert widget.root_edit.isEnabled()
    assert widget.root_browse_button.isEnabled()
    widget.episode_start_button.click()
    assert requests[-1].payload['base_root'] == '/tmp/new collection folder'
    recorder.update(episode_active=False, buffered_frames=3)
    widget.refresh_from_presenter()
    assert not widget.root_edit.isEnabled()
    assert not widget.root_browse_button.isEnabled()
    recorder.update(buffered_frames=0, session_active=True)
    widget.refresh_from_presenter()
    assert widget.root_edit.isEnabled()
    assert widget.root_edit.text() == '/tmp/new collection folder'


@pytest.mark.parametrize('saved', [39, 40, 100])
def test_episode_start_recovers_after_camera_frames_resume(window, saved):
    widget, recorder, requests, app = window
    service = widget._presenter._service
    service.dispatch(Command.CONNECT)
    service.dispatch(Command.POWER_ON)
    readiness = {
        'master_connected': True, 'o6_connected': True,
        'cameras_running': True, 'camera_frames_ready': False,
        'camera_error': '腕部相机帧过期（13.00s）；请停止后重新启动双相机',
    }
    widget._presenter._readiness_snapshot = lambda: readiness
    recorder.update(saved_episodes=saved, session_active=True)
    widget.refresh_from_presenter()
    assert '请先启动主从跟随' in widget.episode_status.text()

    service.dispatch(Command.START_TELEOP)
    widget.refresh_from_presenter()
    assert not widget.episode_start_button.isEnabled()
    assert '腕部相机帧过期' in widget.episode_status.text()
    assert '请先启动主从跟随' not in widget.episode_status.text()
    assert widget.stop_cameras_button.isEnabled()

    readiness.update(camera_frames_ready=True, camera_error='')
    widget.refresh_from_presenter()
    assert widget.episode_start_button.isEnabled()
    assert widget.camera_status.text() == 'fresh'
    assert '暂不能开始' not in widget.episode_status.text()
    assert f'{saved} saved' in widget.episode_status.text()
    widget.episode_start_button.click()
    assert [r.command for r in requests] == [Command.START_EPISODE]


def test_space_toggles_twice_only_in_dagger_and_ignores_key_repeat(window):
    from PySide6.QtCore import QEvent
    from PySide6.QtGui import QKeyEvent

    widget, _, requests, app = window
    service = widget._presenter._service
    widget.activateWindow()
    widget.task_edit.setFocus()
    widget.task_edit.setText('task')
    widget.task_edit.setCursorPosition(4)
    app.processEvents()
    QTest.keyClick(widget.task_edit, Qt.Key_Space)
    assert widget.task_edit.text() == 'task ' and not requests

    service.dispatch(Command.CONNECT)
    service.dispatch(Command.POWER_ON)
    service.dispatch(Command.START_DAGGER)
    widget.refresh_from_presenter()
    widget.setFocus()
    app.processEvents()
    QTest.keyClick(widget, Qt.Key_Space)
    app.sendEvent(widget, QKeyEvent(QEvent.KeyPress, Qt.Key_Space, Qt.NoModifier, ' ', True))
    QTest.keyClick(widget, Qt.Key_Space)
    assert [r.command for r in requests] == [Command.TOGGLE_INTERVENTION] * 2
    assert not widget.task_edit.isEnabled()
    assert not widget.episode_start_button.isEnabled()
    assert not widget.start_teleop_button.isEnabled()
    QTest.keyClick(widget, Qt.Key_F12)
    assert requests[-1].command is Command.EMERGENCY_STOP

    service.dispatch(Command.EMERGENCY_STOP)
    widget.refresh_from_presenter()
    QTest.keyClick(widget, Qt.Key_Space)
    assert len(requests) == 3
    assert not widget._dagger_shortcut.isEnabled()


def test_dagger_start_requires_fresh_devices_and_submits_selected_fields(window):
    widget, _, requests, _ = window
    service = widget._presenter._service
    service.dispatch(Command.CONNECT)
    service.dispatch(Command.POWER_ON)
    widget.refresh_from_presenter()
    assert not widget.dagger_start_button.isEnabled()
    widget._presenter._readiness_snapshot = lambda: {
        'master_connected': True, 'o6_connected': True,
        'cameras_running': True, 'camera_frames_ready': True,
    }
    widget.dagger_host.setText('192.168.2.11')
    widget.dagger_root.setText('/tmp/expert sessions')
    widget.task_edit.setText('correct this task')
    widget.refresh_from_presenter()
    assert widget.dagger_start_button.isEnabled()
    widget.dagger_start_button.click()
    assert requests[-1].command is Command.START_DAGGER
    assert requests[-1].payload == {
        'host': '192.168.2.11', 'port': 8000, 'round_id': 'round1',
        'task': 'correct this task', 'base_root': '/tmp/expert sessions',
    }

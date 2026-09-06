
"""PySide6 operator window.

The window consumes only presentation objects and CommandPort. It does not
import vendor SDKs, runtime implementations, recorder implementations or device adapters.
"""

from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from gello_cr.app import AppEvent, ApplicationViewModel
from gello_cr.core.state_machine import Command, WorkflowState

from .command_port import CommandPort, CommandRequest
from .presenter import OperatorUiPresenter
from .readiness import OperatorReadiness


class OperatorMainWindow(QMainWindow):
    def __init__(
        self,
        presenter: OperatorUiPresenter,
        command_port: CommandPort,
        *,
        refresh_ms: int = 100,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._presenter = presenter
        self._command_port = command_port
        self.setWindowTitle("GELLO · CR3A · O6 Operator")
        self.resize(1180, 820)

        root = QWidget(self)
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        self.workflow_label = QLabel("OFFLINE")
        self.workflow_detail = QLabel(
            "正在等待 ApplicationViewModel"
        )
        layout.addWidget(self.workflow_label)
        layout.addWidget(self.workflow_detail)

        layout.addWidget(self._build_workflow_group())
        layout.addWidget(self._build_preparation_group())
        layout.addWidget(self._build_episode_group())

        self.event_log = QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setPlaceholderText("Application events")
        layout.addWidget(self.event_log, 1)

        self._timer = QTimer(self)
        self._timer.setInterval(max(20, int(refresh_ms)))
        self._timer.timeout.connect(self.refresh_from_presenter)
        self._timer.start()

        self.refresh_from_presenter()

    def _build_workflow_group(self) -> QGroupBox:
        group = QGroupBox("机器人工作流")
        grid = QGridLayout(group)

        self.connect_button = QPushButton("1. 连接 CR3A")
        self.disconnect_button = QPushButton("断开 CR3A")
        self.power_on_button = QPushButton("3. CR3A 上使能")
        self.power_off_button = QPushButton("CR3A 下使能")
        self.start_teleop_button = QPushButton("4. 开始主从跟随")
        self.stop_teleop_button = QPushButton("停止主从跟随")
        self.reset_fault_button = QPushButton("复位 FAULT")
        self.reset_estop_button = QPushButton("复位 ESTOP")
        self.estop_button = QPushButton("软件紧急停止")

        buttons = (
            (self.connect_button, Command.CONNECT),
            (self.disconnect_button, Command.DISCONNECT),
            (self.power_on_button, Command.POWER_ON),
            (self.power_off_button, Command.POWER_OFF),
            (self.start_teleop_button, Command.START_TELEOP),
            (self.stop_teleop_button, Command.STOP_TELEOP),
            (self.reset_fault_button, Command.RESET_FAULT),
            (self.reset_estop_button, Command.RESET_ESTOP),
            (self.estop_button, Command.EMERGENCY_STOP),
        )
        for button, command in buttons:
            button.clicked.connect(
                lambda _checked=False, cmd=command: self._submit(cmd)
            )

        grid.addWidget(self.connect_button, 0, 0)
        grid.addWidget(self.disconnect_button, 0, 1)
        grid.addWidget(self.power_on_button, 0, 2)
        grid.addWidget(self.power_off_button, 0, 3)
        grid.addWidget(self.start_teleop_button, 1, 0, 1, 2)
        grid.addWidget(self.stop_teleop_button, 1, 2, 1, 2)
        grid.addWidget(self.reset_fault_button, 2, 0)
        grid.addWidget(self.reset_estop_button, 2, 1)
        grid.addWidget(self.estop_button, 2, 2, 1, 2)
        return group

    def _build_preparation_group(self) -> QGroupBox:
        group = QGroupBox("设备准备")
        grid = QGridLayout(group)

        self.prepare_devices_button = QPushButton(
            "2. 连接 GELLO + O6"
        )
        self.start_cameras_button = QPushButton("启动双相机")
        self.stop_cameras_button = QPushButton("停止双相机")
        self.device_status_label = QLabel(
            "Master: -- | O6: --"
        )
        self.camera_status_label = QLabel(
            "Cameras: stopped"
        )

        self.prepare_devices_button.clicked.connect(
            lambda: self._submit(Command.PREPARE_DEVICES)
        )
        self.start_cameras_button.clicked.connect(
            lambda: self._submit(Command.START_CAMERAS)
        )
        self.stop_cameras_button.clicked.connect(
            lambda: self._submit(Command.STOP_CAMERAS)
        )

        grid.addWidget(self.prepare_devices_button, 0, 0)
        grid.addWidget(self.start_cameras_button, 0, 1)
        grid.addWidget(self.stop_cameras_button, 0, 2)
        grid.addWidget(self.device_status_label, 1, 0, 1, 3)
        grid.addWidget(self.camera_status_label, 2, 0, 1, 3)
        return group

    def _build_episode_group(self) -> QGroupBox:
        group = QGroupBox("LeRobot Episode")
        outer = QVBoxLayout(group)

        form = QFormLayout()
        self.task_edit = QLineEdit()
        self.root_edit = QLineEdit()
        form.addRow("Task", self.task_edit)
        form.addRow("Dataset root", self.root_edit)
        outer.addLayout(form)

        row = QHBoxLayout()
        self.episode_start_button = QPushButton("开始 Episode")
        self.episode_stop_button = QPushButton("结束录制")
        self.save_success_button = QPushButton("保存 Success")
        self.save_failure_button = QPushButton("保存 Failure")
        self.discard_button = QPushButton("丢弃 Episode")
        for button in (
            self.episode_start_button,
            self.episode_stop_button,
            self.save_success_button,
            self.save_failure_button,
            self.discard_button,
        ):
            row.addWidget(button)
        outer.addLayout(row)

        self.episode_status = QLabel("0 buffered · 0 saved")
        outer.addWidget(self.episode_status)

        self.episode_start_button.clicked.connect(
            self._start_episode
        )
        self.episode_stop_button.clicked.connect(
            lambda: self._submit(Command.STOP_EPISODE)
        )
        self.save_success_button.clicked.connect(
            lambda: self._submit(Command.SAVE_SUCCESS)
        )
        self.save_failure_button.clicked.connect(
            lambda: self._submit(Command.SAVE_FAILURE)
        )
        self.discard_button.clicked.connect(
            lambda: self._submit(Command.DISCARD_EPISODE)
        )
        return group

    def _start_episode(self) -> None:
        self._submit(
            Command.START_EPISODE,
            {
                "task": self.task_edit.text().strip(),
                "base_root": self.root_edit.text().strip(),
            },
        )

    def _submit(self, command: Command, payload=None) -> None:
        try:
            self._command_port.submit(
                CommandRequest.create(command, payload)
            )
        except Exception as exc:
            self.event_log.appendPlainText(
                f"LOCAL ERROR · {command.name}: "
                f"{type(exc).__name__}: {exc}"
            )
            self.statusBar().showMessage(
                f"{command.name} submit failed: {exc}",
                10000,
            )

    def refresh_from_presenter(self) -> None:
        frame = self._presenter.poll()
        self.render_view_model(frame.view_model)
        self.render_readiness(
            frame.readiness,
            frame.view_model,
        )
        for event in frame.events:
            self.append_event(event)

    def render_view_model(
        self,
        view: ApplicationViewModel,
    ) -> None:
        self.workflow_label.setText(view.workflow_label)
        self.workflow_detail.setText(view.headline)
        self.workflow_detail.setToolTip(
            view.application_error
            or view.runtime_error
            or view.recording_error
            or (
                f"Runtime={view.runtime_state}; "
                f"Recovery={view.recovery_state.name}"
            )
        )

        policy = view.policy
        self.connect_button.setEnabled(policy.connect_robot)
        self.disconnect_button.setEnabled(policy.disconnect_robot)
        self.power_on_button.setEnabled(policy.power_on)
        self.power_off_button.setEnabled(policy.power_off)
        self.start_teleop_button.setEnabled(policy.start_teleop)
        self.stop_teleop_button.setEnabled(
            view.workflow_state is WorkflowState.TELEOP_RUNNING
        )
        self.reset_fault_button.setEnabled(policy.reset_fault)
        self.reset_estop_button.setEnabled(policy.reset_estop)
        self.estop_button.setEnabled(policy.emergency_stop)

        self.episode_start_button.setEnabled(
            policy.start_episode
        )
        self.episode_stop_button.setEnabled(
            policy.stop_episode
        )
        self.save_success_button.setEnabled(
            policy.save_success
        )
        self.save_failure_button.setEnabled(
            policy.save_failure
        )
        self.discard_button.setEnabled(
            policy.discard_episode
        )

        self.episode_status.setText(
            f"{view.buffered_frames} buffered · "
            f"{view.saved_episodes} saved · "
            f"review="
            f"{'yes' if view.quality_needs_review else 'no'}"
        )

    def render_readiness(
        self,
        readiness: OperatorReadiness,
        view: ApplicationViewModel,
    ) -> None:
        self.device_status_label.setText(
            f"{readiness.master_type}: "
            f"{'ready' if readiness.master_connected else 'offline'}"
            " | O6: "
            f"{'ready' if readiness.o6_connected else 'offline'}"
        )

        if readiness.camera_error:
            camera_text = (
                f"Cameras ERROR: {readiness.camera_error}"
            )
        elif readiness.camera_frames_ready:
            camera_text = "Cameras: running · frames fresh"
        elif readiness.cameras_running:
            camera_text = "Cameras: running · waiting fresh frames"
        else:
            camera_text = "Cameras: stopped"
        self.camera_status_label.setText(camera_text)

        state = view.workflow_state
        prepare_allowed = state in (
            WorkflowState.CONNECTED,
            WorkflowState.ROBOT_ENABLED,
        )
        self.prepare_devices_button.setEnabled(
            prepare_allowed and not readiness.devices_ready
        )

        start_camera_allowed = state in (
            WorkflowState.OFFLINE,
            WorkflowState.CONNECTED,
            WorkflowState.ROBOT_ENABLED,
            WorkflowState.TELEOP_RUNNING,
        )
        self.start_cameras_button.setEnabled(
            start_camera_allowed
            and not readiness.cameras_running
        )
        self.stop_cameras_button.setEnabled(
            readiness.cameras_running
            and not view.episode_active
            and not view.episode_pending
        )

        # Readiness can only narrow Application workflow permissions.
        self.power_on_button.setEnabled(
            self.power_on_button.isEnabled()
            and readiness.devices_ready
        )
        self.start_teleop_button.setEnabled(
            self.start_teleop_button.isEnabled()
            and readiness.devices_ready
        )
        self.episode_start_button.setEnabled(
            self.episode_start_button.isEnabled()
            and readiness.camera_frames_ready
        )

    def append_event(self, event: AppEvent) -> None:
        command = (
            event.command.name
            if event.command is not None
            else "-"
        )
        self.event_log.appendPlainText(
            f"#{event.sequence} {event.level.value.upper()} "
            f"[{event.state.name}] {command} · "
            f"{event.kind}: {event.message}"
        )

    def closeEvent(self, event) -> None:  # noqa: N802
        self._timer.stop()
        self._command_port.close(timeout=1.0)
        self._presenter.close()
        super().closeEvent(event)

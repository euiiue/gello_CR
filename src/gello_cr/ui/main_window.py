
"""PySide6 operator window using only presentation and command boundaries."""

from __future__ import annotations

import traceback

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from gello_cr.app import AppEvent, ApplicationViewModel
from gello_cr.core.state_machine import Command, WorkflowState

from .command_port import CommandPort, CommandRequest
from .presenter import OperatorUiPresenter
from .preview_model import CameraPreview
from .readiness import OperatorReadiness


class OperatorMainWindow(QMainWindow):
    def __init__(
        self,
        presenter: OperatorUiPresenter,
        command_port: CommandPort,
        *,
        refresh_ms: int = 100,
        close_callback=None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._close_callback = close_callback
        self._presenter = presenter
        self._command_port = command_port
        self._last_preview_timestamps = (-1.0, -1.0)

        self.setWindowTitle("GELLO · CR3A · O6 Operator")
        self.setMinimumSize(1180, 820)
        self.resize(1440, 940)

        root = QWidget(self)
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)

        layout.addWidget(self._build_header())
        layout.addWidget(self._build_status_strip())
        layout.addWidget(self._build_primary_controls())
        layout.addWidget(self._build_preview_group(), 1)
        layout.addWidget(self._build_episode_group())
        layout.addWidget(self._build_log_group())

        self._apply_operator_style()
        self._estop_shortcut = QShortcut(QKeySequence("F12"), self)
        self._estop_shortcut.activated.connect(
            lambda: self._submit(Command.EMERGENCY_STOP)
        )

        self._timer = QTimer(self)
        self._timer.setInterval(max(20, int(refresh_ms)))
        self._timer.timeout.connect(self.refresh_from_presenter)
        self._timer.start()
        self.refresh_from_presenter()

    def _build_header(self) -> QWidget:
        widget = QWidget(self)
        row = QHBoxLayout(widget)
        row.setContentsMargins(0, 0, 0, 0)

        text = QVBoxLayout()
        self.workflow_label = QLabel("OFFLINE")
        self.workflow_label.setObjectName("workflowTitle")
        self.workflow_detail = QLabel("等待应用状态")
        self.workflow_detail.setObjectName("workflowDetail")
        text.addWidget(self.workflow_label)
        text.addWidget(self.workflow_detail)
        row.addLayout(text, 1)

        self.estop_button = QPushButton("软件紧急停止")
        self.estop_button.setObjectName("estopButton")
        self.estop_button.setMinimumSize(230, 64)
        self.estop_button.clicked.connect(
            lambda: self._submit(Command.EMERGENCY_STOP)
        )
        row.addWidget(self.estop_button)
        return widget

    def _make_status_card(
        self,
        title: str,
    ) -> tuple[QFrame, QLabel]:
        frame = QFrame(self)
        frame.setObjectName("statusCard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(12, 8, 12, 8)
        title_label = QLabel(title)
        title_label.setObjectName("statusCardTitle")
        value = QLabel("--")
        value.setObjectName("statusCardValue")
        layout.addWidget(title_label)
        layout.addWidget(value)
        return frame, value

    def _build_status_strip(self) -> QWidget:
        widget = QWidget(self)
        row = QHBoxLayout(widget)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)

        cr3a_card, self.cr3a_status = self._make_status_card("CR3A")
        master_card, self.master_status = self._make_status_card("GELLO / Master")
        o6_card, self.o6_status = self._make_status_card("O6")
        camera_card, self.camera_status = self._make_status_card("Cameras")

        for card in (
            cr3a_card,
            master_card,
            o6_card,
            camera_card,
        ):
            row.addWidget(card, 1)
        return widget

    def _build_primary_controls(self) -> QGroupBox:
        group = QGroupBox("操作流程")
        grid = QGridLayout(group)

        self.connect_button = QPushButton("1. 连接 CR3A")
        self.prepare_devices_button = QPushButton("2. 连接 GELLO + O6")
        self.power_on_button = QPushButton("3. CR3A 上使能")
        self.start_teleop_button = QPushButton("4. 开始主从跟随")

        self.disconnect_button = QPushButton("断开 CR3A")
        self.power_off_button = QPushButton("CR3A 下使能")
        self.stop_teleop_button = QPushButton("停止主从跟随")
        self.start_cameras_button = QPushButton("启动双相机")
        self.stop_cameras_button = QPushButton("停止双相机")
        self.reset_fault_button = QPushButton("复位 FAULT")
        self.reset_estop_button = QPushButton("复位 ESTOP")

        command_buttons = (
            (self.connect_button, Command.CONNECT),
            (self.prepare_devices_button, Command.PREPARE_DEVICES),
            (self.power_on_button, Command.POWER_ON),
            (self.start_teleop_button, Command.START_TELEOP),
            (self.disconnect_button, Command.DISCONNECT),
            (self.power_off_button, Command.POWER_OFF),
            (self.stop_teleop_button, Command.STOP_TELEOP),
            (self.start_cameras_button, Command.START_CAMERAS),
            (self.stop_cameras_button, Command.STOP_CAMERAS),
            (self.reset_fault_button, Command.RESET_FAULT),
            (self.reset_estop_button, Command.RESET_ESTOP),
        )
        for button, command in command_buttons:
            button.clicked.connect(
                lambda _checked=False, cmd=command: self._submit(cmd)
            )

        grid.addWidget(self.connect_button, 0, 0)
        grid.addWidget(self.prepare_devices_button, 0, 1)
        grid.addWidget(self.power_on_button, 0, 2)
        grid.addWidget(self.start_teleop_button, 0, 3)

        grid.addWidget(self.start_cameras_button, 1, 0)
        grid.addWidget(self.stop_cameras_button, 1, 1)
        grid.addWidget(self.stop_teleop_button, 1, 2)
        grid.addWidget(self.power_off_button, 1, 3)

        grid.addWidget(self.disconnect_button, 2, 0)
        grid.addWidget(self.reset_fault_button, 2, 1)
        grid.addWidget(self.reset_estop_button, 2, 2)
        return group

    def _make_preview_label(
        self,
        title: str,
    ) -> tuple[QWidget, QLabel]:
        container = QWidget(self)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)

        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setObjectName("previewTitle")

        image_label = QLabel("等待相机")
        image_label.setObjectName("previewImage")
        image_label.setAlignment(Qt.AlignCenter)
        image_label.setMinimumSize(260, 195)

        layout.addWidget(title_label)
        layout.addWidget(image_label, 1)
        return container, image_label

    def _build_preview_group(self) -> QGroupBox:
        group = QGroupBox("RGB Preview")
        row = QHBoxLayout(group)

        base_widget, self.base_preview = self._make_preview_label(
            "Base RGB"
        )
        wrist_widget, self.wrist_preview = self._make_preview_label(
            "Wrist RGB"
        )
        roi_widget, self.roi_preview = self._make_preview_label(
            "Base ROI"
        )

        row.addWidget(base_widget, 1)
        row.addWidget(wrist_widget, 1)
        row.addWidget(roi_widget, 1)
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

        self.episode_status = QLabel("未启动数据集")
        outer.addWidget(self.episode_status)

        self.episode_start_button.clicked.connect(self._start_episode)
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

    def _build_log_group(self) -> QGroupBox:
        group = QGroupBox("Application Events")
        layout = QVBoxLayout(group)
        self.event_log = QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumHeight(145)
        self.event_log.document().setMaximumBlockCount(400)
        layout.addWidget(self.event_log)
        return group

    def _apply_operator_style(self) -> None:
        self.setStyleSheet(
            """
            QGroupBox {
                font-weight: 600;
                margin-top: 8px;
                padding-top: 10px;
            }
            QPushButton {
                min-height: 34px;
                padding: 4px 10px;
            }
            QLabel#workflowTitle {
                font-size: 24px;
                font-weight: 700;
            }
            QLabel#workflowDetail {
                font-size: 14px;
            }
            QFrame#statusCard {
                border: 1px solid palette(mid);
                border-radius: 6px;
            }
            QLabel#statusCardTitle {
                font-size: 12px;
            }
            QLabel#statusCardValue {
                font-size: 17px;
                font-weight: 700;
            }
            QPushButton#estopButton {
                font-size: 18px;
                font-weight: 800;
                background: #b3261e;
                color: white;
                border-radius: 8px;
            }
            QPushButton#estopButton:disabled {
                background: #8c8c8c;
                color: #dddddd;
            }
            QLabel#previewTitle {
                font-weight: 600;
            }
            QLabel#previewImage {
                border: 1px solid palette(mid);
                background: #111111;
                color: #d0d0d0;
            }
            """
        )

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
            self.refresh_from_presenter()
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
        pending = getattr(self._command_port, "pending_commands", frozenset())
        if pending:
            for button in self.findChildren(QPushButton):
                if button is not self.estop_button:
                    button.setEnabled(False)
            self.statusBar().showMessage("BUSY · " + ", ".join(sorted(c.name for c in pending)))
        else:
            self.statusBar().clearMessage()
        self.render_preview(frame.preview)
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
        self.reset_fault_button.setEnabled(policy.reset_fault and not (view.episode_active or view.episode_pending))
        self.reset_estop_button.setEnabled(policy.reset_estop and not (view.episode_active or view.episode_pending))
        self.estop_button.setEnabled(policy.emergency_stop)

        self.episode_start_button.setEnabled(policy.start_episode)
        self.episode_stop_button.setEnabled(policy.stop_episode)
        self.save_success_button.setEnabled(policy.save_success)
        self.save_failure_button.setEnabled(policy.save_failure)
        self.discard_button.setEnabled(policy.discard_episode)

        if (
            not view.dataset_session_active
            and not view.episode_active
            and not view.episode_pending
            and view.saved_episodes == 0
        ):
            episode_text = "未启动数据集"
        else:
            episode_text = (
                f"{view.buffered_frames} buffered · "
                f"{view.saved_episodes} saved"
            )
            if view.quality_needs_review:
                episode_text += " · NEEDS REVIEW"
        self.episode_status.setText(episode_text)

    def render_readiness(
        self,
        readiness: OperatorReadiness,
        view: ApplicationViewModel,
    ) -> None:
        state = view.workflow_state
        if state is WorkflowState.OFFLINE:
            cr3a_text = "offline"
        elif state is WorkflowState.CONNECTED:
            cr3a_text = "connected"
        elif state is WorkflowState.ROBOT_ENABLED:
            cr3a_text = "enabled"
        elif state in (
            WorkflowState.TELEOP_RUNNING,
            WorkflowState.RECORDING,
        ):
            cr3a_text = "active"
        elif state is WorkflowState.FAULT:
            cr3a_text = "FAULT"
        else:
            cr3a_text = "ESTOP"

        self.cr3a_status.setText(cr3a_text)
        self.master_status.setText(
            f"{readiness.master_type}: "
            f"{'ready' if readiness.master_connected else 'offline'}"
        )
        self.o6_status.setText(
            "ready" if readiness.o6_connected else "offline"
        )

        if readiness.camera_error:
            camera_text = "ERROR · " + readiness.camera_error[:80]
            self.camera_status.setToolTip(readiness.camera_error)
        elif readiness.camera_frames_ready:
            camera_text = "fresh"
            self.camera_status.setToolTip("")
        elif readiness.cameras_running:
            camera_text = "starting"
            self.camera_status.setToolTip("")
        else:
            camera_text = "stopped"
            self.camera_status.setToolTip("")
        self.camera_status.setText(camera_text)

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

    @staticmethod
    def _pixmap_from_rgb(image_rgb) -> QPixmap | None:
        if image_rgb is None:
            return None

        image = np.asarray(image_rgb)
        if (
            image.ndim != 3
            or image.shape[2] != 3
            or image.dtype != np.uint8
        ):
            return None

        image = np.ascontiguousarray(image)
        height, width = image.shape[:2]
        qimage = QImage(
            image.data,
            width,
            height,
            int(image.strides[0]),
            QImage.Format_RGB888,
        ).copy()
        return QPixmap.fromImage(qimage)

    def _set_preview_image(
        self,
        label: QLabel,
        image_rgb,
        empty_text: str,
    ) -> None:
        pixmap = self._pixmap_from_rgb(image_rgb)
        if pixmap is None:
            label.clear()
            label.setText(empty_text)
            return

        target = label.size()
        if target.width() > 4 and target.height() > 4:
            pixmap = pixmap.scaled(
                target,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        label.setPixmap(pixmap)

    def render_preview(self, preview: CameraPreview) -> None:
        timestamps = (
            preview.base_timestamp,
            preview.wrist_timestamp,
        )
        if timestamps == self._last_preview_timestamps:
            return
        self._last_preview_timestamps = timestamps

        self._set_preview_image(
            self.base_preview,
            preview.base_rgb,
            "Base RGB · waiting",
        )
        self._set_preview_image(
            self.wrist_preview,
            preview.wrist_rgb,
            "Wrist RGB · waiting",
        )
        self._set_preview_image(
            self.roi_preview,
            preview.roi_rgb,
            "Base ROI · waiting",
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
        if self._presenter.closed:
            event.accept()
            return
        view = self._presenter.poll().view_model
        if view.episode_active or view.episode_pending:
            QMessageBox.warning(
                self, "Episode 尚未处理",
                "请先 STOP_EPISODE，再明确保存成功、保存失败或丢弃。当前窗口保持打开。",
            )
            event.ignore()
            return
        try:
            if self._close_callback is not None:
                self._close_callback()
            else:
                self._command_port.close(timeout=1.0)
                self._presenter.close()
        except Exception as exc:
            traceback.print_exception(exc)
            QMessageBox.critical(self, "退出未完成", f"{exc!r}\n请查看终端诊断并重试退出。")
            self.event_log.appendPlainText(f"SHUTDOWN ERROR: {exc!r}")
            event.ignore()
            return
        self._timer.stop()
        event.accept()

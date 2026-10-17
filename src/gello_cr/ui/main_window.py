"""Single Workbench shell: page navigation, snapshots, and application commands."""

from __future__ import annotations

from datetime import datetime

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage, QKeySequence, QPixmap, QShortcut
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QStackedWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from gello_cr import __version__
from gello_cr.app.background import BackgroundJobs
from gello_cr.app.datasets import preview_dataset_name
from gello_cr.app.view_model import command_activity
from gello_cr.core.state_machine import Command, WorkflowState

from .command_port import CommandRequest
from .emergency_shortcut import EmergencyStopFilter
from .pages.collection import CollectionPage
from .pages.dataset_manager import DatasetManagerPage
from .pages.robot_status import RobotStatusPage
from .planning_panel import PlanningPanel
from .theme import WORKBENCH_STYLESHEET, set_state
from .trajectory_panel import TrajectoryPanel


class OperatorMainWindow(QMainWindow):
    shutdown_finished = Signal(object)

    def __init__(
        self,
        presenter,
        command_port,
        *,
        refresh_ms=100,
        close_callback=None,
        settings_factory=None,
        parent=None,
    ):
        super().__init__(parent)
        self._jobs = BackgroundJobs()
        self._presenter, self._command_port = presenter, command_port
        self._close_callback, self._settings_factory = close_callback, settings_factory
        self._closing = self._shutdown_complete = self._settings_restart_required = False
        self._last_preview_timestamps = (-1.0, -1.0, -1.0)
        self._last_preview_sizes = ()
        self._shown_batch = ""
        self.settings_editor = None
        self._local_error = ""
        self.setWindowTitle(f"Robot Learning Workbench v{__version__}")
        self.setMinimumSize(1180, 820)
        self.resize(1440, 940)
        root = QWidget()
        root.setObjectName("workbenchRoot")
        self.setCentralWidget(root)
        shell = QHBoxLayout(root)
        shell.setContentsMargins(12, 12, 12, 8)
        shell.setSpacing(14)
        content = QWidget()
        shell.addWidget(content, 1)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        header_panel = QFrame()
        header_panel.setObjectName("headerPanel")
        header = QHBoxLayout(header_panel)
        header.setContentsMargins(18, 12, 12, 12)
        header.setSpacing(10)
        title = QVBoxLayout()
        eyebrow = QLabel("ROBOT LEARNING / OPERATOR")
        eyebrow.setObjectName("eyebrow")
        title.addWidget(eyebrow)
        self.workflow_label = QLabel("数据采集")
        self.workflow_label.setObjectName("workflowTitle")
        self.workflow_detail = QLabel("OFFLINE · 手动连接设备后开始")
        self.workflow_detail.setWordWrap(True)
        self.workflow_detail.setObjectName("mutedText")
        title.addWidget(self.workflow_label)
        title.addWidget(self.workflow_detail)
        header.addLayout(title, 1)
        self.home_button = QPushButton("回零 HOME")
        self.stop_motion_button = QPushButton("停止运动")
        self.stop_motion_button.setProperty("role", "danger")
        self.home_button.setToolTip("停止当前运动后返回配置的 HOME；到位后保持停止")
        self.stop_motion_button.setToolTip("停止当前运动；数据录制需在采集区单独结束")
        self.estop_button = QPushButton("软件紧急停止  F12")
        self.estop_button.setObjectName("estopButton")
        self.estop_button.setMinimumSize(230, 64)
        for button, command in (
            (self.home_button, Command.RETURN_HOME),
            (self.stop_motion_button, Command.STOP_MOTION),
            (self.estop_button, Command.EMERGENCY_STOP),
        ):
            button.clicked.connect(lambda checked=False, cmd=command: self._submit(cmd))
            header.addWidget(button)
        layout.addWidget(header_panel)
        state_panel = QFrame()
        state_panel.setObjectName("statePanel")
        state_row = QHBoxLayout(state_panel)
        state_row.setContentsMargins(12, 9, 12, 9)
        self.robot_badge = QLabel()
        self.motion_badge = QLabel()
        self.recording_badge = QLabel()
        for badge in (self.robot_badge, self.motion_badge, self.recording_badge):
            badge.setObjectName("statusPill")
            state_row.addWidget(badge)
        self.state_headline = QLabel()
        self.state_headline.setObjectName("stateHeadline")
        self.state_headline.setWordWrap(True)
        self.state_headline.setMinimumWidth(0)
        state_row.addWidget(self.state_headline, 1)
        layout.addWidget(state_panel)
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(176)
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(10, 22, 10, 16)
        brand = QLabel("WORKBENCH")
        brand.setObjectName("brandTitle")
        brand.setMargin(8)
        sidebar_layout.addWidget(brand)
        version = QLabel(f"ROBOT LEARNING  /  v{__version__}")
        version.setObjectName("brandSubtitle")
        version.setMargin(8)
        sidebar_layout.addWidget(version)
        sidebar_layout.addSpacing(22)
        self.navigation = QListWidget()
        self.navigation.setObjectName("navigation")
        self.navigation.addItems(
            ["数据采集", "轨迹与回放", "精确走点", "数据集管理", "机器人状态", "设置"]
        )
        sidebar_layout.addWidget(self.navigation, 1)
        note = QLabel("实验操作台\n手动连接 · 明确执行")
        note.setObjectName("sidebarNote")
        note.setMargin(8)
        sidebar_layout.addWidget(note)
        shell.insertWidget(0, sidebar)
        self.pages = QStackedWidget()
        self.collection = CollectionPage(self._submit)
        self.trajectory_panel = TrajectoryPanel(self._submit)
        self.planning_panel = PlanningPanel(self._submit)
        self.dataset_page = DatasetManagerPage(self._submit)
        self.robot_page = RobotStatusPage(self._submit)
        self.settings_page = QWidget()
        self.settings_layout = QVBoxLayout(self.settings_page)
        self.settings_note = QLabel("设置仅在设备和数据集会话未运行时编辑；保存后重新启动生效。")
        self.settings_note.setWordWrap(True)
        self.settings_layout.addWidget(self.settings_note)
        self.settings_layout.addStretch()
        for page in (
            self.collection,
            self.trajectory_panel,
            self.planning_panel,
            self.dataset_page,
            self.robot_page,
            self.settings_page,
        ):
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(page)
            self.pages.addWidget(scroll)
        self.navigation.currentRowChanged.connect(self._navigate)
        body.addWidget(self.pages, 1)
        layout.addLayout(body, 1)
        self.notification = QLabel("")
        self.notification.setObjectName("mutedText")
        self.notification.setMinimumHeight(24)
        self.notification.setTextFormat(Qt.PlainText)
        self.notification.setWordWrap(True)
        self.notification.setTextInteractionFlags(Qt.TextSelectableByMouse)
        footer = QHBoxLayout()
        footer.addWidget(self.notification, 1)
        self.recovery_button = QPushButton("查看未完成批次")
        self.recovery_button.setProperty("role", "quiet")
        self.recovery_button.setVisible(False)
        self.recovery_button.clicked.connect(lambda: self.navigation.setCurrentRow(3))
        footer.addWidget(self.recovery_button)
        self.log_toggle = QToolButton()
        self.log_toggle.setText("运行日志  ▾")
        self.log_toggle.setCheckable(True)
        footer.addWidget(self.log_toggle)
        layout.addLayout(footer)
        self.event_log = QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumHeight(145)
        self.event_log.document().setMaximumBlockCount(400)
        self.event_log.setVisible(False)
        self.log_toggle.toggled.connect(self.event_log.setVisible)
        self.log_toggle.toggled.connect(
            lambda expanded: self.log_toggle.setText("运行日志  ▴" if expanded else "运行日志  ▾")
        )
        layout.addWidget(self.event_log)
        self.summary_status = QLabel()
        self.summary_status.setWordWrap(True)
        self.summary_status.setMinimumWidth(0)
        self.statusBar().addPermanentWidget(self.summary_status, 1)
        self.setStyleSheet(WORKBENCH_STYLESHEET)
        self._emergency_filter = EmergencyStopFilter(
            self, lambda: self._submit(Command.EMERGENCY_STOP)
        )
        self._dagger_shortcut = QShortcut(QKeySequence("Space"), self)
        self._dagger_shortcut.setAutoRepeat(False)
        self._dagger_shortcut.activated.connect(lambda: self._submit(Command.TOGGLE_INTERVENTION))
        self.shutdown_finished.connect(self._finish_shutdown)
        self._timer = QTimer(self)
        self._timer.setInterval(max(50, int(refresh_ms)))
        self._timer.timeout.connect(self.refresh_from_presenter)
        self.navigation.setCurrentRow(0)
        self._timer.start()
        self.refresh_from_presenter()

    def _navigate(self, index):
        self.pages.setCurrentIndex(index)
        if 0 <= index < self.navigation.count():
            self.workflow_label.setText(self.navigation.item(index).text())
            self.workflow_detail.setText(
                (
                    "准备设备，观察画面，记录每一次有效操作",
                    "记录示教轨迹，预览并执行回放",
                    "先规划预览，再明确执行目标运动",
                    "浏览数据集，校验质量，处理未完成批次",
                    "查看设备反馈与故障，明确复位后保持停止",
                    "设备连接、运动与采集参数",
                )[index]
            )
        if index == 5 and self.settings_editor is None:
            self._open_settings()
            self.refresh_from_presenter()

    def _open_settings(self):
        if self._settings_factory is None:
            self.settings_note.setText("此启动方式未配置设置存储；其余离线页面可正常使用。")
            return
        from .settings_dialog import OperatorSettingsDialog

        try:
            editor = OperatorSettingsDialog(self._settings_factory(), self, embedded=True)
        except (ValueError, OSError, RuntimeError) as exc:
            self._local_error = f"无法读取设置：{exc}"
            self.notification.setText(self._local_error)
            return
        self.settings_editor = editor
        self.settings_layout.insertWidget(1, editor, 1)
        editor.saved_changes.connect(self._settings_saved)
        editor.reload_requested.connect(self._reload_settings)

    def _reload_settings(self):
        old = self.settings_editor
        if old:
            try:
                old.close_jobs(timeout=0)
            except TimeoutError:
                self.notification.setText("相机发现仍在执行，请完成后再重新加载设置。")
                return
            self.settings_editor = None
            self.settings_layout.removeWidget(old)
            old.deleteLater()
        self._open_settings()

    def _settings_saved(self):
        self._settings_restart_required = True
        self.notification.setText("参数已保存，请重新启动后连接设备。")
        self.refresh_from_presenter()

    def _submit(self, command, payload=None):
        if (
            command is Command.RESET_ESTOP
            and QMessageBox.question(
                self,
                "解除软件急停",
                "确认现场允许解除软件急停？复位后保持停止。",
                QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            != QMessageBox.Yes
        ):
            return
        try:
            self._command_port.submit(CommandRequest.create(command, payload))
            self._local_error = ""
        except Exception as exc:
            self._local_error = f"{command.name}: {exc}"
            self.notification.setText(self._local_error)
            self.event_log.appendPlainText(f"ERROR · {command.name}: {exc}")
        self.refresh_from_presenter()

    def refresh_from_presenter(self):
        if self._presenter.closed:
            self._timer.stop()
            return
        frame = self._presenter.poll()
        view, readiness = frame.view_model, frame.readiness
        pending = getattr(self._command_port, "pending_commands", frozenset())
        busy = bool(pending)
        dataset = dict(frame.dataset)
        dataset["dataset_name_preview"] = preview_dataset_name(
            self.collection.task_edit.text(), view.dataset_format
        )
        self.collection.render(view, readiness, dataset, busy)
        self.trajectory_panel.render(frame.trajectory, view, readiness, busy)
        self.planning_panel.render(frame.planning, view, busy)
        self.dataset_page.render(frame.catalog, dataset, busy)
        if not self.dataset_page.root_edit.text():
            self.dataset_page.root_edit.setText(self.collection.root_edit.text())
        self.robot_page.render(view, readiness, frame.diagnostics, busy)
        self._dagger_shortcut.setEnabled(
            view.workflow_state is WorkflowState.DAGGER_RUNNING and not busy and not self._closing
        )
        self.home_button.setEnabled(
            view.actions[Command.RETURN_HOME]
            and not pending.intersection(
                {
                    Command.RETURN_HOME,
                    Command.STOP_MOTION,
                    Command.EMERGENCY_STOP,
                    Command.REPORT_FAULT,
                }
            )
        )
        self.stop_motion_button.setEnabled(view.actions[Command.STOP_MOTION])
        self.estop_button.setEnabled(view.actions[Command.EMERGENCY_STOP])
        headline = view.activity_text if view.active_command else view.headline
        for prefix, label in (
            ("OFFLINE：", ""),
            ("CONNECTED：", ""),
            ("ROBOT_ENABLED：", ""),
            ("TELEOP_RUNNING：", ""),
            ("ESTOP：", "软件急停 · "),
            ("FAULT：", "设备故障 · "),
            ("RECORDING：", "录制中 · "),
            ("PENDING：", "待保存 · "),
            (" frames", " 帧"),
        ):
            headline = headline.replace(prefix, label)
        self.state_headline.setText(headline)
        set_state(self.state_headline, view.severity)
        robot_names = {
            "OFFLINE": "未连接",
            "CONNECTED": "已连接",
            "ENABLED": "已使能",
            "FAULT": "故障",
            "ESTOP": "急停锁定",
        }
        motion_names = {
            "IDLE": "待机",
            "TELEOP": "遥操跟随",
            "REPLAY": "轨迹回放",
            "POINT_MOVE": "精确走点",
            "HOME": "回零",
            "POLICY": "模型控制",
        }
        recording_names = {
            "IDLE": "未录制",
            "RECORDING": "录制中",
            "PENDING": "待保存",
            "FINALIZING": "合并中",
            "RECOVERING": "恢复中",
        }
        for badge, prefix, state, names, color in (
            (
                self.robot_badge,
                "机器人",
                view.robot_state.name,
                robot_names,
                "error"
                if view.robot_state.name in ("FAULT", "ESTOP")
                else "success"
                if view.robot_state.name == "ENABLED"
                else "neutral",
            ),
            (
                self.motion_badge,
                "运动",
                view.motion_state.name,
                motion_names,
                "success" if view.motion_running else "neutral",
            ),
            (
                self.recording_badge,
                "采集",
                view.recording_state.name,
                recording_names,
                "warning"
                if view.episode_pending
                else "success"
                if view.episode_active
                else "neutral",
            ),
        ):
            badge.setText(f"{prefix} · {names.get(state, state)}")
            set_state(badge, color)
        master = (
            "FAULT"
            if readiness.master_error
            else "CONNECTED"
            if readiness.master_connected
            else "OFFLINE"
        )
        hand = (
            "FAULT" if readiness.o6_error else "CONNECTED" if readiness.o6_connected else "OFFLINE"
        )
        cameras = (
            "READY"
            if readiness.camera_frames_ready
            else "STALE"
            if readiness.cameras_running
            else "OFFLINE"
        )
        fault = (
            "YES"
            if view.runtime_error
            or view.application_error
            or readiness.master_error
            or readiness.o6_error
            or view.robot_state.name in ("FAULT", "ESTOP")
            else "NONE"
        )
        self.summary_status.setText(
            f"Robot {view.robot_state.name} | {readiness.master_type.upper()} {master} | O6 {hand}"
            f" | Cameras {cameras} | Motion {view.motion_state.name}"
            f" | Recording {view.recording_state.name} | Fault {fault}"
        )
        error = (
            view.runtime_error
            or view.application_error
            or view.recording_error
            or readiness.camera_error
            or readiness.master_error
            or readiness.o6_error
            or self._local_error
        )
        self.notification.setText(
            error
            or ("正在处理：" + ", ".join(command_activity(c) for c in pending) if busy else "")
        )
        self.notification.setStyleSheet("color: #b3261e;" if error else "")
        settings_allowed = view.settings_editable and not busy
        if self.settings_editor:
            self.settings_editor.setEnabled(settings_allowed)
        if self._settings_restart_required:
            self.state_headline.setText("参数已保存 · 请关闭并重新启动程序后连接设备")
            set_state(self.state_headline, "warning")
            self.collection.connect_button.setEnabled(False)
            self.collection.start_cameras_button.setEnabled(False)
        batch_id = dataset.get("batch_id", "")
        if batch_id and batch_id != self._shown_batch:
            self.collection.task_edit.setText(dataset.get("batch_task", ""))
            self.collection.description_edit.setText(dataset.get("task_description", ""))
            self.collection.root_edit.setText(dataset.get("output_root", ""))
        self._shown_batch = batch_id
        recoverable = dataset.get("recoverable_batches", ())
        self.recovery_button.setVisible(bool(recoverable))
        self.recovery_button.setText(f"{len(recoverable)} 个未完成批次 · 查看")
        self.render_preview(frame.preview)
        for event in frame.events:
            stamp = datetime.fromtimestamp(event.occurred_at).strftime("%H:%M:%S")
            self.event_log.appendPlainText(f"{stamp} {event.level.value.upper()} · {event.message}")

    @staticmethod
    def _pixmap_from_rgb(image_rgb):
        if image_rgb is None:
            return None
        image = np.asarray(image_rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            return None
        image = np.ascontiguousarray(image)
        height, width = image.shape[:2]
        return QPixmap.fromImage(
            QImage(image.data, width, height, int(image.strides[0]), QImage.Format_RGB888).copy()
        )

    def render_preview(self, preview):
        labels = (
            self.collection.base_preview,
            self.collection.wrist_preview,
            self.collection.roi_preview,
        )
        for i, title in enumerate(self.collection.preview_titles):
            title.setText(preview.stream_names[i])
            title.setToolTip(preview.stream_descriptions[i] if preview.stream_descriptions else "")
            if i < len(preview.streams):
                stream = preview.streams[i]
                age = f"{stream['age_s'] * 1000:.0f} ms" if stream.get("age_s") is not None else "—"
                self.collection.camera_metrics[i].setText(
                    f"{stream['status']} · {stream['fps']:.1f} FPS · Age {age}"
                )
                set_state(
                    self.collection.camera_metrics[i],
                    "success" if stream["status"] == "OK" else "warning",
                )
            else:
                self.collection.camera_metrics[i].setText("OFFLINE · FPS — · Age —")
                set_state(self.collection.camera_metrics[i], "neutral")
        timestamps = (preview.base_timestamp, preview.wrist_timestamp, preview.roi_timestamp)
        sizes = tuple((label.width(), label.height()) for label in labels)
        if timestamps == self._last_preview_timestamps and sizes == self._last_preview_sizes:
            return
        self._last_preview_timestamps = timestamps
        self._last_preview_sizes = sizes
        for label, rgb, name in zip(
            labels,
            (preview.base_rgb, preview.wrist_rgb, preview.roi_rgb),
            preview.stream_names,
            strict=True,
        ):
            self._set_preview_image(label, rgb, f"{name} · 等待相机")

    def _set_preview_image(self, label, rgb, empty_text):
        pixmap = self._pixmap_from_rgb(rgb)
        if pixmap is None:
            label.clear()
            label.setText(empty_text)
        else:
            label.setPixmap(
                pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )

    def closeEvent(self, event):
        if self._shutdown_complete:
            self._timer.stop()
            self._emergency_filter.detach()
            event.accept()
            return
        event.ignore()
        if self._closing:
            return
        save_pending = False
        if not self._presenter.closed:
            frame = self._presenter.poll()
            view = frame.view_model
            if frame.dataset.get("partial_episodes"):
                self.notification.setText("请先在数据集管理处理 Partial，或保留当前批次稍后继续。")
                self.navigation.setCurrentRow(3)
                return
            trajectory = frame.trajectory
            save_pending = (
                view.episode_active
                or view.episode_pending
                or bool(
                    trajectory.get("recording")
                    or trajectory.get("pending")
                    or trajectory.get("frame_count")
                )
            )
            if (
                save_pending
                and QMessageBox.question(
                    self,
                    "保存数据并退出",
                    "停止运动，保存未完成轨迹及 Episode（Episode 标为失败）后退出？",
                    QMessageBox.Yes | QMessageBox.Cancel,
                    QMessageBox.Cancel,
                )
                != QMessageBox.Yes
            ):
                return
        self._closing = True
        self._timer.stop()
        self._dagger_shortcut.setEnabled(False)
        self.notification.setText("正在停止运动、保存数据并释放设备…")

        def close():
            errors = []
            try:
                if self._close_callback is not None:
                    self._close_callback(**({"save_pending": True} if save_pending else {}))
                else:
                    self._command_port.close(timeout=1)
                    self._presenter.close()
            except Exception as exc:
                errors.append(exc)
            try:
                if self.settings_editor is not None:
                    self.settings_editor.close_jobs()
            except Exception as exc:
                errors.append(exc)
            if errors:
                raise ExceptionGroup("Workbench shutdown incomplete", errors)

        self._jobs.start(
            close,
            lambda value, error: self.shutdown_finished.emit(error),
            name="Workbench-Shutdown",
        )

    def _finish_shutdown(self, error):
        self._closing = False
        if error:
            self._local_error = f"退出未完成：{error}；可再次关闭重试。"
            self.notification.setText(self._local_error)
            if not self._presenter.closed:
                self._timer.start()
            return
        self._shutdown_complete = True
        self.close()

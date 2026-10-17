"""Collection page: device intents, logical camera streams, Dataset and Episode controls."""

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from gello_cr.core.state_machine import Command, WorkflowState


class CollectionPage(QWidget):
    def __init__(self, submit, parent=None):
        super().__init__(parent)
        self._submit = submit
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self._build_primary_controls())

        self.workspace_splitter = QSplitter(Qt.Horizontal)
        self.workspace_splitter.setChildrenCollapsible(False)
        self.workspace_splitter.addWidget(self._build_preview_group())
        self.workspace_splitter.addWidget(self._build_session_panel())
        self.workspace_splitter.setStretchFactor(0, 3)
        self.workspace_splitter.setStretchFactor(1, 2)
        self.workspace_splitter.setSizes([620, 390])
        layout.addWidget(self.workspace_splitter, 1)
        advanced = self._build_dagger_group()
        advanced.layout().removeWidget(self.dagger_expander)
        self._session_layout.addWidget(self.dagger_expander, 0, Qt.AlignLeft)
        advanced.setVisible(False)
        self.dagger_expander.toggled.connect(advanced.setVisible)
        layout.addWidget(advanced)

    @staticmethod
    def _label(text, name="mutedText"):
        label = QLabel(text)
        label.setObjectName(name)
        label.setWordWrap(True)
        label.setMinimumWidth(0)
        return label

    @staticmethod
    def _role(widget, role):
        if widget.property("role") != role:
            widget.setProperty("role", role)
            widget.style().unpolish(widget)
            widget.style().polish(widget)

    @staticmethod
    def _state(widget, state):
        if widget.property("state") != state:
            widget.setProperty("state", state)
            widget.style().unpolish(widget)
            widget.style().polish(widget)

    def _build_session_panel(self):
        panel = QFrame()
        panel.setObjectName("surfacePanel")
        panel.setMinimumWidth(345)
        layout = QVBoxLayout(panel)
        self._session_layout = layout
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(7)
        layout.addWidget(self._label("采集控制", "sectionTitle"))
        self.session_summary = self._label("尚未设置任务 · LeRobot V3")
        self.session_summary.setWordWrap(False)
        self.session_summary.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        layout.addWidget(self.session_summary)
        self.task_edit = QLineEdit()
        self.task_edit.setPlaceholderText("例如 motor_pick")
        self.description_edit = QLineEdit()
        self.description_edit.setPlaceholderText("描述要完成的动作")
        form = QFormLayout()
        form.setContentsMargins(0, 0, 0, 0)
        form.setVerticalSpacing(7)
        form.addRow("任务名称", self.task_edit)
        form.addRow("任务描述", self.description_edit)
        layout.addWidget(self._build_episode_group(form))
        layout.addStretch()
        return panel

    def _build_primary_controls(self):
        group = QWidget()
        outer = QVBoxLayout(group)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(6)
        heading = QHBoxLayout()
        heading.addWidget(self._label("操作准备", "sectionTitle"))
        self.preparation_hint = self._label("下一步：连接机器人")
        heading.addWidget(self.preparation_hint, 1)
        outer.addLayout(heading)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        outer.addLayout(grid)

        self.connect_button = QPushButton("连接机器人")
        self.prepare_devices_button = QPushButton("连接主臂 / O6")
        self.power_on_button = QPushButton("上使能")
        self.start_teleop_button = QPushButton("开始遥操")

        self.disconnect_button = QPushButton("断开机器人")
        self.power_off_button = QPushButton("下使能")
        self.stop_teleop_button = QPushButton("停止遥操")
        self.start_cameras_button = QPushButton("启动相机")
        self.stop_cameras_button = QPushButton("停止相机")
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
        )
        for button, command in command_buttons:
            button.clicked.connect(lambda _checked=False, cmd=command: self._submit(cmd))
            self._role(button, "quiet")

        stages = (
            ("01", "连接机器人", (self.connect_button, self.disconnect_button)),
            ("02", "准备主臂与手", (self.prepare_devices_button,)),
            ("03", "使能机器人", (self.power_on_button, self.power_off_button)),
            ("04", "开始遥操", (self.start_teleop_button, self.stop_teleop_button)),
        )
        self.stage_status_labels = []
        for column, (number, title, buttons) in enumerate(stages):
            card = QFrame()
            card.setObjectName("surfacePanel")
            stage_layout = QVBoxLayout(card)
            stage_layout.setContentsMargins(10, 8, 10, 8)
            stage_layout.setSpacing(4)
            stage_layout.addWidget(self._label(f"{number}  {title}", "eyebrow"))
            status = self._label("等待准备")
            self.stage_status_labels.append(status)
            stage_layout.addWidget(status)
            row = QHBoxLayout()
            row.setSpacing(5)
            for button in buttons:
                row.addWidget(button)
            stage_layout.addLayout(row)
            grid.addWidget(card, 0, column)
            grid.setColumnStretch(column, 1)
        return group

    def _make_preview_label(
        self,
        title: str,
        *,
        primary=False,
    ) -> tuple[QWidget, QLabel]:
        container = QWidget(self)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)

        title_label = QLabel(title)
        title_label.setObjectName("previewTitle")

        image_label = QLabel("等待相机")
        image_label.setObjectName("previewImage")
        image_label.setAlignment(Qt.AlignCenter)
        image_label.setMinimumSize(180, 150 if primary else 65)
        image_label.setMaximumHeight(230 if primary else 110)
        image_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

        layout.addWidget(title_label)
        layout.addWidget(image_label, 1)
        return container, image_label

    def _build_preview_group(self):
        group = QFrame()
        group.setObjectName("surfacePanel")
        group.setMinimumWidth(410)
        outer = QVBoxLayout(group)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(5)
        heading = QHBoxLayout()
        heading.addWidget(self._label("相机工作区", "sectionTitle"), 1)
        self.control_source = QComboBox()
        self.control_source.addItem("GELLO", "gello")
        self.control_source.setEnabled(False)
        self.control_source.setFixedWidth(86)
        self.control_source.setToolTip("当前遥操控制源，在设置中配置")
        heading.addWidget(self.control_source)
        heading.addWidget(self.start_cameras_button)
        heading.addWidget(self.stop_cameras_button)
        outer.addLayout(heading)

        controls = QHBoxLayout()
        self.camera_hint = self._label("相机未启动 · 可独立于遥操准备")
        controls.addWidget(self.camera_hint, 1)
        outer.addLayout(controls)

        base_widget, self.base_preview = self._make_preview_label("Base RGB", primary=True)
        wrist_widget, self.wrist_preview = self._make_preview_label("Wrist RGB")
        roi_widget, self.roi_preview = self._make_preview_label("Base ROI")

        self.preview_titles = [
            widget.findChild(QLabel, "previewTitle")
            for widget in (base_widget, wrist_widget, roi_widget)
        ]
        self.camera_metrics = []
        for widget in (base_widget, wrist_widget, roi_widget):
            metric = self._label("OFFLINE · FPS — · Age —")
            widget.layout().addWidget(metric)
            self.camera_metrics.append(metric)
        outer.addWidget(base_widget, 2)
        row = QHBoxLayout()
        row.setSpacing(12)
        row.addWidget(wrist_widget, 1)
        row.addWidget(roi_widget, 1)
        outer.addLayout(row, 1)
        return group

    def _build_episode_group(self, task_form):
        group = QWidget()
        outer = QVBoxLayout(group)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(6)
        self.dataset_setup = QWidget()
        setup = QVBoxLayout(self.dataset_setup)
        setup.setContentsMargins(0, 0, 0, 0)
        setup.setSpacing(8)
        setup.addWidget(self._label("任务与数据集", "sectionTitle"))
        setup.addLayout(task_form)

        form = QFormLayout()
        self.dataset_format = QComboBox()
        for label, value in (
            ("LeRobot V3", "lerobot_v3"),
            ("LeRobot V2.1", "lerobot_v21"),
            ("HDF5", "hdf5"),
        ):
            self.dataset_format.addItem(label, value)
        self.dataset_format.currentIndexChanged.connect(
            lambda _: self._submit(
                Command.SELECT_DATASET_FORMAT, {"format": self.dataset_format.currentData()}
            )
        )
        self.create_dataset_button = QPushButton("创建数据集")
        self.finalize_dataset_button = QPushButton("完成数据集")
        self._role(self.create_dataset_button, "quiet")
        self._role(self.finalize_dataset_button, "quiet")
        self.create_dataset_button.clicked.connect(
            lambda: self._submit(
                Command.CREATE_DATASET,
                {
                    "task": self.task_edit.text().strip(),
                    "description": self.description_edit.text().strip(),
                    "base_root": self.root_edit.text().strip(),
                },
            )
        )
        self.finalize_dataset_button.clicked.connect(lambda: self._submit(Command.FINALIZE_DATASET))
        form.setVerticalSpacing(7)
        form.addRow("保存格式", self.dataset_format)
        self.root_edit = QLineEdit()
        self.root_edit.setPlaceholderText("选择数据集父目录")
        self.root_browse_button = QPushButton("选择…")
        self._role(self.root_browse_button, "quiet")
        self.root_browse_button.clicked.connect(self._choose_dataset_root)
        root_row = QWidget()
        root_layout = QHBoxLayout(root_row)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.addWidget(self.root_edit)
        root_layout.addWidget(self.root_browse_button)
        self.root_edit.setToolTip("数据集会话开始后锁定目录；完成当前数据集后可更换目录")
        form.addRow("保存目录", root_row)
        setup.addLayout(form)
        dataset_actions = QHBoxLayout()
        dataset_actions.addWidget(self.create_dataset_button)
        dataset_actions.addWidget(self.finalize_dataset_button)
        setup.addLayout(dataset_actions)
        self.name_preview = self._label("填写任务名称后创建数据集")
        setup.addWidget(self.name_preview)

        self.episode_state_panel = QFrame()
        state_layout = QVBoxLayout(self.episode_state_panel)
        state_layout.setContentsMargins(10, 10, 10, 10)
        state_layout.setSpacing(5)
        state_layout.addWidget(self._label("EPISODE · 单段采集", "eyebrow"))
        self.episode_headline = self._label("等待采集准备", "stateHeadline")
        self.episode_hint = self._label("连接并使能机器人，相机画面就绪后开始。")
        state_layout.addWidget(self.episode_headline)
        state_layout.addWidget(self.episode_hint)
        outer.addWidget(self.episode_state_panel)

        metrics = QHBoxLayout()
        self.saved_metric = self._label("0", "metricValue")
        self.frame_metric = self._label("0", "metricValue")
        self.duration_metric = self._label("0.0s", "metricValue")
        for value, caption in (
            (self.saved_metric, "已保存段数"),
            (self.frame_metric, "当前帧数"),
            (self.duration_metric, "本段时长"),
        ):
            metric = QVBoxLayout()
            metric.setSpacing(1)
            metric.addWidget(value)
            metric.addWidget(self._label(caption, "metricLabel"))
            metrics.addLayout(metric, 1)

        self.episode_start_button = QPushButton("开始 Episode")
        self.episode_stop_button = QPushButton("结束录制")
        self.save_success_button = QPushButton("成功 · 保存")
        self.save_failure_button = QPushButton("失败 · 保存")
        self.discard_button = QPushButton("丢弃 Episode")
        for button in (
            self.episode_start_button,
            self.episode_stop_button,
            self.save_success_button,
            self.save_failure_button,
            self.discard_button,
        ):
            self._role(button, "primary" if button is not self.discard_button else "quiet")
        outer.addWidget(self.episode_start_button)
        outer.addWidget(self.episode_stop_button)
        save_actions = QHBoxLayout()
        save_actions.addWidget(self.save_success_button)
        save_actions.addWidget(self.save_failure_button)
        outer.addLayout(save_actions)
        outer.addWidget(self.discard_button)
        outer.addLayout(metrics)

        self.episode_status = self._label("未启动数据集")
        self.episode_status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.episode_details_toggle = QToolButton()
        self.episode_details_toggle.setText("采集详情")
        self.episode_details_toggle.setCheckable(True)
        self.episode_details_toggle.toggled.connect(self.episode_status.setVisible)
        self.dataset_setup_toggle = QToolButton()
        self.dataset_setup_toggle.setText("任务与数据集设置")
        self.dataset_setup_toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.dataset_setup_toggle.setCheckable(True)
        self._dataset_setup_context = None

        def show_setup(visible):
            self.dataset_setup.setVisible(visible)
            self.dataset_setup_toggle.setArrowType(Qt.DownArrow if visible else Qt.RightArrow)

        self.dataset_setup_toggle.toggled.connect(show_setup)
        self.dataset_setup_toggle.setChecked(True)
        detail_actions = QHBoxLayout()
        detail_actions.addWidget(self.dataset_setup_toggle, 1)
        detail_actions.addWidget(self.episode_details_toggle)
        outer.addLayout(detail_actions)
        outer.addWidget(self.dataset_setup)
        outer.addWidget(self.episode_status)
        self.episode_status.setVisible(False)

        self.episode_start_button.clicked.connect(self._start_episode)
        self.episode_stop_button.clicked.connect(lambda: self._submit(Command.STOP_EPISODE))
        self.save_success_button.clicked.connect(lambda: self._submit(Command.SAVE_SUCCESS))
        self.save_failure_button.clicked.connect(lambda: self._submit(Command.SAVE_FAILURE))
        self.discard_button.clicked.connect(lambda: self._submit(Command.DISCARD_EPISODE))
        return group

    def _build_dagger_group(self):
        group = QWidget()
        outer = QVBoxLayout(group)
        outer.setContentsMargins(0, 0, 0, 0)
        self.dagger_expander = QToolButton()
        self.dagger_expander.setText("高级 · DAgger 专家纠正")
        self.dagger_expander.setCheckable(True)
        self.dagger_expander.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.dagger_expander.setArrowType(Qt.RightArrow)
        outer.addWidget(self.dagger_expander, 0, Qt.AlignLeft)
        content = QGroupBox("专家纠正采集")
        layout = QVBoxLayout(content)
        outer.addWidget(content)
        content.setVisible(False)

        def toggle(visible):
            content.setVisible(visible)
            self.dagger_expander.setArrowType(Qt.DownArrow if visible else Qt.RightArrow)

        self.dagger_expander.toggled.connect(toggle)
        row = QHBoxLayout()
        self.dagger_host = QLineEdit("127.0.0.1")
        self.dagger_host.setMaximumWidth(180)
        self.dagger_port = QSpinBox()
        self.dagger_port.setRange(1, 65535)
        self.dagger_port.setValue(8000)
        self.dagger_round = QLineEdit("round1")
        self.dagger_round.setMaximumWidth(100)
        self.dagger_start_button = QPushButton("启动模型控制")
        self.dagger_toggle_button = QPushButton("空格：接管并录制")
        self.dagger_stop_button = QPushButton("停止 DAgger")
        for widget in (
            QLabel("模型服务"),
            self.dagger_host,
            self.dagger_port,
            QLabel("轮次"),
            self.dagger_round,
            self.dagger_start_button,
            self.dagger_toggle_button,
            self.dagger_stop_button,
        ):
            row.addWidget(widget)
        layout.addLayout(row)
        paths = QHBoxLayout()
        paths.addWidget(QLabel("纠正数据目录"))
        default_dagger_root = Path.home() / "datasets" / "cr3_o6_dagger_raw"
        self.dagger_root = QLineEdit(str(default_dagger_root))
        paths.addWidget(self.dagger_root)
        layout.addLayout(paths)
        self.dagger_status = QLabel(
            "空格按一次接管并记录，再按一次结束纠正、恢复模型；Task 使用上方任务文本"
        )
        self.dagger_status.setWordWrap(True)
        layout.addWidget(self.dagger_status)
        self.dagger_start_button.clicked.connect(self._start_dagger)
        self.dagger_toggle_button.clicked.connect(lambda: self._submit(Command.TOGGLE_INTERVENTION))
        self.dagger_stop_button.clicked.connect(lambda: self._submit(Command.STOP_DAGGER))
        return group

    def _start_dagger(self) -> None:
        self._submit(
            Command.START_DAGGER,
            {
                "host": self.dagger_host.text().strip(),
                "port": self.dagger_port.value(),
                "task": self.task_edit.text().strip(),
                "round_id": self.dagger_round.text().strip(),
                "base_root": self.dagger_root.text().strip(),
            },
        )

    def _choose_dataset_root(self) -> None:
        path = QFileDialog.getExistingDirectory(
            self,
            "选择数据集父目录",
            self.root_edit.text(),
            options=QFileDialog.DontUseNativeDialog,
        )
        if path:
            self.root_edit.setText(path)
            self.root_edit.editingFinished.emit()

    def _start_episode(self) -> None:
        self._submit(
            Command.START_EPISODE,
            {
                "task": self.task_edit.text().strip(),
                "description": self.description_edit.text().strip(),
                "base_root": self.root_edit.text().strip(),
            },
        )

    def _render_preparation(self, view, readiness, busy):
        state = view.workflow_state
        connected = view.robot_state.name not in ("OFFLINE", "FAULT", "ESTOP")
        enabled = view.robot_state.name == "ENABLED"
        teleop = state is WorkflowState.TELEOP_RUNNING
        device_text = (
            "主臂与 O6 已就绪"
            if readiness.devices_ready
            else "等待连接主臂与 O6"
            if not (readiness.master_connected or readiness.o6_connected)
            else "等待连接主臂"
            if not readiness.master_connected
            else "等待连接 O6"
        )
        statuses = (
            "机器人已连接" if connected else "机器人未连接",
            device_text,
            "机器人已使能" if enabled else "等待使能",
            "主从跟随运行中" if teleop else "主从跟随未启动",
        )
        for label, text, done in zip(
            self.stage_status_labels,
            statuses,
            (connected, readiness.devices_ready, enabled, teleop),
            strict=True,
        ):
            label.setText(text)
            self._state(label, "success" if done else "")

        next_button = None
        motion = {
            WorkflowState.RETURNING_HOME: "正在回零",
            WorkflowState.REPLAY_RUNNING: "正在回放轨迹",
            WorkflowState.POINT_MOVE_RUNNING: "正在执行走点",
            WorkflowState.DAGGER_RUNNING: "DAgger 控制运行中",
        }.get(state)
        if busy:
            hint = "正在处理上一项请求，请稍候。"
        elif state is WorkflowState.FAULT:
            hint = "机器人故障，请在机器人状态页排查并复位。"
        elif state is WorkflowState.ESTOP:
            hint = "软件急停已触发，确认现场后在机器人状态页复位。"
        elif state is WorkflowState.OFFLINE:
            hint = "下一步：连接机器人；相机可独立启动。"
            next_button = self.connect_button
        elif motion:
            hint = f"{motion}；停止当前运动后可开始遥操。"
        elif teleop and view.episode_pending:
            hint = "本段采集待保存 · 请先选择成功、失败或丢弃。"
        elif teleop and view.episode_active:
            hint = "遥操运行中 · 正在录制，完成动作后结束本段。"
        elif teleop and not readiness.devices_ready:
            hint = "遥操设备未就绪，请先停止运动并检查连接。"
        elif not readiness.devices_ready:
            hint = "下一步：连接遥操设备，确认主臂与 O6 已就绪。"
            next_button = self.prepare_devices_button
        elif state is WorkflowState.CONNECTED:
            hint = "下一步：上使能机器人，然后开始遥操。"
            next_button = self.power_on_button
        elif not teleop:
            hint = "下一步：开始遥操，建立主从跟随。"
            next_button = self.start_teleop_button
        elif not readiness.camera_frames_ready:
            hint = "遥操运行中；相机画面就绪后可开始采集。"
        else:
            hint = "遥操运行中 · 开始 Episode，结束后选择采集结果。"
        self.preparation_hint.setText(hint)
        for button in (
            self.connect_button,
            self.prepare_devices_button,
            self.power_on_button,
            self.start_teleop_button,
        ):
            self._role(button, "primary" if button is next_button else "quiet")

        if busy:
            reason = "正在处理上一项请求，请稍候。"
        elif state is WorkflowState.OFFLINE:
            reason = "请先连接机器人，再使能并连接主臂与 O6。"
        elif state in (WorkflowState.FAULT, WorkflowState.ESTOP):
            reason = hint
        elif not readiness.master_connected:
            reason = readiness.master_error or "请先连接遥操设备中的主臂。"
        elif not readiness.o6_connected:
            reason = readiness.o6_error or "请先连接 O6 手部设备。"
        elif state is WorkflowState.CONNECTED:
            reason = "机器人尚未使能，请先上使能。"
        elif motion:
            reason = hint
        elif teleop:
            reason = "遥操已在运行；可使用停止遥操结束跟随。"
        else:
            reason = "以当前主臂位置建立跟随起点，开始遥操。"
        self.start_teleop_button.setToolTip(reason)
        self.prepare_devices_button.setToolTip(
            "正在处理上一项请求，请稍候。"
            if busy
            else readiness.master_error
            or readiness.o6_error
            or "连接主臂与 O6；不会自动使能机器人或启动跟随。"
        )
        self.power_on_button.setToolTip(
            "请先连接机器人。"
            if state is WorkflowState.OFFLINE
            else "使能机器人；跟随仍需单独启动。"
        )
        self.disconnect_button.setToolTip(
            "请先结束本段并保存或丢弃，再断开机器人。"
            if view.episode_active or view.episode_pending
            else "断开机器人控制器连接。"
        )
        self.stop_cameras_button.setToolTip(
            "本段录制或待保存时不能停止相机，请先处理本段。"
            if view.episode_active or view.episode_pending
            else "停止相机，不改变机器人使能状态。"
        )
        if readiness.camera_error:
            camera_hint = readiness.camera_error
        elif readiness.camera_frames_ready:
            camera_hint = "画面已就绪 · 采集与预览使用相同来源"
        elif readiness.cameras_running:
            camera_hint = "相机运行中 · 等待新鲜画面"
        else:
            camera_hint = "相机未启动 · 可独立于遥操准备"
        self.camera_hint.setText(camera_hint)

    def _render_episode_guidance(self, view, readiness, dataset, busy):
        pending, recording = view.episode_pending, view.episode_active
        task = dataset.get("batch_task") or self.task_edit.text().strip() or "尚未设置任务"
        format_name = {
            "lerobot_v3": "LeRobot V3",
            "lerobot_v21": "LeRobot V2.1",
            "hdf5": "HDF5",
        }.get(view.dataset_format, view.dataset_format)
        self.session_summary.setText(f"{task} · {format_name}")
        self.session_summary.setToolTip(
            dataset.get("root") or dataset.get("output_root") or self.root_edit.text().strip()
        )
        setup_context = bool(view.dataset_session_active or recording or pending)
        if setup_context != self._dataset_setup_context:
            self._dataset_setup_context = setup_context
            self.dataset_setup_toggle.setChecked(not setup_context)
        self.episode_start_button.setVisible(not (recording or pending))
        self.episode_stop_button.setVisible(recording)
        for button in (self.save_success_button, self.save_failure_button, self.discard_button):
            button.setVisible(pending)
        self._role(self.save_failure_button, "quiet")

        state = view.workflow_state
        severity = ""
        if pending:
            headline = "本段未保存"
            severity = "warning"
            if state in (WorkflowState.FAULT, WorkflowState.ESTOP):
                source = "机器人故障" if state is WorkflowState.FAULT else "软件急停已触发"
                hint = f"{source}，成功保存暂不可用；可保存失败或丢弃本段。"
            else:
                hint = "请选择成功或失败保存，或丢弃本段后继续。"
        elif recording:
            headline = "录制中"
            severity = "success"
            hint = "完成动作后结束录制，再选择成功或失败保存。"
            if state in (WorkflowState.FAULT, WorkflowState.ESTOP):
                severity = "warning"
                hint = "故障或急停已触发；请核对现场运动状态，再结束本段并保存失败或丢弃。"
        elif state in (WorkflowState.FAULT, WorkflowState.ESTOP):
            headline = "故障待处理" if state is WorkflowState.FAULT else "软件急停中"
            severity = "error"
            hint = "请在机器人状态页排查并复位后继续采集。"
        elif state is WorkflowState.DAGGER_RUNNING:
            headline = "DAgger 控制中"
            hint = "使用下方高级面板或空格切换专家接管与模型控制。"
        elif state in (WorkflowState.OFFLINE, WorkflowState.CONNECTED):
            headline = "等待机器人使能"
            hint = "请先连接并使能机器人；相机画面就绪后可记录。"
        elif not readiness.camera_frames_ready:
            headline = "等待相机就绪"
            hint = readiness.camera_error or "启动相机，等待三路画面就绪后开始录制。"
        elif (
            view.actions.get(Command.START_EPISODE, False)
            and not view.dataset_session_active
            and not self.task_edit.text().strip()
        ):
            headline = "设置本次采集任务"
            hint = "在任务与数据集设置中填写任务名称和保存目录，再开始录制。"
        elif view.actions.get(Command.START_EPISODE, False):
            headline = "可以开始录制"
            hint = "开始记录当前控制源的状态与动作；结束后选择成功或失败保存。"
        else:
            headline = "等待采集准备"
            hint = "请完成当前操作后再开始新的一段。"
        if busy:
            hint = "正在处理上一项请求，请稍候；完成后操作会自动恢复。"
        if view.quality_needs_review:
            hint += " 本段采集质量需复核，展开采集详情查看。"
            if severity != "error":
                severity = "warning"
        self.episode_headline.setText(headline)
        self.episode_hint.setText(hint)
        self._state(self.episode_state_panel, severity)
        self._state(self.episode_headline, severity)
        self._state(self.episode_headline, severity)
        self.episode_start_button.setToolTip(hint)
        self.create_dataset_button.setToolTip(
            "当前数据集会话已创建，请先完成当前数据集。" if view.dataset_session_active else hint
        )
        self.finalize_dataset_button.setToolTip(
            "请先保存或丢弃当前 Episode。"
            if pending or recording
            else "完成当前数据集会话，合并并校验已保存的采集数据。"
        )
        self.save_success_button.setToolTip(
            "当前故障或急停未解除，可选择保存失败或丢弃。"
            if pending and not view.actions.get(Command.SAVE_SUCCESS, False)
            else "保存本段并标记任务成功。"
        )
        self.saved_metric.setText(str(view.saved_episodes))
        self.frame_metric.setText(str(view.buffered_frames))
        self.duration_metric.setText(f"{dataset.get('duration', 0):.1f}s")

    def render(self, view, readiness, dataset, busy=False):
        for button, command in (
            (self.connect_button, Command.CONNECT),
            (self.disconnect_button, Command.DISCONNECT),
            (self.power_on_button, Command.POWER_ON),
            (self.power_off_button, Command.POWER_OFF),
            (self.start_teleop_button, Command.START_TELEOP),
            (self.stop_teleop_button, Command.STOP_TELEOP),
            (self.prepare_devices_button, Command.PREPARE_DEVICES),
            (self.start_cameras_button, Command.START_CAMERAS),
            (self.stop_cameras_button, Command.STOP_CAMERAS),
            (self.episode_start_button, Command.START_EPISODE),
            (self.episode_stop_button, Command.STOP_EPISODE),
            (self.save_success_button, Command.SAVE_SUCCESS),
            (self.save_failure_button, Command.SAVE_FAILURE),
            (self.discard_button, Command.DISCARD_EPISODE),
            (self.create_dataset_button, Command.CREATE_DATASET),
            (self.finalize_dataset_button, Command.FINALIZE_DATASET),
        ):
            button.setEnabled(view.actions.get(command, False) and not busy)
        self._render_preparation(view, readiness, busy)
        self._render_episode_guidance(view, readiness, dataset, busy)
        editable = not (
            view.episode_active
            or view.episode_pending
            or busy
            or view.dataset_session_active
            or view.workflow_state is WorkflowState.DAGGER_RUNNING
        )
        for widget in (
            self.task_edit,
            self.description_edit,
            self.root_edit,
            self.root_browse_button,
        ):
            widget.setEnabled(editable)
        self.dataset_format.setEnabled(not view.dataset_session_active and not busy)
        self.dataset_format.blockSignals(True)
        self.dataset_format.setCurrentIndex(self.dataset_format.findData(view.dataset_format))
        self.dataset_format.blockSignals(False)
        self.control_source.blockSignals(True)
        label = readiness.master_type if readiness.master_type != "unknown" else "GELLO"
        self.control_source.setItemText(0, label.upper())
        self.control_source.blockSignals(False)
        quality = dataset.get("quality", {})
        text = (
            f"{view.recording_state.name} · Episode {dataset.get('next_episode_index', 0)} · "
            f"{view.buffered_frames} Frames · {quality.get('effective_fps', 0):.1f} FPS · "
            f"{dataset.get('duration', 0):.1f}s\n"
            f"Episodes {view.saved_episodes} · Success {dataset.get('success_episodes', 0)} · "
            f"Failure {dataset.get('failure_episodes', 0)}"
        )
        if dataset.get("batch_id"):
            text += f"\nBatch {dataset['batch_id'][:8]} · {dataset.get('segment_id', '')}"
        if dataset.get("root"):
            text += "\n实际保存：" + dataset["root"]
        if view.quality_needs_review:
            text += " · NEEDS REVIEW"
        if view.dataset_validation:
            report = view.dataset_validation
            text += "\nDataset validation: " + ("PASS" if report["valid"] else "FAILED")
            text += " · " + "; ".join(report["errors"])
        if not readiness.camera_frames_ready:
            text += "\n" + (readiness.camera_error or "启动相机并等待画面就绪后可开始记录")
        self.episode_status.setText(text)
        self.name_preview.setText(dataset.get("dataset_name_preview", ""))
        dagger = view.workflow_state is WorkflowState.DAGGER_RUNNING
        self.dagger_start_button.setEnabled(
            view.workflow_state is WorkflowState.ROBOT_ENABLED
            and readiness.devices_ready
            and readiness.camera_frames_ready
            and not busy
            and not (view.episode_active or view.episode_pending)
        )
        self.dagger_toggle_button.setEnabled(dagger and not busy)
        self.dagger_stop_button.setEnabled(dagger and not busy)
        self.dagger_status.setText(
            f"{view.dagger_phase or '未运行'} · {view.dagger_expert_frames} 专家帧\n"
            f"{view.dagger_root}"
        )

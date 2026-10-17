"""Trajectory controls using snapshots and application commands only."""

from __future__ import annotations

from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QWidget,
)

from gello_cr.core.state_machine import Command, WorkflowState


class TrajectoryPanel(QWidget):
    def __init__(self, submit, parent=None):
        super().__init__(parent)
        self._submit = submit
        self._snapshot = {}
        self._context = None
        self._catalog = []
        layout = QGridLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)
        self.setStyleSheet("QPushButton { min-height: 22px; padding: 2px 8px; }")
        self.setMinimumHeight(216)

        self.record_name = QLineEdit()
        self.record_name.setPlaceholderText("轨迹名称，如 motor_pick_demo_01")
        self.record_button = QPushButton("开始记录")
        self.finish_button = QPushButton("停止并保存")
        self.save_button = QPushButton("重试保存")
        self.cancel_button = QPushButton("取消记录")
        record = QHBoxLayout()
        for widget in (
            QLabel("名称"),
            self.record_name,
            self.record_button,
            self.finish_button,
            self.save_button,
            self.cancel_button,
        ):
            record.addWidget(widget)
        layout.addLayout(record, 0, 0, 1, 2)

        self.saved_combo = QComboBox()
        self.saved_combo.setMinimumWidth(210)
        self.refresh_button = QPushButton("刷新")
        self.import_json_button = QPushButton("导入旧 JSON")
        self.import_episode_button = QPushButton("导入 LeRobot")
        self.episode_spin = QSpinBox()
        self.episode_spin.setRange(0, 1000000)
        saved = QHBoxLayout()
        for widget in (
            QLabel("已保存"),
            self.saved_combo,
            self.refresh_button,
            self.import_json_button,
            self.import_episode_button,
            QLabel("Episode"),
            self.episode_spin,
        ):
            saved.addWidget(widget)
        saved.setStretch(1, 1)
        layout.addLayout(saved, 1, 0, 1, 2)

        self.mode_combo = QComboBox()
        self.mode_combo.addItem("相对 Relative", "relative")
        self.mode_combo.addItem("绝对 Absolute", "absolute")
        self.mode_combo.setToolTip("相对：从当前位置复现位移；绝对：先用 3 秒过渡到轨迹起点")
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.1, 2.0)
        self.speed_spin.setSingleStep(0.1)
        self.speed_spin.setValue(1.0)
        self.speed_spin.setSuffix("×")
        self.repeat_spin = QSpinBox()
        self.repeat_spin.setRange(1, 1000)
        self.arm_check = QCheckBox("Arm")
        self.hand_check = QCheckBox("Hand")
        self.arm_check.setChecked(True)
        self.hand_check.setChecked(True)
        self.preview_button = QPushButton("预览")
        self.play_button = QPushButton("回放")
        options = QHBoxLayout()
        for widget in (
            self.mode_combo,
            QLabel("速度"),
            self.speed_spin,
            QLabel("次数"),
            self.repeat_spin,
            self.arm_check,
            self.hand_check,
            self.preview_button,
            self.play_button,
        ):
            options.addWidget(widget)
        layout.addLayout(options, 2, 0, 1, 2)
        mask = QHBoxLayout()
        mask.addWidget(QLabel("回放关节"))
        self.joint_checks = [QCheckBox(f"J{i}") for i in range(1, 7)]
        for check in self.joint_checks:
            check.setChecked(True)
            check.toggled.connect(self._update_enabled)
            mask.addWidget(check)
        self.status_label = QLabel("未录制")
        mask.addWidget(self.status_label, 1)
        layout.addLayout(mask, 3, 0, 1, 2)
        self.preview_text = QPlainTextEdit()
        self.preview_text.setReadOnly(True)
        self.preview_text.setMinimumHeight(64)
        self.preview_text.setMaximumHeight(92)
        self.preview_text.setPlaceholderText(
            "选择轨迹后点击预览：时长、关节范围与速度、TCP 和 Hand 范围"
        )
        layout.addWidget(self.preview_text, 4, 0, 1, 2)

        self.record_button.clicked.connect(
            lambda: self._submit(
                Command.START_TRAJECTORY_RECORDING, {"name": self.record_name.text().strip()}
            )
        )
        for button, command in (
            (self.finish_button, Command.STOP_TRAJECTORY_RECORDING),
            (self.save_button, Command.SAVE_TRAJECTORY),
            (self.cancel_button, Command.CANCEL_TRAJECTORY_RECORDING),
            (self.refresh_button, Command.LIST_TRAJECTORIES),
        ):
            button.clicked.connect(lambda _checked=False, cmd=command: self._submit(cmd))
        self.preview_button.clicked.connect(
            lambda: self._submit(
                Command.PREVIEW_TRAJECTORY, {"trajectory_id": self.saved_combo.currentData()}
            )
        )
        self.play_button.clicked.connect(self._play)
        self.import_json_button.clicked.connect(self._import_json)
        self.import_episode_button.clicked.connect(self._import_episode)
        self.saved_combo.currentIndexChanged.connect(self._selection_changed)
        self.record_name.textChanged.connect(self._update_enabled)
        self.arm_check.toggled.connect(self._update_enabled)
        self.hand_check.toggled.connect(self._update_enabled)

    def _play(self):
        self._submit(
            Command.REPLAY_TRAJECTORY,
            {
                "trajectory_id": self.saved_combo.currentData(),
                "options": {
                    "mode": self.mode_combo.currentData(),
                    "time_scale": self.speed_spin.value(),
                    "repeat": self.repeat_spin.value(),
                    "arm_enabled": self.arm_check.isChecked(),
                    "hand_enabled": self.hand_check.isChecked(),
                    "joint_mask": [check.isChecked() for check in self.joint_checks],
                },
            },
        )

    def _import_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "导入旧轨迹", "", "JSON (*.json)", options=QFileDialog.DontUseNativeDialog
        )
        if path:
            self._submit(Command.IMPORT_TRAJECTORY, {"path": path, "source": "legacy_json"})

    def _import_episode(self):
        path = QFileDialog.getExistingDirectory(
            self, "选择 LeRobot 数据集目录", options=QFileDialog.DontUseNativeDialog
        )
        if path:
            self._submit(
                Command.IMPORT_TRAJECTORY,
                {
                    "path": path,
                    "source": "lerobot",
                    "episode_index": self.episode_spin.value(),
                },
            )

    def _selection_changed(self):
        selected = self.saved_combo.currentData()
        metadata = next((v for v in self._catalog if v["trajectory_id"] == selected), {})
        mask = metadata.get("joint_mask") or [True] * 6
        for check, enabled in zip(self.joint_checks, mask, strict=True):
            check.blockSignals(True)
            check.setChecked(enabled)
            check.blockSignals(False)
        if "hand_type" in metadata:
            self.hand_check.setChecked(metadata["hand_type"] is not None)
        self._render_preview()
        self._update_enabled()

    def render(self, snapshot, view, readiness, pending=False):
        self._snapshot = snapshot
        self._context = view, readiness, pending
        catalog = snapshot.get("items", [])
        if catalog != self._catalog:
            current = self.saved_combo.currentData()
            self._catalog = catalog
            self.saved_combo.blockSignals(True)
            self.saved_combo.clear()
            for value in catalog:
                self.saved_combo.addItem(value["name"], value["trajectory_id"])
            index = self.saved_combo.findData(current)
            self.saved_combo.setCurrentIndex(max(0, index) if catalog else -1)
            self.saved_combo.blockSignals(False)
            self._selection_changed()
        count, duration = snapshot.get("frame_count", 0), snapshot.get("duration_s", 0.0)
        state = "正在记录" if snapshot.get("recording") else "待保存" if count else "未录制"
        self.status_label.setText(
            snapshot.get("recording_error") or f"{state} · {count} 帧 · {duration:.2f}s"
        )
        self._render_preview()
        self._update_enabled()

    def _update_enabled(self):
        if self._context is None:
            return
        view, readiness, busy = self._context
        state = view.workflow_state
        recording = bool(self._snapshot.get("recording"))
        playing = bool(self._snapshot.get("playing")) or state is WorkflowState.REPLAY_RUNNING
        unsaved = (
            recording or bool(self._snapshot.get("frame_count")) or self._snapshot.get("pending")
        )
        record_allowed = view.actions.get(Command.START_TRAJECTORY_RECORDING, False)
        self.record_button.setEnabled(
            record_allowed and not busy and not unsaved and bool(self.record_name.text().strip())
        )
        self.finish_button.setEnabled(
            bool(unsaved) and not self._snapshot.get("pending") and not busy
        )
        self.save_button.setEnabled(bool(self._snapshot.get("pending")) and not busy)
        self.cancel_button.setEnabled(bool(unsaved) and not busy)
        self.record_name.setEnabled(not unsaved and not busy)
        selected = self.saved_combo.currentData() is not None
        arm, hand = self.arm_check.isChecked(), self.hand_check.isChecked()
        replay_allowed = view.actions.get(Command.REPLAY_TRAJECTORY, False)
        self.play_button.setEnabled(
            replay_allowed
            and selected
            and not busy
            and not unsaved
            and (arm or hand)
            and (not arm or any(c.isChecked() for c in self.joint_checks))
            and (not hand or readiness.o6_connected)
            and not view.episode_pending
        )
        editable = not busy and not playing
        for widget in (
            self.mode_combo,
            self.speed_spin,
            self.repeat_spin,
            self.arm_check,
            self.hand_check,
            self.saved_combo,
            self.import_json_button,
            self.import_episode_button,
            self.episode_spin,
            self.refresh_button,
        ):
            widget.setEnabled(editable)
        for check in self.joint_checks:
            check.setEnabled(editable and arm)
        self.preview_button.setEnabled(selected and editable)

    def _render_preview(self):
        preview = self._snapshot.get("preview") or {}
        if not preview or preview.get("trajectory_id") != self.saved_combo.currentData():
            text = ""
        else:

            def values(vector):
                return "无" if vector is None else ", ".join(f"{v:.4g}" for v in vector)

            def ranges(bounds):
                return "无" if bounds is None else "; ".join(f"{a:.4g}～{b:.4g}" for a, b in bounds)

            text = (
                f"{preview['name']} · {preview['robot_type']} · {preview['frame_count']} 帧 · "
                f"{preview['duration_s']:.2f}s · {preview['fps']:.1f}Hz\n"
                f"来源：{preview.get('source', '—')} · 创建：{preview.get('created_at', '—')}\n"
                f"关节范围 rad：{ranges(preview['joint_range_rad'])}\n"
                f"最大帧间变化 rad：{values(preview.get('max_joint_delta_rad'))}\n"
                f"最大关节速度 rad/s：{values(preview['max_joint_velocity_rad_s'])}\n"
                f"起点 rad：{values(preview['start_joint_rad'])}\n"
                f"终点 rad：{values(preview['end_joint_rad'])}\n"
                f"TCP 范围 m/rad：{ranges(preview['tcp_range'])}\n"
                f"Hand 范围：{ranges(preview['hand_range'])}\n"
                + "\n".join(preview.get("warnings", ()))
            )
        if self.preview_text.toPlainText() != text:
            self.preview_text.setPlainText(text)

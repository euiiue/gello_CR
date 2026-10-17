"""Point and waypoint editor: commands and read-only previews, without device I/O."""

from __future__ import annotations

import math

from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QWidget,
)

from gello_cr.core.state_machine import Command, WorkflowState


class PlanningPanel(QWidget):
    def __init__(self, submit, parent=None):
        super().__init__(parent)
        self._submit = submit
        self._snapshot = {}
        self._context = None
        self._revision = 0
        self._planned_revision = None
        self._requested_revision = None
        self._previous_plan_id = None
        self._position_serial = 0
        self._add_current_pending = False
        self._configured = False
        layout = QGridLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        self.setStyleSheet("QPushButton { min-height: 22px; padding: 2px 8px; }")
        self.mode_combo = QComboBox()
        for text, key in [
            ("Joint PTP", "joint"),
            ("TCP PTP", "tcp_ptp"),
            ("TCP Linear", "tcp_linear"),
            ("Waypoints", "waypoints"),
        ]:
            self.mode_combo.addItem(text, key)
        self.name_edit = QLineEdit("Planned motion")
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(1, 100)
        self.speed_spin.setValue(30)
        self.speed_spin.setSuffix(" %")
        header = QHBoxLayout()
        for widget in (
            QLabel("模式"),
            self.mode_combo,
            QLabel("名称"),
            self.name_edit,
            QLabel("速度"),
            self.speed_spin,
        ):
            header.addWidget(widget)
        layout.addLayout(header, 0, 0, 1, 2)
        self.offline_start = QLineEdit("0, 0, 0, 0, 0, 0")
        self.offline_start.setToolTip("离线规划起点 J1–J6，逗号分隔，单位 °；连接后需重新规划。")
        self.offline_start.textChanged.connect(self._changed)
        layout.addWidget(self.offline_start, 6, 0, 1, 2)
        target = QHBoxLayout()
        self.target_spins, self.target_labels = [], []
        for i in range(6):
            label = QLabel(f"J{i + 1} °")
            spin = QDoubleSpinBox()
            spin.setRange(-100000, 100000)
            spin.setDecimals(3)
            spin.valueChanged.connect(self._changed)
            self.target_labels.append(label)
            self.target_spins.append(spin)
            target.addWidget(label)
            target.addWidget(spin)
        layout.addLayout(target, 1, 0, 1, 2)
        self.table = QTableWidget(0, 9)
        self.table.setHorizontalHeaderLabels(
            ["X mm", "Y mm", "Z mm", "Rx °", "Ry °", "Rz °", "Mode", "速度 %", "停留 s"]
        )
        self.table.setMaximumHeight(170)
        self.table.setMinimumHeight(110)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.itemChanged.connect(self._changed)
        layout.addWidget(self.table, 2, 0, 1, 2)
        editor = QHBoxLayout()
        self.read_button = QPushButton("读取当前位置")
        self.add_current_button = QPushButton("添加当前位置")
        self.add_button = QPushButton("添加 Waypoint")
        self.delete_button = QPushButton("删除")
        self.up_button = QPushButton("上移")
        self.down_button = QPushButton("下移")
        for widget in (
            self.read_button,
            self.add_current_button,
            self.add_button,
            self.delete_button,
            self.up_button,
            self.down_button,
        ):
            editor.addWidget(widget)
        layout.addLayout(editor, 3, 0, 1, 2)
        actions = QHBoxLayout()
        self.plan_button = QPushButton("规划 / 预览")
        self.execute_button = QPushButton("执行")
        self.save_button = QPushButton("保存到轨迹库")
        self.status_label = QLabel("IDLE")
        for widget in (
            self.plan_button,
            self.execute_button,
            self.save_button,
            self.status_label,
        ):
            actions.addWidget(widget)
        layout.addLayout(actions, 4, 0, 1, 2)
        self.preview_text = QPlainTextEdit()
        self.preview_text.setReadOnly(True)
        self.preview_text.setMinimumHeight(150)
        layout.addWidget(self.preview_text, 5, 0, 1, 2)
        self.read_button.clicked.connect(lambda: self._read(False))
        self.add_current_button.clicked.connect(lambda: self._read(True))
        self.add_button.clicked.connect(self._add)
        self.delete_button.clicked.connect(self._delete)
        self.up_button.clicked.connect(lambda: self._move(-1))
        self.down_button.clicked.connect(lambda: self._move(1))
        self.plan_button.clicked.connect(self._plan)
        self.execute_button.clicked.connect(
            lambda: self._submit(Command.EXECUTE_PLAN, {"plan_id": self._snapshot["plan_id"]})
        )
        self.save_button.clicked.connect(
            lambda: self._submit(Command.SAVE_MOTION_PLAN, {"plan_id": self._snapshot["plan_id"]})
        )
        self.mode_combo.currentIndexChanged.connect(self._mode_changed)
        self.name_edit.textChanged.connect(self._changed)
        self.speed_spin.valueChanged.connect(self._changed)
        self._mode_changed()
        self.execute_button.setEnabled(False)
        self.save_button.setEnabled(False)

    def _changed(self, *args):
        self._revision += 1
        self._update_enabled()

    def _mode_changed(self, *args):
        joint = self.mode_combo.currentData() == "joint"
        for label, text in zip(
            self.target_labels,
            [f"J{i + 1} °" for i in range(6)]
            if joint
            else ["X mm", "Y mm", "Z mm", "Rx °", "Ry °", "Rz °"],
            strict=True,
        ):
            label.setText(text)
        self.table.setVisible(self.mode_combo.currentData() == "waypoints")
        self._changed()

    def _read(self, add):
        self._add_current_pending = add
        self._submit(Command.READ_MOTION_POSITION)

    def _add(self):
        self._append([spin.value() for spin in self.target_spins] + ["linear", 100, 0])

    def _append(self, values):
        row = self.table.rowCount()
        self.table.insertRow(row)
        for column, value in enumerate(values):
            self.table.setItem(row, column, QTableWidgetItem(str(value)))
        self.table.selectRow(row)
        self._changed()

    def _delete(self):
        row = self.table.currentRow()
        if row >= 0:
            self.table.removeRow(row)
            self._changed()

    def _move(self, offset):
        row = self.table.currentRow()
        other = row + offset
        if row < 0 or not 0 <= other < self.table.rowCount():
            return
        for column in range(self.table.columnCount()):
            first, second = self.table.takeItem(row, column), self.table.takeItem(other, column)
            self.table.setItem(row, column, second)
            self.table.setItem(other, column, first)
        self.table.selectRow(other)
        self._changed()

    @staticmethod
    def _tcp(values):
        return [v / 1000 for v in values[:3]] + [math.radians(v) for v in values[3:]]

    def _plan(self):
        kind = self.mode_combo.currentData()
        payload = {
            "kind": kind,
            "name": self.name_edit.text(),
            "speed_scale": self.speed_spin.value() / 100,
        }
        try:
            values = [spin.value() for spin in self.target_spins]
            payload["target"] = (
                [math.radians(v) for v in values] if kind == "joint" else self._tcp(values)
            )
            if kind == "waypoints":
                payload["waypoints"] = []
                for row in range(self.table.rowCount()):
                    values = [self.table.item(row, col).text() for col in range(9)]
                    mode = values[6].strip().lower()
                    if mode not in ("ptp", "linear"):
                        raise ValueError("Waypoint Mode 需要 ptp 或 linear")
                    payload["waypoints"].append(
                        {
                            "pose": self._tcp([float(v) for v in values[:6]]),
                            "mode": mode,
                            "speed_scale": float(values[7]) / 100,
                            "hold_time_s": float(values[8]),
                        }
                    )
            if self._context and self._context[0].robot_state.name == "OFFLINE":
                start = [float(v.strip()) for v in self.offline_start.text().split(",")]
                if len(start) != 6:
                    raise ValueError("离线起点需要六个角度")
                payload["start_joint_rad"] = [math.radians(v) for v in start]
            self._planned_revision = None
            self._requested_revision = self._revision
            self._previous_plan_id = self._snapshot.get("plan_id")
            self._submit(Command.PLAN_MOTION, payload)
        except (ValueError, AttributeError) as exc:
            self.status_label.setText(f"输入错误：{exc}")
            self._planned_revision = self._requested_revision = None
        self._update_enabled()

    def render(self, snapshot, view, busy=False):
        self._snapshot = dict(snapshot)
        self._context = view, busy
        plan_id = snapshot.get("plan_id")
        if self._requested_revision is not None and plan_id and plan_id != self._previous_plan_id:
            self._planned_revision = self._requested_revision
            self._requested_revision = None
        if not self._configured and snapshot:
            self.speed_spin.setValue(float(snapshot.get("default_speed_scale", 0.3)) * 100)
            self._configured = True
        serial = snapshot.get("position_serial", 0)
        if serial != self._position_serial:
            self._position_serial = serial
            position = snapshot.get("position")
            if position:
                joint = self.mode_combo.currentData() == "joint"
                values = (
                    [math.degrees(v) for v in position["joint_rad"]]
                    if joint
                    else (
                        [v * 1000 for v in position["tcp"][:3]]
                        + [math.degrees(v) for v in position["tcp"][3:]]
                    )
                )
                for spin, value in zip(self.target_spins, values, strict=True):
                    spin.setValue(value)
                if self._add_current_pending:
                    self._append(values + ["linear", 100, 0])
                self._add_current_pending = False
        preview = snapshot.get("preview") or {}
        metrics = snapshot.get("metrics") or {}

        def numbers(values):
            return "无" if values is None else ", ".join(f"{v:.4g}" for v in values)

        text = "碰撞检查：不可用\n"
        if preview:
            text += (
                f"{metrics.get('frame_count', 0)} 帧 · {metrics.get('duration_s', 0):.3f}s\n"
                f"起始 Joint rad：{numbers(preview.get('start_joint_rad'))}\n"
                f"目标 Joint rad：{numbers(preview.get('target_joint_rad'))}\n"
                f"起始 TCP m/rad：{numbers(preview.get('start_pose'))}\n"
                f"目标 TCP m/rad：{numbers(preview.get('target_pose'))}\n"
                f"最大 Joint 速度：{metrics.get('max_joint_velocity', 0):.4g} rad/s · "
                f"加速度：{metrics.get('max_joint_acceleration', 0):.4g} rad/s² · "
                f"帧间变化：{metrics.get('max_joint_delta', 0):.4g} rad\n"
                f"TCP 路径长度：{metrics.get('path_length_tcp_m')} m\n"
            )
        text += "\n".join(snapshot.get("warnings", ()))
        if snapshot.get("saved_path"):
            text += "\n已保存：" + snapshot["saved_path"]
        text += "\n" + str(snapshot.get("error", ""))
        if self.preview_text.toPlainText() != text:
            self.preview_text.setPlainText(text)
        status = snapshot.get("status", "IDLE")
        if self._planned_revision != self._revision and preview:
            status += " · 输入已修改，请重新规划"
        self.status_label.setText(status)
        self._update_enabled()

    def _update_enabled(self):
        if self._context is None:
            return
        view, busy = self._context
        running = (
            self._snapshot.get("running") or view.workflow_state is WorkflowState.POINT_MOVE_RUNNING
        )
        editable = not busy and not running
        waypoints = self.mode_combo.currentData() == "waypoints"
        for widget in (
            self.mode_combo,
            self.name_edit,
            self.speed_spin,
            self.table,
            *self.target_spins,
        ):
            widget.setEnabled(editable)
        available = view.workflow_state in (
            WorkflowState.CONNECTED,
            WorkflowState.ROBOT_ENABLED,
            WorkflowState.TELEOP_RUNNING,
        )
        self.read_button.setEnabled(editable and available)
        for button in (
            self.add_current_button,
            self.add_button,
            self.delete_button,
            self.up_button,
            self.down_button,
        ):
            button.setEnabled(
                editable and waypoints and (available or button is not self.add_current_button)
            )
        offline = view.robot_state.name == "OFFLINE"
        self.offline_start.setVisible(offline)
        self.offline_start.setEnabled(editable)
        self.offline_start.setPlaceholderText("离线起点 J1–J6，单位 °")
        self.plan_button.setEnabled(
            editable
            and view.actions.get(Command.PLAN_MOTION, False)
            and (available or offline and self.mode_combo.currentData() == "joint")
        )
        valid = bool(self._snapshot.get("plan_id")) and self._planned_revision == self._revision
        self.execute_button.setEnabled(
            valid and editable and view.actions.get(Command.EXECUTE_PLAN, False)
        )
        self.save_button.setEnabled(valid and editable)

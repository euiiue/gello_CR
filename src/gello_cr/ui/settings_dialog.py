"""Offline parameter editor with explicit camera discovery; no motion commands."""

from __future__ import annotations

import copy
import threading
import traceback

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from gello_cr.app.settings import SETTINGS_GROUPS, OperatorSettings
from gello_cr.devices.camera_streams import resolve_camera_streams
from gello_cr.devices.realsense import discover_realsense_cameras

PAGE_NOTES = {
    "机械臂": "ServoJ v/a/j 按 SDK 原值保存。关节保护对 J1–J6 生效；"
    "XYZ / 姿态限制仅用于笛卡尔模式，范围相对于跟随起点，不是绝对关节软限位。",
    "GELLO": "当前标定、关节方向与控制模式沿用原配置。反馈频率同时决定关节跟随循环频率。",
    "O6 动作": "编辑动作库并选择 J7 张开 / 闭合动作。仅保存参数，不发送手部动作。"
    "六列顺序：拇指弯曲、拇指侧摆、食指、中指、无名指、小指。",
    "数采与相机": "质量阈值用于标记 needs_review；机械臂停止阈值在“机械臂”页。"
    "建议保存高 480 / 宽 640，训练时再缩放；画面来源和 ROI 在“相机画面”页设置。",
    "控制器连接": "连接参数在下次启动后使用；启动后仍需手动连接与使能。",
}


class OperatorSettingsDialog(QDialog):
    cameras_discovered = Signal(object, str)

    def __init__(self, settings: OperatorSettings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.saved = False
        self.setWindowTitle("参数设置")
        self.resize(1020, 780)
        self.setMinimumSize(900, 650)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(24, 20, 24, 20)
        title = QLabel("参数设置")
        title.setStyleSheet("font-size: 22px; font-weight: 600")
        outer.addWidget(title)
        note = QLabel("保存后下次启动生效 · 当前会话参数保持不变")
        outer.addWidget(note)
        path_label = QLabel(str(settings.path))
        path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        path_label.setWordWrap(True)
        outer.addWidget(path_label)
        self.tabs = QTabWidget()
        outer.addWidget(self.tabs, 1)
        self.editors: dict[str, list[QLineEdit]] = {}
        for name, fields in SETTINGS_GROUPS.items():
            page = QWidget()
            layout = QVBoxLayout(page)
            layout.setContentsMargins(16, 16, 16, 16)
            description = QLabel(PAGE_NOTES[name])
            description.setWordWrap(True)
            layout.addWidget(description)
            if name == "GELLO":
                layout.addWidget(
                    QLabel(
                        f"主设备：{settings.data['master']['type']}  ·  "
                        f"控制模式：{settings.data['gello']['control_mode']}"
                    )
                )
            if name == "O6 动作":
                self._build_actions(layout)
            form = QFormLayout()
            form.setVerticalSpacing(12)
            form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
            if name == "数采与相机":
                self.recording_mode = QComboBox()
                self.recording_mode.addItem("TCP 位姿（m / rad）＋ TCP 差值动作", "tcp")
                self.recording_mode.addItem("CR3 关节 q1–q6（rad）＋绝对关节目标", "joint")
                self.recording_mode.setCurrentIndex(
                    self.recording_mode.findData(settings.data["dataset"]["recording_mode"])
                )
                form.addRow("LeRobot 记录模式", self.recording_mode)
            for field in fields:
                section, key = field.path.split(".")
                value = settings.data[section][key]
                values = value if field.count > 1 else [value]
                row = QWidget()
                row_layout = QHBoxLayout(row)
                row_layout.setContentsMargins(0, 0, 0, 0)
                edits = []
                for index, item in enumerate(values):
                    edit = QLineEdit(str(item))
                    edit.setObjectName(f"{field.path}.{index}")
                    edit.setAccessibleName(f"{field.label} {index + 1}")
                    edit.setToolTip(field.path)
                    edit.setMinimumWidth(45)
                    row_layout.addWidget(edit)
                    edits.append(edit)
                self.editors[field.path] = edits
                form.addRow(field.label, row)
            layout.addLayout(form)
            layout.addStretch()
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QScrollArea.NoFrame)
            scroll.setWidget(page)
            self.tabs.addTab(scroll, name)
        self._build_camera_page()
        self.error_label = QLabel()
        self.error_label.setWordWrap(True)
        self.error_label.setStyleSheet("color: #b3261e")
        outer.addWidget(self.error_label)
        self.buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        self.buttons.button(QDialogButtonBox.Save).setText("保存参数")
        self.buttons.button(QDialogButtonBox.Cancel).setText("取消")
        self.buttons.accepted.connect(self._save)
        self.buttons.rejected.connect(self.reject)
        outer.addWidget(self.buttons)
        self._initial_candidate = self._candidate()

    def _build_camera_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        description = QLabel(
            "三个窗口分别选择相机和画面类型，预览与 LeRobot 使用同一来源。"
            "同一相机可用于多个窗口；ROI 为 0–1 的归一化坐标。保存后重新启动软件生效。"
        )
        description.setWordWrap(True)
        layout.addWidget(description)
        self.camera_scan_button = QPushButton("刷新已连接相机")
        self.camera_scan_button.clicked.connect(self._scan_cameras)
        layout.addWidget(self.camera_scan_button)
        self.camera_scan_status = QLabel("可刷新选择设备，也可以直接填写相机序列号。")
        self.camera_scan_status.setWordWrap(True)
        layout.addWidget(self.camera_scan_status)
        self.camera_editors = []
        for index, stream in enumerate(resolve_camera_streams(self.settings.data["dataset"])):
            group = QGroupBox(f"录制窗口 {index + 1}")
            form = QFormLayout(group)
            name = QLineEdit(stream.name)
            serial = QComboBox()
            serial.setEditable(True)
            serial.addItem(stream.serial)
            mode = QComboBox()
            mode.addItem("相机完整画面", "camera")
            mode.addItem("该相机的 ROI 裁剪", "roi")
            mode.setCurrentIndex(mode.findData(stream.mode))
            roi_row = QWidget()
            roi_layout = QHBoxLayout(roi_row)
            roi_layout.setContentsMargins(0, 0, 0, 0)
            roi_edits = []
            for label, value in zip(("x1", "y1", "x2", "y2"), stream.roi_norm, strict=True):
                roi_layout.addWidget(QLabel(label))
                edit = QLineEdit(str(value))
                edit.setAccessibleName(f"窗口 {index + 1} ROI {label}")
                roi_layout.addWidget(edit)
                roi_edits.append(edit)
            form.addRow("窗口名称", name)
            form.addRow("相机序列号", serial)
            form.addRow("画面类型", mode)
            form.addRow("裁剪范围", roi_row)
            roi_row.setEnabled(stream.mode == "roi")
            mode.currentIndexChanged.connect(
                lambda _index, row=roi_row, combo=mode: row.setEnabled(combo.currentData() == "roi")
            )
            self.camera_editors.append((name, serial, mode, roi_edits))
            layout.addWidget(group)
        layout.addStretch()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setWidget(page)
        self.tabs.addTab(scroll, "相机画面")
        self.cameras_discovered.connect(self._show_cameras)

    def _scan_cameras(self):
        self.camera_scan_button.setEnabled(False)
        self.camera_scan_status.setText("正在读取相机列表…")

        def scan():
            try:
                devices = discover_realsense_cameras()
            except Exception as exc:
                traceback.print_exception(exc)
                self.cameras_discovered.emit([], f"{type(exc).__name__}: {exc}")
            else:
                self.cameras_discovered.emit(devices, "")

        threading.Thread(target=scan, name="CameraDiscovery", daemon=True).start()

    def _show_cameras(self, devices, error):
        self.camera_scan_button.setEnabled(True)
        if error:
            self.camera_scan_status.setText(f"读取相机失败：{error}")
            return
        for _name, combo, _mode, _roi in self.camera_editors:
            selected = combo.currentText().split(" · ", 1)[0].strip()
            combo.clear()
            for device in devices:
                combo.addItem(f"{device['serial']} · {device['name']}", device["serial"])
            index = combo.findData(selected)
            if index >= 0:
                combo.setCurrentIndex(index)
            else:
                combo.setEditText(selected)
        self.camera_scan_status.setText(f"检测到 {len(devices)} 台相机；刷新不会更改已选序列号。")

    def _build_actions(self, layout):
        self.actions_table = QTableWidget(0, 7)
        self.actions_table.setHorizontalHeaderLabels(
            [
                "动作名称",
                "拇指弯曲",
                "拇指侧摆",
                "食指弯曲",
                "中指弯曲",
                "无名指弯曲",
                "小指弯曲",
            ]
        )
        self.actions_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.actions_table.verticalHeader().setVisible(False)
        self.actions_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.actions_table.setMinimumHeight(220)
        for name, values in self.settings.data["o6"]["actions"].items():
            self._add_action(name, values)
        self.actions_table.itemChanged.connect(self._refresh_action_choices)
        layout.addWidget(self.actions_table)
        actions_row = QHBoxLayout()
        self.add_action_button = QPushButton("新增动作")
        self.add_action_button.clicked.connect(self._new_action)
        self.delete_action_button = QPushButton("删除选中动作")
        self.delete_action_button.clicked.connect(self._delete_action)
        actions_row.addWidget(self.add_action_button)
        actions_row.addWidget(self.delete_action_button)
        actions_row.addStretch()
        layout.addLayout(actions_row)
        self.open_action = QComboBox()
        self.closed_action = QComboBox()
        self._refresh_action_choices()
        self.open_action.setCurrentText(self.settings.data["o6"]["open_action"])
        self.closed_action.setCurrentText(self.settings.data["o6"]["closed_action"])
        form = QFormLayout()
        form.addRow("J7 张开动作", self.open_action)
        form.addRow("J7 闭合动作", self.closed_action)
        layout.addLayout(form)

    def _add_action(self, name, values):
        row = self.actions_table.rowCount()
        self.actions_table.insertRow(row)
        self.actions_table.setItem(row, 0, QTableWidgetItem(name))
        for column, value in enumerate(values, 1):
            spin = QSpinBox()
            spin.setRange(0, 255)
            spin.setValue(value)
            self.actions_table.setCellWidget(row, column, spin)

    def _new_action(self):
        names = {
            self.actions_table.item(row, 0).text() for row in range(self.actions_table.rowCount())
        }
        index = 1
        while f"新动作 {index}" in names:
            index += 1
        # Start from the selected saved pose, never an invented hardware target.
        values = self.settings.data["o6"]["actions"]["张开手"]
        selected = self.actions_table.currentRow()
        if selected >= 0:
            values = [self.actions_table.cellWidget(selected, c).value() for c in range(1, 7)]
        self._add_action(f"新动作 {index}", values)
        self._refresh_action_choices()
        self.actions_table.selectRow(self.actions_table.rowCount() - 1)

    def _delete_action(self):
        row = self.actions_table.currentRow()
        if row < 0:
            self.error_label.setText("请先选择要删除的动作。")
            return
        name = self.actions_table.item(row, 0).text().strip()
        if name in (
            "张开手",
            "抓取",
            self.open_action.currentText(),
            self.closed_action.currentText(),
        ):
            self.error_label.setText("张开手、抓取与已选开合动作需要保留；请先切换开合选择。")
            return
        self.actions_table.removeRow(row)
        self._refresh_action_choices()

    def _refresh_action_choices(self):
        names = [
            self.actions_table.item(row, 0).text().strip()
            for row in range(self.actions_table.rowCount())
        ]
        for combo in (self.open_action, self.closed_action):
            current = combo.currentText()
            combo.clear()
            combo.addItems(names)
            combo.setCurrentIndex(combo.findText(current))

    def _candidate(self):
        candidate = copy.deepcopy(self.settings.data)
        for fields in SETTINGS_GROUPS.values():
            for field in fields:
                section, key = field.path.split(".")
                candidate[section][key] = field.parse(
                    [edit.text() for edit in self.editors[field.path]]
                )
        candidate["dataset"]["recording_mode"] = self.recording_mode.currentData()
        streams = [
            {"name": name.text().strip(),
             "serial": serial.currentText().split(" · ", 1)[0].strip(),
             "mode": mode.currentData(),
             "roi_norm": [float(edit.text()) for edit in roi_edits]}
            for name, serial, mode, roi_edits in self.camera_editors
        ]
        # Preserve an untouched legacy config; persist explicit slots on the first edit.
        original = resolve_camera_streams(self.settings.data["dataset"])
        parsed = resolve_camera_streams({"camera_streams": streams})
        if self.settings.data["dataset"].get("camera_streams") is not None or any(
            (a.name, a.serial, a.mode, tuple(a.roi_norm)) !=
            (b.name, b.serial, b.mode, tuple(b.roi_norm))
            for a, b in zip(original, parsed, strict=True)
        ):
            candidate["dataset"]["camera_streams"] = streams
        actions = {}
        for row in range(self.actions_table.rowCount()):
            name = self.actions_table.item(row, 0).text().strip()
            if not name or name in actions:
                raise ValueError("O6 动作名称不能为空或重复")
            actions[name] = [self.actions_table.cellWidget(row, c).value() for c in range(1, 7)]
        if not {"张开手", "抓取"} <= actions.keys():
            raise ValueError("请保留内置动作名称“张开手”和“抓取”，可修改其目标值")
        o6 = candidate["o6"]
        o6["actions"] = actions
        for key, combo, endpoint in (
            ("open_action", self.open_action, "open"),
            ("closed_action", self.closed_action, "closed"),
        ):
            name = combo.currentText()
            if name not in actions:
                raise ValueError("请选择有效的 J7 张开 / 闭合动作")
            o6[key] = name
            previous = self.settings.data["o6"]
            if name != previous[key] or actions[name] != previous["actions"][name]:
                o6[endpoint] = actions[name].copy()
        return candidate

    def _save(self):
        try:
            self.settings.save(self._candidate())
        except (ValueError, OSError, RuntimeError) as exc:
            traceback.print_exception(exc)
            self.error_label.setText(f"保存失败：{exc}")
            return
        self.saved = True
        self.accept()

    def reject(self):
        try:
            dirty = self._candidate() != self._initial_candidate
        except ValueError:
            dirty = True
        if (
            dirty
            and QMessageBox.question(
                self,
                "放弃未保存修改",
                "参数尚未保存，是否放弃本次修改？",
                QMessageBox.Discard | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            != QMessageBox.Discard
        ):
            return
        super().reject()

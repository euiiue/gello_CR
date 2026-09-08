"""Offline parameter editor. No device access or motion commands."""

from __future__ import annotations

import copy
import traceback

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
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

PAGE_NOTES = {
    "机械臂": "ServoJ v/a/j 按 SDK 原值保存。关节保护对 J1–J6 生效；"
    "XYZ / 姿态限制仅用于笛卡尔模式，范围相对于跟随起点，不是绝对关节软限位。",
    "GELLO": "当前标定、关节方向与控制模式沿用原配置。反馈频率同时决定关节跟随循环频率。",
    "O6 动作": "编辑动作库并选择 J7 张开 / 闭合动作。仅保存参数，不发送手部动作。"
    "六列顺序：拇指弯曲、拇指侧摆、食指、中指、无名指、小指。",
    "数采与相机": "质量阈值用于标记 needs_review；机械臂停止阈值在“机械臂”页。"
    "输出保持三路 224×224 RGB；ROI 坐标归一化至 0–1。",
    "控制器连接": "连接参数在下次启动后使用；启动后仍需手动连接与使能。",
}


class OperatorSettingsDialog(QDialog):
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

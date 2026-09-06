"""RoArm-M2-Pro or Inverse3 -> NRC/CR5 -> LinkerHand O6 runtime.

The module deliberately contains no Qt code.  Hardware I/O runs in background
threads and the GUI only polls snapshots/events.  This keeps blocking serial and
NRC calls out of Qt's event loop.
"""

from __future__ import annotations

import copy
import json
import math
import os
import queue
import select
import shutil
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

import numpy as np

# V2 package compatibility during the staged migration. The legacy application
# is still launched from the repository root, while new code uses a src layout.
try:
    from gello_cr.control.hand_mapping import (
        binary_o6_action,
        interpolate_o6_target,
        should_send_o6_target,
    )
    from gello_cr.control.joint_mapping import RelativeJointMapper
    from gello_cr.control.joint_safety import (
        LeaderSpeedViolationCounter,
        max_command_step_violation,
        max_tracking_error_violation,
    )
    from gello_cr.devices.gello import GelloConfig, GelloDevice
    from gello_cr.devices.nrc_robot import NrcRobotSession, NrcServoTransition
    from gello_cr.devices.o6 import O6Config, O6Device
except ModuleNotFoundError as exc:
    if exc.name != "gello_cr":
        raise
    _V2_SRC_DIR = Path(__file__).resolve().parent / "src"
    if str(_V2_SRC_DIR) not in sys.path:
        sys.path.insert(0, str(_V2_SRC_DIR))
    from gello_cr.control.hand_mapping import (
        binary_o6_action,
        interpolate_o6_target,
        should_send_o6_target,
    )
    from gello_cr.control.joint_mapping import RelativeJointMapper
    from gello_cr.control.joint_safety import (
        LeaderSpeedViolationCounter,
        max_command_step_violation,
        max_tracking_error_violation,
    )
    from gello_cr.devices.gello import GelloConfig, GelloDevice
    from gello_cr.devices.nrc_robot import NrcRobotSession, NrcServoTransition
    from gello_cr.devices.o6 import O6Config, O6Device



ROARM_DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "36df624f63e4ee118642737d4c11a646-if00-port0"
)
O6_DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
O6_MOTOR_NAMES = (
    "大拇指弯曲",
    "大拇指侧摆",
    "食指弯曲",
    "中指弯曲",
    "无名指弯曲",
    "小拇指弯曲",
)
O6_FAULT_NAMES = {
    0: "正常",
    1: "电流过载",
    2: "温度过高",
    3: "编码器错误",
    4: "过压/欠压",
}
NRC_RESULT_NAMES = {
    -6: "通信超时",
    -5: "SDK 异常",
    -4: "当前状态不允许该操作",
    -3: "参数错误",
    -2: "控制柜连接断开",
    -1: "接收控制柜响应失败",
    0: "成功",
}
NRC_SERVO_STATE_NAMES = {
    0: "停止",
    1: "就绪",
    2: "报警",
    3: "运行",
}


DEFAULT_CONFIG: dict[str, Any] = {
    "version": 1,
    "master": {
        "type": "roarm",
    },
    "robot": {
        "ip": "192.168.1.245",
        "command_port": 6001,
        "servo_port": 7000,
        "robot_num": 1,
        "motion_mode": "servoj",
        "movej_velocity": 10.0,
        "movej_acc": 20.0,
        "movej_dec": 20.0,
        "movej_period_s": 0.1,
        "movej_low_latency": False,
        "movej_max_segment_deg": 2.0,
        "movej_duplicate_deadband_deg": 0.05,
        "safety_max_command_step_rad": math.radians(20.0),
        "safety_max_command_speed_rad_s": 0.8,
        "safety_speed_violation_cycles": 3,
        "safety_max_tracking_error_rad": math.radians(20.0),
        "servoj_vmax": 80.0,
        "servoj_amax": 200.0,
        "servoj_jmax": 150.0,
    },
    "roarm": {
        "port": ROARM_DEFAULT_PORT,
        "baudrate": 115200,
        "feedback_hz": 20.0,
        "feedback_timeout_s": 0.6,
        "m5_open_rad": 0.7977,
        "m5_closed_rad": 3.1447,
    },
    "inverse3": {
        "uri": "ws://127.0.0.1:10001",
        "device_id": "",
        "grip_device_id": "",
        "basis": "XYZ",
        "feedback_hz": 20.0,
        "feedback_timeout_s": 0.6,
    },
    "gello": {
        "port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB6W7LM-if00-port0",
        "baudrate": 57600,
        "software_root": str(Path(__file__).resolve().parent / "TESTMaster_GELLO/gello_software-main"),
        "control_mode": "joint",
        "joint_scale": 1.0,
        "kinematics_urdf": str(Path(__file__).resolve().parent / "TESTMaster_GELLO/gello_software-main/gello/factr/urdf/yam_active_gello/robot.urdf"),
        "xyz_axis_map": [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        "j6_sign": 1,
        "xyz_scale": [1.0, 1.0, 1.0],
        "joint_ids": [1, 2, 3, 4, 5, 6],
        "joint_offsets": [
            4.574337507945,
            3.060291671832,
            1.402049963022,
            1.363700443325,
            1.471087575582,
            2.994338975058,
        ],
        "joint_signs": [1, 1, -1, 1, 1, 1],
        "locked_joints": [],
        "gripper_config": [7, 200.478515625, 147.3046875],
        "feedback_hz": 100.0,
        "feedback_timeout_s": 0.5,
        "startup_settle_s": 0.5,
    },
    "o6": {
        "port": O6_DEFAULT_PORT,
        "baudrate": 115200,
        "hand_id": 0x27,
        "open": [253, 253, 253, 253, 253, 253],
        "closed": [90, 99, 50, 50, 50, 50],
        "speed": [20, 20, 20, 20, 20, 20],
        "torque": [30, 30, 30, 30, 30, 30],
        "command_deadband": 2,
        "feedback_timeout_s": 1.0,
        "action_timeout_s": 15.0,
        "actions": {
            "张开手": [250, 250, 250, 250, 250, 250],
            "中指": [91, 132, 0, 250, 0, 0],
            "轻微抓取": [107, 90, 129, 250, 250, 250],
            "小拇指": [30, 250, 40, 40, 40, 250],
            "抓取": [78, 85, 123, 250, 250, 250],
        },
    },
    "teleop": {
        "scale_xyz": [2.0, 2.0, 2.0],
        "scale_rpy": [1.0, 1.0, 1.0],
        "max_delta_xyz_mm": [100.0, 100.0, 100.0],
        "max_delta_rpy_rad": [0.35, 0.35, 0.35],
        "max_tcp_speed_mm_s": 30.0,
        "max_tcp_angular_speed_rad_s": 0.1,
        "max_master_jump_mm": 35.0,
        "max_master_angular_jump_rad": 0.35,
        "period_s": 0.05,
    },
    "preset": {
        "robot_velocity_percent": 10.0,
        "robot_acc_percent": 20.0,
        "robot_dec_percent": 20.0,
        "roarm_duration_s": 3.0,
        "timeout_s": 90.0,
    },
    "replay": {
        "file": "joint_replay.json",
        "robot_joint_indices": [1, 2, 3, 4, 5, 6],
        # Selected joints replay their recorded displacement from the position
        # at which replay is requested.  This makes an Episode insertion safe
        # to use at a taught point instead of jumping back to the old absolute
        # recording pose first.
        "relative_to_current": True,
        "servoj_period_s": 0.01,
        "mode_switch_settle_s": 0.2,
        "sample_period_s": 0.05,
        "max_duration_s": 300.0,
        "initial_move_velocity_percent": 10.0,
        "initial_move_acc_percent": 20.0,
        "initial_move_dec_percent": 20.0,
        "initial_tolerance_deg": 1.0,
        "initial_timeout_s": 90.0,
        "max_joint_speed_deg_s": 120.0,
    },
    "dataset": {
        "root": "/home/ace/datasets/cr5_o6_low_latency_test",
        "repo_prefix": "ace/cr5_o6",
        "task": "Pick up the object and place it in the target container.",
        "fps": 20,
        "image_size": [224, 224],
        "camera_max_age_s": 0.5,
        "camera_max_skew_s": 0.1,
        "action_max_age_s": 0.25,
        "max_tracking_error_deg": 5.0,
        "wrist_camera_serial": "317222074617",
        "base_camera_serial": "254622075848",
        "base_roi_norm": [0.37, 0.56, 0.55, 0.79],
        "worker_python": "/home/ace/miniconda3/envs/lerobot/bin/python",
    },
    "shortcuts": {
        "prepare": "F5",
        "power_on": "Ctrl+F6",
        "start_follow": "F6",
        "stop_follow": "F7",
        "episode_start": "F8",
        "episode_save_next": "F9",
        "episode_discard_retry": "F10",
        "home": "F11",
        "emergency_stop": "F12",
        "replay_start": "",
        "replay_stop": "",
        "master_free": "M",
        "o6_open": "1",
        "o6_grasp": "2",
        "o6_light_grasp": "3",
        "o6_m5": "4",
        "preset_a": "A",
        "preset_b": "B",
        "preset_c": "C",
        "preset_d": "D",
        "preset_save_a": "Shift+A",
        "preset_save_b": "Shift+B",
        "preset_save_c": "Shift+C",
        "preset_save_d": "Shift+D",
    },
    "presets": {},
}


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _six_uint8(values: Sequence[Any], name: str) -> list[int]:
    if len(values) != 6:
        raise ValueError(f"{name} 必须包含 6 个数值")
    converted = [int(round(float(value))) for value in values]
    if any(value < 0 or value > 255 for value in converted):
        raise ValueError(f"{name} 必须在 0..255 范围内")
    return converted


class TeleopConfigStore:
    """Load and atomically persist teleoperation settings and taught poses."""

    def __init__(self, path: Path | str):
        self.path = Path(path)
        self._lock = threading.RLock()
        self.data = self.load()

    def load(self) -> dict[str, Any]:
        with self._lock:
            if not self.path.exists():
                data = copy.deepcopy(DEFAULT_CONFIG)
                self._validate(data)
                return data
            try:
                loaded = json.loads(self.path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"读取遥操作配置失败: {self.path}: {exc}") from exc
            data = _deep_merge(DEFAULT_CONFIG, loaded)
            self._validate(data)
            return data

    def save(self) -> None:
        with self._lock:
            self._validate(self.data)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temp_path = self.path.with_suffix(self.path.suffix + ".tmp")
            temp_path.write_text(
                json.dumps(self.data, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            os.replace(temp_path, self.path)

    def update_runtime(self, values: dict[str, Any]) -> None:
        with self._lock:
            previous = self.data
            self.data = _deep_merge(self.data, values)
            try:
                self.save()
            except Exception:
                self.data = previous
                raise

    @staticmethod
    def _validate(data: dict[str, Any]) -> None:
        master_type = data["master"]["type"]
        if master_type not in ("roarm", "inverse3", "gello"):
            raise ValueError("主臂类型必须是 roarm、inverse3 或 gello")
        robot_num = data["robot"]["robot_num"]
        if (
            isinstance(robot_num, bool)
            or not isinstance(robot_num, int)
            or not 1 <= robot_num <= 4
        ):
            raise ValueError("NRC robot_num 必须是 1～4 的整数")
        robot = data["robot"]
        if not isinstance(robot["movej_low_latency"], bool):
            raise ValueError("MoveJ 低延迟开关必须是布尔值")
        segment_deg = float(robot["movej_max_segment_deg"])
        if not math.isfinite(segment_deg) or not 0.1 <= segment_deg <= 5.0:
            raise ValueError("MoveJ 单段上限必须在 0.1..5.0 度之间")
        duplicate_deadband = float(robot["movej_duplicate_deadband_deg"])
        if not math.isfinite(duplicate_deadband) or not 0.0 <= duplicate_deadband <= 1.0:
            raise ValueError("MoveJ 重复目标抑制阈值必须在 0.0..1.0 度之间")
        for key, label in (
            ("safety_max_command_step_rad", "GELLO 单帧跳变限制"),
            ("safety_max_command_speed_rad_s", "GELLO 速度限制"),
            ("safety_max_tracking_error_rad", "CR5 跟踪误差限制"),
        ):
            if not math.isfinite(float(robot[key])) or float(robot[key]) <= 0:
                raise ValueError(f"{label}必须大于 0")
        if int(robot["safety_speed_violation_cycles"]) < 1:
            raise ValueError("GELLO 速度超限连续帧数必须为正整数")
        scales = data["teleop"]["scale_xyz"]
        limits = data["teleop"]["max_delta_xyz_mm"]
        if len(scales) != 3 or len(limits) != 3:
            raise ValueError("XYZ 倍率和范围必须各包含 3 个值")
        if any(not math.isfinite(float(value)) or float(value) == 0 for value in scales):
            raise ValueError("XYZ 倍率必须是非零有限数")
        if any(float(value) <= 0 for value in limits):
            raise ValueError("XYZ 安全范围必须大于 0")
        if float(data["teleop"]["max_tcp_speed_mm_s"]) <= 0:
            raise ValueError("TCP 速度限制必须大于 0")
        rpy_scales = data["teleop"]["scale_rpy"]
        rpy_limits = data["teleop"]["max_delta_rpy_rad"]
        if len(rpy_scales) != 3 or len(rpy_limits) != 3:
            raise ValueError("RPY 倍率和范围必须各包含 3 个值")
        if any(
            not math.isfinite(float(value)) or float(value) == 0
            for value in rpy_scales
        ):
            raise ValueError("RPY 倍率必须是非零有限数")
        if any(
            not 0 < float(value) <= math.pi
            for value in rpy_limits
        ):
            raise ValueError("RPY 安全范围必须在 0..pi rad 内")
        if float(data["teleop"]["max_tcp_angular_speed_rad_s"]) <= 0:
            raise ValueError("TCP 姿态速度限制必须大于 0")
        angular_jump = float(data["teleop"]["max_master_angular_jump_rad"])
        if not 0 < angular_jump <= math.pi:
            raise ValueError("主臂姿态单帧跳变限制必须在 0..pi rad 内")
        inverse3 = data["inverse3"]
        if not str(inverse3["uri"]).startswith(("ws://", "wss://")):
            raise ValueError("Inverse Service 地址必须以 ws:// 或 wss:// 开头")
        if float(inverse3["feedback_hz"]) <= 0:
            raise ValueError("Inverse3 反馈频率必须大于 0")
        if float(inverse3["feedback_timeout_s"]) <= 0:
            raise ValueError("Inverse3 反馈超时必须大于 0")
        if not str(inverse3["basis"]).strip():
            raise ValueError("Inverse3 basis 不能为空")
        gello = data["gello"]
        if not str(gello["port"]).strip():
            raise ValueError("GELLO 串口不能为空")
        if int(gello["baudrate"]) <= 0:
            raise ValueError("GELLO 波特率必须大于 0")
        if not str(gello["software_root"]).strip():
            raise ValueError("GELLO 软件目录不能为空")
        if gello["control_mode"] not in (
            "joint",
            "xyz_j6",
            "xyz_j6_tcp_ik",
            "tcp_6d",
        ):
            raise ValueError(
                "GELLO control_mode 必须是 joint、xyz_j6、xyz_j6_tcp_ik 或 tcp_6d"
            )
        if not str(gello["kinematics_urdf"]).strip():
            raise ValueError("GELLO kinematics_urdf 不能为空")
        if gello["control_mode"] != "joint" and not Path(
            str(gello["kinematics_urdf"])
        ).expanduser().is_file():
            raise ValueError("GELLO TCP/XYZ 模式的 URDF 文件不存在")
        xyz_axis_map = gello["xyz_axis_map"]
        if (
            not isinstance(xyz_axis_map, list)
            or len(xyz_axis_map) != 3
            or any(
                not isinstance(row, list)
                or len(row) != 3
                or any(not math.isfinite(float(value)) for value in row)
                for row in xyz_axis_map
            )
        ):
            raise ValueError("GELLO xyz_axis_map 必须是 3x3 有限矩阵")
        if int(gello["j6_sign"]) not in (-1, 1):
            raise ValueError("GELLO j6_sign 必须为 ±1")
        xyz_scale = gello["xyz_scale"]
        if len(xyz_scale) != 3 or any(
            not math.isfinite(float(value)) or float(value) == 0
            for value in xyz_scale
        ):
            raise ValueError("GELLO xyz_scale 必须包含 3 个非零有限数")
        joint_ids = gello["joint_ids"]
        offsets = gello["joint_offsets"]
        signs = gello["joint_signs"]
        if len(joint_ids) != 6 or len(offsets) != 6 or len(signs) != 6:
            raise ValueError("GELLO 必须配置 6 个关节")
        if any(int(value) < 1 for value in joint_ids):
            raise ValueError("GELLO 关节 ID 必须为正整数")
        if any(not math.isfinite(float(value)) for value in offsets):
            raise ValueError("GELLO joint_offsets 必须是有限数")
        if any(int(value) not in (-1, 1) for value in signs):
            raise ValueError("GELLO joint_signs 必须为 ±1")
        locked_joints = gello["locked_joints"]
        if (
            not isinstance(locked_joints, list)
            or any(
                isinstance(value, bool)
                or not isinstance(value, int)
                or value < 1
                or value > 6
                for value in locked_joints
            )
            or len(set(locked_joints)) != len(locked_joints)
        ):
            raise ValueError("GELLO locked_joints 必须是 1～6 的不重复整数列表")
        gripper_config = gello["gripper_config"]
        if len(gripper_config) != 3:
            raise ValueError("GELLO gripper_config 必须包含 ID、张开端和闭合端")
        if int(gripper_config[0]) < 1:
            raise ValueError("GELLO gripper ID 必须为正整数")
        if float(gello["feedback_hz"]) <= 0 or float(gello["feedback_timeout_s"]) <= 0:
            raise ValueError("GELLO 反馈频率和超时必须大于 0")
        joint_scale = float(gello["joint_scale"])
        if not math.isfinite(joint_scale) or not 0.5 <= joint_scale <= 1.5:
            raise ValueError("GELLO J1-J6 关节倍率必须在 0.5..1.5 之间")
        if float(gello["startup_settle_s"]) < 0:
            raise ValueError("GELLO 启动稳定时间不能为负数")
        preset = data["preset"]
        for key, label in (
            ("robot_velocity_percent", "CR5 回位速度"),
            ("robot_acc_percent", "CR5 回位加速度"),
            ("robot_dec_percent", "CR5 回位减速度"),
        ):
            value = float(preset[key])
            if not 0 < value <= 100:
                raise ValueError(f"{label}必须在 0..100% 之间")
        if float(preset["roarm_duration_s"]) <= 0:
            raise ValueError("RoArm 主动回位时长必须大于 0")
        if float(data["preset"]["timeout_s"]) <= 0:
            raise ValueError("预设回位等待上限必须大于 0")
        replay = data["replay"]
        if not str(replay["file"]).strip():
            raise ValueError("关节轨迹文件名不能为空")
        joint_indices = replay["robot_joint_indices"]
        if (
            not isinstance(joint_indices, list)
            or not joint_indices
            or any(
                isinstance(value, bool)
                or not isinstance(value, int)
                or value < 1
                or value > 6
                for value in joint_indices
            )
            or len(set(joint_indices)) != len(joint_indices)
        ):
            raise ValueError("轨迹关节必须是 J1～J6 中至少一个且不能重复")
        if not 0.01 <= float(replay["sample_period_s"]) <= 1.0:
            raise ValueError("关节录制采样周期必须在 0.01..1.0 秒")
        if not isinstance(replay["relative_to_current"], bool):
            raise ValueError("关节重放 relative_to_current 必须是布尔值")
        if not 0.005 <= float(replay["servoj_period_s"]) <= 0.02:
            raise ValueError("ServoJ 重放周期必须在 0.005..0.02 秒")
        if not 0.0 <= float(replay["mode_switch_settle_s"]) <= 1.0:
            raise ValueError("运动模式切换稳定时间必须在 0..1 秒")
        if float(replay["max_duration_s"]) <= 0:
            raise ValueError("关节录制最长时间必须大于 0")
        if float(replay["initial_timeout_s"]) <= 0:
            raise ValueError("关节重放初始对齐超时必须大于 0")
        if not 0 < float(replay["initial_move_velocity_percent"]) <= 100:
            raise ValueError("关节重放首帧对齐速度必须在 0..100% 之间")
        if float(replay["initial_tolerance_deg"]) <= 0:
            raise ValueError("关节重放初始对齐容差必须大于 0")
        if float(replay["max_joint_speed_deg_s"]) <= 0:
            raise ValueError("关节重放速度安全上限必须大于 0")
        m5_open = float(data["roarm"]["m5_open_rad"])
        m5_closed = float(data["roarm"]["m5_closed_rad"])
        if abs(m5_closed - m5_open) < 0.1:
            raise ValueError("RoArm M5 开合端点过于接近")
        _six_uint8(data["o6"]["open"], "O6 张开端点")
        _six_uint8(data["o6"]["closed"], "O6 闭合端点")
        _six_uint8(data["o6"]["speed"], "O6 速度")
        _six_uint8(data["o6"]["torque"], "O6 力矩")
        if float(data["o6"]["action_timeout_s"]) <= 0:
            raise ValueError("O6 动作等待上限必须大于 0")
        actions = data["o6"]["actions"]
        if not isinstance(actions, dict) or not actions:
            raise ValueError("O6 快捷动作必须是非空字典")
        for action_name, target in actions.items():
            if not isinstance(action_name, str) or not action_name.strip():
                raise ValueError("O6 快捷动作名称不能为空")
            _six_uint8(target, f"O6 快捷动作 {action_name}")
        dataset = data["dataset"]
        if not str(dataset["root"]).strip():
            raise ValueError("LeRobot 数据集根目录不能为空")
        repo_prefix = str(dataset["repo_prefix"]).strip()
        if repo_prefix.count("/") != 1 or any(
            not part for part in repo_prefix.split("/", 1)
        ):
            raise ValueError("LeRobot repo_prefix 必须是 owner/name 格式")
        if not str(dataset["task"]).strip():
            raise ValueError("LeRobot 任务文字不能为空")
        if not 1 <= int(dataset["fps"]) <= 60:
            raise ValueError("LeRobot FPS 必须在 1..60 之间")
        image_size = dataset["image_size"]
        if (
            not isinstance(image_size, list)
            or len(image_size) != 2
            or any(int(value) <= 0 for value in image_size)
        ):
            raise ValueError("LeRobot image_size 必须是 [height, width]")
        if float(dataset["camera_max_age_s"]) <= 0:
            raise ValueError("LeRobot 相机帧超时必须大于 0")
        for key in ("camera_max_skew_s", "action_max_age_s", "max_tracking_error_deg"):
            if not math.isfinite(float(dataset[key])) or float(dataset[key]) <= 0:
                raise ValueError(f"LeRobot {key} 必须是大于 0 的有限数")
        if not str(dataset["wrist_camera_serial"]).strip():
            raise ValueError("CAMERA1 腕部相机序列号不能为空")
        if not str(dataset["base_camera_serial"]).strip():
            raise ValueError("CAMERA2 基座相机序列号不能为空")
        if dataset["wrist_camera_serial"] == dataset["base_camera_serial"]:
            raise ValueError("CAMERA1 和 CAMERA2 不能配置为同一个序列号")
        roi = dataset["base_roi_norm"]
        if not isinstance(roi, list) or len(roi) != 4:
            raise ValueError("base_roi_norm 必须是 [x1, y1, x2, y2]")
        x1, y1, x2, y2 = [float(value) for value in roi]
        if not (0.0 <= x1 < x2 <= 1.0 and 0.0 <= y1 < y2 <= 1.0):
            raise ValueError("base_roi_norm 必须满足 0<=x1<x2<=1 且 0<=y1<y2<=1")
        if not str(dataset["worker_python"]).strip():
            raise ValueError("LeRobot worker_python 不能为空")
        shortcuts = data["shortcuts"]
        if not isinstance(shortcuts, dict):
            raise ValueError("shortcuts 必须是字典")
        missing_shortcuts = set(DEFAULT_CONFIG["shortcuts"]) - set(shortcuts)
        if missing_shortcuts:
            raise ValueError(
                "缺少快捷键配置: " + ", ".join(sorted(missing_shortcuts))
            )
        assigned: dict[str, str] = {}
        for action_name in DEFAULT_CONFIG["shortcuts"]:
            sequence = str(shortcuts[action_name]).strip()
            if not sequence:
                continue
            normalized = sequence.casefold()
            if normalized in assigned:
                raise ValueError(
                    f"快捷键 {sequence} 同时分配给 {assigned[normalized]} 和 {action_name}"
                )
            assigned[normalized] = action_name


@dataclass(frozen=True)
class RoArmFeedback:
    timestamp: float
    x: float
    y: float
    z: float
    b: float
    s: float
    e: float
    t: float
    torques: tuple[float, float, float, float]

    @property
    def xyz(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def joints(self) -> tuple[float, float, float, float]:
        return (self.b, self.s, self.e, self.t)

    @classmethod
    def from_json(cls, value: dict[str, Any]) -> "RoArmFeedback":
        required = ("x", "y", "z", "b", "s", "e", "t")
        if any(key not in value for key in required):
            raise ValueError("RoArm T=1051 反馈字段不完整")
        numbers = [float(value[key]) for key in required]
        if any(not math.isfinite(number) for number in numbers):
            raise ValueError("RoArm 反馈包含非有限数")
        return cls(
            timestamp=time.monotonic(),
            x=numbers[0],
            y=numbers[1],
            z=numbers[2],
            b=numbers[3],
            s=numbers[4],
            e=numbers[5],
            t=numbers[6],
            torques=(
                float(value.get("torB", 0.0)),
                float(value.get("torS", 0.0)),
                float(value.get("torE", 0.0)),
                float(value.get("torH", 0.0)),
            ),
        )


def _normalize_quaternion(values: Sequence[Any]) -> tuple[float, float, float, float]:
    if len(values) != 4:
        raise ValueError("四元数必须包含 w/x/y/z 四个值")
    quaternion = tuple(float(value) for value in values)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("四元数无效")
    return tuple(value / norm for value in quaternion)


def _quaternion_multiply(
    left: Sequence[Any], right: Sequence[Any]
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = _normalize_quaternion(left)
    rw, rx, ry, rz = _normalize_quaternion(right)
    return _normalize_quaternion(
        (
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        )
    )


def _quaternion_inverse(values: Sequence[Any]) -> tuple[float, float, float, float]:
    w, x, y, z = _normalize_quaternion(values)
    return (w, -x, -y, -z)


def _rotation_matrix_to_quaternion(
    matrix: Sequence[Sequence[float]],
) -> tuple[float, float, float, float]:
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("旋转矩阵必须是 3x3")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            0.25 * scale,
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
        )
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        quaternion = (
            (rotation[2, 1] - rotation[1, 2]) / scale,
            0.25 * scale,
            (rotation[0, 1] + rotation[1, 0]) / scale,
            (rotation[0, 2] + rotation[2, 0]) / scale,
        )
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        quaternion = (
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[0, 1] + rotation[1, 0]) / scale,
            0.25 * scale,
            (rotation[1, 2] + rotation[2, 1]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        quaternion = (
            (rotation[1, 0] - rotation[0, 1]) / scale,
            (rotation[0, 2] + rotation[2, 0]) / scale,
            (rotation[1, 2] + rotation[2, 1]) / scale,
            0.25 * scale,
        )
    return _normalize_quaternion(quaternion)


def _nrc_abc_to_quaternion(
    values: Sequence[Any],
) -> tuple[float, float, float, float]:
    if len(values) != 3:
        raise ValueError("NRC ABC 必须包含三个值")
    a, b, c = (float(value) for value in values)
    qa = (math.cos(a / 2.0), math.sin(a / 2.0), 0.0, 0.0)
    qb = (math.cos(b / 2.0), 0.0, math.sin(b / 2.0), 0.0)
    qc = (math.cos(c / 2.0), 0.0, 0.0, math.sin(c / 2.0))
    return _quaternion_multiply(_quaternion_multiply(qa, qb), qc)


def _quaternion_to_nrc_abc(
    values: Sequence[Any],
) -> tuple[float, float, float]:
    """Convert a relative quaternion to NRC intrinsic X'-Y'-Z' Euler radians."""
    w, x, y, z = _normalize_quaternion(values)
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - w * z)
    r02 = 2.0 * (x * z + w * y)
    r10 = 2.0 * (x * y + w * z)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    b = math.asin(max(-1.0, min(1.0, r02)))
    if abs(math.cos(b)) > 1e-8:
        a = math.atan2(-r12, r22)
        c = math.atan2(-r01, r00)
    elif b > 0:
        a = math.atan2(r10, r11)
        c = 0.0
    else:
        a = math.atan2(-r10, r11)
        c = 0.0
    return (a, b, c)


def _quaternion_to_nrc_abc_near(
    values: Sequence[Any], reference: Sequence[Any]
) -> tuple[float, float, float]:
    if len(reference) != 3:
        raise ValueError("NRC ABC 参考值必须包含三个值")
    a, b, c = _quaternion_to_nrc_abc(values)
    raw_candidates = (
        (a, b, c),
        (a + math.pi, math.pi - b, c + math.pi),
        (a + math.pi, -math.pi - b, c + math.pi),
    )
    target = tuple(float(value) for value in reference)
    candidates = []
    for candidate in raw_candidates:
        adjusted = tuple(
            value + 2.0 * math.pi * round((target[index] - value) / (2.0 * math.pi))
            for index, value in enumerate(candidate)
        )
        candidates.append(adjusted)
    return min(
        candidates,
        key=lambda candidate: sum(
            (candidate[index] - target[index]) ** 2 for index in range(3)
        ),
    )


def _wrapped_angle_delta(target: float, origin: float) -> float:
    return (float(target) - float(origin) + math.pi) % (2.0 * math.pi) - math.pi


def _quaternion_distance(left: Sequence[Any], right: Sequence[Any]) -> float:
    first = _normalize_quaternion(left)
    second = _normalize_quaternion(right)
    dot = abs(sum(a * b for a, b in zip(first, second)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _quaternion_slerp(
    origin: Sequence[Any], target: Sequence[Any], ratio: float
) -> tuple[float, float, float, float]:
    first = _normalize_quaternion(origin)
    second = _normalize_quaternion(target)
    amount = max(0.0, min(1.0, float(ratio)))
    dot = sum(a * b for a, b in zip(first, second))
    if dot < 0.0:
        second = tuple(-value for value in second)
        dot = -dot
    if dot > 0.9995:
        return _normalize_quaternion(
            tuple(
                first[index] + amount * (second[index] - first[index])
                for index in range(4)
            )
        )
    angle = math.acos(max(-1.0, min(1.0, dot)))
    denominator = math.sin(angle)
    first_weight = math.sin((1.0 - amount) * angle) / denominator
    second_weight = math.sin(amount * angle) / denominator
    return _normalize_quaternion(
        tuple(
            first_weight * first[index] + second_weight * second[index]
            for index in range(4)
        )
    )


@dataclass(frozen=True)
class Inverse3Feedback:
    timestamp: float
    x: float
    y: float
    z: float
    orientation_wxyz: Optional[tuple[float, float, float, float]]
    inverse3_device_id: str
    grip_device_id: str
    grip_type: str
    buttons: tuple[bool, bool, bool]

    @property
    def xyz(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def rpy_rad(self) -> Optional[tuple[float, float, float]]:
        if self.orientation_wxyz is None:
            return None
        return _quaternion_to_nrc_abc(self.orientation_wxyz)


class Inverse3Controller:
    """Read-only Haply Inverse Service client for Inverse3 + VerseGrip."""

    master_type = "inverse3"
    passive = True

    def __init__(
        self,
        uri: str,
        device_id: str = "",
        grip_device_id: str = "",
        feedback_hz: float = 20.0,
        basis: str = "XYZ",
        websocket_factory: Optional[Any] = None,
    ):
        self.uri = str(uri)
        self.device_id = str(device_id)
        self.grip_device_id = str(grip_device_id)
        self.feedback_hz = float(feedback_hz)
        self.basis = str(basis)
        self._websocket_factory = websocket_factory
        self._socket: Optional[Any] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._stream_ready = threading.Event()
        self._lock = threading.RLock()
        self._latest: Optional[Inverse3Feedback] = None
        self._error = ""
        self._selected_inverse3_id = ""
        self._selected_grip_id = ""
        self._selected_grip_type = ""

    @property
    def io_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def connected(self) -> bool:
        return self.io_alive and not self.error

    @property
    def error(self) -> str:
        with self._lock:
            return self._error

    def latest(self) -> Optional[Inverse3Feedback]:
        with self._lock:
            return self._latest

    def connect(self, timeout: float = 4.0) -> Inverse3Feedback:
        if self.connected:
            latest = self.latest()
            if latest is None:
                raise RuntimeError("Inverse3 已连接但没有反馈")
            if latest.orientation_wxyz is not None:
                return latest
            self.close()
        if self._thread is not None:
            self.close()

        self._stop.clear()
        self._stream_ready.clear()
        with self._lock:
            self._latest = None
            self._error = ""
        factory = self._websocket_factory
        if factory is None:
            import websocket

            factory = lambda uri, connect_timeout: websocket.create_connection(
                uri,
                timeout=connect_timeout,
                enable_multithread=True,
            )
        self._socket = factory(self.uri, float(timeout))
        try:
            first_frame = json.loads(self._socket.recv())
            self._select_devices(first_frame)
            self._update_feedback(first_frame)
        except Exception:
            self._socket.close()
            self._socket = None
            raise
        self._thread = threading.Thread(
            target=self._run,
            name="Inverse3-ReadOnly",
            daemon=True,
        )
        self._thread.start()
        if not self._stream_ready.wait(timeout):
            error = self.error or "Inverse3 只读状态流启动超时"
            self.close()
            raise TimeoutError(error)
        if self.error:
            error = self.error
            self.close()
            raise RuntimeError(error)
        latest = self.latest()
        if latest is None:
            raise RuntimeError("Inverse3 没有有效反馈")
        return latest

    def close(self, hold: bool = False) -> None:
        del hold
        self._stop.set()
        socket = self._socket
        if socket is not None:
            socket.close()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        if thread is not None and thread.is_alive():
            raise TimeoutError("Inverse3 只读状态线程未能退出")
        self._socket = None
        self._thread = None

    @staticmethod
    def _find_device(
        frame: dict[str, Any], group: str, device_id: str
    ) -> Optional[dict[str, Any]]:
        devices = frame.get(group, [])
        if device_id:
            return next(
                (device for device in devices if device.get("device_id") == device_id),
                None,
            )
        return devices[0] if devices else None

    @staticmethod
    def _validate_inverse3_status(device: dict[str, Any]) -> None:
        status = device.get("status", {})
        missing = [
            name
            for name in ("calibrated", "ready", "started")
            if not status.get(name)
        ]
        if missing:
            raise RuntimeError("Inverse3 状态未就绪: " + ", ".join(missing))
        if status.get("in_use"):
            raise RuntimeError(
                "Inverse3 正被其他 Haply 控制会话占用；"
                "请先退出 GEL/其他力或位置控制演示"
            )
        control_mode = str(
            device.get("state", {}).get(
                "control_mode", device.get("state", {}).get("mode", "idle")
            )
        )
        if control_mode not in ("", "idle"):
            raise RuntimeError(
                f"Inverse3 当前 control_mode={control_mode}，只读遥操作要求 idle"
            )

    @staticmethod
    def _validate_grip_status(group: str, device: dict[str, Any]) -> None:
        status = device.get("status", {})
        if not status.get("ready"):
            raise RuntimeError("VerseGrip 状态未就绪: ready")
        if group == "wireless_verse_grip":
            missing = [name for name in ("connected", "awake") if not status.get(name)]
            if missing:
                raise RuntimeError("Wireless VerseGrip 状态未就绪: " + ", ".join(missing))
        elif int(status.get("error", 0)) != 0:
            raise RuntimeError(f"VerseGrip 报错: {status['error']}")

    def _select_devices(self, frame: dict[str, Any]) -> None:
        inverse3 = self._find_device(frame, "inverse3", self.device_id)
        if inverse3 is None:
            target = f" {self.device_id}" if self.device_id else ""
            raise RuntimeError(f"Inverse Service 未发现 Inverse3{target}")
        self._validate_inverse3_status(inverse3)
        self._selected_inverse3_id = str(inverse3["device_id"])
        actual_basis = str(
            inverse3.get("config", {}).get("basis", {}).get("permutation", "")
        )
        if actual_basis != self.basis:
            raise RuntimeError(
                f"Inverse3 会话 basis={actual_basis or '--'}，"
                f"配置要求 {self.basis}；禁止在未知坐标映射下跟随"
            )

        candidates: list[tuple[str, dict[str, Any]]] = []
        for group in ("wireless_verse_grip", "verse_grip"):
            candidates.extend(
                (group, grip)
                for grip in frame.get(group, [])
                if not self.grip_device_id
                or grip.get("device_id") == self.grip_device_id
            )
        if len(candidates) > 1:
            raise RuntimeError("检测到多个 VerseGrip，请在配置中填写唯一 grip_device_id")
        if candidates:
            self._selected_grip_type, grip = candidates[0]
            self._selected_grip_id = str(grip["device_id"])
            self._validate_grip_status(self._selected_grip_type, grip)
            grip_basis = str(
                grip.get("config", {}).get("basis", {}).get("permutation", "")
            )
            if grip_basis != self.basis:
                raise RuntimeError(
                    f"VerseGrip 会话 basis={grip_basis or '--'}，"
                    f"配置要求 {self.basis}；禁止混用位置/姿态坐标系"
                )
        else:
            self._selected_grip_type = ""
            self._selected_grip_id = ""

    def _update_feedback(self, frame: dict[str, Any]) -> None:
        inverse3 = self._find_device(
            frame, "inverse3", self._selected_inverse3_id
        )
        if inverse3 is None:
            raise RuntimeError("Inverse3 状态流中目标设备消失")
        self._validate_inverse3_status(inverse3)
        position = inverse3["state"]["cursor_position"]
        xyz_m = tuple(float(position[axis]) for axis in "xyz")
        if any(not math.isfinite(value) for value in xyz_m):
            raise RuntimeError("Inverse3 cursor_position 包含非有限数")

        orientation: Optional[tuple[float, float, float, float]] = None
        buttons = (False, False, False)
        if self._selected_grip_type:
            grip = self._find_device(
                frame, self._selected_grip_type, self._selected_grip_id
            )
            if grip is None:
                raise RuntimeError("VerseGrip 状态流中目标设备消失")
            self._validate_grip_status(self._selected_grip_type, grip)
            raw_orientation = grip["state"]["orientation"]
            orientation = _normalize_quaternion(
                tuple(raw_orientation[name] for name in "wxyz")
            )
            state = grip["state"]
            if self._selected_grip_type == "wireless_verse_grip":
                raw_buttons = state.get("buttons", {})
                buttons = tuple(bool(raw_buttons.get(name, False)) for name in "abc")
            else:
                buttons = (bool(state.get("button", False)), False, False)

        feedback = Inverse3Feedback(
            timestamp=time.monotonic(),
            x=xyz_m[0] * 1000.0,
            y=xyz_m[1] * 1000.0,
            z=xyz_m[2] * 1000.0,
            orientation_wxyz=orientation,
            inverse3_device_id=self._selected_inverse3_id,
            grip_device_id=self._selected_grip_id,
            grip_type=self._selected_grip_type,
            buttons=buttons,
        )
        with self._lock:
            self._latest = feedback

    def _probe_message(self) -> dict[str, Any]:
        message: dict[str, Any] = {
            "inverse3": [
                {
                    "device_id": self._selected_inverse3_id,
                    "commands": {"probe_position": {}},
                }
            ]
        }
        if self._selected_grip_type:
            message[self._selected_grip_type] = [
                {
                    "device_id": self._selected_grip_id,
                    "commands": {"probe_orientation": {}},
                }
            ]
        return message

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._error = message
        self._stream_ready.set()

    def _run(self) -> None:
        period = 1.0 / self.feedback_hz
        next_cycle = time.monotonic()
        try:
            while not self._stop.is_set():
                self._socket.send(
                    json.dumps(
                        self._probe_message(),
                        separators=(",", ":"),
                    )
                )
                frame = json.loads(self._socket.recv())
                self._update_feedback(frame)
                self._stream_ready.set()
                next_cycle += period
                wait_time = next_cycle - time.monotonic()
                if wait_time > 0:
                    self._stop.wait(wait_time)
                else:
                    next_cycle = time.monotonic()
        except Exception as exc:
            if not self._stop.is_set():
                self._set_error(
                    f"Inverse3 只读状态线程异常: {type(exc).__name__}: {exc}"
                )
        finally:
            self._stream_ready.set()


@dataclass(frozen=True)
class GelloFeedback:
    """Legacy GELLO feedback view kept while the old UI/runtime is migrated."""

    timestamp: float
    joints_rad: tuple[float, float, float, float, float, float, float]

    @property
    def arm_joints_rad(self) -> tuple[float, float, float, float, float, float]:
        return self.joints_rad[:6]

    @property
    def gripper(self) -> float:
        return float(self.joints_rad[6])

    @property
    def joints(self) -> tuple[float, float, float, float]:
        return self.joints_rad[:4]


class GelloController:
    """Legacy-compatible facade over the V2 GelloDevice."""

    master_type = "gello"
    passive = True

    def __init__(
        self,
        port: str,
        software_root: str,
        joint_ids: Sequence[int],
        joint_offsets: Sequence[float],
        joint_signs: Sequence[int],
        gripper_config: Sequence[float],
        baudrate: int = 57600,
        feedback_hz: float = 20.0,
        feedback_timeout_s: float = 0.5,
    ):
        self.port = str(port)
        self.software_root = str(software_root)
        self.joint_ids = tuple(int(value) for value in joint_ids)
        self.joint_offsets = tuple(float(value) for value in joint_offsets)
        self.joint_signs = tuple(int(value) for value in joint_signs)
        self.gripper_config = tuple(gripper_config)
        self.baudrate = int(baudrate)
        self.feedback_hz = float(feedback_hz)
        self.feedback_timeout_s = float(feedback_timeout_s)

        if len(self.gripper_config) != 3:
            raise ValueError("GELLO gripper_config 必须包含 ID/open/close 三个值")

        config = GelloConfig(
            port=self.port,
            software_root=self.software_root,
            joint_ids=self.joint_ids,
            joint_offsets=self.joint_offsets,
            joint_signs=self.joint_signs,
            gripper_config=(
                int(self.gripper_config[0]),
                float(self.gripper_config[1]),
                float(self.gripper_config[2]),
            ),
            baudrate=self.baudrate,
            feedback_hz=self.feedback_hz,
            feedback_timeout_s=self.feedback_timeout_s,
        )
        self._device = GelloDevice(config)

    @property
    def io_alive(self) -> bool:
        return self._device.io_alive

    @property
    def connected(self) -> bool:
        return self._device.connected

    @property
    def error(self) -> str:
        return self._device.error

    @staticmethod
    def _legacy_feedback(snapshot: Any) -> GelloFeedback:
        arm = tuple(float(value) for value in snapshot.joints.positions_rad)
        if len(arm) != 6:
            raise RuntimeError("V2 GELLO arm feedback must contain 6 joints")
        try:
            gripper = float(snapshot.auxiliary["gripper"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("V2 GELLO feedback is missing auxiliary gripper") from exc
        return GelloFeedback(
            timestamp=float(snapshot.timestamp),
            joints_rad=(
                arm[0],
                arm[1],
                arm[2],
                arm[3],
                arm[4],
                arm[5],
                gripper,
            ),
        )

    def latest(self) -> Optional[GelloFeedback]:
        snapshot = self._device.latest()
        if snapshot is None:
            return None
        return self._legacy_feedback(snapshot)

    def connect(self, timeout: float = 5.0) -> GelloFeedback:
        return self._legacy_feedback(self._device.connect(timeout=float(timeout)))

    def close(self, hold: bool = False) -> None:
        del hold
        self._device.close()


@dataclass(frozen=True)
class _GelloUrdfJoint:
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


class GelloKinematics:
    """Minimal URDF FK for the six joints of the physical GELLO."""

    def __init__(self, urdf_path: str | Path):
        root = ET.parse(str(Path(urdf_path).expanduser())).getroot()
        joints: dict[int, _GelloUrdfJoint] = {}
        for element in root.findall("joint"):
            name = str(element.attrib["name"])
            if not name.startswith("joint"):
                continue
            number = int(name.removeprefix("joint"))
            origin = element.find("origin")
            axis = element.find("axis")
            if origin is None or axis is None:
                raise ValueError(f"GELLO URDF 关节 {name} 缺少 origin 或 axis")
            origin_xyz = tuple(
                float(value)
                for value in str(origin.attrib.get("xyz", "0 0 0")).split()
            )
            origin_rpy = tuple(
                float(value)
                for value in str(origin.attrib.get("rpy", "0 0 0")).split()
            )
            joint_axis = tuple(float(value) for value in str(axis.attrib["xyz"]).split())
            if len(origin_xyz) != 3 or len(origin_rpy) != 3 or len(joint_axis) != 3:
                raise ValueError(f"GELLO URDF 关节 {name} 参数长度错误")
            joints[number] = _GelloUrdfJoint(origin_xyz, origin_rpy, joint_axis)
        if tuple(sorted(joints)) != (1, 2, 3, 4, 5, 6):
            raise ValueError("GELLO URDF 必须包含 joint1～joint6")
        self._joints = tuple(joints[index] for index in range(1, 7))

    @staticmethod
    def _rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
        roll, pitch, yaw = (float(value) for value in rpy)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=float,
        )

    @staticmethod
    def _axis_matrix(axis: Sequence[float], angle: float) -> np.ndarray:
        vector = np.asarray(axis, dtype=float)
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-12:
            raise ValueError("GELLO URDF 关节轴不能为零")
        x, y, z = vector / norm
        cosine = math.cos(float(angle))
        sine = math.sin(float(angle))
        one_minus_cosine = 1.0 - cosine
        return np.array(
            [
                [cosine + x * x * one_minus_cosine,
                 x * y * one_minus_cosine - z * sine,
                 x * z * one_minus_cosine + y * sine],
                [y * x * one_minus_cosine + z * sine,
                 cosine + y * y * one_minus_cosine,
                 y * z * one_minus_cosine - x * sine],
                [z * x * one_minus_cosine - y * sine,
                 z * y * one_minus_cosine + x * sine,
                 cosine + z * z * one_minus_cosine],
            ],
            dtype=float,
        )

    def pose(self, joints_rad: Sequence[float]) -> np.ndarray:
        if len(joints_rad) != 6:
            raise ValueError("GELLO FK 输入必须包含 6 个关节")
        transform = np.eye(4, dtype=float)
        for joint, angle in zip(self._joints, joints_rad):
            origin_rotation = self._rpy_matrix(joint.origin_rpy)
            origin_transform = np.eye(4, dtype=float)
            origin_transform[:3, :3] = origin_rotation
            origin_transform[:3, 3] = np.asarray(joint.origin_xyz, dtype=float)
            joint_transform = np.eye(4, dtype=float)
            joint_transform[:3, :3] = self._axis_matrix(joint.axis, float(angle))
            transform = transform @ origin_transform @ joint_transform
        return transform

    def position(self, joints_rad: Sequence[float]) -> np.ndarray:
        return self.pose(joints_rad)[:3, 3].copy()


class RoArmSerialController:
    """Long-lived RoArm JSON/UART connection that never toggles DTR/RTS."""

    master_type = "roarm"
    passive = False

    def __init__(self, port: str, baudrate: int = 115200, feedback_hz: float = 20.0):
        self.port = port
        self.baudrate = int(baudrate)
        self.feedback_hz = max(2.0, float(feedback_hz))
        self._fd: Optional[int] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._feedback_ready = threading.Event()
        self._commands: queue.Queue[tuple[dict[str, Any], Optional[threading.Event], list[Any]]] = queue.Queue()
        self._lock = threading.RLock()
        self._latest: Optional[RoArmFeedback] = None
        self._error = ""
        self._boot_detected = False

    @property
    def io_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def connected(self) -> bool:
        return self.io_alive and not self._error

    @property
    def error(self) -> str:
        with self._lock:
            return self._error

    def latest(self) -> Optional[RoArmFeedback]:
        with self._lock:
            return self._latest

    def connect(self, timeout: float = 4.0) -> RoArmFeedback:
        if self.connected:
            latest = self.latest()
            if latest is None:
                raise RuntimeError("RoArm 已连接但没有反馈")
            return latest
        if self._thread is not None:
            self.close(hold=False)
        if not Path(self.port).exists():
            raise FileNotFoundError(f"RoArm 串口不存在: {self.port}")

        # stty opens the port without manipulating modem-control lines, then
        # persists -hupcl so subsequent low-level opens/closes do not reset ESP32.
        subprocess.run(
            [
                "stty",
                "-F",
                self.port,
                str(self.baudrate),
                "raw",
                "-echo",
                "-hupcl",
                "-crtscts",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        self._fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._stop.clear()
        self._feedback_ready.clear()
        with self._lock:
            self._latest = None
            self._error = ""
            self._boot_detected = False
        self._thread = threading.Thread(target=self._run, name="RoArm-UART", daemon=True)
        self._thread.start()
        if not self._feedback_ready.wait(timeout):
            error = self.error or "RoArm 状态查询超时"
            self.close(hold=False)
            raise TimeoutError(error)
        latest = self.latest()
        if latest is None:
            raise RuntimeError(self.error or "RoArm 没有有效反馈")
        return latest

    def command(self, value: dict[str, Any], wait: bool = False, timeout: float = 1.0) -> None:
        if self._thread is None or not self._thread.is_alive():
            raise RuntimeError("RoArm 尚未连接")
        done = threading.Event() if wait else None
        result: list[Any] = []
        self._commands.put((value, done, result))
        if done is not None:
            if not done.wait(timeout):
                raise TimeoutError("RoArm 命令发送超时")
            if result and isinstance(result[0], BaseException):
                raise RuntimeError(f"RoArm 命令发送失败: {result[0]}")

    def release_torque(self) -> None:
        self.command({"T": 210, "cmd": 0}, wait=True)

    def lock_torque(self) -> None:
        self.command({"T": 210, "cmd": 1}, wait=True)

    def move_joints(self, joints: Sequence[float], duration_s: float = 3.0) -> None:
        if len(joints) != 4:
            raise ValueError("RoArm 目标必须包含 b/s/e/t 四个关节")
        target = [float(value) for value in joints]
        if any(not math.isfinite(value) for value in target):
            raise ValueError("RoArm 目标关节包含非有限数")
        current = self.latest()
        max_delta = max(
            abs(target[index] - current.joints[index]) if current else 0.5
            for index in range(4)
        )
        steps_per_second = int(math.ceil(max_delta * 4096.0 / (2.0 * math.pi) / max(0.5, duration_s)))
        steps_per_second = max(10, min(800, steps_per_second))
        self.command(
            {
                "T": 102,
                "base": target[0],
                "shoulder": target[1],
                "elbow": target[2],
                "hand": target[3],
                "spd": steps_per_second,
                "acc": 10,
            },
            wait=True,
        )

    def hold_current(self) -> None:
        latest = self.latest()
        if latest is not None:
            self.move_joints(latest.joints, duration_s=1.0)
            time.sleep(0.1)
        self.lock_torque()

    def wait_until_joints(
        self,
        target: Sequence[float],
        tolerance_rad: float = 0.04,
        timeout: float = 15.0,
    ) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest = self.latest()
            if latest and max(abs(a - b) for a, b in zip(latest.joints, target)) <= tolerance_rad:
                return True
            if self.error:
                return False
            time.sleep(0.05)
        return False

    def close(self, hold: bool = True) -> None:
        if hold and self._thread is not None and self._thread.is_alive():
            try:
                self.hold_current()
            except Exception:
                pass
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
        self._fd = None
        self._thread = None

    def _write_json(self, value: dict[str, Any]) -> None:
        if self._fd is None:
            raise RuntimeError("RoArm 串口已关闭")
        payload = (json.dumps(value, separators=(",", ":")) + "\n").encode("ascii")
        offset = 0
        while offset < len(payload):
            offset += os.write(self._fd, payload[offset:])

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._error = message
        self._feedback_ready.set()

    def _run(self) -> None:
        pending = b""
        next_query = time.monotonic()
        period = 1.0 / self.feedback_hz
        try:
            while not self._stop.is_set():
                for _ in range(20):
                    try:
                        value, done, result = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self._write_json(value)
                    except Exception as exc:
                        result.append(exc)
                        self._set_error(str(exc))
                    finally:
                        if done is not None:
                            done.set()

                now = time.monotonic()
                if now >= next_query:
                    self._write_json({"T": 105})
                    next_query = now + period

                readable, _, _ = select.select([self._fd], [], [], min(0.02, max(0.0, next_query - now)))
                if not readable:
                    continue
                try:
                    chunk = os.read(self._fd, 4096)
                except BlockingIOError:
                    continue
                if not chunk:
                    continue
                pending += chunk
                while b"\n" in pending:
                    raw, pending = pending.split(b"\n", 1)
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    if any(
                        marker in line
                        for marker in (
                            "ServoCtrl init",
                            "Bus servos status check",
                            "Moving BASE_JOINT",
                            "Moving SHOULDER_JOINT",
                        )
                    ):
                        self._boot_detected = True
                        self._set_error("检测到 RoArm ESP32 重启/初始化，已禁止跟随")
                        continue
                    try:
                        value = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if value.get("T") != 1051:
                        continue
                    feedback = RoArmFeedback.from_json(value)
                    with self._lock:
                        self._latest = feedback
                    self._feedback_ready.set()
        except Exception as exc:
            self._set_error(f"RoArm 串口线程异常: {type(exc).__name__}: {exc}")


class O6Controller:
    """Legacy-compatible facade over the V2 O6Device."""

    def __init__(
        self,
        port: str,
        sdk_root: Path | str,
        hand_id: int = 0x27,
        baudrate: int = 115200,
    ):
        self.port = str(port)
        self.sdk_root = str(sdk_root)
        self.hand_id = int(hand_id)
        self.baudrate = int(baudrate)

        self._device = O6Device(
            O6Config(
                port=self.port,
                sdk_root=self.sdk_root,
                hand_id=self.hand_id,
                baudrate=self.baudrate,
            )
        )

    @property
    def connected(self) -> bool:
        return self._device.connected

    @property
    def error(self) -> str:
        return self._device.error

    @property
    def phase(self) -> str:
        return self._device.phase

    def latest(
        self,
    ) -> tuple[
        Optional[tuple[int, ...]],
        Optional[tuple[int, ...]],
        float,
    ]:
        snapshot = self._device.latest()
        if snapshot is None:
            return None, None, self._device.latest_position_timestamp
        return (
            tuple(int(value) for value in snapshot.positions),
            tuple(int(value) for value in snapshot.faults),
            float(self._device.latest_position_timestamp),
        )

    @property
    def last_sent_target(self) -> Optional[tuple[int, ...]]:
        return self._device.last_sent_target

    def connect(self, timeout: float = 10.0) -> tuple[int, ...]:
        snapshot = self._device.connect(timeout=float(timeout))
        return tuple(int(value) for value in snapshot.positions)

    def set_profile(self, speed: Sequence[Any], torque: Sequence[Any]) -> None:
        self._device.set_profile(speed, torque)

    def set_target(self, target: Sequence[Any]) -> None:
        self._device.set_target(target)

    def hold_current(self) -> None:
        self._device.hold_current()

    def recover_motor(
        self,
        motor_number: int,
        speed: Any,
        torque: Any,
        timeout: float = 4.0,
        stop_event: Optional[threading.Event] = None,
    ) -> dict[str, Any]:
        return self._device.recover_motor(
            motor_number,
            speed,
            torque,
            timeout=float(timeout),
            stop_event=stop_event,
        )

    def wait_until_position(
        self,
        target: Sequence[Any],
        tolerance: int = 6,
        timeout: float = 15.0,
        stop_event: Optional[threading.Event] = None,
    ) -> bool:
        return self._device.wait_until_position(
            target,
            tolerance=int(tolerance),
            timeout=float(timeout),
            stop_event=stop_event,
        )

    def close(self) -> None:
        self._device.close()

    @property
    def _thread(self) -> Any:
        return self._device._thread

    @_thread.setter
    def _thread(self, value: Any) -> None:
        self._device._thread = value

    @property
    def _recovery_requests(self) -> Any:
        return self._device._recovery_requests

    def _process_recovery(self, hand: Any, request: Any) -> None:
        self._device._process_recovery(hand, request)


class NrcRobotAdapter(NrcRobotSession):
    """Legacy class name for the V2 NRC control session.

    Socket creation/closure is still owned by the existing application path.
    All serialized NRC state-machine and motion logic now lives in
    src/gello_cr/devices/nrc_robot.py.
    """

    pass


class TeleopEngine:
    """Coordinates master feedback, CR5 Cartesian tracking, O6 and presets."""

    def __init__(
        self,
        store: TeleopConfigStore,
        roarm: Any,
        o6: O6Controller,
    ):
        self.store = store
        self.roarm = roarm
        self.o6 = o6
        self.robot: Optional[NrcRobotAdapter] = None
        self.events: queue.Queue[tuple[str, str]] = queue.Queue()
        self._state_lock = threading.RLock()
        self._state = "idle"
        self._last_error = ""
        self._stop_generation = 0
        self._follow_stop = threading.Event()
        self._preset_stop = threading.Event()
        self._hand_action_stop = threading.Event()
        self._record_stop = threading.Event()
        self._replay_stop = threading.Event()
        self._shutdown_requested = threading.Event()
        self._follow_thread: Optional[threading.Thread] = None
        self._preset_thread: Optional[threading.Thread] = None
        self._record_thread: Optional[threading.Thread] = None
        self._replay_thread: Optional[threading.Thread] = None
        self._record_frames: list[dict[str, Any]] = []
        self._record_started_at = 0.0
        self._record_slot = 1
        self._replay_slot = 1
        self._record_joint_indices: tuple[int, ...] = tuple(
            int(value) for value in store.data["replay"]["robot_joint_indices"]
        )
        self._o6_manual_override = False
        self._o6_action_active = False
        self._o6_action_name = ""
        self._dataset_target_tcp_controller: Optional[
            tuple[float, float, float, float, float, float]
        ] = None
        self._dataset_action_timestamp = 0.0
        self._gello_telemetry: Optional[dict[str, Any]] = None
        self._replay_resume_follow = False
        self._replay_file_signatures: dict[int, Optional[tuple[int, int]]] = {}
        self._replay_file_statuses: dict[int, tuple[bool, str]] = {}

    @property
    def master_type(self) -> str:
        return str(getattr(self.roarm, "master_type", "roarm"))

    def set_master(self, master: Any) -> None:
        if self.state not in ("idle", "fault"):
            raise RuntimeError("当前遥操作状态不允许切换主臂")
        if getattr(self.roarm, "connected", False):
            raise RuntimeError("主臂已连接，不能直接切换类型")
        self.roarm = master
        if self.state == "fault" and self.last_error.startswith("主臂"):
            self._set_state("idle")

    @property
    def state(self) -> str:
        with self._state_lock:
            return self._state

    @property
    def last_error(self) -> str:
        with self._state_lock:
            return self._last_error

    @property
    def o6_manual_override(self) -> bool:
        with self._state_lock:
            return self._o6_manual_override

    @property
    def o6_action_active(self) -> bool:
        with self._state_lock:
            return self._o6_action_active

    @property
    def recording(self) -> bool:
        thread = self._record_thread
        return bool(thread is not None and thread.is_alive() and not self._record_stop.is_set())

    @property
    def replaying(self) -> bool:
        thread = self._replay_thread
        return bool(thread is not None and thread.is_alive())

    def _replay_path(self, slot: int = 1) -> Path:
        slot_number = int(slot)
        if slot_number not in (1, 2, 3):
            raise ValueError("轨迹槽位必须是 1、2 或 3")
        configured = Path(str(self.store.data["replay"]["file"])).expanduser()
        path = configured if configured.is_absolute() else self.store.path.parent / configured
        if slot_number == 1:
            return path
        return path.with_name(f"{path.stem}_{slot_number}{path.suffix}")

    def _get_replay_file_status(self, slot: int = 1) -> tuple[bool, str]:
        """Cache the lightweight compatibility check used by the Qt status timer."""
        slot_number = int(slot)
        path = self._replay_path(slot_number)
        try:
            stat = path.stat()
        except FileNotFoundError:
            self._replay_file_signatures[slot_number] = None
            self._replay_file_statuses[slot_number] = (False, "尚无轨迹文件")
            return self._replay_file_statuses[slot_number]
        signature = (int(stat.st_mtime_ns), int(stat.st_size))
        if signature == self._replay_file_signatures.get(slot_number):
            return self._replay_file_statuses[slot_number]
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            frames = payload.get("frames") if isinstance(payload, dict) else None
            version = int(payload.get("version", 0)) if isinstance(payload, dict) else 0
            if (
                version < 2
                or not isinstance(frames, list)
                or len(frames) < 2
                or not isinstance(frames[0], dict)
                or "o6_position" not in frames[0]
            ):
                status = (False, "旧轨迹缺少 O6 数据，请重新录制")
            else:
                raw_indices = payload.get("robot_joint_indices", [1, 2, 3, 4, 5, 6])
                joint_indices = self._validate_replay_joint_indices(raw_indices)
                joints_text = "/".join(f"J{value}" for value in joint_indices)
                status = (
                    True,
                    f"可重放：{len(frames)} 帧（CR5 {joints_text} + O6）",
                )
        except Exception as exc:
            status = (False, f"轨迹文件无效：{exc}")
        self._replay_file_signatures[slot_number] = signature
        self._replay_file_statuses[slot_number] = status
        return status

    def attach_robot(self, robot: NrcRobotAdapter) -> None:
        self.robot = robot
        if self.state == "fault" and self.last_error.startswith("CR5 连接断开"):
            self._set_state("idle")
        self._event("info", "CR5 已接入遥操作引擎")

    def detach_robot(self, robot: Optional[NrcRobotAdapter], reason: str) -> bool:
        """Detach only the expected stale adapter and stop local command producers."""
        with self._state_lock:
            if robot is not None and self.robot is not robot:
                return False
            if self.robot is None:
                return False
            self.robot = None
        self._follow_stop.set()
        self._preset_stop.set()
        self._replay_stop.set()
        try:
            self.o6.hold_current()
        except Exception as exc:
            self._event("error", f"CR5 掉线后 O6 保持失败: {exc}")
        try:
            self._hold_roarm()
        except Exception as exc:
            self._event("error", f"CR5 掉线后主臂保持失败: {exc}")
        message = f"CR5 连接断开：{reason}"
        if self.state != "closed":
            self._set_state("fault", message)
        self._event("error", message)
        return True

    def _set_state(self, state: str, error: str = "") -> None:
        with self._state_lock:
            self._state = state
            self._last_error = error

    def _event(self, level: str, message: str) -> None:
        self.events.put((level, message))

    def _hold_roarm(self) -> None:
        if getattr(self.roarm, "passive", False):
            return
        if self.roarm.connected:
            self.roarm.hold_current()
        elif getattr(self.roarm, "io_alive", False):
            # 通信线程仍存活但反馈已报错时，不再使用旧位置移动，只重新上力矩。
            self.roarm.lock_torque()

    def connect_master(self) -> Any:
        if self._shutdown_requested.is_set():
            raise RuntimeError("程序正在退出，禁止连接主臂")
        master_feedback = self.roarm.connect()
        if self._shutdown_requested.is_set():
            self.roarm.close(hold=False)
            raise RuntimeError("程序退出已取消主臂连接")
        if self.master_type == "gello":
            self._event(
                "info",
                "GELLO 主臂已连接: "
                + ", ".join(
                    f"J{index + 1}={value:.3f}"
                    for index, value in enumerate(master_feedback.joints_rad)
                ),
            )
            return master_feedback
        if self.master_type == "inverse3":
            orientation_status = (
                f"，ABC={list(master_feedback.rpy_rad)}"
                if master_feedback.rpy_rad is not None
                else "，VerseGrip 未检测到（仅 XYZ 监视，禁止 6D 跟随）"
            )
        else:
            orientation_status = ""
        self._event(
            "info",
            f"{self.master_type} 主臂已连接: XYZ="
            f"[{master_feedback.x:.1f}, {master_feedback.y:.1f}, "
            f"{master_feedback.z:.1f}] mm{orientation_status}",
        )
        return master_feedback

    def connect_devices(self) -> None:
        master_feedback = self.connect_master()
        if self._shutdown_requested.is_set():
            self.roarm.close(hold=False)
            raise RuntimeError("程序退出已取消主臂/O6 连接")
        o6_position = self.o6.connect()
        if self._shutdown_requested.is_set():
            self.o6.close()
            self.roarm.close(hold=False)
            raise RuntimeError("程序退出已取消主臂/O6 连接")
        o6_cfg = self.store.data["o6"]
        self.o6.set_profile(o6_cfg["speed"], o6_cfg["torque"])
        if self.master_type == "gello":
            self._event(
                "info",
                f"GELLO/O6 已连接: J1-J6={list(master_feedback.arm_joints_rad)}, "
                f"J7={master_feedback.gripper:.3f}, O6={list(o6_position)}",
            )
            return
        self._event(
            "info",
            f"{self.master_type}/O6 已连接: 主臂 XYZ="
            f"[{master_feedback.x:.1f}, {master_feedback.y:.1f}, {master_feedback.z:.1f}], "
            f"O6={list(o6_position)}",
        )

    def _set_dataset_target_tcp(self, tcp: Sequence[Any]) -> None:
        # NRC Cartesian targets stay in their native [mm, rad] convention
        # here. dataset_sample() combines this command with same-cycle TCP
        # feedback and exposes the OpenPI action as delta [m, rad].
        controller_values = tuple(float(value) for value in tcp[:6])
        if len(controller_values) != 6 or any(
            not math.isfinite(value) for value in controller_values
        ):
            raise ValueError("CR5 数据集 TCP action 必须是 6 个有限数值")
        with self._state_lock:
            self._dataset_target_tcp_controller = controller_values
            self._dataset_action_timestamp = time.monotonic()

    def start_follow(
        self,
        _resume_from_episode_replay: bool = False,
        _resume_from_replay: bool = False,
        _resume_from_preset: bool = False,
    ) -> None:
        if self._shutdown_requested.is_set():
            raise RuntimeError("程序正在退出，禁止启动主从跟随")
        if self.robot is None:
            raise RuntimeError("请先连接纳博特控制柜")
        if not self.roarm.connected or not self.o6.connected:
            raise RuntimeError("请先连接主臂和 O6")
        if self.master_type == "gello" and self.o6_action_active:
            raise RuntimeError("O6 单独动作尚未结束，不能交接给 GELLO J7")
        if (
            self._preset_thread is not None
            and self._preset_thread.is_alive()
            and not _resume_from_preset
        ):
            raise RuntimeError("预设回位正在执行")
        with self._state_lock:
            state = self._state
            stop_generation = self._stop_generation
        if _resume_from_episode_replay and state != "episode_replay":
            raise RuntimeError("Episode 轨迹状态已改变，禁止自动恢复 XYZ 跟随")
        if _resume_from_replay and state != "replay":
            raise RuntimeError("关节重放状态已改变，禁止自动恢复 XYZ 跟随")
        if _resume_from_preset and state != "preset":
            raise RuntimeError("示教点回位状态已改变，禁止自动恢复 XYZ 跟随")
        if (state == "preset" and not _resume_from_preset) or state in (
            "recovering",
            "closed",
        ) or (state == "replay" and not _resume_from_replay) or (
            state == "episode_replay" and not _resume_from_episode_replay
        ):
            raise RuntimeError("恢复或轨迹重放正在执行，请等待完成")
        if self._follow_thread is not None and self._follow_thread.is_alive():
            return
        if self.robot.servo_state() != 3:
            raise RuntimeError("CR5 尚未上电运行，请先使能机器人")
        if self._shutdown_requested.is_set():
            raise RuntimeError("程序退出已取消主从跟随启动")

        cfg = self.store.data
        master = self.roarm.latest()
        o6_position, o6_fault, o6_timestamp = self.o6.latest()
        now = time.monotonic()
        master_timeout = float(
            cfg["inverse3"]["feedback_timeout_s"]
            if self.master_type == "inverse3"
            else cfg["gello"]["feedback_timeout_s"]
            if self.master_type == "gello"
            else cfg["roarm"]["feedback_timeout_s"]
        )
        if master is None or now - master.timestamp > master_timeout:
            raise RuntimeError("主臂反馈超时")
        if self.master_type == "inverse3" and master.orientation_wxyz is None:
            raise RuntimeError(
                "Inverse3 只能提供 XYZ；未检测到 VerseGrip 姿态，"
                "禁止启动不锁 RPY 的 6D 跟随"
            )
        if o6_position is None or now - o6_timestamp > float(cfg["o6"]["feedback_timeout_s"]):
            raise RuntimeError("O6 反馈超时")
        if o6_fault is None or any(o6_fault):
            raise RuntimeError(f"O6 故障: {o6_fault}")

        if self.master_type == "gello":
            self._start_gello_follow(master, stop_generation)
            return

        slave_tcp = self.robot.tcp_position()
        self._set_dataset_target_tcp(slave_tcp)
        robot_cfg = cfg["robot"]
        with self._state_lock:
            if (
                self._shutdown_requested.is_set()
                or self._stop_generation != stop_generation
            ):
                raise RuntimeError("XYZ 跟随启动已被软件紧急停止")
            self._follow_stop.clear()
        try:
            if getattr(self.robot, "motion_mode", "servoj") != "movej":
                self.robot.open_servoj(
                    float(robot_cfg["servoj_vmax"]),
                    float(robot_cfg["servoj_amax"]),
                    float(robot_cfg["servoj_jmax"]),
                )
            if (
                self._follow_stop.is_set()
                or self._shutdown_requested.is_set()
                or self._stop_generation != stop_generation
            ):
                raise RuntimeError("XYZ 跟随启动已被软件紧急停止")
            if not getattr(self.roarm, "passive", False):
                self.roarm.release_torque()
            if (
                self._follow_stop.is_set()
                or self._shutdown_requested.is_set()
                or self._stop_generation != stop_generation
            ):
                raise RuntimeError("XYZ 跟随启动已被软件紧急停止")
        except Exception as exc:
            if self._stop_generation == stop_generation and self.state != "closed":
                self._set_state("fault", str(exc))
            try:
                if getattr(self.robot, "motion_mode", "servoj") == "movej":
                    self.robot.stop_motion()
                else:
                    self.robot.stop_servoj()
            finally:
                self._hold_roarm()
            raise

        with self._state_lock:
            cancelled = (
                self._follow_stop.is_set()
                or self._shutdown_requested.is_set()
                or self._stop_generation != stop_generation
            )
            if not cancelled:
                self._state = "following"
                self._last_error = ""
        if cancelled:
            if getattr(self.robot, "motion_mode", "servoj") == "movej":
                self.robot.stop_motion()
            else:
                self.robot.stop_servoj()
            self._hold_roarm()
            raise RuntimeError("XYZ 跟随启动已被软件紧急停止")
        self._follow_thread = threading.Thread(
            target=self._follow_loop,
            args=(master, slave_tcp),
            name=f"{self.master_type}-CR5-Follow",
            daemon=True,
        )
        self._follow_thread.start()
        if self.master_type == "inverse3":
            self._event(
                "info",
                "跟随已启动：Inverse3 XYZ + VerseGrip 相对 RPY；"
                "O6 仅使用独立快捷动作",
            )
        else:
            hand_mode = (
                "O6 保持独立动作，M5 已屏蔽"
                if self.o6_manual_override
                else "M5 控制 O6"
            )
            self._event("info", f"跟随已启动：XYZ 增量映射，RPY 已锁定；{hand_mode}")

    def _start_gello_follow(self, master: GelloFeedback, stop_generation: int) -> None:
        if self.robot is None:
            raise RuntimeError("请先连接 CR5")
        slave_joints = [float(value) for value in self.robot.joint_position()]
        if len(slave_joints) != 7:
            raise RuntimeError("CR5 反馈必须包含 7 个关节值")
        gello_cfg = self.store.data["gello"]
        gello_mode = str(gello_cfg.get("control_mode", "joint"))
        kinematics = (
            GelloKinematics(gello_cfg["kinematics_urdf"])
            if gello_mode != "joint"
            else None
        )
        robot_cfg = self.store.data["robot"]
        with self._state_lock:
            if (
                self._shutdown_requested.is_set()
                or self._stop_generation != stop_generation
            ):
                raise RuntimeError("GELLO 跟随启动已被软件紧急停止")
            self._follow_stop.clear()
            self._gello_telemetry = None
            self._dataset_target_tcp_controller = None
            self._dataset_action_timestamp = 0.0
        try:
            if getattr(self.robot, "motion_mode", "servoj") != "movej":
                self.robot.open_servoj(
                    float(robot_cfg["servoj_vmax"]),
                    float(robot_cfg["servoj_amax"]),
                    float(robot_cfg["servoj_jmax"]),
                )
        except Exception as exc:
            if self._stop_generation == stop_generation and self.state != "closed":
                self._set_state("fault", str(exc))
            try:
                self.robot.stop_motion()
            finally:
                self.o6.hold_current()
            raise
        with self._state_lock:
            cancelled = (
                self._follow_stop.is_set()
                or self._shutdown_requested.is_set()
                or self._stop_generation != stop_generation
            )
            if not cancelled:
                self._state = "following"
                self._last_error = ""
        if cancelled:
            self.robot.stop_motion()
            self.o6.hold_current()
            raise RuntimeError("GELLO 跟随启动已被软件紧急停止")
        motion_mode = getattr(self.robot, "motion_mode", "servoj")
        if motion_mode == "movej":
            self._event(
                "info",
                "CR3A MoveJ 跟随已准备："
                f"速度/加速度/减速度={robot_cfg['movej_velocity']}/"
                f"{robot_cfg['movej_acc']}/{robot_cfg['movej_dec']}%，"
                f"指令周期={robot_cfg['movej_period_s']}s；"
                f"短段实验={'开' if robot_cfg['movej_low_latency'] else '关'}，"
                f"单段上限={robot_cfg['movej_max_segment_deg']}°，"
                f"J1-J6倍率={gello_cfg['joint_scale']:.2f}×",
            )
        else:
            self._event(
                "info",
                f"CR3A {motion_mode} 指令通道已准备；SDK 返回成功不代表从臂已执行。"
                f" ServoJ 参数 v/a/j={robot_cfg['servoj_vmax']}/"
                f"{robot_cfg['servoj_amax']}/{robot_cfg['servoj_jmax']}",
            )
        self._follow_thread = threading.Thread(
            target=self._gello_follow_loop,
            args=(master, slave_joints, stop_generation, kinematics),
            name="GELLO-CR5-Follow",
            daemon=True,
        )
        self._follow_thread.start()
        self._event(
            "info",
            "GELLO 跟随已启动："
            + (
                "完整 TCP 6D IK：J1-J6 控制 CR3A TCP XYZ+ABC，J7 控制 O6"
                if gello_mode == "tcp_6d"
                else
                "J1-J5 控制 CR3A XYZ，J6 通过 TCP-C IK 控制 CR3A J6，J7 控制 O6"
                if gello_mode == "xyz_j6_tcp_ik"
                else "J1-J5 控制 CR3A XYZ，J6 直接控制 CR3A J6，J7 控制 O6"
                if gello_mode == "xyz_j6"
                else "J1-J6 控制 CR3A 关节，J7 控制 O6"
            )
            + "；使用当前姿态相对零点",
        )

    def _gello_follow_loop(
        self,
        master_origin: GelloFeedback,
        slave_origin: Sequence[float],
        stop_generation: int,
        kinematics: Optional[GelloKinematics] = None,
    ) -> None:
        if kinematics is not None:
            self._gello_xyz_j6_follow_loop(
                master_origin, slave_origin, stop_generation, kinematics
            )
            return
        cfg = self.store.data
        robot_cfg = cfg["robot"]
        gello_cfg = cfg["gello"]
        o6_cfg = cfg["o6"]
        joint_mapper = RelativeJointMapper(
            joint_scale=float(gello_cfg["joint_scale"]),
            locked_joints=tuple(
                int(value) for value in gello_cfg["locked_joints"]
            ),
        )
        motion_mode = getattr(self.robot, "motion_mode", "servoj")
        low_latency = motion_mode == "movej" and bool(robot_cfg["movej_low_latency"])
        max_segment_deg = float(robot_cfg["movej_max_segment_deg"])
        period = 1.0 / float(gello_cfg["feedback_hz"])
        if getattr(self.robot, "motion_mode", "servoj") == "movej":
            period = max(period, float(getattr(self.robot, "movej_period_s", period)))
        master_timeout = float(gello_cfg["feedback_timeout_s"])
        o6_timeout = float(o6_cfg["feedback_timeout_s"])
        step_limit = float(robot_cfg.get("safety_max_command_step_rad", math.radians(20.0)))
        speed_limit = float(robot_cfg.get("safety_max_command_speed_rad_s", 0.8))
        tracking_limit = float(robot_cfg.get("safety_max_tracking_error_rad", math.radians(20.0)))
        speed_cycles = int(robot_cfg.get("safety_speed_violation_cycles", 3))
        leader_origin = np.asarray(master_origin.arm_joints_rad, dtype=float)
        last_leader = leader_origin.copy()
        last_leader_timestamp = master_origin.timestamp
        commanded = np.asarray(slave_origin[:6], dtype=float)
        last_target = commanded.copy()
        speed_guard = LeaderSpeedViolationCounter()
        next_cycle = time.monotonic()
        startup_settle_deadline = next_cycle + float(gello_cfg.get("startup_settle_s", 0.5))
        last_hand_action: Optional[str] = None
        last_servoj_log_time = 0.0
        sent_count = 0
        first_sent_at = 0.0
        last_sent_at = 0.0
        last_validated_at = 0.0
        last_diagnostic_at = next_cycle
        window_sent = 0
        recent_send_hz = 0.0
        busy_since: Optional[float] = None
        last_busy_wait_ms = 0.0
        error = ""
        try:
            while not self._follow_stop.is_set():
                now = time.monotonic()
                master = self.roarm.latest()
                if master is None or now - master.timestamp > master_timeout:
                    raise RuntimeError("GELLO 反馈掉线/超时")
                if self.roarm.error:
                    raise RuntimeError(self.roarm.error)
                o6_position, o6_fault, o6_timestamp = self.o6.latest()
                if self.o6.error:
                    raise RuntimeError(self.o6.error)
                if o6_position is None or now - o6_timestamp > o6_timeout:
                    raise RuntimeError("O6 反馈掉线/超时")
                if o6_fault is None or any(o6_fault):
                    raise RuntimeError(f"O6 故障: {o6_fault}")
                if self._stop_generation != stop_generation:
                    raise RuntimeError("GELLO 跟随已被软件紧急停止")
                if now < startup_settle_deadline:
                    leader_origin = np.asarray(master.arm_joints_rad, dtype=float)
                    last_leader = leader_origin.copy()
                    last_leader_timestamp = master.timestamp
                    last_target = commanded.copy()
                    speed_guard.reset()
                    next_cycle += period
                    wait_time = next_cycle - time.monotonic()
                    if wait_time > 0:
                        self._follow_stop.wait(wait_time)
                    else:
                        next_cycle = time.monotonic()
                    continue
                live_robot_cfg = self.store.data["robot"]
                step_limit = float(
                    live_robot_cfg.get(
                        "safety_max_command_step_rad", math.radians(20.0)
                    )
                )
                speed_limit = float(
                    live_robot_cfg.get("safety_max_command_speed_rad_s", 0.8)
                )
                tracking_limit = float(
                    live_robot_cfg.get(
                        "safety_max_tracking_error_rad", math.radians(20.0)
                    )
                )
                speed_cycles = int(live_robot_cfg.get("safety_speed_violation_cycles", 3))

                warning_before = self.robot.controller_job_warning if motion_mode == "movej" else ""
                if warning_before and busy_since is None:
                    busy_since = time.monotonic()
                busy_check_ms = 0.0
                dispatch_ready = True
                if low_latency:
                    check_started = time.monotonic()
                    dispatch_ready = self.robot.clear_movej_busy_if_stopped()
                    busy_check_ms = (time.monotonic() - check_started) * 1000.0
                    read_started = time.monotonic()
                    actual = np.asarray(self.robot.joint_position()[:6], dtype=float)
                    feedback_at = time.monotonic()
                    feedback_read_ms = (feedback_at - read_started) * 1000.0
                    # The NRC request may block; construct the segment from samples
                    # fetched after it, and recheck freshness before issuing motion.
                    master = self.roarm.latest()
                    now = time.monotonic()
                    if master is None or now - master.timestamp > master_timeout:
                        raise RuntimeError("GELLO 反馈掉线/超时")
                    if self.roarm.error:
                        raise RuntimeError(self.roarm.error)
                    o6_position, o6_fault, o6_timestamp = self.o6.latest()
                    if self.o6.error:
                        raise RuntimeError(self.o6.error)
                    if o6_position is None or now - o6_timestamp > o6_timeout:
                        raise RuntimeError("O6 反馈掉线/超时")
                    if o6_fault is None or any(o6_fault):
                        raise RuntimeError(f"O6 故障: {o6_fault}")
                    if self._follow_stop.is_set():
                        break
                    if self._stop_generation != stop_generation:
                        raise RuntimeError("GELLO 跟随已被软件紧急停止")

                leader = np.asarray(master.arm_joints_rad, dtype=float)
                target = np.asarray(
                    joint_mapper.target_deg(
                        leader,
                        leader_origin,
                        commanded,
                    ),
                    dtype=float,
                )
                step_violation = max_command_step_violation(
                    target,
                    last_target,
                    step_limit,
                )
                if step_violation is not None:
                    raise RuntimeError(
                        f"GELLO J{step_violation.joint_number} command step "
                        f"{step_violation.value:.4f}rad exceeds "
                        f"{step_violation.limit:.4f}rad"
                    )
                if master.timestamp != last_leader_timestamp:
                    leader_delta_rad = np.asarray(
                        joint_mapper.leader_delta_rad(
                            leader,
                            last_leader,
                        ),
                        dtype=float,
                    )
                    speed_sample = speed_guard.update(
                        leader_delta_rad,
                        master.timestamp - last_leader_timestamp,
                        speed_limit,
                        speed_cycles,
                    )
                    if speed_sample.violation is not None:
                        violation = speed_sample.violation
                        raise RuntimeError(
                            f"GELLO J{violation.joint_number} leader speed "
                            f"{violation.value:.3f}rad/s exceeds "
                            f"{violation.limit:.3f}rad/s"
                        )
                    last_leader = leader.copy()
                    last_leader_timestamp = master.timestamp

                if not low_latency:
                    read_started = time.monotonic()
                    actual = np.asarray(self.robot.joint_position()[:6], dtype=float)
                    feedback_at = time.monotonic()
                    feedback_read_ms = (feedback_at - read_started) * 1000.0
                tracking_violation = max_tracking_error_violation(
                    last_target,
                    actual,
                    tracking_limit,
                )
                if tracking_violation is not None:
                    index = tracking_violation.joint_number - 1
                    raise RuntimeError(
                        f"CR5 J{tracking_violation.joint_number} tracking error "
                        f"{tracking_violation.value:.4f}rad exceeds "
                        f"{tracking_violation.limit:.4f}rad; "
                        f"target={last_target[index]:.3f}deg, "
                        f"actual={actual[index]:.3f}deg"
                    )

                desired_target = target.copy()
                desired_error_deg = float(np.max(np.abs(desired_target - actual)))
                if low_latency:
                    # A short segment must not hide an unsafe full leader/follower gap.
                    if math.radians(desired_error_deg) > tracking_limit:
                        raise RuntimeError(
                            f"CR3A 完整期望目标跟踪误差 {desired_error_deg:.2f}° "
                            f"超过 {math.degrees(tracking_limit):.2f}°；短段不会放宽安全限位"
                        )
                    if desired_error_deg > max_segment_deg:
                        target = actual + (desired_target - actual) * (
                            max_segment_deg / desired_error_deg
                        )
                segment_delta_deg = float(np.max(np.abs(target - actual)))
                target_full = target.tolist() + [float(slave_origin[6])]
                validated_target = target.copy()
                dispatch_started = time.monotonic()
                last_validated_at = dispatch_started
                duplicate_deadband = float(
                    live_robot_cfg["movej_duplicate_deadband_deg"]
                ) if motion_mode == "movej" else 0.0
                duplicate_hold = (
                    motion_mode == "movej"
                    and sent_count > 0
                    and float(np.max(np.abs(validated_target - last_target))) <= duplicate_deadband
                    and float(np.max(np.abs(validated_target - actual))) <= duplicate_deadband
                )
                sent = (
                    self.robot.send_servoj(target_full)
                    if dispatch_ready and not duplicate_hold else False
                )
                dispatch_finished = time.monotonic()
                dispatch_ms = (dispatch_finished - dispatch_started) * 1000.0
                leader_age_ms = (dispatch_started - master.timestamp) * 1000.0
                if sent is not False:
                    last_target = target.copy()
                    last_sent_at = time.monotonic()
                    if sent_count == 0:
                        first_sent_at = last_sent_at
                    sent_count += 1
                    window_sent += 1
                    if now - last_servoj_log_time >= 1.0:
                        current_error_deg = np.abs(target - actual)
                        self._event(
                            "info",
                            f"{getattr(self.robot, 'motion_mode', 'servoj')} 发送/反馈："
                            f"target={[round(float(value), 2) for value in target]}，"
                            f"actual={[round(float(value), 2) for value in actual]}，"
                            f"max_err={float(np.max(current_error_deg)):.2f}°",
                        )
                        last_servoj_log_time = now
                warning_after = self.robot.controller_job_warning if motion_mode == "movej" else ""
                if warning_after:
                    if busy_since is None:
                        busy_since = dispatch_finished
                    busy_wait_ms = (dispatch_finished - busy_since) * 1000.0
                else:
                    if busy_since is not None:
                        last_busy_wait_ms = (dispatch_finished - busy_since) * 1000.0
                        busy_since = None
                    busy_wait_ms = 0.0
                phase = (
                    "duplicate_hold" if duplicate_hold else
                    "busy" if warning_after or not dispatch_ready else
                    "submitted" if sent is not False else
                    "idle_deferred" if warning_before else "rate_limited"
                )
                diagnostic_due = dispatch_finished - last_diagnostic_at >= 1.0
                if diagnostic_due:
                    recent_send_hz = window_sent / (dispatch_finished - last_diagnostic_at)
                    last_diagnostic_at = dispatch_finished
                    window_sent = 0
                # Reuse the safety-loop feedback in the UI and recorder. FK is
                # needed only at dataset rate, not on every ServoJ cycle.
                with self._state_lock:
                    pending_target_deg = float(np.max(np.abs(validated_target - last_target)))
                    self._gello_telemetry = {
                        "actual_deg": tuple(float(v) for v in actual),
                        "target_deg": tuple(float(v) for v in last_target),
                        "target_full_deg": tuple(last_target) + (float(slave_origin[6]),),
                        "feedback_at": feedback_at,
                        "command_at": last_sent_at,
                        "validated_at": last_validated_at,
                        "sent_count": sent_count,
                        "send_hz": ((sent_count - 1) / (last_sent_at - first_sent_at)
                                    if sent_count > 1 else 0.0),
                        "tracking_error_deg": float(np.max(np.abs(last_target - actual))),
                        "desired_deg": tuple(float(v) for v in desired_target),
                        "desired_tracking_error_deg": desired_error_deg,
                        "validated_deg": tuple(float(v) for v in validated_target),
                        "pending_target_deg": pending_target_deg,
                        "low_latency": low_latency,
                        "latency": {
                            "phase": phase,
                            "leader_age_ms": leader_age_ms,
                            "feedback_read_ms": feedback_read_ms,
                            "busy_check_ms": busy_check_ms,
                            "dispatch_ms": dispatch_ms,
                            "busy_wait_ms": busy_wait_ms,
                            "last_busy_wait_ms": last_busy_wait_ms,
                            "recent_send_hz": recent_send_hz,
                            "segment_delta_deg": segment_delta_deg,
                        },
                    }
                if diagnostic_due and motion_mode == "movej":
                    self._event(
                        "info",
                        f"MoveJ 软件时序：短段={'开' if low_latency else '关'}，phase={phase}，"
                        f"GELLO_age={leader_age_ms:.1f}ms，read={feedback_read_ms:.1f}ms，"
                        f"busy_check={busy_check_ms:.1f}ms，dispatch={dispatch_ms:.1f}ms，"
                        f"busy_wait={busy_wait_ms:.1f}ms（上次 {last_busy_wait_ms:.1f}ms），"
                        f"SDK_submit={recent_send_hz:.1f}Hz，span={segment_delta_deg:.2f}°，"
                        f"pending={pending_target_deg:.2f}°，desired_err={desired_error_deg:.2f}°；"
                        "提交不等于执行确认",
                    )

                hand_action = binary_o6_action(master.gripper)
                if last_hand_action != hand_action:
                    self.o6.set_target(o6_cfg["actions"][hand_action])
                    last_hand_action = hand_action
                    self._event(
                        "info",
                        f"GELLO J7 触发 O6 动作：{hand_action}；"
                        f"目标={list(o6_cfg['actions'][hand_action])}",
                    )

                next_cycle += period
                wait_time = next_cycle - time.monotonic()
                if wait_time > 0:
                    self._follow_stop.wait(wait_time)
                else:
                    next_cycle = time.monotonic()
        except Exception as exc:
            error = str(exc)
            self._set_state("fault", error)
            self._event("error", f"GELLO 安全停止：{error}")
        finally:
            try:
                if self.robot is not None:
                    self.robot.stop_motion()
            except Exception as exc:
                self._event("error", f"CR5 停止失败: {exc}")
            try:
                self.o6.hold_current()
            except Exception as exc:
                self._event("error", f"O6 保持失败: {exc}")
            if not error and self.state == "following":
                self._set_state("idle")

    def _gello_xyz_j6_follow_loop(
        self,
        master_origin: GelloFeedback,
        slave_origin: Sequence[float],
        stop_generation: int,
        kinematics: GelloKinematics,
    ) -> None:
        cfg = self.store.data
        robot_cfg = cfg["robot"]
        gello_cfg = cfg["gello"]
        o6_cfg = cfg["o6"]
        period = 1.0 / float(gello_cfg["feedback_hz"])
        if getattr(self.robot, "motion_mode", "servoj") == "movej":
            period = max(period, float(getattr(self.robot, "movej_period_s", period)))
        master_timeout = float(gello_cfg["feedback_timeout_s"])
        o6_timeout = float(o6_cfg["feedback_timeout_s"])
        step_limit = float(
            robot_cfg.get("safety_max_command_step_rad", math.radians(20.0))
        )
        speed_limit = float(robot_cfg.get("safety_max_command_speed_rad_s", 0.8))
        tracking_limit = float(
            robot_cfg.get("safety_max_tracking_error_rad", math.radians(20.0))
        )
        speed_cycles = int(robot_cfg.get("safety_speed_violation_cycles", 3))
        j6_sign = int(gello_cfg["j6_sign"])
        tcp_ik_mode = str(gello_cfg.get("control_mode")) == "xyz_j6_tcp_ik"
        full_tcp_ik_mode = str(gello_cfg.get("control_mode")) == "tcp_6d"
        leader_origin_joints = np.asarray(master_origin.arm_joints_rad, dtype=float)
        leader_position_joints = leader_origin_joints.copy()
        if not full_tcp_ik_mode:
            leader_position_joints[5] = leader_origin_joints[5]
        leader_origin_pose = kinematics.pose(leader_position_joints)
        leader_origin_xyz = leader_origin_pose[:3, 3].copy()
        leader_origin_rotation = leader_origin_pose[:3, :3].copy()
        base_tcp = np.asarray(self.robot.tcp_position()[:7], dtype=float)
        if len(base_tcp) != 7:
            raise RuntimeError("CR3A TCP 反馈必须包含 7 个值")
        slave_origin_orientation = _nrc_abc_to_quaternion(base_tcp[3:6])
        commanded_orientation = slave_origin_orientation
        mapping_slave_origin = base_tcp[:3].copy()
        commanded_xyz = mapping_slave_origin.copy()
        last_sent_target = np.asarray(slave_origin[:7], dtype=float)
        last_j6_target_rad = 0.0
        last_leader_xyz = leader_origin_xyz.copy()
        last_leader_rotation = leader_origin_rotation.copy()
        last_master_timestamp = master_origin.timestamp
        last_j6_change_time = time.monotonic()
        j6_speed_count = 0
        next_cycle = time.monotonic()
        last_cycle_time = next_cycle
        startup_settle_deadline = next_cycle + float(gello_cfg.get("startup_settle_s", 0.5))
        last_hand: Optional[tuple[int, ...]] = None
        error = ""
        try:
            while not self._follow_stop.is_set():
                now = time.monotonic()
                master = self.roarm.latest()
                if master is None or now - master.timestamp > master_timeout:
                    raise RuntimeError("GELLO 反馈掉线/超时")
                if self.roarm.error:
                    raise RuntimeError(self.roarm.error)
                o6_position, o6_fault, o6_timestamp = self.o6.latest()
                if self.o6.error:
                    raise RuntimeError(self.o6.error)
                if o6_position is None or now - o6_timestamp > o6_timeout:
                    raise RuntimeError("O6 反馈掉线/超时")
                if o6_fault is None or any(o6_fault):
                    raise RuntimeError(f"O6 故障: {o6_fault}")
                if self._stop_generation != stop_generation:
                    raise RuntimeError("GELLO 跟随已被软件紧急停止")

                leader_joints = np.asarray(master.arm_joints_rad, dtype=float)
                leader_position_joints = leader_joints.copy()
                if not full_tcp_ik_mode:
                    leader_position_joints[5] = leader_origin_joints[5]
                leader_pose = kinematics.pose(leader_position_joints)
                leader_xyz = leader_pose[:3, 3].copy()
                leader_rotation = leader_pose[:3, :3].copy()
                if now < startup_settle_deadline:
                    leader_origin_joints = leader_joints.copy()
                    if not full_tcp_ik_mode:
                        leader_position_joints[5] = leader_origin_joints[5]
                    leader_origin_pose = kinematics.pose(leader_position_joints)
                    leader_origin_xyz = leader_origin_pose[:3, 3].copy()
                    leader_origin_rotation = leader_origin_pose[:3, :3].copy()
                    commanded_orientation = slave_origin_orientation
                    last_leader_xyz = leader_origin_xyz.copy()
                    last_leader_rotation = leader_origin_rotation.copy()
                    last_sent_target = np.asarray(slave_origin[:7], dtype=float)
                    last_j6_target_rad = 0.0
                    last_j6_change_time = now
                    j6_speed_count = 0
                    next_cycle += period
                    wait_time = next_cycle - time.monotonic()
                    if wait_time > 0:
                        self._follow_stop.wait(wait_time)
                    else:
                        next_cycle = time.monotonic()
                    continue

                live_teleop_cfg = self.store.data["teleop"]
                live_robot_cfg = self.store.data["robot"]
                step_limit = float(
                    live_robot_cfg.get(
                        "safety_max_command_step_rad", math.radians(20.0)
                    )
                )
                speed_limit = float(
                    live_robot_cfg.get("safety_max_command_speed_rad_s", 0.8)
                )
                tracking_limit = float(
                    live_robot_cfg.get(
                        "safety_max_tracking_error_rad", math.radians(20.0)
                    )
                )
                speed_cycles = int(
                    live_robot_cfg.get("safety_speed_violation_cycles", 3)
                )
                live_gello_cfg = self.store.data["gello"]
                axis_map = np.asarray(live_gello_cfg["xyz_axis_map"], dtype=float)
                scales = np.asarray(live_gello_cfg["xyz_scale"], dtype=float)
                if master.timestamp != last_master_timestamp:
                    xyz_jump_mm = float(np.linalg.norm(leader_xyz - last_leader_xyz) * 1000.0)
                    max_master_jump_mm = float(live_teleop_cfg["max_master_jump_mm"])
                    if xyz_jump_mm > max_master_jump_mm:
                        raise RuntimeError(
                            f"GELLO XYZ 单帧跳变 {xyz_jump_mm:.1f} mm，"
                            f"超过 {max_master_jump_mm:.1f} mm"
                        )
                    if full_tcp_ik_mode:
                        master_rotation_delta = leader_rotation @ last_leader_rotation.T
                        angular_jump = _quaternion_distance(
                            _rotation_matrix_to_quaternion(master_rotation_delta),
                            (1.0, 0.0, 0.0, 0.0),
                        )
                        max_angular_jump = float(
                            live_teleop_cfg["max_master_angular_jump_rad"]
                        )
                        if angular_jump > max_angular_jump:
                            raise RuntimeError(
                                f"GELLO 姿态单帧跳变 {angular_jump:.3f}rad，"
                                f"超过 {max_angular_jump:.3f}rad"
                            )
                    last_leader_xyz = leader_xyz.copy()
                    last_leader_rotation = leader_rotation.copy()
                    last_master_timestamp = master.timestamp

                mapped_delta_mm = axis_map @ (leader_xyz - leader_origin_xyz) * 1000.0
                mapped_delta_mm = mapped_delta_mm * scales
                desired_xyz = mapping_slave_origin + mapped_delta_mm
                xyz_limits = np.asarray(live_teleop_cfg["max_delta_xyz_mm"], dtype=float)
                total_delta_mm = desired_xyz - mapping_slave_origin
                for index, axis in enumerate("XYZ"):
                    if abs(total_delta_mm[index]) > xyz_limits[index]:
                        raise RuntimeError(
                            f"{axis} 相对位移 {total_delta_mm[index]:.1f} mm 超过 "
                            f"±{xyz_limits[index]:.1f} mm"
                        )

                cycle_dt = max(0.001, now - last_cycle_time)
                last_cycle_time = now
                remaining = desired_xyz - commanded_xyz
                distance = float(np.linalg.norm(remaining))
                max_step_mm = float(live_teleop_cfg["max_tcp_speed_mm_s"]) * cycle_dt
                ratio = min(1.0, max_step_mm / distance) if distance > 1e-9 else 1.0
                commanded_xyz = commanded_xyz + remaining * ratio

                orientation_requested = False
                desired_j6_rad = 0.0
                if full_tcp_ik_mode:
                    leader_relative_rotation = leader_rotation @ leader_origin_rotation.T
                    mapped_relative_rotation = (
                        axis_map @ leader_relative_rotation @ axis_map.T
                    )
                    relative_orientation = _rotation_matrix_to_quaternion(
                        mapped_relative_rotation
                    )
                    relative_abc = np.asarray(
                        _quaternion_to_nrc_abc(relative_orientation), dtype=float
                    )
                    relative_abc *= np.asarray(
                        live_teleop_cfg["scale_rpy"], dtype=float
                    )
                    desired_orientation = _quaternion_multiply(
                        _nrc_abc_to_quaternion(relative_abc),
                        slave_origin_orientation,
                    )
                    desired_relative = _quaternion_multiply(
                        desired_orientation,
                        _quaternion_inverse(slave_origin_orientation),
                    )
                    desired_abc = _quaternion_to_nrc_abc(desired_relative)
                    rpy_limits = [
                        float(value) for value in live_teleop_cfg["max_delta_rpy_rad"]
                    ]
                    for index, axis in enumerate("ABC"):
                        if abs(desired_abc[index]) > rpy_limits[index]:
                            raise RuntimeError(
                                f"{axis} 相对转角 {desired_abc[index]:.3f}rad 超过 "
                                f"±{rpy_limits[index]:.3f}rad"
                            )
                    angular_distance = _quaternion_distance(
                        commanded_orientation, desired_orientation
                    )
                    max_angular_step = (
                        float(live_teleop_cfg["max_tcp_angular_speed_rad_s"])
                        * cycle_dt
                    )
                    angular_ratio = (
                        min(1.0, max_angular_step / angular_distance)
                        if angular_distance > 1e-9
                        else 1.0
                    )
                    commanded_orientation = _quaternion_slerp(
                        commanded_orientation,
                        desired_orientation,
                        angular_ratio,
                    )
                    target_abc = _quaternion_to_nrc_abc_near(
                        commanded_orientation, base_tcp[3:6]
                    )
                    orientation_requested = angular_distance > 1e-7
                else:
                    desired_j6_rad = _wrapped_angle_delta(
                        leader_joints[5], leader_origin_joints[5]
                    ) * j6_sign
                    j6_step_rad = abs(
                        _wrapped_angle_delta(desired_j6_rad, last_j6_target_rad)
                    )
                    if j6_step_rad > step_limit:
                        raise RuntimeError(
                            f"GELLO J6 command step {j6_step_rad:.4f}rad exceeds "
                            f"{step_limit:.4f}rad"
                        )
                    if j6_step_rad > 1e-9:
                        j6_speed = j6_step_rad / max(
                            0.001, now - last_j6_change_time
                        )
                        if j6_speed > speed_limit:
                            j6_speed_count += 1
                        else:
                            j6_speed_count = 0
                        if j6_speed_count >= speed_cycles:
                            raise RuntimeError(
                                f"GELLO J6 leader speed {j6_speed:.3f}rad/s exceeds "
                                f"{speed_limit:.3f}rad/s"
                            )
                        last_j6_change_time = now

                target_tcp = base_tcp.copy()
                target_tcp[:3] = commanded_xyz
                if full_tcp_ik_mode:
                    target_tcp[3:6] = target_abc
                else:
                    target_tcp[3] = base_tcp[3]
                    target_tcp[4] = base_tcp[4]
                    target_tcp[5] = base_tcp[5] + desired_j6_rad
                movement_requested = bool(
                    np.max(np.abs(commanded_xyz - mapping_slave_origin)) > 1e-7
                    or orientation_requested
                    or (not full_tcp_ik_mode and abs(desired_j6_rad) > 1e-7)
                )
                if movement_requested:
                    ik_target_tcp = target_tcp if (tcp_ik_mode or full_tcp_ik_mode) else base_tcp.copy()
                    if not tcp_ik_mode and not full_tcp_ik_mode:
                        ik_target_tcp[:3] = commanded_xyz
                        ik_target_tcp[3] = base_tcp[3]
                        ik_target_tcp[4] = base_tcp[4]
                        # Direct J6 mode keeps C fixed while the controller
                        # solves J1-J5 for XYZ, then sets CR3A J6 below.
                        ik_target_tcp[5] = base_tcp[5]
                    target_joints = np.asarray(
                        self.robot.inverse_kinematics(ik_target_tcp.tolist()), dtype=float
                    )
                    if len(target_joints) != 7:
                        raise RuntimeError("CR3A 逆运动学必须返回 7 个关节值")
                    target_joints[6] = float(slave_origin[6])
                    if not tcp_ik_mode and not full_tcp_ik_mode:
                        target_joints[5] = (
                            float(slave_origin[5]) + math.degrees(desired_j6_rad)
                        )
                else:
                    target_joints = np.asarray(slave_origin[:7], dtype=float)

                target_delta_rad = np.abs(
                    np.deg2rad(target_joints[:6] - last_sent_target[:6])
                )
                if float(np.max(target_delta_rad)) > step_limit:
                    index = int(np.argmax(target_delta_rad))
                    raise RuntimeError(
                        f"CR3A J{index + 1} command step {target_delta_rad[index]:.4f}rad "
                        f"exceeds {step_limit:.4f}rad"
                    )
                actual = np.asarray(self.robot.joint_position()[:6], dtype=float)
                tracking_error = np.abs(np.deg2rad(last_sent_target[:6] - actual))
                if float(np.max(tracking_error)) > tracking_limit:
                    index = int(np.argmax(tracking_error))
                    raise RuntimeError(
                        f"CR3A J{index + 1} tracking error {tracking_error[index]:.4f}rad "
                        f"exceeds {tracking_limit:.4f}rad"
                    )

                sent = self.robot.send_servoj(target_joints.tolist())
                if sent is not False:
                    last_sent_target = target_joints.copy()
                    last_j6_target_rad = desired_j6_rad
                self._set_dataset_target_tcp(target_tcp.tolist())

                hand_target = interpolate_o6_target(
                    master.gripper,
                    o6_cfg["open"],
                    o6_cfg["closed"],
                )
                if should_send_o6_target(
                    hand_target,
                    last_hand,
                    int(o6_cfg["command_deadband"]),
                ):
                    self.o6.set_target(hand_target)
                    last_hand = hand_target

                next_cycle += period
                wait_time = next_cycle - time.monotonic()
                if wait_time > 0:
                    self._follow_stop.wait(wait_time)
                else:
                    next_cycle = time.monotonic()
        except Exception as exc:
            error = str(exc)
            self._set_state("fault", error)
            self._event("error", f"GELLO XYZ/J6 安全停止：{error}")
        finally:
            try:
                if self.robot is not None:
                    self.robot.stop_motion()
            except Exception as exc:
                self._event("error", f"CR3A 停止失败: {exc}")
            try:
                self.o6.hold_current()
            except Exception as exc:
                self._event("error", f"O6 保持失败: {exc}")
            if not error and self.state == "following":
                self._set_state("idle")

    def stop_follow(self, reason: str = "人工停止") -> None:
        if self.state in ("replay", "episode_replay") and self.replaying:
            # A general stop-follow request must not silently restart following.
            self.stop_joint_replay(resume_follow=False)
            self._event("info", f"关节重放已停止：{reason}")
            return
        self._follow_stop.set()
        thread = self._follow_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        if self.robot is not None:
            try:
                if (
                    self.master_type == "gello"
                    or getattr(self.robot, "motion_mode", "servoj") == "movej"
                ):
                    self.robot.stop_motion()
                else:
                    self.robot.stop_servoj()
            except Exception as exc:
                self._event("error", str(exc))
        try:
            self._hold_roarm()
        except Exception as exc:
            self._event("error", f"主臂保持失败: {exc}")
        if thread is not None and thread.is_alive():
            message = "XYZ 跟随线程未能在 2 秒内停止"
            self._set_state("fault", message)
            raise TimeoutError(message)
        if self.state in ("following", "master_free"):
            self._set_state("idle")
        self._event("info", f"跟随已停止：{reason}")

    def release_master_hold_slave(self) -> None:
        """Hold CR5/O6 at their current state and leave the master free."""
        if self.robot is None:
            raise RuntimeError("请先连接纳博特控制柜")
        if not self.roarm.connected or not self.o6.connected:
            raise RuntimeError("请先连接主臂和 O6")
        if self._preset_thread is not None and self._preset_thread.is_alive():
            raise RuntimeError("预设回位正在执行，请先点击软件紧急停止")
        if self.state in ("recovering", "replay", "episode_replay") or self.o6_action_active:
            raise RuntimeError("O6 操作正在执行，请等待完成或点击软件紧急停止")
        if self.robot.servo_state() != 3:
            raise RuntimeError("CR5 尚未上电运行，无法保证从臂保持")
        if self.state == "following":
            self.stop_follow("切换到主臂自由模式")
        self.robot.stop_motion()
        self.o6.hold_current()
        if not getattr(self.roarm, "passive", False):
            self.roarm.release_torque()
        self._set_state("master_free")
        self._event("info", "已锁住从臂并保持 O6，主臂保持自由")

    def recover_o6_motor(self, motor_number: int) -> dict[str, Any]:
        """Stop motion and safely re-issue one O6 motor's current-position control."""
        number = int(motor_number)
        with self._state_lock:
            stop_generation = self._stop_generation
        if number < 1 or number > len(O6_MOTOR_NAMES):
            raise ValueError("O6 电机编号必须为 1～6")
        if self._preset_thread is not None and self._preset_thread.is_alive():
            raise RuntimeError("预设回位正在执行，请先点击软件紧急停止")
        if self.state in ("recovering", "replay", "episode_replay") or self.o6_action_active:
            raise RuntimeError("O6 操作正在执行")
        if not self.o6.connected:
            raise RuntimeError("请先连接 O6")
        if self.state == "following":
            self.stop_follow(f"准备恢复 O6 {number} 号电机")

        position, fault, timestamp = self.o6.latest()
        if position is None or fault is None:
            raise RuntimeError("O6 尚无完整位置/故障反馈")
        if time.monotonic() - timestamp > float(self.store.data["o6"]["feedback_timeout_s"]):
            raise RuntimeError("O6 反馈超时，禁止重新上力")
        fault_code = int(fault[number - 1])
        if fault_code not in (0, 1):
            fault_name = O6_FAULT_NAMES.get(fault_code, "未知故障")
            raise RuntimeError(
                f"{number} 号电机为 {fault_name}（故障码 {fault_code}），"
                "请先排除硬件故障，程序不会强制上力"
            )

        previous_state = self.state
        cfg = self.store.data["o6"]
        with self._state_lock:
            if self._stop_generation != stop_generation:
                raise RuntimeError("O6 电机恢复启动已被软件紧急停止")
            self._hand_action_stop.clear()
            self._state = "recovering"
            self._last_error = ""
        self._event(
            "info",
            f"开始恢复 O6 {number} 号电机（{O6_MOTOR_NAMES[number - 1]}）："
            "先保持反馈位置，再重发速度和力矩",
        )
        try:
            result = self.o6.recover_motor(
                number,
                cfg["speed"][number - 1],
                cfg["torque"][number - 1],
                stop_event=self._hand_action_stop,
            )
        except Exception as exc:
            if (
                self._stop_generation == stop_generation
                and self.state == "recovering"
            ):
                self._set_state("fault", str(exc))
            raise

        if (
            self._stop_generation != stop_generation
            or self.state != "recovering"
            or self._hand_action_stop.is_set()
        ):
            raise RuntimeError("O6 电机恢复已被软件紧急停止")

        after_fault = tuple(int(value) for value in result["fault_after"])
        after_code = after_fault[number - 1]
        if after_code:
            fault_name = O6_FAULT_NAMES.get(after_code, "未知故障")
            message = (
                f"O6 {number} 号电机恢复后仍为 {fault_name}（故障码 {after_code}）；"
                "请勿继续遥操作，断电检查受阻、温度和供电"
            )
            self._set_state("fault", message)
            self._event("error", message)
        elif any(after_fault):
            message = (
                f"O6 {number} 号电机故障反馈已为 0，但其他电机仍有故障 "
                f"{list(after_fault)}；请先处理全部故障"
            )
            self._set_state("fault", message)
            self._event("error", message)
        else:
            restored_state = "master_free" if previous_state == "master_free" else "idle"
            self._set_state(restored_state)
            self._event(
                "info",
                f"O6 {number} 号电机（{result['motor_name']}）已重发控制，"
                f"位置 {result['position_before']}→{result['position_after']}，故障反馈为 0；"
                "请先低速测试，程序不会自动恢复 XYZ 跟随",
            )
        return result

    def execute_o6_action(self, action_name: str) -> tuple[int, ...]:
        """Move O6 independently while allowing CR5 XYZ following to continue."""
        if self.master_type == "gello" and self.state == "following":
            raise RuntimeError("GELLO 跟随时 O6 由 J7 独占控制；请先停止跟随再执行手势")
        name = str(action_name).strip()
        cfg = self.store.data["o6"]
        actions = cfg.get("actions", {})
        if name not in actions:
            raise ValueError(f"未配置 O6 快捷动作: {name}")
        target = tuple(_six_uint8(actions[name], f"O6 快捷动作 {name}"))
        if self._preset_thread is not None and self._preset_thread.is_alive():
            raise RuntimeError("预设回位正在执行，请先点击软件紧急停止")
        if self.state in ("recovering", "preset", "replay", "episode_replay", "closed"):
            raise RuntimeError(f"当前状态 {self.state} 不允许执行 O6 快捷动作")
        if self.o6_action_active:
            raise RuntimeError("已有 O6 快捷动作正在执行")
        if not self.o6.connected:
            raise RuntimeError("请先连接 O6")

        position, fault, timestamp = self.o6.latest()
        if position is None or fault is None:
            raise RuntimeError("O6 尚无完整位置/故障反馈")
        if time.monotonic() - timestamp > float(cfg["feedback_timeout_s"]):
            raise RuntimeError("O6 反馈超时，禁止执行快捷动作")
        if any(fault):
            raise RuntimeError(f"O6 存在故障，禁止执行快捷动作: {list(fault)}")

        self._hand_action_stop.clear()
        with self._state_lock:
            self._o6_manual_override = True
            self._o6_action_active = True
            self._o6_action_name = name
        self._event(
            "info",
            f"开始执行 O6 快捷动作“{name}”：目标={list(target)}；"
            "XYZ 跟随可继续，M5 对 O6 的控制已屏蔽",
        )
        try:
            self.o6.set_profile(cfg["speed"], cfg["torque"])
            self.o6.set_target(target)
            arrived = self.o6.wait_until_position(
                target,
                tolerance=6,
                timeout=float(cfg["action_timeout_s"]),
                stop_event=self._hand_action_stop,
            )
            if not arrived:
                if self._hand_action_stop.is_set():
                    raise RuntimeError("O6 快捷动作已被软件紧急停止")
                current, current_fault, _ = self.o6.latest()
                if self.o6.error:
                    raise RuntimeError(self.o6.error)
                if current_fault is not None and any(current_fault):
                    raise RuntimeError(f"O6 动作过程中出现故障: {list(current_fault)}")
                raise TimeoutError(
                    f"O6 未在 {float(cfg['action_timeout_s']):g}s 内到达动作“{name}”"
                    f"（当前={list(current) if current is not None else '--'}）"
                )
        except Exception:
            try:
                self.o6.hold_current()
            except Exception as hold_exc:
                self._event("error", f"O6 失败后保持当前位置失败: {hold_exc}")
            raise
        finally:
            with self._state_lock:
                self._o6_action_active = False

        self._event(
            "info",
            f"O6 快捷动作“{name}”完成；手势保持，M5 仍不会控制 O6",
        )
        return target

    def enable_m5_o6_control(self) -> None:
        """Return O6 command ownership to RoArm M5 during XYZ following."""
        if self.master_type == "inverse3":
            raise RuntimeError("Inverse3 没有 RoArm M5；请使用 O6 快捷动作")
        if self.state in ("preset", "recovering", "replay", "episode_replay", "closed"):
            raise RuntimeError(f"当前状态 {self.state} 不能恢复 M5 控制")
        self._hand_action_stop.set()
        with self._state_lock:
            self._o6_manual_override = False
            self._o6_action_name = ""
        self._event("info", "已恢复 M5 对 O6 的控制；XYZ 跟随时手爪将再次随 M5 开合")

    def _follow_loop(self, master_origin: Any, slave_origin: Sequence[float]) -> None:
        cfg = self.store.data
        teleop_cfg = cfg["teleop"]
        inverse3_mode = self.master_type == "inverse3"
        scales = [float(value) for value in teleop_cfg["scale_xyz"]]
        rpy_scales = [float(value) for value in teleop_cfg["scale_rpy"]]
        max_jump = float(teleop_cfg["max_master_jump_mm"])
        max_angular_jump = float(teleop_cfg["max_master_angular_jump_rad"])
        period = float(teleop_cfg["period_s"])
        master_timeout = float(
            cfg["inverse3"]["feedback_timeout_s"]
            if inverse3_mode
            else cfg["roarm"]["feedback_timeout_s"]
        )
        o6_timeout = float(cfg["o6"]["feedback_timeout_s"])
        m5_open = float(cfg["roarm"]["m5_open_rad"])
        m5_closed = float(cfg["roarm"]["m5_closed_rad"])
        o6_open = [float(value) for value in cfg["o6"]["open"]]
        o6_closed = [float(value) for value in cfg["o6"]["closed"]]
        hand_deadband = int(cfg["o6"]["command_deadband"])

        commanded_xyz = [float(value) for value in slave_origin[:3]]
        commanded_rpy = [float(value) for value in slave_origin[3:6]]
        slave_orientation = _nrc_abc_to_quaternion(slave_origin[3:6])
        commanded_orientation = slave_orientation
        mapping_master_origin = list(master_origin.xyz)
        mapping_slave_origin = list(commanded_xyz)
        mapping_orientation_origin = (
            master_origin.orientation_wxyz if inverse3_mode else None
        )
        mapping_slave_orientation = commanded_orientation
        last_master = master_origin
        last_master_timestamp = master_origin.timestamp
        last_hand: Optional[list[int]] = None
        next_cycle = time.monotonic()
        error = ""
        try:
            while not self._follow_stop.is_set():
                now = time.monotonic()
                master = self.roarm.latest()
                if master is None or now - master.timestamp > master_timeout:
                    raise RuntimeError("主臂反馈掉线/超时")
                if self.roarm.error:
                    raise RuntimeError(self.roarm.error)
                if inverse3_mode and master.orientation_wxyz is None:
                    raise RuntimeError("VerseGrip 姿态反馈丢失")

                o6_position, o6_fault, o6_timestamp = self.o6.latest()
                if self.o6.error:
                    raise RuntimeError(self.o6.error)
                if o6_position is None or now - o6_timestamp > o6_timeout:
                    raise RuntimeError("O6 反馈掉线/超时")
                if o6_fault is None or any(o6_fault):
                    raise RuntimeError(f"O6 故障: {o6_fault}")

                if master.timestamp != last_master_timestamp:
                    jump = math.dist(master.xyz, last_master.xyz)
                    if jump > max_jump:
                        raise RuntimeError(
                            f"主臂单帧跳变 {jump:.1f} mm，超过 {max_jump:.1f} mm"
                        )
                    if inverse3_mode:
                        angular_jump = _quaternion_distance(
                            master.orientation_wxyz,
                            last_master.orientation_wxyz,
                        )
                        if angular_jump > max_angular_jump:
                            raise RuntimeError(
                                "VerseGrip 单帧姿态跳变 "
                                f"{angular_jump:.3f} rad，超过 {max_angular_jump:.3f} rad"
                            )
                    last_master = master
                    last_master_timestamp = master.timestamp

                live_teleop_cfg = self.store.data["teleop"]
                live_scales = [
                    float(value) for value in live_teleop_cfg["scale_xyz"]
                ]
                if live_scales != scales:
                    # Rebase only the mapping origin so changing gain/direction
                    # cannot jump the current target. The original CR5 origin
                    # below remains the fixed safety-envelope reference.
                    mapping_master_origin = list(master.xyz)
                    mapping_slave_origin = list(commanded_xyz)
                    scales = live_scales
                live_rpy_scales = [
                    float(value) for value in live_teleop_cfg["scale_rpy"]
                ]
                if inverse3_mode and live_rpy_scales != rpy_scales:
                    mapping_orientation_origin = master.orientation_wxyz
                    mapping_slave_orientation = commanded_orientation
                    rpy_scales = live_rpy_scales
                limits = [
                    float(value)
                    for value in live_teleop_cfg["max_delta_xyz_mm"]
                ]
                scaled_delta = [
                    (master.xyz[index] - mapping_master_origin[index])
                    * scales[index]
                    for index in range(3)
                ]
                desired_xyz = [
                    mapping_slave_origin[index] + scaled_delta[index]
                    for index in range(3)
                ]
                total_delta = [
                    desired_xyz[index] - float(slave_origin[index])
                    for index in range(3)
                ]
                for index, axis in enumerate("XYZ"):
                    if abs(total_delta[index]) > limits[index]:
                        raise RuntimeError(
                            f"{axis} 相对位移 {total_delta[index]:.1f} mm 超过 ±{limits[index]:.1f} mm"
                        )

                dt = max(0.001, now - (next_cycle - period))
                remaining = [desired_xyz[index] - commanded_xyz[index] for index in range(3)]
                distance = math.sqrt(sum(value * value for value in remaining))
                # The GUI may update the speed while following.  Read this one
                # value every cycle so the new limit takes effect without
                # stopping ServoJ or resetting the teleoperation origin.
                max_speed = float(live_teleop_cfg["max_tcp_speed_mm_s"])
                max_step = max_speed * dt
                ratio = min(1.0, max_step / distance) if distance > 1e-9 else 1.0
                commanded_xyz = [
                    commanded_xyz[index] + remaining[index] * ratio for index in range(3)
                ]

                target_tcp = list(slave_origin[:7])
                target_tcp[:3] = commanded_xyz
                if inverse3_mode:
                    relative_orientation = _quaternion_multiply(
                        master.orientation_wxyz,
                        _quaternion_inverse(mapping_orientation_origin),
                    )
                    relative_rpy = _quaternion_to_nrc_abc(relative_orientation)
                    scaled_relative_orientation = _nrc_abc_to_quaternion(
                        [
                            relative_rpy[index] * rpy_scales[index]
                            for index in range(3)
                        ]
                    )
                    desired_orientation = _quaternion_multiply(
                        scaled_relative_orientation,
                        mapping_slave_orientation,
                    )
                    rpy_limits = [
                        float(value)
                        for value in live_teleop_cfg["max_delta_rpy_rad"]
                    ]
                    total_rpy_delta = _quaternion_to_nrc_abc(
                        _quaternion_multiply(
                            desired_orientation,
                            _quaternion_inverse(slave_orientation),
                        )
                    )
                    for index, axis in enumerate("ABC"):
                        if abs(total_rpy_delta[index]) > rpy_limits[index]:
                            raise RuntimeError(
                                f"{axis} 相对转角 {total_rpy_delta[index]:.3f} rad "
                                f"超过 ±{rpy_limits[index]:.3f} rad"
                            )
                    angular_distance = _quaternion_distance(
                        commanded_orientation,
                        desired_orientation,
                    )
                    max_angular_step = (
                        float(live_teleop_cfg["max_tcp_angular_speed_rad_s"]) * dt
                    )
                    angular_ratio = (
                        min(1.0, max_angular_step / angular_distance)
                        if angular_distance > 1e-9
                        else 1.0
                    )
                    commanded_orientation = _quaternion_slerp(
                        commanded_orientation,
                        desired_orientation,
                        angular_ratio,
                    )
                    commanded_rpy = [
                        _wrapped_angle_delta(value, 0.0)
                        for value in _quaternion_to_nrc_abc_near(
                            commanded_orientation,
                            commanded_rpy,
                        )
                    ]
                    target_tcp[3:6] = commanded_rpy
                target_joints = self.robot.inverse_kinematics(target_tcp)
                self.robot.send_servoj(target_joints)
                self._set_dataset_target_tcp(target_tcp)

                if not inverse3_mode and not self.o6_manual_override:
                    grip = (master.t - m5_open) / (m5_closed - m5_open)
                    grip = max(0.0, min(1.0, grip))
                    hand_target = [
                        int(round(o6_open[index] + grip * (o6_closed[index] - o6_open[index])))
                        for index in range(6)
                    ]
                    if (
                        last_hand is None
                        or max(abs(a - b) for a, b in zip(hand_target, last_hand))
                        >= hand_deadband
                    ):
                        self.o6.set_target(hand_target)
                        last_hand = hand_target

                next_cycle += period
                wait_time = next_cycle - time.monotonic()
                if wait_time > 0:
                    self._follow_stop.wait(wait_time)
                else:
                    next_cycle = time.monotonic()
        except Exception as exc:
            error = str(exc)
            self._set_state("fault", error)
            self._event("error", f"安全停止：{error}")
        finally:
            try:
                if self.robot is not None:
                    if getattr(self.robot, "motion_mode", "servoj") == "movej":
                        self.robot.stop_motion()
                    else:
                        self.robot.stop_servoj()
            except Exception as exc:
                self._event("error", f"CR5 停止失败: {exc}")
            try:
                self._hold_roarm()
            except Exception as exc:
                self._event("error", f"主臂保持失败: {exc}")
            if not error and self.state == "following":
                self._set_state("idle")

    def start_joint_recording(self, slot: int = 1) -> None:
        """Record synchronized CR5 joint and O6 feedback trajectories."""
        slot_number = int(slot)
        self._replay_path(slot_number)
        if self.robot is None:
            raise RuntimeError("请先连接 CR5")
        if not self.o6.connected:
            raise RuntimeError("请先连接 O6")
        if self.state in ("preset", "recovering", "replay", "episode_replay", "closed"):
            raise RuntimeError(f"当前状态 {self.state} 不允许开始录制")
        if self.recording:
            raise RuntimeError("关节轨迹正在录制")

        joint_indices = self._validate_replay_joint_indices(
            self.store.data["replay"]["robot_joint_indices"]
        )
        with self._state_lock:
            self._record_frames = []
            self._record_started_at = time.monotonic()
            self._record_slot = slot_number
            self._record_joint_indices = joint_indices
        self._record_stop.clear()
        self._record_thread = threading.Thread(
            target=self._record_joint_worker,
            args=(slot_number,),
            name=f"Joint-Record-{slot_number}",
            daemon=True,
        )
        self._record_thread.start()
        cfg = self.store.data["replay"]
        self._event(
            "info",
            f"开始录制轨迹 {slot_number}：CR5 "
            + "/".join(f"J{value}" for value in joint_indices)
            + " + O6 6 关节，采样周期 "
            f"{float(cfg['sample_period_s']):g}s",
        )

    def _record_joint_worker(self, slot: int) -> None:
        cfg = self.store.data["replay"]
        period = float(cfg["sample_period_s"])
        max_duration = float(cfg["max_duration_s"])
        started = self._record_started_at
        next_sample = started
        error = ""
        try:
            while not self._record_stop.is_set():
                now = time.monotonic()
                elapsed = now - started
                if elapsed > max_duration:
                    self._event("warning", f"录制达到最长 {max_duration:g}s，已自动停止")
                    break
                if self.robot is None:
                    raise RuntimeError("CR5 连接已丢失")
                robot_joints = [float(value) for value in self.robot.joint_position()]
                if len(robot_joints) != 7 or any(
                    not math.isfinite(value) for value in robot_joints
                ):
                    raise RuntimeError("CR5 关节反馈必须是 7 个有限数值")
                o6_position, o6_fault, o6_timestamp = self.o6.latest()
                if o6_position is None or o6_fault is None:
                    raise RuntimeError("O6 没有完整位置/故障反馈")
                if now - o6_timestamp > float(
                    self.store.data["o6"]["feedback_timeout_s"]
                ):
                    raise RuntimeError("O6 反馈超时")
                if any(o6_fault):
                    raise RuntimeError(f"O6 故障: {list(o6_fault)}")
                master = self.roarm.latest()
                frame = {
                    "t": round(elapsed, 6),
                    "robot_joints_deg": robot_joints,
                    "o6_position": list(_six_uint8(o6_position, "O6 录制反馈")),
                    "roarm_joints_rad": list(master.joints) if master is not None else None,
                }
                with self._state_lock:
                    self._record_frames.append(frame)

                next_sample += period
                wait_time = next_sample - time.monotonic()
                if wait_time > 0:
                    self._record_stop.wait(wait_time)
                else:
                    next_sample = time.monotonic()
        except Exception as exc:
            error = str(exc)
            self._event("error", f"关节轨迹录制停止: {error}")
        finally:
            self._record_stop.set()
            try:
                path = self._save_joint_recording(slot)
                with self._state_lock:
                    count = len(self._record_frames)
                if count:
                    self._event("info", f"关节轨迹已保存：{path}，共 {count} 帧")
            except Exception as exc:
                self._event("error", f"保存关节轨迹失败: {exc}")

    def _save_joint_recording(self, slot: int = 1) -> Path:
        with self._state_lock:
            frames = copy.deepcopy(self._record_frames)
        if not frames:
            raise RuntimeError("没有可保存的关节轨迹帧")
        path = self._replay_path(slot)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 3,
            "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "sample_period_s": float(self.store.data["replay"]["sample_period_s"]),
            "robot_joint_indices": list(self._record_joint_indices),
            "frame_count": len(frames),
            "frames": frames,
        }
        temp_path = path.with_suffix(path.suffix + ".tmp")
        temp_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        if path.exists():
            try:
                old_payload = json.loads(path.read_text(encoding="utf-8"))
                old_frames = old_payload.get("frames", [])
                old_has_o6 = bool(
                    old_frames
                    and isinstance(old_frames[0], dict)
                    and "o6_position" in old_frames[0]
                )
                if int(old_payload.get("version", 0)) < 2 or not old_has_o6:
                    backup = path.with_name(
                        f"{path.stem}.v1-backup-{time.strftime('%Y%m%d-%H%M%S')}"
                        f"{path.suffix}"
                    )
                    shutil.copy2(path, backup)
                    self._event("warning", f"旧版无 O6 轨迹已备份：{backup}")
            except Exception as exc:
                temp_path.unlink(missing_ok=True)
                raise RuntimeError(f"备份旧轨迹失败，已取消覆盖: {exc}") from exc
        os.replace(temp_path, path)
        return path

    def stop_joint_recording(self, slot: Optional[int] = None) -> Path:
        slot_number = self._record_slot if slot is None else int(slot)
        self._replay_path(slot_number)
        thread = self._record_thread
        if thread is None or not thread.is_alive():
            path = self._replay_path(slot_number)
            if path.exists():
                return path
            raise RuntimeError("当前没有正在录制的关节轨迹")
        if slot_number != self._record_slot:
            raise RuntimeError(f"当前正在录制轨迹 {self._record_slot}")
        self._record_stop.set()
        if thread is not threading.current_thread():
            thread.join(timeout=3.0)
        if thread.is_alive():
            raise TimeoutError("关节录制线程未能及时停止")
        return self._replay_path(slot_number)

    @staticmethod
    def _validate_replay_joint_indices(values: Any) -> tuple[int, ...]:
        if not isinstance(values, list) or not values:
            raise ValueError("轨迹至少需要选择一个 CR5 关节")
        if any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 1
            or value > 6
            for value in values
        ):
            raise ValueError("轨迹关节编号必须在 J1～J6 范围内")
        if len(set(values)) != len(values):
            raise ValueError("轨迹关节编号不能重复")
        return tuple(sorted(values))

    def _load_joint_recording(
        self,
        slot: int = 1,
    ) -> tuple[list[dict[str, Any]], tuple[int, ...]]:
        path = self._replay_path(slot)
        if not path.exists():
            raise FileNotFoundError(f"尚无关节轨迹文件: {path}")
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"读取关节轨迹失败: {exc}") from exc
        frames = payload.get("frames") if isinstance(payload, dict) else None
        if not isinstance(frames, list) or len(frames) < 2:
            raise ValueError("关节轨迹至少需要 2 帧")
        if any("o6_position" not in frame for frame in frames if isinstance(frame, dict)):
            raise ValueError("旧轨迹不包含 O6 灵巧手数据，请重新录制后再重放")
        joint_indices = self._validate_replay_joint_indices(
            payload.get("robot_joint_indices", [1, 2, 3, 4, 5, 6])
        )
        selected_zero_based = tuple(value - 1 for value in joint_indices)
        validated: list[dict[str, Any]] = []
        previous_t = -1.0
        previous_joints: Optional[list[float]] = None
        max_joint_speed = float(self.store.data["replay"]["max_joint_speed_deg_s"])
        for index, frame in enumerate(frames):
            if not isinstance(frame, dict):
                raise ValueError(f"轨迹第 {index + 1} 帧格式错误")
            timestamp = float(frame["t"])
            joints = [float(value) for value in frame["robot_joints_deg"]]
            if not math.isfinite(timestamp) or (index and timestamp <= previous_t):
                raise ValueError("轨迹时间戳必须严格递增且为有限数")
            if len(joints) != 7 or any(not math.isfinite(value) for value in joints):
                raise ValueError(f"轨迹第 {index + 1} 帧 CR5 关节数据无效")
            hand = _six_uint8(frame["o6_position"], f"轨迹第 {index + 1} 帧 O6")
            if previous_joints is not None:
                dt = timestamp - previous_t
                frame_speed = max(
                    abs(joints[index] - previous_joints[index]) / dt
                    for index in selected_zero_based
                )
                if frame_speed > max_joint_speed:
                    raise ValueError(
                        f"轨迹第 {index + 1} 帧关节速度 {frame_speed:.1f} deg/s "
                        f"超过安全上限 {max_joint_speed:.1f} deg/s"
                    )
            validated.append(
                {
                    "t": timestamp,
                    "robot_joints_deg": joints,
                    "o6_position": hand,
                }
            )
            previous_t = timestamp
            previous_joints = joints
        start_t = validated[0]["t"]
        for frame in validated:
            frame["t"] -= start_t
        return validated, joint_indices

    def start_joint_replay(
        self,
        insert_into_episode: bool = False,
        slot: int = 1,
    ) -> None:
        """Replay CR5/O6, optionally without interrupting an active Episode."""
        slot_number = int(slot)
        self._replay_path(slot_number)
        insert_mode = bool(insert_into_episode)
        with self._state_lock:
            stop_generation = self._stop_generation
        if self.robot is None:
            raise RuntimeError("请先连接 CR5")
        if not self.o6.connected or not self.roarm.connected:
            raise RuntimeError("请先连接主臂和 O6")
        if self.replaying:
            raise RuntimeError("关节轨迹正在重放")
        if self.o6_action_active:
            raise RuntimeError("O6 快捷动作正在执行，请等待完成或点击软件紧急停止")
        if self.state in ("preset", "recovering", "replay", "episode_replay", "closed"):
            raise RuntimeError(f"当前状态 {self.state} 不允许重放")
        if insert_mode and self.state != "following":
            raise RuntimeError("Episode 内插入式重放必须从 XYZ 跟随状态开始")
        if self.recording:
            self.stop_joint_recording()
        frames, joint_indices = self._load_joint_recording(slot_number)
        try:
            was_following = self.state == "following"
            with self._state_lock:
                if self._stop_generation != stop_generation:
                    raise RuntimeError("关节轨迹重放启动已被软件紧急停止")
                self._replay_stop.clear()
                # Claim the higher-priority state before stopping follow.  This
                # closes the small interval in which a second preset/replay command
                # could otherwise enter while ServoJ is being stopped.
                self._state = "episode_replay" if insert_mode else "replay"
                self._last_error = ""
                self._replay_slot = slot_number
            if was_following:
                self.stop_follow(
                    "Episode 内准备插入关节轨迹"
                    if insert_mode
                    else "准备重放关节轨迹"
                )
                time.sleep(float(self.store.data["replay"]["mode_switch_settle_s"]))
            self._hold_roarm()
            self._hand_action_stop.set()
            with self._state_lock:
                self._o6_manual_override = True
                self._o6_action_active = False
                self._o6_action_name = (
                    f"Episode 插入轨迹 {slot_number}"
                    if insert_mode
                    else f"轨迹 {slot_number} 重放"
                )
                self._replay_resume_follow = was_following
        except Exception as exc:
            if (
                self._stop_generation == stop_generation
                and self.state in ("replay", "episode_replay")
            ):
                self._set_state("fault", str(exc))
            raise
        expected_state = "episode_replay" if insert_mode else "replay"
        if (
            self._stop_generation != stop_generation
            or self._replay_stop.is_set()
            or self.state != expected_state
        ):
            raise RuntimeError("关节轨迹重放启动已被软件紧急停止")
        self._replay_thread = threading.Thread(
            target=self._joint_replay_worker,
            args=(frames, joint_indices, insert_mode),
            name=(
                f"Episode-Insert-Replay-{slot_number}"
                if insert_mode
                else f"Joint-Replay-{slot_number}"
            ),
            daemon=True,
        )
        self._replay_thread.start()
        joints_text = "/".join(f"J{value}" for value in joint_indices)
        self._event(
            "info",
            ("Episode 保持录制，开始插入" if insert_mode else "开始重放")
            + f"轨迹 {slot_number}，共 {len(frames)} 帧；CR5 仅运动 {joints_text}；"
            + (
                "以点击时的关节位置为零点执行相对位移"
                if bool(self.store.data["replay"]["relative_to_current"])
                else "使用轨迹文件中的绝对关节角"
            ),
        )

    def _joint_replay_worker(
        self,
        frames: list[dict[str, Any]],
        joint_indices: tuple[int, ...],
        insert_mode: bool,
    ) -> None:
        error = ""
        cancelled = False
        cfg = self.store.data
        replay_cfg = cfg["replay"]
        try:
            if self._replay_stop.is_set():
                cancelled = True
                raise RuntimeError("关节轨迹重放已停止")
            if self.robot is None:
                raise RuntimeError("CR5 未连接")
            if self.robot.servo_state() != 3:
                raise RuntimeError("CR5 尚未上电运行")
            held_joints = [float(value) for value in self.robot.joint_position()]
            if len(held_joints) != 7 or any(
                not math.isfinite(value) for value in held_joints
            ):
                raise RuntimeError("CR5 重放起始关节反馈必须是 7 个有限数值")
            selected_zero_based = tuple(value - 1 for value in joint_indices)
            relative_replay = bool(replay_cfg["relative_to_current"])
            recorded_origin = [
                float(value) for value in frames[0]["robot_joints_deg"]
            ]

            def masked_target(recorded: Sequence[Any]) -> list[float]:
                target = held_joints.copy()
                for index in selected_zero_based:
                    recorded_value = float(recorded[index])
                    target[index] = (
                        held_joints[index]
                        + recorded_value
                        - recorded_origin[index]
                        if relative_replay
                        else recorded_value
                    )
                return target

            # The commanded TCP is needed to derive each LeRobot delta action.
            # Precompute it from every masked joint target so replay frames never
            # reuse the stale leader-follow target and wrist RPY is represented.
            current_tcp = self.robot.tcp_position()
            self._set_dataset_target_tcp(current_tcp)
            if insert_mode:
                self._event("info", "正在生成插入轨迹的 TCP/RPY action 标签")
            prepared_frames: list[dict[str, Any]] = []
            for frame_index, frame in enumerate(frames):
                if self._replay_stop.is_set():
                    cancelled = True
                    raise RuntimeError("关节轨迹重放已停止")
                target_joints = masked_target(frame["robot_joints_deg"])
                target_tcp = self.robot.forward_kinematics(target_joints)
                prepared_frames.append(
                    {
                        "t": float(frame["t"]),
                        "target_joints": target_joints,
                        "target_tcp": target_tcp,
                        "o6_position": frame["o6_position"],
                    }
                )
                if frame_index % 20 == 0:
                    self._set_dataset_target_tcp(current_tcp)

            first = prepared_frames[0]
            first_joints = first["target_joints"]
            first_tcp = first["target_tcp"]
            first_hand = first["o6_position"]
            if self._replay_stop.is_set():
                cancelled = True
                raise RuntimeError("关节轨迹重放已停止")
            self.o6.set_profile(cfg["o6"]["speed"], cfg["o6"]["torque"])
            self.o6.set_target(first_hand)
            self._set_dataset_target_tcp(first_tcp)
            if not relative_replay:
                self.robot.movej(
                    first_joints,
                    float(replay_cfg["initial_move_velocity_percent"]),
                    float(replay_cfg["initial_move_acc_percent"]),
                    float(replay_cfg["initial_move_dec_percent"]),
                )
                deadline = time.monotonic() + float(replay_cfg["initial_timeout_s"])
                tolerance = float(replay_cfg["initial_tolerance_deg"])
                while time.monotonic() < deadline:
                    if self._replay_stop.is_set():
                        cancelled = True
                        raise RuntimeError("关节轨迹重放已停止")
                    self._set_dataset_target_tcp(first_tcp)
                    current = self.robot.joint_position()
                    if max(
                        abs(current[index] - first_joints[index])
                        for index in selected_zero_based
                    ) <= tolerance:
                        break
                    time.sleep(0.05)
                else:
                    raise TimeoutError("CR5 未在规定时间内到达轨迹起点")
                if not self.robot.wait_until_motion_stopped(
                    float(replay_cfg["initial_timeout_s"]),
                    stop_event=self._replay_stop,
                ):
                    if self._replay_stop.is_set():
                        cancelled = True
                        raise RuntimeError("关节轨迹重放已停止")
                    raise TimeoutError("CR5 到达轨迹起点后仍未完全停止")
                time.sleep(float(replay_cfg["mode_switch_settle_s"]))
            if not self.o6.wait_until_position(
                first_hand,
                tolerance=6,
                timeout=float(cfg["o6"]["action_timeout_s"]),
                stop_event=self._replay_stop,
            ):
                if self._replay_stop.is_set():
                    cancelled = True
                    raise RuntimeError("关节轨迹重放已停止")
                raise TimeoutError("O6 未在规定时间内到达轨迹起点")

            if self._replay_stop.is_set():
                cancelled = True
                raise RuntimeError("关节轨迹重放已停止")
            robot_cfg = cfg["robot"]
            self.robot.open_servoj(
                float(robot_cfg["servoj_vmax"]),
                float(robot_cfg["servoj_amax"]),
                float(robot_cfg["servoj_jmax"]),
            )
            servo_period = float(replay_cfg["servoj_period_s"])

            # Recordings are normally sampled at 20 Hz.  Interpolate selected
            # joint targets to the controller's 10 ms ServoJ cadence instead of
            # dispatching coarse 50 ms position steps.
            servo_frames: list[dict[str, Any]] = []
            for start_frame, end_frame in zip(prepared_frames, prepared_frames[1:]):
                duration = float(end_frame["t"]) - float(start_frame["t"])
                steps = max(1, int(math.ceil(duration / servo_period)))
                for step in range(steps):
                    alpha = min(1.0, (step * servo_period) / duration)
                    target_joints = list(start_frame["target_joints"])
                    for index in selected_zero_based:
                        target_joints[index] = (
                            float(start_frame["target_joints"][index])
                            + alpha
                            * (
                                float(end_frame["target_joints"][index])
                                - float(start_frame["target_joints"][index])
                            )
                        )
                    target_tcp = [
                        float(a) + alpha * (float(b) - float(a))
                        for a, b in zip(
                            start_frame["target_tcp"], end_frame["target_tcp"]
                        )
                    ]
                    servo_frames.append(
                        {
                            "t": float(start_frame["t"]) + step * servo_period,
                            "target_joints": target_joints,
                            "target_tcp": target_tcp,
                            "o6_position": start_frame["o6_position"],
                        }
                    )
            servo_frames.append(prepared_frames[-1])

            started = time.monotonic()
            last_hand: Optional[tuple[int, ...]] = None
            next_state_check = started
            for frame in servo_frames:
                wait_time = started + float(frame["t"]) - time.monotonic()
                if wait_time > 0 and self._replay_stop.wait(wait_time):
                    cancelled = True
                    raise RuntimeError("关节轨迹重放已停止")
                if self._replay_stop.is_set():
                    cancelled = True
                    raise RuntimeError("关节轨迹重放已停止")
                now = time.monotonic()
                if now >= next_state_check:
                    if self.robot.servo_state() != 3:
                        raise RuntimeError("CR5 在重放过程中退出运行状态")
                    next_state_check = now + 0.1
                _, fault, o6_timestamp = self.o6.latest()
                if time.monotonic() - o6_timestamp > float(cfg["o6"]["feedback_timeout_s"]):
                    raise RuntimeError("O6 重放过程中反馈超时")
                if self.o6.error:
                    raise RuntimeError(self.o6.error)
                if fault is None or any(fault):
                    raise RuntimeError(f"O6 重放过程中故障: {fault}")
                self._set_dataset_target_tcp(frame["target_tcp"])
                self.robot.send_servoj(frame["target_joints"])
                hand = tuple(frame["o6_position"])
                if hand != last_hand:
                    self.o6.set_target(hand)
                    last_hand = hand
        except Exception as exc:
            error = str(exc)
        finally:
            try:
                if self.robot is not None:
                    if error:
                        self.robot.stop_motion()
                    else:
                        self.robot.stop_servoj()
            except Exception as exc:
                self._event("error", f"重放结束时 CR5 停止失败: {exc}")
            try:
                self.o6.hold_current()
                self._hold_roarm()
            except Exception as exc:
                self._event("error", f"重放结束时保持失败: {exc}")
            with self._state_lock:
                expected_state = "episode_replay" if insert_mode else "replay"
                resume_requested = (
                    self._replay_resume_follow
                    and self._state == expected_state
                    and (not error or cancelled)
                )
            resumed = False
            if resume_requested:
                try:
                    time.sleep(float(replay_cfg["mode_switch_settle_s"]))
                    if insert_mode:
                        self.start_follow(_resume_from_episode_replay=True)
                    else:
                        self.start_follow(_resume_from_replay=True)
                    resumed = True
                except Exception as exc:
                    error = f"轨迹结束后恢复 XYZ 跟随失败: {exc}"
                    cancelled = False
            with self._state_lock:
                self._replay_resume_follow = False

            if resumed:
                with self._state_lock:
                    self._o6_action_name = "轨迹最终手势"
                result_text = "提前停止" if cancelled else "完成"
                self._event(
                    "info",
                    (
                        "Episode 插入轨迹" if insert_mode else "关节轨迹重放"
                    )
                    + f"已{result_text}；已用当前主从位姿重建 XYZ 零点并恢复跟随"
                    + ("，Episode 继续录制" if insert_mode else ""),
                )
            elif cancelled:
                if self.state in ("replay", "episode_replay"):
                    self._set_state("idle")
                self._event("info", "关节轨迹重放已停止")
            elif error:
                self._set_state("fault", error)
                self._event("error", f"关节轨迹重放失败: {error}")
            elif self.state in ("replay", "episode_replay"):
                with self._state_lock:
                    self._o6_action_name = "轨迹最终手势"
                self._set_state("idle")
                self._event("info", "CR5/O6 关节轨迹重放完成；O6 保持最后一帧手势")

    def stop_joint_replay(self, resume_follow: Optional[bool] = None) -> None:
        thread = self._replay_thread
        if thread is None or not thread.is_alive():
            raise RuntimeError("当前没有正在重放的关节轨迹")
        if resume_follow is not None:
            with self._state_lock:
                self._replay_resume_follow = bool(resume_follow)
        self._replay_stop.set()
        if self.robot is not None:
            self.robot.stop_motion()
        self.o6.hold_current()
        self._hold_roarm()
        if thread is not threading.current_thread():
            thread.join(timeout=3.0)
        if thread.is_alive():
            raise TimeoutError("关节重放线程未能及时停止")

    def save_preset(self, name: str, resume_follow: bool = True) -> dict[str, Any]:
        key = name.upper()
        if key not in ("A", "B", "C", "D", "HOME"):
            raise ValueError("预设名称必须是 A/B/C/D/HOME")
        if self.robot is None:
            raise RuntimeError("请先连接 CR5")
        if not self.roarm.connected or not self.o6.connected:
            raise RuntimeError("请先连接主臂和 O6")
        if self.master_type == "inverse3":
            raise RuntimeError(
                "Inverse3 采用只读 probe，不支持主动锁定/回位；"
                "请切换 RoArm 后再保存同步示教点"
            )
        if self.state in ("preset", "recovering", "replay", "episode_replay", "closed") or self.o6_action_active:
            raise RuntimeError(f"当前状态 {self.state} 不允许保存示教点")
        was_following = self.state == "following"
        if was_following:
            self.stop_follow("保存示教点")
        master = self.roarm.latest()
        o6_position, o6_fault, _ = self.o6.latest()
        if master is None:
            raise RuntimeError("没有 RoArm 反馈")
        if o6_position is None or o6_fault is None or any(o6_fault):
            raise RuntimeError(f"O6 状态无效: {o6_fault}")
        master_joints = (
            list(master.joints_rad)
            if self.master_type == "gello"
            else list(master.joints)
        )
        preset = {
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "roarm_joints_rad": master_joints,
            "robot_joints_deg": self.robot.joint_position(),
            "robot_tcp": self.robot.tcp_position(),
            "o6_position": list(o6_position),
        }
        if self.master_type == "gello":
            preset["gello_joints_rad"] = master_joints
        self.store.data["presets"][key] = preset
        self.store.save()
        label = "初始位 HOME" if key == "HOME" else f"示教点 {key}"
        self._event("info", f"{label} 已保存")
        if was_following and resume_follow:
            self.start_follow()
        return preset

    def recall_preset(self, name: str, resume_follow: bool = True) -> None:
        key = name.upper()
        with self._state_lock:
            stop_generation = self._stop_generation
        if key not in ("A", "B", "C", "D", "HOME"):
            raise ValueError("预设名称必须是 A/B/C/D/HOME")
        preset = self.store.data.get("presets", {}).get(key)
        if preset is None:
            label = "初始位 HOME" if key == "HOME" else f"示教点 {key}"
            raise RuntimeError(f"{label} 尚未保存")
        if self.robot is None:
            raise RuntimeError("请先连接 CR5")
        if not self.roarm.connected or not self.o6.connected:
            raise RuntimeError("请先连接主臂和 O6")
        if self.master_type == "inverse3":
            raise RuntimeError(
                "Inverse3 采用只读 probe，不支持同步主臂回位；"
                "请切换 RoArm 后再使用同步示教点"
            )
        if self._preset_thread is not None and self._preset_thread.is_alive():
            raise RuntimeError("已有预设回位任务正在执行")
        if self.state in ("preset", "recovering", "replay", "episode_replay", "closed") or self.o6_action_active:
            raise RuntimeError(f"当前状态 {self.state} 不允许预设回位")
        was_following = self.state == "following"
        resume_after_recall = bool(resume_follow and was_following)
        with self._state_lock:
            if self._stop_generation != stop_generation:
                raise RuntimeError("预设回位启动已被软件紧急停止")
            self._preset_stop.clear()
            self._state = "preset"
            self._last_error = ""
        if was_following:
            self.stop_follow(f"准备回到示教点 {key}")
        if (
            self._stop_generation != stop_generation
            or self._preset_stop.is_set()
            or self.state != "preset"
        ):
            raise RuntimeError("预设回位启动已被软件紧急停止")
        self._preset_thread = threading.Thread(
            target=self._recall_worker,
            args=(key, copy.deepcopy(preset), resume_after_recall),
            name=f"Preset-{key}",
            daemon=True,
        )
        self._preset_thread.start()

    def _recall_worker(self, key: str, preset: dict[str, Any], resume_follow: bool) -> None:
        self._event("info", f"开始回到示教点 {key}")
        cfg = self.store.data["preset"]
        timeout = float(cfg["timeout_s"])
        try:
            if self._preset_stop.is_set():
                raise RuntimeError("预设回位已被紧急停止")
            if self.robot is None:
                raise RuntimeError("CR5 未连接")
            if self.robot.servo_state() != 3:
                raise RuntimeError("CR5 尚未上电运行")
            roarm_target = [float(value) for value in preset["roarm_joints_rad"]]
            robot_target = [float(value) for value in preset["robot_joints_deg"]]
            robot_tcp_target = [float(value) for value in preset["robot_tcp"]]
            o6_target = _six_uint8(preset["o6_position"], "示教点 O6")

            if self._preset_stop.is_set():
                raise RuntimeError("预设回位已被紧急停止")
            self.o6.set_target(o6_target)
            # 主臂可能处于卸力拖动状态：先以当前反馈位置上力矩，避免启动瞬间跳回旧目标。
            if self.master_type != "gello":
                self.roarm.hold_current()
                self.roarm.move_joints(roarm_target, float(cfg["roarm_duration_s"]))
            self._set_dataset_target_tcp(robot_tcp_target)
            self.robot.movej(
                robot_target,
                float(cfg["robot_velocity_percent"]),
                float(cfg["robot_acc_percent"]),
                float(cfg["robot_dec_percent"]),
            )

            deadline = time.monotonic() + timeout
            robot_ok = roarm_ok = o6_ok = False
            while time.monotonic() < deadline:
                if self._preset_stop.is_set():
                    raise RuntimeError("预设回位已被紧急停止")
                current_robot = self.robot.joint_position()
                current_master = self.roarm.latest()
                current_o6, current_fault, _ = self.o6.latest()
                if current_fault and any(current_fault):
                    raise RuntimeError(f"O6 故障: {current_fault}")
                robot_ok = max(
                    abs(a - b) for a, b in zip(current_robot[:6], robot_target[:6])
                ) <= 1.0
                roarm_ok = (
                    self.master_type == "gello"
                    or bool(
                        current_master
                        and max(
                            abs(a - b)
                            for a, b in zip(current_master.joints, roarm_target)
                        ) <= 0.04
                    )
                )
                o6_ok = bool(
                    current_o6
                    and max(abs(a - b) for a, b in zip(current_o6, o6_target)) <= 6
                )
                if robot_ok and roarm_ok and o6_ok:
                    break
                time.sleep(0.08)
            if not (robot_ok and roarm_ok and o6_ok):
                missing = []
                if not robot_ok:
                    missing.append("CR5")
                if not roarm_ok and self.master_type != "gello":
                    missing.append("RoArm")
                if not o6_ok:
                    missing.append("O6")
                raise TimeoutError("/".join(missing) + " 未在规定时间内到达示教点")
            # MoveJ has reached the saved pose. End its controller-side command
            # ownership before reopening ServoJ for XYZ following.
            self.robot.stop_motion()
            if not self.robot.wait_until_motion_stopped(
                min(timeout, 5.0),
                stop_event=self._preset_stop,
            ):
                if self._preset_stop.is_set():
                    raise RuntimeError("预设回位已被紧急停止")
                raise TimeoutError("CR5 到达示教点后仍未完全停止")
            current_robot = self.robot.joint_position()
            if max(
                abs(a - b) for a, b in zip(current_robot[:6], robot_target[:6])
            ) > 1.0:
                raise RuntimeError("CR5 停止后偏离示教点")
            if self._preset_stop.is_set():
                raise RuntimeError("预设回位已被紧急停止")
            time.sleep(float(self.store.data["replay"]["mode_switch_settle_s"]))
            if self._preset_stop.is_set() or self.state != "preset":
                raise RuntimeError("预设回位已被紧急停止")
            if key == "HOME":
                self._hand_action_stop.set()
                with self._state_lock:
                    self._o6_manual_override = False
                    self._o6_action_name = ""
            if resume_follow:
                # This callback still runs inside _preset_thread.  Allow only
                # this internal transition to establish a fresh master/slave
                # origin after the higher-priority preset motion has finished.
                self.start_follow(_resume_from_preset=True)
                self._event(
                    "info",
                    f"示教点 {key} 回位完成；已用当前主从位姿重建 XYZ 零点并恢复跟随",
                )
            else:
                self._set_state("idle")
                self._event(
                    "info",
                    f"示教点 {key} 回位完成；保持主从臂锁定"
                    + ("；已恢复 M5 对 O6 的控制权" if key == "HOME" else ""),
                )
        except Exception as exc:
            message = str(exc)
            if self.state != "closed":
                self._set_state("fault", message)
            self._event("error", f"示教点 {key} 回位失败: {message}")
            try:
                if self.robot is not None:
                    self.robot.stop_motion()
            except Exception:
                pass
            try:
                self._hold_roarm()
            except Exception:
                pass

    def acknowledge_fault(self) -> None:
        """Clear the local runtime fault latch after hardware recovery.

        This method sends no robot/hand/master command. The caller must first
        verify that the underlying hardware recovery/reset has succeeded.
        """
        with self._state_lock:
            if self._state == "closed":
                raise RuntimeError("遥操作引擎已经关闭")
            if self._state not in ("fault", "idle"):
                raise RuntimeError(
                    f"当前运行态 {self._state} 不允许确认故障恢复"
                )
            self._state = "idle"
            self._last_error = ""
        self._event(
            "info",
            "运行时故障已确认清除；保持 idle，不会自动恢复主从跟随",
        )

    def emergency_stop(self, reason: str = "软件紧急停止") -> None:
        """Stop commanded motion; this does not replace the cabinet E-stop."""
        with self._state_lock:
            self._stop_generation += 1
            self._follow_stop.set()
            self._preset_stop.set()
            self._hand_action_stop.set()
            self._record_stop.set()
            self._replay_stop.set()
        errors: list[str] = []
        if self.robot is not None:
            try:
                self.robot.stop_motion()
            except Exception as exc:
                errors.append(f"CR5: {exc}")
        try:
            self.o6.hold_current()
        except Exception as exc:
            errors.append(f"O6: {exc}")
        try:
            self._hold_roarm()
        except Exception as exc:
            errors.append(f"主臂: {exc}")
        message = reason if not errors else reason + "；" + "；".join(errors)
        self._set_state("fault", message)
        self._event("error", message)

    def shutdown(self) -> None:
        self._shutdown_requested.set()
        with self._state_lock:
            self._stop_generation += 1
            self._follow_stop.set()
            self._preset_stop.set()
            self._hand_action_stop.set()
            self._record_stop.set()
            self._replay_stop.set()
        self.stop_follow("程序退出")
        if self.robot is not None:
            try:
                self.robot.stop_motion()
            except Exception:
                pass
        preset_thread = self._preset_thread
        if preset_thread is not None and preset_thread is not threading.current_thread():
            preset_thread.join(timeout=2.0)
        for thread in (self._record_thread, self._replay_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=3.0)
        self.o6.close()
        self.roarm.close(hold=True)
        self._set_state("closed")

    def dataset_sample(self) -> dict[str, Any]:
        """Read a bounded-age state/action sample (not hardware synchronized).

        The observation follows the portable OpenPI interface contract: six CR5
        joints in radians, TCP XYZ+RPY in metres/radians, then six raw O6
        positions.  The action is the shortest TCP delta from current feedback
        to the last command sent by follow/preset/replay plus the last O6 RS485
        target. SDK send success is not a controller execution acknowledgement.
        """
        state = self.state
        if state == "closed":
            raise RuntimeError("遥操已关闭，无法记录 LeRobot Episode")
        robot = self.robot
        if robot is None:
            raise RuntimeError("CR5 连接已丢失")

        sample_started_at = time.monotonic()
        gello_joint_follow = (
            self.master_type == "gello" and state == "following"
            and self.store.data["gello"]["control_mode"] == "joint"
        )
        with self._state_lock:
            telemetry = self._gello_telemetry
            target_tcp = self._dataset_target_tcp_controller
            action_timestamp = self._dataset_action_timestamp
        quality = {}
        if gello_joint_follow and (telemetry is None or not telemetry["sent_count"]):
            quality["gello_telemetry_missing"] = 1.0
            gello_joint_follow = False
        if gello_joint_follow:
            age_limit = float(self.store.data["dataset"]["action_max_age_s"])
            feedback_age = sample_started_at - telemetry["feedback_at"]
            quality["feedback_age_s"] = feedback_age
            quality["feedback_stale"] = float(feedback_age > age_limit)
            pending_target_deg = float(telemetry.get("pending_target_deg", 0.0))
            duplicate_deadband = (
                float(self.store.data["robot"]["movej_duplicate_deadband_deg"])
                if getattr(self.robot, "motion_mode", "servoj") == "movej"
                else 0.0
            )
            validated_at = float(telemetry.get("validated_at", telemetry["command_at"]))
            effective_action_at = (
                validated_at if pending_target_deg <= duplicate_deadband
                else telemetry["command_at"]
            )
            action_age = sample_started_at - effective_action_at
            quality["sdk_submit_age_s"] = sample_started_at - telemetry["command_at"]
            quality["action_stale"] = float(action_age > age_limit)
            tracking_limit = float(self.store.data["dataset"]["max_tracking_error_deg"])
            tracking_error_deg = float(telemetry["tracking_error_deg"])
            quality["tracking_error_deg"] = tracking_error_deg
            quality["tracking_error_limit_deg"] = tracking_limit
            quality["tracking_error_exceeded"] = float(
                tracking_error_deg > tracking_limit
            )
            if telemetry.get("low_latency", False):
                desired_error = float(telemetry["desired_tracking_error_deg"])
                quality["desired_tracking_error_deg"] = desired_error
                quality["desired_tracking_error_exceeded"] = float(
                    desired_error > tracking_limit
                )
            joints = list(telemetry["actual_deg"])
            target_tcp = robot.forward_kinematics(telemetry["target_full_deg"])
            quality["validated_target_age_s"] = sample_started_at - validated_at
            quality["pending_target_deg"] = pending_target_deg
            action_timestamp = effective_action_at
        else:
            joints = [float(value) for value in robot.joint_position()]
        if len(joints) < 6 or any(not math.isfinite(value) for value in joints[:6]):
            raise RuntimeError("CR5 六关节反馈无效")
        current_tcp = [float(value) for value in robot.tcp_position()]
        if len(current_tcp) < 6 or any(
            not math.isfinite(value) for value in current_tcp[:6]
        ):
            raise RuntimeError("CR5 TCP 反馈无效")

        o6_position, o6_fault, o6_timestamp = self.o6.latest()
        now = time.monotonic()
        if o6_position is None or len(o6_position) != 6:
            raise RuntimeError("O6 位置反馈不完整")
        quality["o6_feedback_stale"] = float(
            now - o6_timestamp > float(self.store.data["o6"]["feedback_timeout_s"])
        )
        quality["o6_fault_present"] = float(o6_fault is None or any(o6_fault))

        if target_tcp is None:
            target_tcp = current_tcp
        if state == "following":
            action_age_limit = float(self.store.data["dataset"]["action_max_age_s"])
            quality["action_stale"] = max(
                quality.get("action_stale", 0.0),
                float(now - action_timestamp > action_age_limit),
            )
        o6_target = getattr(self.o6, "last_sent_target", None)
        if o6_target is None:
            o6_target = o6_position
        if len(o6_target) != 6:
            raise RuntimeError("O6 实际下发目标不完整")

        delta_xyz_m = [
            (float(target_tcp[index]) - current_tcp[index]) / 1000.0
            for index in range(3)
        ]
        delta_rpy_rad = [
            _wrapped_angle_delta(float(target_tcp[index]), current_tcp[index])
            for index in range(3, 6)
        ]
        observation_state = (
            [math.radians(value) for value in joints[:6]]
            + [value / 1000.0 for value in current_tcp[:3]]
            + current_tcp[3:6]
            + [float(value) for value in o6_position]
        )

        quality["sample_latency_s"] = now - sample_started_at
        quality["command_age_s"] = now - action_timestamp
        return {
            "timestamp": now,
            "quality": quality,
            "observation_state": observation_state,
            "action": delta_xyz_m
            + delta_rpy_rad
            + [float(value) for value in o6_target],
        }

    def snapshot(self) -> dict[str, Any]:
        master = self.roarm.latest()
        o6_position, o6_fault, o6_timestamp = self.o6.latest()
        with self._state_lock:
            record_count = len(self._record_frames)
            record_duration = (
                float(self._record_frames[-1]["t"]) if self._record_frames else 0.0
            )
            o6_action_name = self._o6_action_name
            record_slot = self._record_slot
            replay_slot = self._replay_slot
            record_joint_indices = self._record_joint_indices
            gello_telemetry = (
                dict(self._gello_telemetry) if self._gello_telemetry is not None else None
            )
        replay_slots = []
        for slot in (1, 2, 3):
            path = self._replay_path(slot)
            available, file_status = self._get_replay_file_status(slot)
            replay_slots.append(
                {
                    "slot": slot,
                    "path": str(path),
                    "available": available,
                    "file_exists": path.exists(),
                    "file_status": file_status,
                }
            )
        first_slot = replay_slots[0]
        return {
            "state": self.state,
            "error": self.last_error,
            "master_type": self.master_type,
            "gello_telemetry": gello_telemetry,
            "master": master,
            "roarm": master,
            "o6_position": o6_position,
            "o6_fault": o6_fault,
            "o6_timestamp": o6_timestamp,
            "o6_phase": self.o6.phase,
            "o6_manual_override": self.o6_manual_override,
            "o6_action_active": self.o6_action_active,
            "o6_action_name": o6_action_name,
            "recording": self.recording,
            "record_count": record_count,
            "record_duration": record_duration,
            "record_slot": record_slot,
            "record_joint_indices": record_joint_indices,
            "replaying": self.replaying,
            "replay_slot": replay_slot,
            "replay_slots": replay_slots,
            "replay_path": first_slot["path"],
            "replay_available": first_slot["available"],
            "replay_file_exists": first_slot["file_exists"],
            "replay_file_status": first_slot["file_status"],
            "presets": sorted(self.store.data.get("presets", {}).keys()),
        }

"""Saved HOME and compatibility taught-pose orchestration through Motion leases."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from datetime import UTC, datetime
from functools import partial
from typing import Any

from gello_cr.app.config_store import _six_uint8
from gello_cr.motion import (
    JointTarget,
    MotionSource,
)


class HomeService:
    """Application service; device I/O remains behind the runtime adapters."""

    def __init__(self, runtime):
        self.runtime = runtime
        self._event = partial(runtime._event, domain="motion")

    def save_home(self) -> dict[str, Any]:
        """Persist the current robot and hand pose without resuming motion."""
        if self.runtime.robot is None:
            raise RuntimeError("请先连接机械臂")
        if self.runtime.state not in ("idle", "following") or self.runtime.o6_action_active:
            raise RuntimeError("当前状态不允许保存 HOME")
        if self.runtime.state == "following":
            self.runtime.stop_follow("保存 HOME")
        self.runtime.trajectory_source.check_ready(False, True)
        position, fault, _ = self.runtime.o6.latest()
        if position is None or fault is None or any(fault):
            raise RuntimeError(f"O6 状态无效: {fault}")
        home = {
            "saved_at": datetime.now(UTC).isoformat(),
            "robot_joints_deg": self.runtime.robot.joint_position(),
            "robot_tcp": self.runtime.robot.tcp_position(),
            "o6_position": list(position),
        }
        self.runtime.store.data["presets"]["HOME"] = home
        self.runtime.store.save()
        self._event("info", "HOME 已保存；保持停止")
        return home

    def return_home(self, stop_recording: Callable[[], None]) -> None:
        """Preempt motion, return the robot and O6 to HOME, then end the Episode.

        The active Episode keeps sampling throughout the HOME motion so the
        return trajectory is recorded too; ``stop_recording`` is invoked only
        after the robot has settled at HOME. Runs on a dedicated GUI worker;
        software stop can cancel it at any stage.
        """
        if not self.runtime._home_lock.acquire(blocking=False):
            raise RuntimeError("HOME 回位正在执行")
        robot = self.runtime.robot
        lease = None
        generation = self.runtime.stop_generation
        try:
            if robot is None or self.runtime.state == "closed":
                raise RuntimeError("请先连接机械臂")
            preset = self.runtime.store.data["presets"].get("HOME")
            if preset is None:
                raise RuntimeError("初始位 HOME 尚未保存")
            target = [float(value) for value in preset["robot_joints_deg"]]
            if len(target) != 7 or not all(math.isfinite(v) for v in target):
                raise ValueError("HOME 关节目标必须是 7 个有限数值（含 SDK 占位轴）")
            o6_cfg = self.runtime.store.data["o6"]
            o6_target = _six_uint8(o6_cfg["actions"][o6_cfg["open_action"]], "HOME 张开手")
            o6_homing = self.runtime.o6.connected
            if o6_homing:
                self.runtime.trajectory_source.check_ready(False, True)
            # 尽早设置录制动作目标。关节录制在 state 离开 "following" 后只能依赖
            # HOME 关节目标；必须在状态切换前设好，否则回位窗口内 dataset_sample()
            # 会抛「关节记录需要 GELLO 关节跟随」而打断录制。
            home_tcp = preset.get("robot_tcp")
            if home_tcp is not None:
                self.runtime._set_dataset_target_tcp([float(value) for value in home_tcp])
            if self.runtime.store.data["dataset"]["recording_mode"] == "joint":
                self.runtime._set_dataset_target_joints(target)
            with self.runtime._state_lock:
                self.runtime._stop_generation += 1
                generation = self.runtime._stop_generation
                self.runtime._follow_stop.set()
                self.runtime._home_stop.set()
                self.runtime._replay_stop.set()
                self.runtime._record_stop.set()
                self.runtime._hand_action_stop.set()
                producers = (
                    self.runtime._follow_thread,
                    self.runtime._home_thread,
                    self.runtime._replay_thread,
                    self.runtime._record_thread,
                    self.runtime.point_motion.thread,
                )
                self.runtime._state = "recovering"
            lease = self.runtime.motion.acquire(
                MotionSource.HOME,
                allow_preempt=True,
                on_cancel=self.runtime._home_stop.set,
            )
            if self.runtime.external_control is not None:
                self.runtime.external_control.request_stop()
            for thread in producers:
                if thread is not None and thread is not threading.current_thread():
                    thread.join(timeout=3.0)
                    if thread.is_alive():
                        raise TimeoutError(f"{thread.name} 未停止，禁止 HOME 回位")
            self.runtime.finish_external_control()
            # 不在这里停止 Episode：HOME 回位过程本身也要被录制，等回位完成后再 stop_recording()。
            with self.runtime._state_lock:
                if (
                    self.runtime._stop_generation != generation
                    or self.runtime._shutdown_requested.is_set()
                ):
                    raise RuntimeError("HOME 回位已被停止")
                self.runtime._home_thread = threading.current_thread()
                self.runtime._home_stop.clear()
                self.runtime._state = "preset"
            if not robot.wait_until_motion_stopped(5.0, stop_event=self.runtime._home_stop):
                raise RuntimeError("机械臂尚未停止或 HOME 已取消")
            if robot.servo_state() != 3:
                robot.power_on()
            if robot.servo_state() != 3:
                raise RuntimeError("HOME 回位使能失败")
            if self.runtime._home_stop.is_set() or self.runtime._stop_generation != generation:
                raise RuntimeError("HOME 回位已被停止")
            cfg = self.runtime.store.data["preset"]
            self._event(
                "info",
                "机械臂回 HOME：其他运动已停止，录制继续，回位轨迹也会被保存"
                + ("；O6 同步回 HOME" if o6_homing else "；O6 未连接，仅机械臂回位"),
            )
            if o6_homing:
                self.runtime.o6.set_profile(o6_cfg["speed"], o6_cfg["torque"])
                self.runtime.o6.set_target(o6_target)
            lease.move_joint(
                JointTarget.from_degrees(target),
                float(cfg["robot_velocity_percent"]),
                float(cfg["robot_acc_percent"]),
                float(cfg["robot_dec_percent"]),
            )
            deadline = time.monotonic() + float(cfg["timeout_s"])
            robot_ok = False
            o6_ok = not o6_homing
            while True:
                if self.runtime._home_stop.is_set() or self.runtime._stop_generation != generation:
                    raise RuntimeError("HOME 回位已被停止")
                lease.check()
                current = robot.joint_position()
                robot_ok = (
                    max(abs(a - b) for a, b in zip(current[:6], target[:6], strict=True)) <= 1.0
                )
                if o6_homing:
                    self.runtime.trajectory_source.check_ready(False, True)
                    current_o6, current_fault, _ = self.runtime.o6.latest()
                    if current_fault and any(current_fault):
                        raise RuntimeError(f"O6 故障: {current_fault}")
                    o6_ok = bool(
                        current_o6
                        and max(abs(a - b) for a, b in zip(current_o6, o6_target, strict=True)) <= 6
                    )
                if robot_ok and o6_ok:
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError("机械臂或 O6 未在规定时间内到达 HOME")
                self.runtime._home_stop.wait(0.05)
            lease.stop()
            if not robot.wait_until_motion_stopped(5.0, stop_event=self.runtime._home_stop):
                raise RuntimeError("HOME 到达后停止未确认")
            if (
                max(abs(a - b) for a, b in zip(robot.joint_position()[:6], target[:6], strict=True))
                > 1.0
            ):
                raise RuntimeError("机械臂停止后偏离 HOME")
            # 回位完成，结束录制：HOME 轨迹已包含在 Episode 中，随后保存为 pending。
            stop_recording()
            with self.runtime._state_lock:
                if self.runtime._stop_generation != generation or self.runtime._home_stop.is_set():
                    raise RuntimeError("HOME 回位已被停止")
                self.runtime._hand_action_stop.set()
                self.runtime._o6_manual_override = False
                self.runtime._o6_action_name = ""
                self.runtime._state = "idle"
                self.runtime._last_error = ""
            self._event("info", "机械臂与 O6 已回 HOME；录制已结束，请保存 Episode")
        except Exception as exc:
            # 回位失败时也结束录制，避免关节录制在 fault 态继续产出 HOME 目标的
            # 无效样本。尽力而为，不掩盖原始异常。
            try:
                stop_recording()
            except Exception as cleanup_error:
                exc.add_note(f"HOME recording cleanup: {cleanup_error}")
                self._event("error", f"HOME 停止录制失败: {cleanup_error}")
            if self.runtime.state != "closed" and self.runtime.stop_generation == generation:
                self.runtime._set_state("fault", str(exc))
            if lease is not None:
                try:
                    lease.stop()
                except Exception as stop_exc:
                    raise ExceptionGroup("HOME 回位及停止失败", [exc, stop_exc]) from None
            raise
        finally:
            try:
                if lease is not None:
                    lease.release()
            except Exception as exc:
                if self.runtime.state != "closed":
                    self.runtime._set_state("fault", str(exc))
                raise
            finally:
                with self.runtime._state_lock:
                    if self.runtime._home_thread is threading.current_thread():
                        self.runtime._home_thread = None
                self.runtime._home_lock.release()

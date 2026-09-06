"""LeRobot episode recording bridge for the CR5/O6 Qt application.

The Qt application and LeRobot deliberately run in separate Python environments:
the hardware environment owns NRC, PyQt and RealSense, while the LeRobot worker
owns datasets/PyArrow/PyAV.  A bounded synchronous Unix socket carries one
three 224x224 RGB frames plus state/action values at a time, so raw RealSense
frames are never accumulated in memory.  The streams follow the OpenPI contract:
base camera full view, wrist camera full view, and base-camera ROI.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from pathlib import Path
from typing import Any, Callable, Optional

import cv2
import numpy as np

_V2_SRC_DIR = Path(__file__).resolve().parent / "src"
if str(_V2_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_V2_SRC_DIR))

from gello_cr.recording.frame import (
    prepare_recording_frame,
    resize_rgb_for_openpi,
)
from gello_cr.recording.protocol import (
    read_exact as _read_exact,
    receive_packet as _receive_packet,
    send_packet as _send_packet,
)
from gello_cr.recording.worker_client import (
    WorkerClient as _WorkerClient,
    WorkerClientError as LeRobotRecorderError,
)
from gello_cr.recording.schema import (
    dataset_features as v2_dataset_features,
    validate_recording_sample,
)
from gello_cr.recording.worker_service import run_worker as _run_worker_service



class LeRobotEpisodeRecorder:
    """Threaded episode recorder used by the Qt process."""

    def __init__(
        self,
        sample_provider: Callable[[], dict[str, Any]],
        event_callback: Callable[[str, str], None],
        worker_python: str,
        repo_prefix: str,
        fps: int = 20,
        image_size: tuple[int, int] = (224, 224),
    ):
        self.sample_provider = sample_provider
        self.event_callback = event_callback
        self.worker_python = str(worker_python)
        self.repo_prefix = str(repo_prefix)
        self.fps = int(fps)
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self._lock = threading.RLock()
        self._client: Optional[_WorkerClient] = None
        self._session_root = ""
        self._repo_id = ""
        self._episode_thread: Optional[threading.Thread] = None
        self._episode_stop = threading.Event()
        self._episode_active = False
        self._buffered_frames = 0
        self._saved_episodes = 0
        self._task = ""
        self._last_error = ""
        self._started_at = 0.0
        self._stopped_at = 0.0
        self._first_sample_at = 0.0
        self._last_sample_at = 0.0
        self._late_frames = 0
        self._max_frame_gap_s = 0.0
        self._quality_max: dict[str, float] = {}
        self._last_camera_timestamps: dict[str, float] = {}
        self._reused_camera_frames = {"wrist": 0, "base": 0}
        self._episode_metadata: dict[str, Any] = {}
        self._last_saved_summary: dict[str, Any] = {}

    def _event(self, level: str, message: str) -> None:
        self.event_callback(level, message)

    def _ensure_session(self, base_root: str) -> None:
        with self._lock:
            if self._client is not None:
                return
        stamp = (
            time.strftime("%Y%m%d_%H%M%S")
            + f"_{int(time.time_ns() % 1_000_000_000):09d}"
        )
        prefix_owner, prefix_name = self.repo_prefix.split("/", 1)
        repo_id = f"{prefix_owner}/{prefix_name}_{stamp}"
        session_root = Path(base_root).expanduser().resolve() / f"{prefix_name}_{stamp}"
        client = _WorkerClient(
            self.worker_python,
            worker_script=Path(__file__).resolve(),
        )
        try:
            response = client.request(
                {
                    "op": "init",
                    "repo_id": repo_id,
                    "root": str(session_root),
                    "fps": self.fps,
                    "height": self.image_size[0],
                    "width": self.image_size[1],
                },
                timeout=60.0,
            )
        except Exception:
            client.close()
            raise
        with self._lock:
            self._client = client
            self._session_root = str(response["root"])
            self._repo_id = repo_id
            self._saved_episodes = 0
            self._last_saved_summary = {}
        self._event("info", f"LeRobot data session created: {self._session_root}")

    def start_episode(
        self, task: str, base_root: str, *, metadata: Optional[dict[str, Any]] = None
    ) -> None:
        task = str(task).strip()
        if not task:
            raise ValueError("LeRobot task text cannot be empty")
        with self._lock:
            if self._episode_active:
                raise LeRobotRecorderError("A LeRobot Episode is already recording")
            if self._buffered_frames:
                raise LeRobotRecorderError("The previous Episode is not saved or discarded")
        # Validate hardware and camera before creating a dataset directory.
        first_sample = self.sample_provider()
        self._validate_sample(first_sample)
        self._ensure_session(base_root)
        # Worker imports/encoder setup can take seconds. Never record the
        # preflight frame with an action captured before that initialization.
        first_sample = self.sample_provider()
        self._validate_sample(first_sample)
        with self._lock:
            self._task = task
            self._last_error = ""
            self._episode_active = True
            self._started_at = time.monotonic()
            self._stopped_at = 0.0
            self._first_sample_at = 0.0
            self._last_sample_at = 0.0
            self._late_frames = 0
            self._max_frame_gap_s = 0.0
            self._quality_max = {}
            self._last_camera_timestamps = {}
            self._reused_camera_frames = {"wrist": 0, "base": 0}
            self._episode_metadata = {
                "started_at_unix_s": time.time(),
                "context": metadata or {},
            }
            self._episode_stop.clear()
            self._episode_thread = threading.Thread(
                target=self._record_loop,
                args=(first_sample,),
                name="LeRobot-Episode",
                daemon=True,
            )
            self._episode_thread.start()
        self._event("info", f"LeRobot Episode recording started: {task}")

    def _validate_sample(self, sample: dict[str, Any]) -> None:
        validate_recording_sample(sample)

    def _add_sample(self, sample: dict[str, Any]) -> None:
        with self._lock:
            client = self._client
            task = self._task
        if client is None:
            raise LeRobotRecorderError("LeRobot session is not initialized")

        prepared = prepare_recording_frame(
            sample,
            task=task,
            height=self.image_size[0],
            width=self.image_size[1],
        )
        client.request(
            prepared.payload,
            raw=prepared.raw,
            timeout=15.0,
        )
        with self._lock:
            sampled_at = float(sample.get("timestamp", time.monotonic()))
            if self._buffered_frames:
                gap = sampled_at - self._last_sample_at
                self._max_frame_gap_s = max(self._max_frame_gap_s, gap)
                if gap > 1.5 / self.fps:
                    self._late_frames += 1
            else:
                self._first_sample_at = sampled_at
            self._last_sample_at = sampled_at
            for key, value in sample.get("quality", {}).items():
                if key in ("wrist_timestamp", "base_timestamp"):
                    if self._last_camera_timestamps.get(key) == value:
                        self._reused_camera_frames[key.split("_")[0]] += 1
                    self._last_camera_timestamps[key] = value
                else:
                    self._quality_max[key] = max(self._quality_max.get(key, 0.0), float(value))
            self._buffered_frames += 1

    def _record_loop(self, first_sample: dict[str, Any]) -> None:
        period = 1.0 / self.fps
        next_sample = time.monotonic()
        pending_first: Optional[dict[str, Any]] = first_sample
        try:
            while not self._episode_stop.is_set():
                sample = pending_first if pending_first is not None else self.sample_provider()
                pending_first = None
                self._add_sample(sample)
                next_sample += period
                wait_time = next_sample - time.monotonic()
                if wait_time > 0:
                    self._episode_stop.wait(wait_time)
                else:
                    # Do not create an unbounded backlog if camera/encoding is slow.
                    next_sample = time.monotonic()
        except Exception as exc:
            message = f"{type(exc).__name__}: {exc}"
            with self._lock:
                self._last_error = message
            self._event("error", f"LeRobot Episode recording stopped: {message}")
        finally:
            with self._lock:
                self._episode_active = False
                self._stopped_at = time.monotonic()

    def _stop_sampling(self) -> None:
        self._episode_stop.set()
        with self._lock:
            thread = self._episode_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=16.0)
            if thread.is_alive():
                raise LeRobotRecorderError("LeRobot sampling thread did not stop")

    def stop_episode(self) -> None:
        """End sampling while retaining the current buffer for review."""
        self._stop_sampling()
        self._event("info", "Episode 已结束录制；请标注结果并保存，或明确丢弃")

    def save_episode(self, outcome: str = "success", notes: str = "") -> None:
        if outcome not in ("success", "failure"):
            raise ValueError("Episode outcome must be success or failure")
        self._stop_sampling()
        with self._lock:
            client = self._client
            frame_count = self._buffered_frames
            error = self._last_error
        if client is None or frame_count <= 0:
            raise LeRobotRecorderError("There are no LeRobot Episode frames to save")
        if error and outcome != "failure":
            raise LeRobotRecorderError(
                f"Episode stopped with an error; save it only as failure or discard it: {error}"
            )
        summary = {
            **self._episode_metadata,
            "ended_at_unix_s": time.time(),
            "task": self._task,
            "outcome": outcome,
            "notes": str(notes).strip(),
            "quality": self.snapshot()["quality"],
        }
        try:
            response = client.request(
                {"op": "save_episode", "collection": summary}, timeout=180.0
            )
        except Exception as exc:
            with self._lock:
                self._last_error = f"保存失败，请检查数据目录，不要重复提交：{exc}"
            raise
        with self._lock:
            self._saved_episodes = int(response["episodes"])
            self._buffered_frames = 0
            self._task = ""
            self._started_at = 0.0
            self._last_saved_summary = summary
        self._event(
            "info",
            f"LeRobot Episode saved: {frame_count} frames; total {self._saved_episodes}",
        )

    def discard_episode(self) -> None:
        self._stop_sampling()
        with self._lock:
            client = self._client
            frame_count = self._buffered_frames
        if client is not None and frame_count:
            try:
                client.request({"op": "clear_episode"}, timeout=60.0)
            except Exception as exc:
                with self._lock:
                    self._last_error = f"丢弃失败，数据未确认清除：{exc}"
                raise
        with self._lock:
            self._buffered_frames = 0
            self._task = ""
            self._last_error = ""
            self._started_at = 0.0
        self._event("warning", f"LeRobot Episode discarded: {frame_count} frames")

    def finalize(self) -> Optional[str]:
        with self._lock:
            if self._episode_active or self._buffered_frames:
                raise LeRobotRecorderError("当前 Episode 未处理；请先结束录制并保存或丢弃")
        self._stop_sampling()
        with self._lock:
            client = self._client
            root = self._session_root
        if client is None:
            return None
        try:
            client.request({"op": "finalize"}, timeout=180.0)
        except Exception as exc:
            with self._lock:
                self._last_error = f"数据集收尾失败：{exc}"
            raise
        client.close()
        with self._lock:
            self._client = None
            self._buffered_frames = 0
            self._episode_active = False
            self._task = ""
            self._last_error = ""
            self._started_at = 0.0
        self._event("info", f"LeRobot dataset finalized: {root}")
        return root

    def close(self) -> None:
        self.finalize()

    def preserve_failed_session(self) -> None:
        """Explicit recovery: release a failed worker, never clear disk data."""
        with self._lock:
            if not self._last_error:
                raise LeRobotRecorderError("会话没有错误，请正常保存/丢弃后结束数据集")
            client = self._client
            root = self._session_root
            error = self._last_error
        self._stop_sampling()
        if client is not None:
            client.close()
        with self._lock:
            self._client = None
            self._buffered_frames = 0
            self._episode_active = False
            self._started_at = 0.0
            self._task = ""
            self._last_error = ""
        self._event(
            "warning", f"异常会话已释放，磁盘文件未删除：{root}；"
            f"未完成数据可能不完整，请保留诊断且不要用于训练。原错误：{error}",
        )

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            duration = (
                (self._stopped_at or time.monotonic()) - self._started_at
                if self._started_at else 0.0
            )
            span = self._last_sample_at - self._first_sample_at
            quality = {
                "frames": self._buffered_frames,
                "duration_s": duration,
                "configured_fps": self.fps,
                "effective_fps": (self._buffered_frames - 1) / span
                if self._buffered_frames > 1 and span > 0 else 0.0,
                "late_frames": self._late_frames,
                "max_frame_gap_s": self._max_frame_gap_s,
                "reused_camera_frames": dict(self._reused_camera_frames),
                "max": dict(self._quality_max),
                "timestamp_basis": "host_monotonic_receipt_not_hardware_sync",
            }
            quality["needs_review"] = bool(
                self._buffered_frames < 2
                or quality["effective_fps"] < 0.9 * self.fps
                or self._late_frames
                or any(self._reused_camera_frames.values())
                or self._quality_max.get("tracking_error_exceeded", 0.0) > 0.0
                or self._quality_max.get("desired_tracking_error_exceeded", 0.0) > 0.0
                or self._quality_max.get("feedback_stale", 0.0) > 0.0
                or self._quality_max.get("action_stale", 0.0) > 0.0
                or self._quality_max.get("o6_feedback_stale", 0.0) > 0.0
                or self._quality_max.get("o6_fault_present", 0.0) > 0.0
                or self._quality_max.get("wrist_age_exceeded", 0.0) > 0.0
                or self._quality_max.get("base_age_exceeded", 0.0) > 0.0
                or self._quality_max.get("camera_skew_exceeded", 0.0) > 0.0
            )
            return {
                "session_active": self._client is not None,
                "episode_active": self._episode_active,
                "buffered_frames": self._buffered_frames,
                "saved_episodes": self._saved_episodes,
                "duration": duration,
                "task": self._task,
                "root": self._session_root,
                "repo_id": self._repo_id,
                "error": self._last_error,
                "quality": quality,
                "last_saved": self._last_saved_summary,
            }


def _dataset_features(height: int, width: int) -> dict[str, dict[str, Any]]:
    return v2_dataset_features(height, width)



def _worker_main(fd: int) -> int:
    return _run_worker_service(fd)



def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker-fd", type=int, required=True)
    args = parser.parse_args()
    return _worker_main(args.worker_fd)


if __name__ == "__main__":
    raise SystemExit(main())

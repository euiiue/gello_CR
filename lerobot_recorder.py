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
from gello_cr.recording.schema import (
    dataset_features as v2_dataset_features,
    validate_recording_sample,
)



class LeRobotRecorderError(RuntimeError):
    pass



def _read_exact(sock: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = int(size)
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise EOFError("LeRobot worker connection closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _send_packet(sock: socket.socket, payload: dict[str, Any], raw: bytes = b"") -> None:
    message = dict(payload)
    message["raw_size"] = len(raw)
    encoded = json.dumps(message, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    sock.sendall(struct.pack("!I", len(encoded)) + encoded + raw)


def _receive_packet(sock: socket.socket) -> tuple[dict[str, Any], bytes]:
    header_size = struct.unpack("!I", _read_exact(sock, 4))[0]
    if header_size <= 0 or header_size > 4 * 1024 * 1024:
        raise ValueError(f"Invalid protocol header size: {header_size}")
    payload = json.loads(_read_exact(sock, header_size).decode("utf-8"))
    raw_size = int(payload.pop("raw_size", 0))
    if raw_size < 0 or raw_size > 16 * 1024 * 1024:
        raise ValueError(f"Invalid protocol raw size: {raw_size}")
    return payload, _read_exact(sock, raw_size) if raw_size else b""


class _WorkerClient:
    def __init__(self, python_executable: str):
        executable = Path(python_executable).expanduser()
        if not executable.is_file():
            raise FileNotFoundError(f"LeRobot Python does not exist: {executable}")
        parent_socket, child_socket = socket.socketpair()
        self._socket = parent_socket
        self._request_lock = threading.Lock()
        self._communication_error = ""
        self._stderr_lines: deque[str] = deque(maxlen=30)
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        worker_lib = executable.parent.parent / "lib"
        inherited_library_path = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = str(worker_lib) + (
            f":{inherited_library_path}" if inherited_library_path else ""
        )
        self._process = subprocess.Popen(
            [
                str(executable),
                str(Path(__file__).resolve()),
                "--worker-fd",
                str(child_socket.fileno()),
            ],
            pass_fds=(child_socket.fileno(),),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=env,
        )
        child_socket.close()
        self._stderr_thread = threading.Thread(
            target=self._collect_stderr,
            name="LeRobot-Worker-Stderr",
            daemon=True,
        )
        self._stderr_thread.start()

    def _collect_stderr(self) -> None:
        stream = self._process.stderr
        if stream is None:
            return
        for line in stream:
            text = line.rstrip()
            if text:
                self._stderr_lines.append(text)

    def request(
        self,
        payload: dict[str, Any],
        raw: bytes = b"",
        timeout: float = 30.0,
    ) -> dict[str, Any]:
        with self._request_lock:
            if self._communication_error:
                raise LeRobotRecorderError(self._communication_error)
            if self._process.poll() is not None:
                details = "\n".join(self._stderr_lines)
                raise LeRobotRecorderError(
                    f"LeRobot worker exited with code {self._process.returncode}"
                    + (f":\n{details}" if details else "")
                )
            previous_timeout = self._socket.gettimeout()
            self._socket.settimeout(timeout)
            try:
                _send_packet(self._socket, payload, raw)
                response, _ = _receive_packet(self._socket)
            except (OSError, EOFError, ValueError) as exc:
                details = "\n".join(self._stderr_lines)
                self._communication_error = (
                    f"LeRobot worker communication failed: {exc}"
                    + (f"\n{details}" if details else "")
                )
                # A late response must never be mistaken for the next request's
                # acknowledgement after a timeout (there are no request IDs).
                raise LeRobotRecorderError(self._communication_error) from exc
            finally:
                self._socket.settimeout(previous_timeout)
            if not response.get("ok", False):
                raise LeRobotRecorderError(str(response.get("error", "Unknown worker error")))
            return response

    def close(self) -> None:
        try:
            self._socket.close()
        except OSError:
            pass
        if self._process.poll() is None:
            try:
                self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._process.terminate()
                try:
                    self._process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait(timeout=2.0)
        if self._process.stderr is not None:
            self._process.stderr.close()


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
        client = _WorkerClient(self.worker_python)
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
    # Imported only inside the dedicated LeRobot environment.
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    sock = socket.socket(fileno=fd)
    dataset: Any = None
    buffered_frames = 0
    saved_episodes = 0
    image_shape = (0, 0, 3)
    try:
        while True:
            try:
                request, raw = _receive_packet(sock)
            except EOFError:
                break
            operation = request.get("op")
            try:
                if operation == "init":
                    if dataset is not None:
                        raise RuntimeError("Dataset is already initialized")
                    root = Path(str(request["root"])).expanduser().resolve()
                    if root.exists() and any(root.iterdir()):
                        raise FileExistsError(f"Dataset directory is not empty: {root}")
                    fps = int(request["fps"])
                    height = int(request["height"])
                    width = int(request["width"])
                    if (height, width) != (224, 224):
                        raise ValueError("PI0.5 recording image size must be 224x224")
                    encoder = RGBEncoderConfig(
                        vcodec="h264",
                        pix_fmt="yuv420p",
                        crf=28,
                        preset="fast",
                        g=fps,
                    )
                    dataset = LeRobotDataset.create(
                        repo_id=str(request["repo_id"]),
                        root=root,
                        fps=fps,
                        robot_type="dobot_cr5_o6",
                        features=_dataset_features(height, width),
                        use_videos=True,
                        rgb_encoder=encoder,
                        streaming_encoding=True,
                        encoder_queue_maxsize=max(4, fps // 2),
                        encoder_threads=2,
                        batch_encoding_size=1,
                    )
                    image_shape = (height, width, 3)
                    _send_packet(sock, {"ok": True, "root": str(root)})
                elif operation == "add_frame":
                    if dataset is None:
                        raise RuntimeError("Dataset is not initialized")
                    expected_per_image = int(np.prod(image_shape))
                    expected = expected_per_image * 3
                    if len(raw) != expected:
                        raise ValueError(
                            f"3-stream RGB payload is {len(raw)} bytes, expected {expected}"
                        )
                    images = np.frombuffer(raw, dtype=np.uint8).reshape(
                        (3, *image_shape)
                    ).copy()
                    state = np.asarray(request["state"], dtype=np.float32)
                    action = np.asarray(request["action"], dtype=np.float32)
                    if state.shape != (18,) or action.shape != (12,):
                        raise ValueError(
                            f"Invalid state/action shapes: {state.shape}/{action.shape}"
                        )
                    dataset.add_frame(
                        {
                            "observation.images.base_0_rgb": images[0],
                            "observation.images.left_wrist_0_rgb": images[1],
                            "observation.images.right_wrist_0_rgb": images[2],
                            "observation.state": state,
                            "action": action,
                            "task": str(request["task"]),
                        }
                    )
                    buffered_frames += 1
                    _send_packet(sock, {"ok": True, "frames": buffered_frames})
                elif operation == "save_episode":
                    if dataset is None or buffered_frames <= 0:
                        raise RuntimeError("No Episode frames to save")
                    # The dataset's episode index remains the source of truth.
                    # Write diagnostics first so a sidecar write error cannot
                    # turn an already committed episode into a retryable save.
                    collection_dir = Path(dataset.root) / "meta" / "collection"
                    collection_dir.mkdir(parents=True, exist_ok=True)
                    collection = dict(request["collection"])
                    collection["episode_index"] = saved_episodes
                    (collection_dir / f"episode_{saved_episodes:06d}.json").write_text(
                        json.dumps(collection, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8",
                    )
                    dataset.save_episode()
                    saved_episodes += 1
                    buffered_frames = 0
                    _send_packet(sock, {"ok": True, "episodes": saved_episodes})
                elif operation == "clear_episode":
                    if dataset is not None and buffered_frames:
                        dataset.clear_episode_buffer()
                    buffered_frames = 0
                    _send_packet(sock, {"ok": True})
                elif operation == "finalize":
                    if dataset is not None:
                        if buffered_frames:
                            raise RuntimeError("Unsaved Episode; save or discard explicitly")
                        dataset.finalize()
                    _send_packet(sock, {"ok": True, "episodes": saved_episodes})
                    return 0
                else:
                    raise ValueError(f"Unknown worker operation: {operation}")
            except Exception as exc:
                _send_packet(
                    sock,
                    {"ok": False, "error": f"{type(exc).__name__}: {exc}"},
                )
    finally:
        if dataset is not None:
            try:
                dataset.finalize()
            except Exception:
                traceback.print_exc()
        sock.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker-fd", type=int, required=True)
    args = parser.parse_args()
    return _worker_main(args.worker_fd)


if __name__ == "__main__":
    raise SystemExit(main())

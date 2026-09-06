from __future__ import annotations

import numpy as np
import pytest

from gello_cr.recording.episode_recorder import LeRobotEpisodeRecorder
from gello_cr.recording.worker_client import WorkerClientError


class FakeClient:
    def __init__(self):
        self.requests = []
        self.closed = False

    def request(self, payload, raw=b"", timeout=30.0):
        self.requests.append((payload, raw, timeout))
        if payload["op"] == "finalize":
            return {"ok": True, "episodes": 0}
        if payload["op"] == "add_frame":
            return {"ok": True, "frames": 1}
        return {"ok": True}

    def close(self):
        self.closed = True


def _recorder(**kwargs):
    return LeRobotEpisodeRecorder(
        sample_provider=lambda: {},
        event_callback=lambda level, message: None,
        worker_python="/tmp/fake-python",
        repo_prefix="local/test",
        **kwargs,
    )


def test_constructor_preserves_recording_defaults() -> None:
    recorder = _recorder()
    assert recorder.fps == 20
    assert recorder.image_size == (224, 224)
    assert recorder.repo_prefix == "local/test"


def test_default_worker_script_points_to_root_recorder() -> None:
    recorder = _recorder()
    assert recorder.worker_script.name == "lerobot_recorder.py"
    assert recorder.worker_script.parent.name == "gello_CR"


def test_explicit_worker_script_is_supported(tmp_path) -> None:
    script = tmp_path / "custom_worker.py"
    script.write_text("pass\n", encoding="utf-8")
    recorder = _recorder(worker_script=script)
    assert recorder.worker_script == script.resolve()


def test_initial_snapshot_preserves_quality_contract() -> None:
    recorder = _recorder()
    snapshot = recorder.snapshot()
    assert snapshot["session_active"] is False
    assert snapshot["episode_active"] is False
    assert snapshot["buffered_frames"] == 0
    assert snapshot["saved_episodes"] == 0
    assert snapshot["quality"]["configured_fps"] == 20
    assert (
        snapshot["quality"]["timestamp_basis"]
        == "host_monotonic_receipt_not_hardware_sync"
    )


def test_start_episode_rejects_empty_task_before_worker_use() -> None:
    recorder = _recorder()
    with pytest.raises(ValueError, match="task text cannot be empty"):
        recorder.start_episode("   ", "/tmp/data")


def test_add_sample_uses_frame_preparation_and_updates_count() -> None:
    recorder = _recorder(image_size=(2, 2))
    client = FakeClient()
    recorder._client = client
    recorder._task = "task"

    image = np.zeros((2, 2, 3), dtype=np.uint8)
    recorder._add_sample(
        {
            "timestamp": 10.0,
            "observation_state": [0.0] * 18,
            "action": [0.0] * 12,
            "image_base_rgb": image,
            "image_wrist_rgb": image,
            "image_roi_rgb": image,
            "quality": {},
        }
    )

    assert recorder._buffered_frames == 1
    payload, raw, timeout = client.requests[0]
    assert payload["op"] == "add_frame"
    assert payload["task"] == "task"
    assert len(raw) == 3 * 2 * 2 * 3
    assert timeout == 15.0


def test_finalize_preserves_worker_request_and_close_behavior() -> None:
    recorder = _recorder()
    client = FakeClient()
    recorder._client = client
    recorder._session_root = "/tmp/session"

    result = recorder.finalize()

    assert result == "/tmp/session"
    assert client.requests == [({"op": "finalize"}, b"", 180.0)]
    assert client.closed
    assert recorder._client is None


def test_preserve_failed_session_requires_existing_error() -> None:
    recorder = _recorder()
    with pytest.raises(WorkerClientError, match="会话没有错误"):
        recorder.preserve_failed_session()

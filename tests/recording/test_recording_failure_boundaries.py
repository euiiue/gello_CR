"""Regression coverage for durable capture and pending episode boundaries."""

import time
from pathlib import Path

import h5py
import pytest

from gello_cr.recording.errors import RecordingError
from gello_cr.recording.journal import FrameJournal, read_frames
from gello_cr.recording.session import RecordingSession
from tests.recording.test_recovery import make_manager, source_frame
from tests.recording.test_session import Source, Writer


def test_configured_hd_frame_survives_journal_roundtrip(tmp_path):
    journal = FrameJournal(tmp_path / "frames.bin", task="HD", image_size=(1080, 1920), mode="joint")
    try:
        journal.append(source_frame())
    finally:
        journal.close()
    frames = list(read_frames(journal.path))
    assert len(frames) == 1
    assert all(image.shape == (1080, 1920, 3) for image in frames[0].images.values())
    assert frames[0].action == source_frame().action


def test_backend_rejected_frame_can_be_recovered_as_failure(tmp_path, monkeypatch):
    manager = make_manager(tmp_path)
    manager.create_dataset("capture", tmp_path / "train")
    backend = manager._segment_writer.writer
    original_add = backend.add_frame
    attempts = 0

    def reject_second(frame):
        nonlocal attempts
        attempts += 1
        if attempts == 2:
            raise RuntimeError("backend rejected the second frame")
        return original_add(frame)

    monkeypatch.setattr(backend, "add_frame", reject_second)
    manager.start_episode("capture", tmp_path / "train")
    manager.session._episode_thread.join(timeout=3)
    assert not manager.snapshot()["episode_active"]
    assert manager.snapshot()["buffered_frames"] == 1
    batch_id = manager.snapshot()["batch_id"]
    try:
        with pytest.raises(RecordingError, match="partial|recover|恢复"):
            manager.save_episode("failure")
    finally:
        manager.suspend("backend rejection")
    recovered = make_manager(tmp_path)
    entry = recovered.recovery.scan()[0]
    assert entry["episodes"] == 0
    assert entry["partial_episodes"] == [{"episode_index": 0, "frame_count": 2}]
    recovered.recovery.resolve_partial(batch_id, 0, "failure")
    final = Path(recovered.recovery.finalize(batch_id))
    with h5py.File(final / "dataset.hdf5", "r") as file:
        episode = file["episodes/000000"]
        assert episode["metadata"].attrs["outcome"] == "failure"
        assert len(episode["timestamp"]) == 2


def test_first_frame_failure_remains_pending_until_explicit_discard(tmp_path):
    writer = Writer()

    def reject(frame):
        raise RuntimeError("first frame rejected")

    writer.add_frame = reject
    session = RecordingSession(
        Source(), lambda: writer, event_callback=lambda *args: None, image_size=(2, 2)
    )
    session.start_episode("capture", tmp_path)
    session._episode_thread.join(timeout=1)
    assert session.snapshot()["buffered_frames"] == 0
    assert session.snapshot()["episode_pending"]
    with pytest.raises(RecordingError, match="previous|未处理"):
        session.start_episode("capture", tmp_path)
    with pytest.raises(RecordingError, match="未处理"):
        session.finalize()
    session.discard_episode()
    assert not session.snapshot()["episode_pending"]
    session.finalize()


def test_joint_capture_keeps_last_accepted_action_after_owner_release(tmp_path):
    from gello_cr.app.config_store import TeleopConfigStore
    from gello_cr.app.runtime import TeleopRuntime
    from gello_cr.motion import JointTarget, MotionSource
    from tests.fakes.runtime import FakeO6, FakeRoArm, FakeRobot

    store = TeleopConfigStore(tmp_path / "config.json")
    store.data["dataset"]["recording_mode"] = "joint"
    runtime = TeleopRuntime(store, FakeRoArm(), FakeO6())
    runtime.attach_robot(FakeRobot())
    target = JointTarget((0.01,) * 6, time.monotonic())
    try:
        lease = runtime.motion.acquire(MotionSource.REPLAY)
        lease.servo_joint(target)
        moving = runtime.dataset_sample()
        lease.release()
        completed = runtime.dataset_sample()
        assert completed["action"][:6] == pytest.approx(target.positions_rad)
        assert completed["action"] == moving["action"]
        assert completed["robot_joint_command"] == pytest.approx(target.positions_rad)
        assert completed["metadata"]["motion_source"] is None
    finally:
        runtime.shutdown(close_devices=False)

@pytest.mark.parametrize("state", ["fault", "closed", "motion_error", "new_owner"])
def test_completed_action_is_not_reused_for_fault_or_new_owner(tmp_path, state):
    from gello_cr.app.config_store import TeleopConfigStore
    from gello_cr.app.runtime import TeleopRuntime
    from gello_cr.motion import JointTarget, MotionSource
    from tests.fakes.runtime import FakeO6, FakeRoArm, FakeRobot

    store = TeleopConfigStore(tmp_path / "config.json")
    store.data["dataset"]["recording_mode"] = "joint"
    runtime = TeleopRuntime(store, FakeRoArm(), FakeO6())
    runtime.attach_robot(FakeRobot())
    try:
        lease = runtime.motion.acquire(MotionSource.REPLAY)
        lease.servo_joint(JointTarget((0.01,) * 6, time.monotonic()))
        lease.release(error="stop failed" if state == "motion_error" else None)
        if state == "new_owner":
            runtime.motion.acquire(MotionSource.POLICY)
        elif state in ("fault", "closed"):
            runtime._set_state(state)
        with pytest.raises(RuntimeError):
            runtime.dataset_sample()
    finally:
        runtime.shutdown(close_devices=False)


def test_completed_tcp_action_is_not_reused_after_motion_error(tmp_path):
    from gello_cr.app.config_store import TeleopConfigStore
    from gello_cr.app.runtime import TeleopRuntime
    from gello_cr.motion import JointTarget, MotionSource
    from tests.fakes.runtime import FakeO6, FakeRoArm, FakeRobot

    store = TeleopConfigStore(tmp_path / "config.json")
    store.data["dataset"]["recording_mode"] = "tcp"
    runtime = TeleopRuntime(store, FakeRoArm(), FakeO6())
    runtime.attach_robot(FakeRobot())
    try:
        lease = runtime.motion.acquire(MotionSource.REPLAY)
        lease.servo_joint(JointTarget((0.01,) * 6, time.monotonic()))
        lease.release(error="stop failed")
        with pytest.raises(RuntimeError, match="运动|motion"):
            runtime.dataset_sample()
    finally:
        runtime.shutdown(close_devices=False)

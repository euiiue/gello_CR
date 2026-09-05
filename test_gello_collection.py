"""Offline regression tests. No camera, serial port or robot connection is opened."""

import copy
import json
import os
import tempfile
import threading
import time
import unittest
from collections import deque
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import TEST_INEXBOT as ui
from lerobot_recorder import LeRobotEpisodeRecorder, LeRobotRecorderError, _WorkerClient
from teleop_runtime import GelloFeedback, NrcRobotAdapter, TeleopConfigStore, TeleopEngine
from test_teleop_runtime import FakeGello, FakeNrcServoApi, FakeO6, FakeRobot


def sample(timestamp=None, **quality):
    image = np.zeros((8, 8, 3), dtype=np.uint8)
    return {
        "timestamp": time.monotonic() if timestamp is None else timestamp,
        "observation_state": [0.0] * 18,
        "action": [0.0] * 12,
        "image_base_rgb": image,
        "image_wrist_rgb": image,
        "image_roi_rgb": image,
        "quality": quality,
    }


class FakeWorker:
    def __init__(self):
        self.requests = []
        self.frames = 0
        self.episodes = 0
        self.first_frame = threading.Event()
        self.closed = False
        self.on_init = lambda: None

    def request(self, payload, raw=b"", timeout=30.0):
        self.requests.append(copy.deepcopy(payload))
        if payload["op"] == "init":
            self.on_init()
            return {"ok": True, "root": payload["root"]}
        if payload["op"] == "add_frame":
            self.frames += 1
            self.first_frame.set()
        if payload["op"] == "save_episode":
            self.episodes += 1
            self.frames = 0
        if payload["op"] == "clear_episode":
            self.frames = 0
        return {"ok": True, "frames": self.frames, "episodes": self.episodes}

    def close(self):
        self.closed = True


class RecorderLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.worker = FakeWorker()
        self.worker_patch = patch("lerobot_recorder._WorkerClient", return_value=self.worker)
        self.worker_patch.start()
        self.events = []
        self.recorder = LeRobotEpisodeRecorder(
            sample, lambda *event: self.events.append(event), "/unused", "test/gello", fps=20
        )

    def tearDown(self):
        self.recorder._stop_sampling()
        self.worker_patch.stop()
        self.temp.cleanup()

    def start_and_stop(self, **kwargs):
        self.recorder.start_episode("pick object", self.temp.name, **kwargs)
        self.assertTrue(self.worker.first_frame.wait(1.0))
        self.recorder.stop_episode()

    def test_worker_startup_frame_is_resampled(self):
        value = [1.0]
        def provider():
            frame = sample()
            frame["observation_state"][0] = value[0]
            return frame
        self.recorder.sample_provider = provider
        self.worker.on_init = lambda: value.__setitem__(0, 7.0)
        self.start_and_stop()
        added = [r for r in self.worker.requests if r["op"] == "add_frame"]
        self.assertEqual(added[0]["state"][0], 7.0)

    def test_stop_retains_buffer_and_freezes_elapsed_time(self):
        self.start_and_stop()
        before = self.recorder.snapshot()
        self.assertFalse(before["episode_active"])
        self.assertGreater(before["buffered_frames"], 0)
        with patch("lerobot_recorder.time.monotonic", return_value=time.monotonic() + 100):
            self.assertEqual(self.recorder.snapshot()["duration"], before["duration"])

    def test_multiple_episodes_are_saved_in_one_dataset_session(self):
        self.start_and_stop()
        self.recorder.save_episode("success", "episode 1")
        self.start_and_stop()
        self.recorder.save_episode("success", "episode 2")
        snapshot = self.recorder.snapshot()
        self.assertEqual(snapshot["saved_episodes"], 2)
        self.assertTrue(snapshot["session_active"])
        self.assertEqual(
            [request["op"] for request in self.worker.requests].count("init"), 1
        )
        self.assertEqual(
            [request["op"] for request in self.worker.requests].count("save_episode"), 2
        )

    def test_finalize_never_silently_discards_pending_frames(self):
        self.start_and_stop()
        with self.assertRaisesRegex(LeRobotRecorderError, "未处理"):
            self.recorder.finalize()
        self.assertNotIn("clear_episode", [r["op"] for r in self.worker.requests])
        self.assertGreater(self.recorder.snapshot()["buffered_frames"], 0)

    def test_fault_retains_buffer_for_explicit_discard(self):
        calls = [0]
        def provider():
            calls[0] += 1
            if calls[0] > 2:
                raise RuntimeError("camera expired")
            return sample()
        self.recorder.sample_provider = provider
        self.recorder.start_episode("pick object", self.temp.name)
        self.recorder._episode_thread.join(1.0)
        snapshot = self.recorder.snapshot()
        self.assertIn("camera expired", snapshot["error"])
        self.assertEqual(snapshot["buffered_frames"], 1)
        with self.assertRaisesRegex(LeRobotRecorderError, "failure or discard"):
            self.recorder.save_episode()
        self.recorder.discard_episode()
        self.assertEqual(self.recorder.snapshot()["buffered_frames"], 0)
        self.assertFalse(self.recorder.snapshot()["error"])

    def test_fault_episode_can_be_saved_as_failure_with_existing_frames(self):
        calls = [0]

        def provider():
            calls[0] += 1
            if calls[0] > 2:
                raise RuntimeError("gello jitter")
            return sample()

        self.recorder.sample_provider = provider
        self.recorder.start_episode("pick object", self.temp.name)
        self.recorder._episode_thread.join(1.0)
        self.recorder.save_episode("failure", "保留抖动前有效片段")
        saved = next(r for r in self.worker.requests if r["op"] == "save_episode")
        self.assertEqual(saved["collection"]["outcome"], "failure")
        self.assertEqual(self.recorder.snapshot()["saved_episodes"], 1)

    def test_outcome_notes_context_and_quality_accompany_save(self):
        self.start_and_stop(metadata={"motion_mode": "servoj"})
        self.recorder.save_episode("failure", "object slipped")
        saved = next(r for r in self.worker.requests if r["op"] == "save_episode")
        self.assertEqual(saved["collection"]["outcome"], "failure")
        self.assertEqual(saved["collection"]["notes"], "object slipped")
        self.assertEqual(saved["collection"]["context"]["motion_mode"], "servoj")
        self.assertGreater(saved["collection"]["quality"]["frames"], 0)
        self.assertEqual(self.recorder.snapshot()["saved_episodes"], 1)
        self.assertFalse(self.recorder.snapshot()["episode_active"])
        self.assertEqual(self.recorder.snapshot()["buffered_frames"], 0)
        self.recorder.finalize()
        self.assertTrue(self.worker.closed)

    def test_slow_sampling_and_reused_images_are_not_hidden(self):
        self.recorder._client = self.worker
        self.recorder._task = "test"
        self.recorder._add_sample(sample(10.0, wrist_timestamp=9.99, base_timestamp=9.98, camera_skew_s=0.01))
        self.recorder._add_sample(sample(10.15, wrist_timestamp=9.99, base_timestamp=9.98, camera_skew_s=0.02))
        quality = self.recorder.snapshot()["quality"]
        self.assertEqual(quality["late_frames"], 1)
        self.assertAlmostEqual(quality["effective_fps"], 1 / 0.15)
        self.assertEqual(quality["reused_camera_frames"], {"wrist": 1, "base": 1})
        self.assertEqual(quality["max"]["camera_skew_s"], 0.02)
        self.assertTrue(quality["needs_review"])

    def test_tracking_error_marker_does_not_truncate_30_second_episode(self):
        self.recorder._client = self.worker
        self.recorder._task = "test"
        for frame_index in range(600):
            self.recorder._add_sample(
                sample(
                    10.0 + frame_index / 20.0,
                    tracking_error_deg=8.0,
                    tracking_error_limit_deg=5.0,
                    tracking_error_exceeded=1.0,
                )
            )
        snapshot = self.recorder.snapshot()
        self.assertEqual(snapshot["buffered_frames"], 600)
        self.assertAlmostEqual(snapshot["quality"]["effective_fps"], 20.0)
        self.assertEqual(snapshot["quality"]["max"]["tracking_error_deg"], 8.0)
        self.assertTrue(snapshot["quality"]["needs_review"])

    def test_save_error_prevents_duplicate_retry(self):
        self.start_and_stop()
        self.worker.request = Mock(side_effect=TimeoutError("save timeout"))
        with self.assertRaises(TimeoutError):
            self.recorder.save_episode()
        with self.assertRaisesRegex(LeRobotRecorderError, "discard"):
            self.recorder.save_episode()
        self.assertEqual(self.worker.request.call_count, 1)

    def test_event_callback_errors_are_not_swallowed(self):
        self.recorder.event_callback = Mock(side_effect=RuntimeError("logger error"))
        with self.assertRaisesRegex(RuntimeError, "logger error"):
            self.recorder._event("info", "message")

    def test_failed_session_recovery_does_not_clear_disk_data(self):
        self.start_and_stop()
        self.recorder._last_error = "worker connection failed"
        self.recorder.preserve_failed_session()
        self.assertTrue(self.worker.closed)
        self.assertNotIn("clear_episode", [r["op"] for r in self.worker.requests])
        self.assertFalse(self.recorder.snapshot()["session_active"])
        self.assertIn("磁盘文件未删除", self.events[-1][1])

    def test_finalize_error_remains_visible_for_recovery(self):
        self.recorder._client = self.worker
        self.worker.request = Mock(side_effect=TimeoutError("finalize timeout"))
        with self.assertRaises(TimeoutError):
            self.recorder.finalize()
        self.assertIn("finalize timeout", self.recorder.snapshot()["error"])
        self.assertTrue(self.recorder.snapshot()["session_active"])

    def test_packet_timeout_poisons_connection_instead_of_consuming_late_ack(self):
        client = _WorkerClient.__new__(_WorkerClient)
        client._socket = Mock()
        client._process = Mock()
        client._process.poll.return_value = None
        client._request_lock = threading.Lock()
        client._stderr_lines = deque()
        client._communication_error = ""
        with patch("lerobot_recorder._send_packet"), patch(
            "lerobot_recorder._receive_packet", side_effect=TimeoutError("late ack")
        ) as receive:
            for _ in range(2):
                with self.assertRaisesRegex(LeRobotRecorderError, "late ack"):
                    client.request({"op": "save_episode"})
            self.assertEqual(receive.call_count, 1)


class GelloTelemetryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.store = TeleopConfigStore(Path(self.temp.name) / "teleop.json")
        self.store.data["gello"]["control_mode"] = "joint"
        self.store.data["gello"]["startup_settle_s"] = 0.0
        self.master = FakeGello()
        self.hand = FakeO6()
        self.hand.last_sent_target = self.hand.position
        self.robot = FakeRobot()
        self.engine = TeleopEngine(self.store, self.master, self.hand)
        self.engine.attach_robot(self.robot)

    def tearDown(self):
        self.engine.shutdown()
        self.temp.cleanup()

    def ready_telemetry(self):
        now = time.monotonic()
        self.engine._set_state("following")
        self.engine._gello_telemetry = {
            "actual_deg": (0.0,) * 6, "target_deg": (0.0,) * 6,
            "target_full_deg": (0.0,) * 7, "feedback_at": now,
            "command_at": now, "sent_count": 2, "send_hz": 100.0,
            "tracking_error_deg": 0.0,
        }

    def run_one_follow_cycle(self, master_origin, slave_origin):
        self.engine._set_state("following")
        self.engine._follow_stop.clear()
        with patch.object(
            self.robot, "send_servoj",
            side_effect=lambda _target: self.engine._follow_stop.set() or True,
        ) as submit, patch.object(
            self.engine._follow_stop, "wait",
            side_effect=lambda _timeout: self.engine._follow_stop.set(),
        ):
            self.engine._gello_follow_loop(master_origin, slave_origin, self.engine._stop_generation)
        return submit

    def test_short_segments_preserve_direction_extra_axis_and_off_mode(self):
        self.robot.motion_mode = "movej"
        self.robot.joints = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 7.0]
        origin = self.master.value
        self.master.value = GelloFeedback(
            time.monotonic(), (np.deg2rad(10.0), np.deg2rad(5.0), 0.0, 0.0, 0.0, 0.0, 0.5)
        )
        for enabled, first_two in ((False, [20.0, 25.0]), (True, [12.0, 21.0])):
            with self.subTest(enabled=enabled):
                self.store.data["robot"]["movej_low_latency"] = enabled
                submit = self.run_one_follow_cycle(origin, list(self.robot.joints))
                np.testing.assert_allclose(
                    submit.call_args.args[0], first_two + [30.0, 40.0, 50.0, 60.0, 7.0]
                )
                telemetry = self.engine.snapshot()["gello_telemetry"]
                np.testing.assert_allclose(telemetry["desired_deg"][:2], [20.0, 25.0])
                self.assertAlmostEqual(telemetry["desired_tracking_error_deg"], 10.0)
                if enabled:
                    self.assertAlmostEqual(telemetry["latency"]["segment_delta_deg"], 2.0)
                    self.assertAlmostEqual(telemetry["tracking_error_deg"], 2.0)

    def test_short_segment_keeps_small_target_exact(self):
        self.robot.motion_mode = "movej"
        self.store.data["robot"]["movej_low_latency"] = True
        origin = self.master.value
        self.master.value = GelloFeedback(
            time.monotonic(), (np.deg2rad(0.1), np.deg2rad(-0.2), 0.0, 0.0, 0.0, 0.0, 0.0)
        )
        submit = self.run_one_follow_cycle(origin, [0.0] * 7)
        np.testing.assert_allclose(submit.call_args.args[0], [0.1, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_joint_scale_increases_j1_j6_relative_mapping_only(self):
        self.robot.motion_mode = "movej"
        self.store.data["gello"]["joint_scale"] = 1.1
        origin = self.master.value
        self.master.value = GelloFeedback(
            time.monotonic(), (np.deg2rad(10.0), np.deg2rad(-5.0), 0.0, 0.0, 0.0, 0.0, 0.7)
        )
        submit = self.run_one_follow_cycle(origin, [0.0] * 6 + [9.0])
        np.testing.assert_allclose(submit.call_args.args[0], [11.0, -5.5, 0.0, 0.0, 0.0, 0.0, 9.0])
        telemetry = self.engine.snapshot()["gello_telemetry"]
        np.testing.assert_allclose(telemetry["desired_deg"][:2], [11.0, -5.5])

    def test_joint_scale_also_applies_to_speed_protection(self):
        self.robot.motion_mode = "movej"
        self.store.data["gello"].update({"joint_scale": 1.1, "feedback_timeout_s": 1e10})
        self.store.data["robot"].update({
            "safety_max_command_speed_rad_s": 0.18, "safety_speed_violation_cycles": 1,
        })
        origin = GelloFeedback(0.0, (0.0,) * 7)
        self.master.value = GelloFeedback(1.0, (np.deg2rad(10.0),) + (0.0,) * 6)
        submit = self.run_one_follow_cycle(origin, [0.0] * 7)
        submit.assert_not_called()
        self.assertIn("leader speed", self.engine.last_error)

    def test_short_segment_does_not_hide_full_desired_safety_error(self):
        self.robot.motion_mode = "movej"
        self.store.data["robot"].update({
            "movej_low_latency": True, "safety_max_tracking_error_rad": np.deg2rad(5.0)
        })
        origin = self.master.value
        self.master.value = GelloFeedback(time.monotonic(), (np.deg2rad(10.0),) + (0.0,) * 6)
        submit = self.run_one_follow_cycle(origin, [0.0] * 7)
        submit.assert_not_called()
        self.assertEqual(self.engine.state, "fault")
        self.assertIn("完整期望目标跟踪误差", self.engine.last_error)

    def test_short_segment_refreshes_leader_after_nrc_read(self):
        self.robot.motion_mode = "movej"
        self.store.data["robot"]["movej_low_latency"] = True
        origin = self.master.value
        self.master.value = GelloFeedback(time.monotonic(), (np.deg2rad(4.0),) + (0.0,) * 6)

        def read_feedback():
            self.master.value = GelloFeedback(time.monotonic(), (np.deg2rad(-4.0),) + (0.0,) * 6)
            return [0.0] * 7

        with patch.object(self.robot, "joint_position", side_effect=read_feedback):
            submit = self.run_one_follow_cycle(origin, [0.0] * 7)
        self.assertAlmostEqual(submit.call_args.args[0][0], -2.0)
        self.assertAlmostEqual(self.engine.snapshot()["gello_telemetry"]["desired_deg"][0], -4.0)

    def test_short_segment_uses_feedback_after_idle_confirmation(self):
        self.robot.motion_mode = "movej"
        self.store.data["robot"]["movej_low_latency"] = True
        origin = self.master.value
        self.master.value = GelloFeedback(time.monotonic(), (np.deg2rad(10.0),) + (0.0,) * 6)
        self.robot.controller_job_warning = "code=4098"

        def confirm_idle():
            self.robot.joints[0] = 3.0
            self.robot.controller_job_warning = ""
            return True

        with patch.object(self.robot, "clear_movej_busy_if_stopped", side_effect=confirm_idle):
            submit = self.run_one_follow_cycle(origin, [0.0] * 7)
        self.assertAlmostEqual(submit.call_args.args[0][0], 5.0)

    def test_short_segment_rechecks_expired_leader_after_nrc_read(self):
        self.robot.motion_mode = "movej"
        self.store.data["robot"]["movej_low_latency"] = True
        origin = self.master.value

        def read_feedback():
            self.master.value = GelloFeedback(time.monotonic() - 1.0, (0.0,) * 7)
            return [0.0] * 7

        with patch.object(self.robot, "joint_position", side_effect=read_feedback):
            submit = self.run_one_follow_cycle(origin, [0.0] * 7)
        submit.assert_not_called()
        self.assertIn("GELLO 反馈掉线/超时", self.engine.last_error)

    def test_busy_cycle_has_diagnostics_without_submitting(self):
        self.robot.motion_mode = "movej"
        self.robot.controller_job_warning = "code=4098"
        self.store.data["robot"]["movej_low_latency"] = True
        with patch.object(self.robot, "clear_movej_busy_if_stopped", return_value=False):
            submit = self.run_one_follow_cycle(self.master.value, [0.0] * 7)
        submit.assert_not_called()
        telemetry = self.engine.snapshot()["gello_telemetry"]
        self.assertEqual(telemetry["sent_count"], 0)
        self.assertEqual(telemetry["latency"]["phase"], "busy")
        self.assertGreaterEqual(telemetry["latency"]["feedback_read_ms"], 0.0)

    def test_static_target_is_submitted_once_after_follower_arrives(self):
        self.robot.motion_mode = "movej"
        self.robot.movej_period_s = 0.01
        waits = [0]

        def finish_after_three_cycles(_timeout):
            waits[0] += 1
            if waits[0] >= 3:
                self.engine._follow_stop.set()

        self.engine._set_state("following")
        with patch.object(
            self.robot, "send_servoj", wraps=self.robot.send_servoj
        ) as submit, patch.object(
            self.engine._follow_stop, "wait", side_effect=finish_after_three_cycles
        ):
            self.engine._gello_follow_loop(
                self.master.value, [0.0] * 7, self.engine._stop_generation
            )
        submit.assert_called_once()
        telemetry = self.engine.snapshot()["gello_telemetry"]
        self.assertEqual(telemetry["sent_count"], 1)
        self.assertEqual(telemetry["latency"]["phase"], "duplicate_hold")
        self.assertLessEqual(
            telemetry["pending_target_deg"],
            self.store.data["robot"]["movej_duplicate_deadband_deg"],
        )

    def test_same_target_retries_until_follower_arrives(self):
        self.robot.motion_mode = "movej"
        self.robot.movej_period_s = 0.01
        origin = self.master.value
        self.master.value = GelloFeedback(
            time.monotonic(), (np.deg2rad(0.2),) + (0.0,) * 6
        )
        waits = [0]

        def finish_after_three_cycles(_timeout):
            waits[0] += 1
            if waits[0] >= 3:
                self.engine._follow_stop.set()

        self.engine._set_state("following")
        with patch.object(self.robot, "send_servoj", return_value=True) as submit, patch.object(
            self.engine._follow_stop, "wait", side_effect=finish_after_three_cycles
        ):
            self.engine._gello_follow_loop(origin, [0.0] * 7, self.engine._stop_generation)
        self.assertGreaterEqual(submit.call_count, 2)
        self.assertGreater(
            self.engine.snapshot()["gello_telemetry"]["tracking_error_deg"],
            self.store.data["robot"]["movej_duplicate_deadband_deg"],
        )

    def test_busy_timing_log_uses_observed_times_not_log_line_frequency(self):
        self.robot.motion_mode = "movej"
        self.robot.movej_period_s = 0.1
        self.robot.controller_job_warning = "code=4098"
        self.store.data["robot"]["movej_low_latency"] = True
        clock = [0.0]

        def check_busy():
            clock[0] += 0.01
            return False

        def read_feedback():
            clock[0] += 0.02
            return [0.0] * 7

        def wait_cycle(duration):
            clock[0] += duration
            if clock[0] >= 1.15:
                self.engine._follow_stop.set()

        self.engine._set_state("following")
        with patch("teleop_runtime.time.monotonic", side_effect=lambda: clock[0]), patch.object(
            self.master, "latest", side_effect=lambda: GelloFeedback(clock[0], (0.0,) * 7)
        ), patch.object(self.robot, "clear_movej_busy_if_stopped", side_effect=check_busy), patch.object(
            self.robot, "joint_position", side_effect=read_feedback
        ), patch.object(self.engine._follow_stop, "wait", side_effect=wait_cycle):
            self.engine._gello_follow_loop(GelloFeedback(0.0, (0.0,) * 7), [0.0] * 7, self.engine._stop_generation)
        latency = self.engine.snapshot()["gello_telemetry"]["latency"]
        self.assertAlmostEqual(latency["feedback_read_ms"], 20.0)
        self.assertAlmostEqual(latency["busy_check_ms"], 10.0)
        self.assertEqual(latency["recent_send_hz"], 0.0)
        self.assertGreater(latency["busy_wait_ms"], 1000.0)
        messages = []
        while not self.engine.events.empty():
            messages.append(self.engine.events.get_nowait()[1])
        self.assertTrue(any("MoveJ 软件时序" in message and "SDK_submit=0.0Hz" in message for message in messages))

    def test_short_segment_collection_marks_full_desired_gap_for_review(self):
        self.ready_telemetry()
        self.engine._gello_telemetry.update({"low_latency": True, "desired_tracking_error_deg": 8.0})
        frame = self.engine.dataset_sample()
        self.assertEqual(frame["quality"]["desired_tracking_error_exceeded"], 1.0)
        self.assertEqual(frame["quality"]["desired_tracking_error_deg"], 8.0)

    def test_short_segment_collection_records_submitted_target_not_desired(self):
        self.ready_telemetry()
        submitted = (0.0,) * 5 + (1.0, 0.0)
        self.engine._gello_telemetry.update({
            "low_latency": True, "target_full_deg": submitted,
            "desired_deg": (0.0,) * 5 + (3.0,), "desired_tracking_error_deg": 3.0,
        })
        with patch.object(self.robot, "forward_kinematics", wraps=self.robot.forward_kinematics) as fk:
            frame = self.engine.dataset_sample()
        fk.assert_called_once_with(submitted)
        self.assertEqual(len(frame["observation_state"]), 18)
        self.assertEqual(len(frame["action"]), 12)
        self.assertAlmostEqual(frame["action"][3], np.deg2rad(1.0))
        self.assertEqual(frame["quality"]["desired_tracking_error_deg"], 3.0)

    def test_static_hold_uses_fresh_validated_time_but_submitted_action(self):
        self.ready_telemetry()
        submitted = (0.0,) * 5 + (1.0, 0.0)
        self.engine._gello_telemetry.update({
            "target_full_deg": submitted,
            "command_at": time.monotonic() - 1.0,
            "validated_at": time.monotonic(),
            "pending_target_deg": 0.0,
        })
        with patch.object(self.robot, "forward_kinematics", wraps=self.robot.forward_kinematics) as fk:
            frame = self.engine.dataset_sample()
        fk.assert_called_once_with(submitted)
        self.assertAlmostEqual(frame["action"][3], np.deg2rad(1.0))
        self.assertGreater(frame["quality"]["sdk_submit_age_s"], 0.5)
        self.assertLess(frame["quality"]["validated_target_age_s"], 0.5)

    def test_pending_unsent_target_is_marked_stale_without_interrupting_collection(self):
        self.ready_telemetry()
        self.engine._gello_telemetry.update({
            "command_at": time.monotonic() - 1.0,
            "validated_at": time.monotonic(),
            "pending_target_deg": 0.2,
        })
        frame = self.engine.dataset_sample()
        self.assertEqual(frame["quality"]["action_stale"], 1.0)

    def test_invalid_short_segment_config_is_rejected(self):
        for value in (0.0, -1.0, 6.0, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.store.update_runtime({"robot": {"movej_max_segment_deg": value}})
        with self.assertRaises(ValueError):
            self.store.update_runtime({"robot": {"movej_low_latency": "true"}})
        for value in (-0.01, 1.01, float("nan"), float("inf")):
            with self.subTest(duplicate_deadband=value), self.assertRaises(ValueError):
                self.store.update_runtime({"robot": {"movej_duplicate_deadband_deg": value}})

    def test_invalid_joint_scale_is_rejected(self):
        for value in (0.49, 1.51, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.store.update_runtime({"gello": {"joint_scale": value}})

    def test_invalid_locked_joint_list_is_rejected(self):
        for value in ([0], [7], [2, 2], [True], "5"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.store.update_runtime({"gello": {"locked_joints": value}})

    def test_control_loop_does_not_query_fk_at_servoj_rate(self):
        with patch.object(self.robot, "forward_kinematics", wraps=self.robot.forward_kinematics) as fk:
            self.engine.start_follow()
            deadline = time.monotonic() + 0.5
            while self.engine.snapshot()["gello_telemetry"] is None and time.monotonic() < deadline:
                time.sleep(0.005)
            fk.assert_not_called()
            frame = self.engine.dataset_sample()
            self.assertEqual(fk.call_count, 1)
            self.assertEqual(len(frame["observation_state"]), 18)
            self.assertEqual(len(frame["action"]), 12)

    def test_unsent_target_falls_back_to_current_state_for_collection(self):
        self.robot.send_servoj = Mock(return_value=False)
        self.engine.start_follow()
        deadline = time.monotonic() + 0.5
        while self.engine.snapshot()["gello_telemetry"] is None and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(self.engine.state, "following")
        frame = self.engine.dataset_sample()
        self.assertEqual(frame["quality"]["gello_telemetry_missing"], 1.0)

    def test_stale_action_is_marked_without_interrupting_collection(self):
        self.ready_telemetry()
        self.engine._gello_telemetry["command_at"] -= 1
        frame = self.engine.dataset_sample()
        self.assertEqual(frame["quality"]["action_stale"], 1.0)

    def test_tracking_quality_error_is_recorded_without_stopping_episode(self):
        self.ready_telemetry()
        self.engine._gello_telemetry["tracking_error_deg"] = 8.0
        frame = self.engine.dataset_sample()
        self.assertEqual(frame["quality"]["tracking_error_deg"], 8.0)
        self.assertEqual(frame["quality"]["tracking_error_limit_deg"], 5.0)
        self.assertEqual(frame["quality"]["tracking_error_exceeded"], 1.0)
        self.assertEqual(self.engine.state, "following")

    def test_idle_dataset_sample_uses_current_state_and_zero_motion_action(self):
        frame = self.engine.dataset_sample()
        self.assertEqual(len(frame["observation_state"]), 18)
        self.assertEqual(len(frame["action"]), 12)
        self.assertTrue(all(abs(value) < 1e-9 for value in frame["action"][:6]))

    def test_o6_manual_actions_cannot_compete_with_j7(self):
        self.ready_telemetry()
        with self.assertRaisesRegex(RuntimeError, "J7 独占"):
            self.engine.execute_o6_action("张开手")
        self.assertEqual(self.hand.targets, [])

    def test_gello_start_refuses_o6_action_in_progress(self):
        self.engine._o6_action_active = True
        with self.assertRaisesRegex(RuntimeError, "O6 单独动作"):
            self.engine.start_follow()
        self.assertFalse(self.robot.servoj_open)

    def test_movej_gello_start_does_not_open_servoj(self):
        gello = FakeGello()
        self.master.connected = False
        self.engine.set_master(gello)
        self.robot.motion_mode = "movej"
        self.store.data["gello"]["control_mode"] = "joint"
        self.store.data["gello"]["startup_settle_s"] = 0.0
        self.store.data["robot"]["safety_max_command_speed_rad_s"] = 100.0
        self.store.data["robot"]["safety_max_tracking_error_rad"] = 10.0
        self.engine.start_follow()
        deadline = time.monotonic() + 0.8
        while not self.robot.servoj_targets and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(self.robot.servoj_targets)
        self.assertEqual(self.robot.servoj_events.count("open"), 0)
        self.engine.stop_follow("test")

    def test_servoj_follow_keeps_sending_even_when_target_is_static(self):
        self.robot.motion_mode = "servoj"
        self.store.data["gello"]["feedback_hz"] = 100.0
        waits = [0]

        def finish_after_three_cycles(_timeout):
            waits[0] += 1
            if waits[0] >= 3:
                self.engine._follow_stop.set()

        self.engine._set_state("following")
        with patch.object(
            self.robot, "send_servoj", wraps=self.robot.send_servoj
        ) as submit, patch.object(
            self.engine._follow_stop, "wait", side_effect=finish_after_three_cycles
        ):
            self.engine._gello_follow_loop(
                self.master.value, [0.0] * 7, self.engine._stop_generation
            )
        self.assertEqual(submit.call_count, 3)
        self.assertEqual(
            self.engine.snapshot()["gello_telemetry"]["latency"]["phase"],
            "submitted",
        )

    def test_gello_j7_selects_open_or_configured_grasp_action(self):
        self.robot.motion_mode = "servoj"
        self.store.data["gello"]["feedback_hz"] = 100.0
        waits = [0]
        open_feedback = GelloFeedback(
            time.monotonic(), (0.0,) * 6 + (0.0,)
        )
        grasp_feedback = GelloFeedback(
            time.monotonic(), (0.0,) * 6 + (1.0,)
        )
        self.master.value = open_feedback

        def change_to_grasp_then_stop(_timeout):
            waits[0] += 1
            if waits[0] == 1:
                self.master.value = grasp_feedback
            elif waits[0] == 2:
                self.engine._follow_stop.set()

        self.engine._set_state("following")
        with patch.object(
            self.engine._follow_stop, "wait", side_effect=change_to_grasp_then_stop
        ):
            self.engine._gello_follow_loop(
                open_feedback, [0.0] * 7, self.engine._stop_generation
            )
        self.assertEqual(
            self.hand.targets[:2],
            [
                tuple(self.store.data["o6"]["actions"]["张开手"]),
                tuple(self.store.data["o6"]["actions"]["抓取"]),
            ],
        )

    def test_gello_j5_is_locked_at_follow_start_pose(self):
        self.robot.motion_mode = "servoj"
        self.store.data["gello"]["locked_joints"] = [5]
        origin = self.master.value
        self.master.value = GelloFeedback(
            time.monotonic(), (0.0, 0.0, 0.0, 0.0, np.deg2rad(20.0), 0.0, 0.0)
        )
        submit = self.run_one_follow_cycle(origin, [0.0] * 7)
        self.assertEqual(submit.call_args.args[0][4], 0.0)
        telemetry = self.engine.snapshot()["gello_telemetry"]
        self.assertEqual(telemetry["desired_deg"][4], 0.0)
        self.assertEqual(telemetry["target_deg"][4], 0.0)

    def test_gello_j5_is_unlocked_by_default(self):
        self.robot.motion_mode = "servoj"
        origin = self.master.value
        self.master.value = GelloFeedback(
            time.monotonic(), (0.0, 0.0, 0.0, 0.0, np.deg2rad(5.0), 0.0, 0.0)
        )
        submit = self.run_one_follow_cycle(origin, [0.0] * 7)
        self.assertAlmostEqual(submit.call_args.args[0][4], 5.0)

    def test_movej_follow_has_no_added_mode_gate(self):
        self.robot.motion_mode = "movej"
        self.robot.current_mode = Mock(side_effect=AssertionError("unexpected mode query"))
        with patch.object(self.engine, "_start_gello_follow") as start:
            self.engine.start_follow()
        start.assert_called_once()
        self.robot.current_mode.assert_not_called()

    def test_movej_follow_still_requires_enabled_servo(self):
        self.robot.motion_mode = "movej"
        for state in (0, 1, 2):
            with self.subTest(state=state), patch.object(self.robot, "servo_state", return_value=state):
                with self.assertRaisesRegex(RuntimeError, "尚未上电"):
                    self.engine.start_follow()
        self.assertEqual(self.robot.servoj_targets, [])

    def test_invalid_config_update_keeps_memory_and_file_unchanged(self):
        self.store.save()
        before = copy.deepcopy(self.store.data)
        raw = self.store.path.read_bytes()
        with self.assertRaises(ValueError):
            self.store.update_runtime({"dataset": {"camera_max_skew_s": -1}})
        self.assertEqual(self.store.data, before)
        self.assertEqual(self.store.path.read_bytes(), raw)

    def test_failed_config_write_keeps_previous_runtime_settings(self):
        before = copy.deepcopy(self.store.data)
        with patch.object(self.store, "save", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                self.store.update_runtime({"robot": {"safety_max_tracking_error_rad": 0.1}})
        self.assertEqual(self.store.data, before)

    def test_nan_safety_limit_is_rejected(self):
        with self.assertRaises(ValueError):
            self.store.update_runtime({"robot": {"safety_max_command_speed_rad_s": float("nan")}})

    def test_nan_controller_feedback_is_rejected_at_sdk_boundary(self):
        api = Mock()
        api.VectorDouble = list
        def position(_fd, _coord, output):
            output.extend([float("nan")] + [0.0] * 6)
            return 0
        api.get_current_position.side_effect = position
        adapter = NrcRobotAdapter(api, 1, 2, robot_num=1)
        with self.assertRaisesRegex(RuntimeError, "非有限数值"):
            adapter.joint_position()

    def test_controller_angles_near_full_turn_are_canonicalized(self):
        class PositionApi:
            VectorDouble = list

            def get_current_position(self, _fd, _coord, output):
                output.extend([-359.92, -0.07, -0.02, -0.01, -0.02, 360.0, 0.0])
                return 0

        adapter = NrcRobotAdapter(PositionApi(), 1, 2, robot_num=1)
        joints = adapter.joint_position()
        self.assertAlmostEqual(joints[0], 0.08, places=6)
        self.assertAlmostEqual(joints[5], 0.0, places=6)


class MoveJDispatchTests(unittest.TestCase):
    def test_low_latency_idle_confirmation_can_submit_in_same_cycle(self):
        api = Mock()
        api.VectorDouble = list
        api.PosType_data = 0
        api.robot_movej.return_value = 0
        api.get_robot_running_state_robot.side_effect = [(0, 2), (0, 0)]
        api.get_servo_state_robot.return_value = (0, 3)
        adapter = NrcRobotAdapter(api, 11, 22, robot_num=1, motion_mode="movej", movej_low_latency=True)
        adapter.record_controller_message(1, "机器人1作业文件正在运行中", 4098)
        self.assertFalse(adapter.clear_movej_busy_if_stopped())
        self.assertFalse(adapter.send_servoj([1.0] + [0.0] * 6))
        self.assertEqual(api.get_robot_running_state_robot.call_count, 1)
        api.robot_movej.assert_not_called()
        self.assertTrue(adapter.clear_movej_busy_if_stopped())
        latest = [2.0] + [0.0] * 6
        self.assertTrue(adapter.send_servoj(latest))
        api.robot_movej.assert_called_once()
        self.assertEqual(api.robot_movej.call_args.args[1].targetPosValue, latest)

    def test_low_latency_late_busy_warning_drops_stale_dispatch(self):
        api = Mock()
        adapter = NrcRobotAdapter(api, 11, 22, robot_num=1, motion_mode="movej", movej_low_latency=True)
        self.assertTrue(adapter.clear_movej_busy_if_stopped())
        adapter.record_controller_message(1, "机器人1作业文件正在运行中", 4098)
        self.assertFalse(adapter.send_servoj([1.0] + [0.0] * 6))
        self.assertEqual(api.mock_calls, [])

    def test_movej_power_on_retains_original_sequence(self):
        api = FakeNrcServoApi(state=0)
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)
        adapter.power_on(timeout=0.2)
        self.assertLess(
            api.calls.index(("set_current_mode_robot", 11, 1, 2)),
            api.calls.index(("set_servo_poweron_robot", 11, 1)),
        )

    def test_movej_already_powered_original_state_is_unchanged(self):
        api = FakeNrcServoApi(state=3)
        api.mode = 2
        adapter = NrcRobotAdapter(
            api, command_fd=11, servo_fd=22, robot_num=1, motion_mode="movej"
        )
        transition = adapter.power_on(timeout=0.2)
        self.assertEqual((transition.initial_state, transition.final_state), (3, 3))
        self.assertFalse(any(call[0].startswith(("set_", "clear_")) for call in api.calls))

    def test_movej_follow_uses_configured_faster_speed_and_period(self):
        class MoveJApi:
            VectorDouble = list
            PosType_data = 0

            class MoveCmd:
                pass

            def __init__(self):
                self.commands = []

            def robot_movej(self, socket_fd, command):
                self.commands.append((socket_fd, command))
                return 0

        api = MoveJApi()
        adapter = NrcRobotAdapter(
            api, command_fd=11, servo_fd=22, robot_num=1, motion_mode="movej",
            movej_velocity=30.0, movej_acc=50.0, movej_dec=50.0, movej_period_s=0.05,
        )
        self.assertTrue(adapter.send_servoj([0.0] * 7))
        socket_fd, command = api.commands[0]
        self.assertEqual(socket_fd, 11)
        self.assertEqual(command.targetPosType, 0)
        self.assertEqual(command.coord, 0)
        self.assertEqual(command.velocity, 30.0)
        self.assertEqual(command.acc, 50.0)
        self.assertEqual(command.dec, 50.0)

    def test_movej_busy_waits_and_resumes_with_latest_target(self):
        class MoveJApi(FakeNrcServoApi):
            VectorDouble = list
            PosType_data = 0

            class MoveCmd:
                pass

            def __init__(self):
                super().__init__(state=3)
                self.commands = []
                self.stop_calls = 0

            def robot_movej(self, socket_fd, command):
                self.commands.append((socket_fd, command))
                return 0

            def queue_motion_stop_not_power_off(self, _socket_fd):
                self.stop_calls += 1
                return 0

        api = MoveJApi()
        adapter = NrcRobotAdapter(api, 11, 22, robot_num=1, motion_mode="movej")
        adapter.record_controller_message(1, "机器人1作业文件正在运行中", 4098)
        for state in (2, 1):
            api.running_states = [state]
            self.assertFalse(adapter.send_servoj([1.0] + [0.0] * 6))
            self.assertTrue(adapter.controller_job_warning)
        adapter.stop_motion()
        self.assertEqual(api.commands, [])
        self.assertEqual(api.stop_calls, 0)
        api.running_states = [0]
        self.assertFalse(adapter.send_servoj([2.0] + [0.0] * 6))
        self.assertFalse(adapter.controller_job_warning)
        latest = [3.0] + [0.0] * 6
        self.assertTrue(adapter.send_servoj(latest))
        self.assertEqual(len(api.commands), 1)
        self.assertEqual(api.commands[0][1].targetPosValue, latest)

    def test_movej_busy_state_read_failure_is_not_swallowed(self):
        api = FakeNrcServoApi(state=3)
        api.get_robot_running_state_robot = Mock(return_value=(-1, 0))
        api.robot_movej = Mock()
        adapter = NrcRobotAdapter(api, 11, 22, robot_num=1, motion_mode="movej")
        adapter.record_controller_message(1, "机器人1作业文件正在运行中", 4098)
        with self.assertRaisesRegex(RuntimeError, "运行状态失败"):
            adapter.send_servoj([0.0] * 7)
        api.robot_movej.assert_not_called()

    def test_movej_busy_recovery_does_not_bypass_servo_state(self):
        api = FakeNrcServoApi(state=1)
        api.robot_movej = Mock()
        adapter = NrcRobotAdapter(api, 11, 22, robot_num=1, motion_mode="movej")
        adapter.record_controller_message(1, "机器人1作业文件正在运行中", 4098)
        with self.assertRaisesRegex(RuntimeError, "退出运行状态"):
            adapter.send_servoj([0.0] * 7)
        api.robot_movej.assert_not_called()

class GelloWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = ui.QApplication.instance() or ui.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        (root / "config").mkdir()
        source = Path(ui.__file__).parent / "config" / "roarm_cr5_teleop.json"
        config = json.loads(source.read_text())
        config["dataset"]["root"] = str(root / "datasets")
        (root / "config" / source.name).write_text(json.dumps(config))
        self.patches = [
            patch.object(ui, "BASE_DIR", root), patch.object(ui, "rs", None),
            patch.object(ui.aa, "connect_robot", side_effect=AssertionError("no hardware")),
            patch.object(ui.GelloController, "connect", side_effect=AssertionError("no serial")),
            patch.object(ui.O6Controller, "connect", side_effect=AssertionError("no hand")),
        ]
        for item in self.patches:
            item.start()
        self.window = ui.MyMainForm()
        self.window.timer_teleop_ui.stop()
        self.window.refresh_teleop_ui()

    def tearDown(self):
        recorder = self.window.lerobot_recorder
        recorder._episode_active = False
        recorder._buffered_frames = 0
        recorder._client = None
        self.window.D435_1_Started = False
        self.window.D435_2_Started = False
        if self.window._episode_operation_lock.locked():
            self.window._episode_operation_lock.release()
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        for item in reversed(self.patches):
            item.stop()
        self.temp.cleanup()

    def make_ready(self):
        w = self.window
        master = FakeGello()
        hand = FakeO6()
        hand.last_sent_target = hand.position
        robot = FakeRobot()
        robot.controller_job_warning = ""
        w.teleop_engine.set_master(master)
        w.master_controller = master
        w.teleop_engine.o6 = hand
        w.o6_controller = hand
        w.nrc_adapter = robot
        w.teleop_engine.attach_robot(robot)
        w.robot1_connected = True
        now = time.monotonic()
        w._latest_robot_snapshot = (robot.joint_position(), robot.tcp_position(), 3, "")
        w._latest_robot_snapshot_at = now
        w.D435_1_Started = w.D435_2_Started = True
        w._wrist_rgb_frame = w._base_rgb_frame = w._base_roi_rgb_frame = np.zeros((8, 8, 3), dtype=np.uint8)
        w._wrist_rgb_timestamp = w._base_rgb_timestamp = now
        w.teleop_engine._set_state("following")
        w.teleop_engine._gello_telemetry = {
            "actual_deg": (0.0,) * 6, "target_deg": (0.0,) * 6,
            "target_full_deg": (0.0,) * 7, "feedback_at": now,
            "command_at": now, "sent_count": 5, "send_hz": 80.0,
            "tracking_error_deg": 0.0,
        }

    def test_three_tabs_no_legacy_background_collector_or_hardware_start(self):
        w = self.window
        self.assertIn("ServoJ 实验副本", w.windowTitle())
        self.assertEqual(w.log_path.name, "gello_low_latency_test.log")
        self.assertEqual([w.tabWidget.tabText(i) for i in range(w.tabWidget.count())],
                         ["GELLO 控制", "数采工作台", "参数维护"])
        self.assertFalse(hasattr(w, "my_thread_HDF5_Collect"))
        ui.aa.connect_robot.assert_not_called()
        ui.GelloController.connect.assert_not_called()
        ui.O6Controller.connect.assert_not_called()
        self.assertTrue(w.lerobot_start_button.isEnabled())
        robot_cfg = w.teleop_store.data["robot"]
        self.assertEqual(w.teleop_movej_velocity.value(), robot_cfg["movej_velocity"])
        self.assertEqual(w.teleop_movej_acc.value(), robot_cfg["movej_acc"])
        self.assertEqual(w.teleop_movej_dec.value(), robot_cfg["movej_dec"])
        self.assertEqual(w.teleop_movej_period.value(), robot_cfg["movej_period_s"])
        self.assertEqual(
            w.teleop_movej_duplicate_deadband.value(),
            robot_cfg["movej_duplicate_deadband_deg"],
        )

    def test_short_segment_controls_save_to_existing_adapter(self):
        w = self.window
        api = Mock()
        adapter = NrcRobotAdapter(api, 11, 22, robot_num=1, motion_mode="servoj")
        with patch.object(w, "nrc_adapter", adapter):
            w.teleop_movej_low_latency.setChecked(True)
            w.teleop_movej_segment.setValue(1.5)
            w.teleop_movej_duplicate_deadband.setValue(0.08)
            w.teleop_gello_joint_scale.setValue(1.1)
            self.assertTrue(w.save_teleop_settings())
            self.assertTrue(adapter.movej_low_latency)
            self.assertEqual(w.teleop_store.data["robot"]["movej_max_segment_deg"], 1.5)
            self.assertEqual(w.teleop_store.data["robot"]["movej_duplicate_deadband_deg"], 0.08)
            self.assertEqual(w.teleop_store.data["gello"]["joint_scale"], 1.1)
            w.teleop_movej_low_latency.setChecked(False)
            self.assertFalse(w.teleop_movej_segment.isEnabled())
            self.assertTrue(w.save_teleop_settings())
            self.assertFalse(adapter.movej_low_latency)
        self.assertEqual(api.mock_calls, [])

    def test_joint_lock_buttons_save_selected_joints(self):
        w = self.window
        self.assertFalse(w.teleop_locked_joint_checks[5].isChecked())
        w.teleop_locked_joint_checks[2].setChecked(True)
        w.teleop_locked_joint_checks[5].setChecked(True)
        self.assertTrue(w.save_teleop_settings(show_event=False))
        self.assertEqual(w.teleop_store.data["gello"]["locked_joints"], [2, 5])

    def test_short_segment_preflight_rejects_full_desired_gap(self):
        self.make_ready()
        w = self.window
        w.collection_verified.setChecked(True)
        w.teleop_engine._gello_telemetry.update({"low_latency": True, "desired_tracking_error_deg": 8.0})
        w.refresh_teleop_ui()
        self.assertTrue(w.lerobot_start_button.isEnabled())
        self.assertIn("完整期望误差", w.collection_preflight.toolTip())
        w.teleop_engine._gello_telemetry["desired_tracking_error_deg"] = 3.0
        w.refresh_teleop_ui()
        self.assertTrue(w.lerobot_start_button.isEnabled())

    def test_short_segment_preflight_does_not_block_on_transient_tracking_error(self):
        # 瞬时“已提交目标差”只做质量标记，不阻止开始录制。
        self.make_ready()
        w = self.window
        w.teleop_engine._gello_telemetry["tracking_error_deg"] = 8.0
        w.refresh_teleop_ui()
        self.assertTrue(w.lerobot_start_button.isEnabled())

    def test_latency_display_separates_desired_from_submitted_target(self):
        self.make_ready()
        w = self.window
        w.teleop_engine._gello_telemetry.update({
            "target_deg": (2.0,) + (0.0,) * 5, "desired_deg": (10.0,) + (0.0,) * 5,
            "validated_deg": (4.0,) + (0.0,) * 5,
            "validated_at": time.monotonic(), "pending_target_deg": 2.0,
            "desired_tracking_error_deg": 10.0, "low_latency": True,
            "latency": {
                "phase": "busy", "leader_age_ms": 2.0, "feedback_read_ms": 12.0,
                "busy_check_ms": 3.0, "dispatch_ms": 4.0, "busy_wait_ms": 1200.0,
                "last_busy_wait_ms": 500.0, "recent_send_hz": 1.0, "segment_delta_deg": 2.0,
            },
        })
        w.refresh_teleop_ui()
        self.assertEqual(w.gello_joint_table.item(0, 3).text(), "10.00")
        self.assertEqual(w.gello_joint_table.item(0, 4).text(), "4.00")
        self.assertEqual(w.gello_joint_table.item(0, 5).text(), "2.00")
        self.assertEqual(w.gello_joint_table.item(0, 6).text(), "10.00")
        self.assertIn("1200ms", w.gello_latency_label.text())

    def test_follow_ui_poll_reuses_telemetry_and_only_checks_servo_state(self):
        self.make_ready()
        w = self.window
        w.refresh_teleop_ui()
        w._last_robot_poll_started_at = 0.0
        with patch.object(w.nrc_adapter, "joint_position", wraps=w.nrc_adapter.joint_position) as joints, patch.object(
            w.nrc_adapter, "tcp_position", wraps=w.nrc_adapter.tcp_position
        ) as tcp, patch.object(
            w.nrc_adapter, "servo_state", wraps=w.nrc_adapter.servo_state
        ) as servo:
            w.on_timeout_render()
            w._robot_poll_thread.join(timeout=1.0)
        joints.assert_not_called()
        tcp.assert_not_called()
        servo.assert_called_once()

    def test_idle_ui_poll_keeps_full_status_refresh(self):
        self.make_ready()
        w = self.window
        w.teleop_engine._set_state("idle")
        w.refresh_teleop_ui()
        w._last_robot_poll_started_at = 0.0
        with patch.object(w.nrc_adapter, "joint_position", wraps=w.nrc_adapter.joint_position) as joints, patch.object(
            w.nrc_adapter, "tcp_position", wraps=w.nrc_adapter.tcp_position
        ) as tcp, patch.object(
            w.nrc_adapter, "servo_state", wraps=w.nrc_adapter.servo_state
        ) as servo:
            w.on_timeout_render()
            w._robot_poll_thread.join(timeout=1.0)
        joints.assert_called_once()
        tcp.assert_called_once()
        servo.assert_called_once()

    def test_saved_servoj_parameters_are_written_without_hardware_calls(self):
        w = self.window
        api = Mock()
        api.VectorDouble = list
        api.PosType_data = 0
        adapter = NrcRobotAdapter(api, 11, 22, robot_num=1, motion_mode="servoj")
        w.teleop_servoj_vmax.setValue(25.0)
        w.teleop_servoj_amax.setValue(135.0)
        w.teleop_servoj_jmax.setValue(240.0)
        with patch.object(w, "nrc_adapter", adapter):
            w.teleop_save_settings_button.click()
            saved = json.loads(w.teleop_store.path.read_text())["robot"]
            self.assertEqual(saved["motion_mode"], "servoj")
            self.assertEqual(saved["servoj_vmax"], 25.0)
            self.assertEqual(saved["servoj_amax"], 135.0)
            self.assertEqual(saved["servoj_jmax"], 240.0)
            self.assertEqual(api.mock_calls, [])
            w.refresh_teleop_ui()
            self.assertIn("servoj", w.connection_mode_label.text())

    def test_failed_speed_save_keeps_connected_parameters(self):
        w = self.window
        api = Mock()
        adapter = NrcRobotAdapter(api, 11, 22, robot_num=1, motion_mode="movej")
        before_data = copy.deepcopy(w.teleop_store.data)
        before_velocity = adapter.movej_velocity
        before = w.teleop_store.path.read_bytes()
        w.teleop_movej_velocity.setValue(25.0)
        with patch.object(w, "nrc_adapter", adapter), patch.object(
            w.teleop_store, "save", side_effect=OSError("disk full")
        ):
            self.assertFalse(w.save_teleop_settings())
        self.assertEqual(adapter.movej_velocity, before_velocity)
        self.assertEqual(w.teleop_store.data, before_data)
        self.assertEqual(w.teleop_store.path.read_bytes(), before)
        self.assertEqual(api.mock_calls, [])

    def test_speed_save_is_blocked_during_follow_startup(self):
        w = self.window
        before = copy.deepcopy(w.teleop_store.data)
        w.teleop_movej_velocity.setValue(25.0)
        starting = Mock()
        starting.is_alive.return_value = True
        with patch.dict(w._teleop_async_threads, {"启动跟随": starting}):
            self.assertFalse(w.save_teleop_settings())
        self.assertEqual(w.teleop_store.data, before)

    def test_previous_busy_warning_does_not_permanently_disable_follow(self):
        self.make_ready()
        self.window.teleop_engine._set_state("idle")
        self.window.nrc_adapter.controller_job_warning = "code=4098"
        self.window.refresh_teleop_ui()
        self.assertTrue(self.window.gello_page_start.isEnabled())
        self.assertTrue(self.window.lerobot_start_button.isEnabled())
        self.window.nrc_adapter.controller_job_warning = ""
        self.window.refresh_teleop_ui()
        self.assertNotIn("忙", self.window.gello_page_start.toolTip())

    def test_episode_start_needs_no_manual_follow_confirmation(self):
        self.make_ready()
        self.window.refresh_teleop_ui()
        self.assertTrue(self.window.lerobot_start_button.isEnabled())

    def test_action_record_replay_group_is_on_collection_workbench(self):
        w = self.window
        self.assertIs(w.teleop_replay_group.parent(), w.camera_console_content)
        self.assertTrue(
            any("动作记录 / 重放" in label.text() for label in w.teleop_replay_group.findChildren(ui.QLabel))
        )

    def test_action_record_can_start_during_gello_follow_without_saving_settings(self):
        self.make_ready()
        w = self.window
        with patch.object(w, "save_teleop_settings") as save, patch.object(
            w, "_run_teleop_async"
        ) as run:
            w.TeleopRecordStart(2)
        save.assert_not_called()
        run.assert_called_once()

    def test_stale_servo_state_blocks_preflight(self):
        self.make_ready()
        self.window.collection_verified.setChecked(True)
        self.window._latest_robot_snapshot_at -= 2.0
        self.window.refresh_teleop_ui()
        self.assertTrue(self.window.lerobot_start_button.isEnabled())

    def test_fault_buffer_can_be_discarded_but_not_saved(self):
        w = self.window
        w.lerobot_recorder._buffered_frames = 3
        w.lerobot_recorder._last_error = "camera lost"
        w.teleop_engine._set_state("fault", "camera lost")
        w.refresh_teleop_ui()
        self.assertTrue(w.lerobot_discard_button.isEnabled())
        self.assertIsNone(w.lerobot_outcome.currentData())
        self.assertTrue(w.lerobot_save_button.isEnabled())
        self.assertTrue(w.lerobot_start_button.isEnabled())

    def test_stop_is_available_while_dataset_operation_is_busy(self):
        self.make_ready()
        self.window._episode_operation_lock.acquire()
        self.window.refresh_teleop_ui()
        self.assertTrue(self.window.gello_page_stop.isEnabled())
        self.assertTrue(self.window.gello_page_estop.isEnabled())

    def test_start_records_context_without_reconfiguring_o6(self):
        self.make_ready()
        w = self.window
        w.collection_verified.setChecked(True)
        with patch.object(w, "_run_episode_async", side_effect=lambda name, fn: fn()), patch.object(
            w.lerobot_recorder, "start_episode"
        ) as start:
            w.LeRobotEpisodeStart()
        self.assertEqual(w.o6_controller.profiles, [])
        self.assertEqual(start.call_args.kwargs["metadata"]["robot"]["motion_mode"], "servoj")

    def test_start_auto_saves_stopped_episode_before_recording_next_one(self):
        self.make_ready()
        w = self.window
        w.lerobot_recorder._buffered_frames = 2
        with patch.object(w, "_run_episode_async", side_effect=lambda _name, fn: fn()), patch.object(
            w.lerobot_recorder, "save_episode"
        ) as save, patch.object(w.lerobot_recorder, "start_episode") as start:
            w.LeRobotEpisodeStart()
        save.assert_called_once_with("success", "自动保存后开始下一条")
        start.assert_called_once()

    def test_camera_skew_is_marked_by_actual_provider_without_interrupting_collection(self):
        self.make_ready()
        w = self.window
        w.collection_verified.setChecked(True)
        w._base_rgb_timestamp -= 0.2
        frame = w._lerobot_sample_provider()
        self.assertEqual(frame["quality"]["camera_skew_exceeded"], 1.0)

    def test_camera_stop_is_refused_during_sampling(self):
        self.make_ready()
        w = self.window
        w.pipeline = Mock()
        w.pipeline_D435_2 = Mock()
        w.lerobot_recorder._episode_active = True
        w.D435_1_Stop()
        w.D435_2_Stop()
        w.pipeline.stop.assert_not_called()
        w.pipeline_D435_2.stop.assert_not_called()

    def test_close_refuses_pending_episode_without_deleting_it(self):
        self.window.lerobot_recorder._buffered_frames = 3
        event = ui.QCloseEvent()
        with patch.object(self.window.teleop_engine, "shutdown") as shutdown:
            self.window.closeEvent(event)
            shutdown.assert_not_called()
        self.assertFalse(event.isAccepted())
        self.assertEqual(self.window.lerobot_recorder.snapshot()["buffered_frames"], 3)

    def test_save_shortcut_does_not_start_another_episode(self):
        with patch.object(self.window, "LeRobotEpisodeSave") as save, patch.object(
            self.window, "LeRobotEpisodeStart"
        ) as start:
            self.window._shortcut_action_handlers()["episode_save_next"]()
            save.assert_called_once()
            start.assert_not_called()
        self.assertNotIn("replay_start", self.window._keyboard_shortcuts)
        self.assertNotIn("preset_a", self.window._keyboard_shortcuts)

    def test_emergency_shortcut_survives_shortcut_edit_mode(self):
        editor = self.window.shortcut_editors["prepare"]
        self.window.eventFilter(editor, ui.QEvent(ui.QEvent.FocusIn))
        self.assertTrue(self.window._keyboard_shortcuts["emergency_stop"].isEnabled())
        self.assertFalse(self.window._keyboard_shortcuts["start_follow"].isEnabled())

    def test_logs_persist_and_display_literal_text(self):
        self.window._teleop_event("error", "<alarm> CR3A disconnected")
        self.window.refresh_teleop_ui()
        self.assertIn("<alarm>", self.window.teleop_log.toPlainText())
        self.assertIn("CR3A disconnected", self.window.log_path.read_text())
        self.assertEqual(self.window.teleop_log.document().maximumBlockCount(), 2000)

    def test_save_home_overwrites_without_confirmation(self):
        with patch.object(
            ui.QMessageBox, "question", side_effect=AssertionError("unexpected HOME confirmation")
        ), patch.object(self.window.teleop_engine, "save_preset") as save, patch.object(
            self.window, "_run_teleop_async", side_effect=lambda _name, operation: operation()
        ):
            self.window.TeleopSaveHome()
        save.assert_called_once_with("HOME")

    def test_go_home_executes_without_confirmation(self):
        with patch.object(
            ui.QMessageBox, "warning", side_effect=AssertionError("unexpected HOME confirmation")
        ), patch.object(self.window.teleop_engine, "recall_preset") as recall:
            self.window.TeleopGoHome()
        recall.assert_called_once_with("HOME", resume_follow=False)
        self.assertNotIn("…", self.window.gello_page_go_home.text())
        self.assertIn("立即执行", self.window.gello_page_go_home.toolTip())

    def test_collection_home_takes_over_without_stopping_active_episode(self):
        self.make_ready()
        w = self.window
        w.lerobot_recorder._episode_active = True
        with patch.object(w.lerobot_recorder, "stop_episode") as stop_episode, patch.object(
            w.teleop_engine, "recall_preset"
        ) as recall:
            w.TeleopGoHome()
        stop_episode.assert_not_called()
        recall.assert_called_once_with("HOME", resume_follow=False)

    def test_collection_home_button_is_enabled_while_recording(self):
        self.make_ready()
        w = self.window
        w.teleop_store.data["presets"]["HOME"] = {}
        w.lerobot_recorder._episode_active = True
        w.refresh_teleop_ui()
        self.assertTrue(w.workflow_home_button.isEnabled())

    def test_collection_home_accepts_following_state_for_direct_takeover(self):
        self.make_ready()
        with patch.object(self.window.teleop_engine, "recall_preset") as recall:
            self.window.TeleopGoHome()
        recall.assert_called_once_with("HOME", resume_follow=False)
        self.assertEqual(self.window.workflow_home_button.text(), "一键回 HOME")

    def test_following_rejects_setting_changes_without_touching_config(self):
        self.make_ready()
        before = copy.deepcopy(self.window.teleop_store.data)
        self.window.gello_page_tracking_limit.setValue(10.0)
        self.window.teleop_movej_velocity.setValue(25.0)
        self.window.teleop_movej_low_latency.setChecked(False)
        self.window.teleop_movej_segment.setValue(3.0)
        self.window.teleop_gello_joint_scale.setValue(1.2)
        adapter = NrcRobotAdapter(Mock(), 11, 22, robot_num=1, motion_mode="movej")
        with patch.object(self.window, "nrc_adapter", adapter):
            self.assertFalse(self.window.save_teleop_settings())
        self.assertEqual(adapter.movej_velocity, 10.0)
        self.assertEqual(self.window.teleop_store.data, before)
        self.assertEqual(self.window.o6_controller.profiles, [])

    def test_camera_disconnect_is_visible_and_invalidates_readiness(self):
        self.make_ready()
        w = self.window
        w.pipeline = Mock()
        w.pipeline.poll_for_frames.side_effect = RuntimeError("device disconnected")
        w.timer_D435_1 = Mock()
        w.update_frame_D435_1()
        self.assertFalse(w.D435_1_Started)
        w.timer_D435_1.stop.assert_called_once()
        w.pipeline.stop.assert_called_once()
        w.refresh_teleop_ui()
        self.assertIn("device disconnected", w.teleop_log.toPlainText())


if __name__ == "__main__":
    unittest.main()

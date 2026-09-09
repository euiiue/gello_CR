import json
import math
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from lerobot_recorder import LeRobotEpisodeRecorder, _dataset_features
from teleop_runtime import (
    GelloFeedback,
    GelloKinematics,
    Inverse3Controller,
    Inverse3Feedback,
    NrcRobotAdapter,
    O6Controller,
    RoArmFeedback,
    TeleopConfigStore,
    TeleopEngine,
    _quaternion_to_nrc_abc,
)


def feedback(x=100.0, y=200.0, z=300.0, b=0.1, s=0.2, e=0.3, t=0.8):
    return RoArmFeedback(
        timestamp=time.monotonic(),
        x=x,
        y=y,
        z=z,
        b=b,
        s=s,
        e=e,
        t=t,
        torques=(0.0, 0.0, 0.0, 0.0),
    )


def inverse_feedback(x=100.0, y=200.0, z=300.0, orientation=(1.0, 0.0, 0.0, 0.0)):
    return Inverse3Feedback(
        timestamp=time.monotonic(),
        x=x,
        y=y,
        z=z,
        orientation_wxyz=orientation,
        inverse3_device_id="06DA",
        grip_device_id="VG01",
        grip_type="wireless_verse_grip",
        buttons=(False, False, False),
    )


class Inverse3ControllerTests(unittest.TestCase):
    class FakeWebSocket:
        def __init__(self, frames):
            self.frames = [json.dumps(frame) for frame in frames]
            self.sent = []
            self.closed = threading.Event()

        def recv(self):
            if self.frames:
                return self.frames.pop(0)
            self.closed.wait(1.0)
            raise RuntimeError("socket closed")

        def send(self, payload):
            self.sent.append(payload)

        def close(self):
            self.closed.set()

    @staticmethod
    def frame(with_grip=True, x=0.1):
        frame = {
            "inverse3": [
                {
                    "device_id": "06DA",
                    "config": {"basis": {"permutation": "XYZ"}},
                    "state": {
                        "cursor_position": {"x": x, "y": 0.2, "z": 0.3},
                        "control_mode": "idle",
                        "body_orientation": {
                            "w": 0.7,
                            "x": 0.7,
                            "y": 0.0,
                            "z": 0.0,
                        },
                    },
                    "status": {
                        "calibrated": True,
                        "power_supply": True,
                        "ready": True,
                        "started": True,
                        "in_use": False,
                    },
                }
            ],
            "verse_grip": [],
            "wireless_verse_grip": [],
        }
        if with_grip:
            frame["wireless_verse_grip"] = [
                {
                    "device_id": "VG01",
                    "config": {"basis": {"permutation": "XYZ"}},
                    "state": {
                        "orientation": {
                            "w": 1.0,
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0,
                        },
                        "buttons": {"a": False, "b": True, "c": False},
                    },
                    "status": {
                        "connected": True,
                        "awake": True,
                        "ready": True,
                    },
                }
            ]
        return frame

    def test_readonly_probe_stream_uses_grip_orientation_not_body_orientation(self):
        socket = self.FakeWebSocket([self.frame(), self.frame(x=0.11)])
        controller = Inverse3Controller(
            "ws://127.0.0.1:10001",
            device_id="06DA",
            feedback_hz=20.0,
            websocket_factory=lambda _uri, _timeout: socket,
        )

        latest = controller.connect(timeout=0.5)
        self.assertEqual(latest.xyz, (110.0, 200.0, 300.0))
        self.assertEqual(latest.orientation_wxyz, (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(latest.buttons, (False, True, False))
        first_probe = json.loads(socket.sent[0])
        self.assertEqual(
            first_probe["inverse3"][0]["commands"], {"probe_position": {}}
        )
        self.assertEqual(
            first_probe["wireless_verse_grip"][0]["commands"],
            {"probe_orientation": {}},
        )
        self.assertNotIn("session", first_probe)
        self.assertNotIn("set_cursor", socket.sent[0])
        controller.close()

    def test_missing_versegrip_allows_xyz_probe_but_not_fake_orientation(self):
        frame = self.frame(with_grip=False)
        socket = self.FakeWebSocket([frame, frame])
        controller = Inverse3Controller(
            "ws://127.0.0.1:10001",
            websocket_factory=lambda _uri, _timeout: socket,
        )

        latest = controller.connect(timeout=0.5)

        self.assertEqual(latest.xyz, (100.0, 200.0, 300.0))
        self.assertIsNone(latest.orientation_wxyz)
        probe = json.loads(socket.sent[0])
        self.assertEqual(
            probe["inverse3"][0]["commands"], {"probe_position": {}}
        )
        self.assertNotIn("verse_grip", probe)
        self.assertNotIn("wireless_verse_grip", probe)
        controller.close()

    def test_busy_inverse3_fails_before_any_command(self):
        frame = self.frame()
        frame["inverse3"][0]["status"]["in_use"] = True
        frame["inverse3"][0]["state"]["control_mode"] = "force"
        socket = self.FakeWebSocket([frame])
        controller = Inverse3Controller(
            "ws://127.0.0.1:10001",
            websocket_factory=lambda _uri, _timeout: socket,
        )

        with self.assertRaisesRegex(RuntimeError, "其他 Haply 控制会话"):
            controller.connect(timeout=0.5)
        self.assertEqual(socket.sent, [])

    def test_multiple_or_sleeping_grips_are_rejected(self):
        multiple = self.frame()
        second = json.loads(json.dumps(multiple["wireless_verse_grip"][0]))
        second["device_id"] = "VG02"
        multiple["wireless_verse_grip"].append(second)
        socket = self.FakeWebSocket([multiple])
        controller = Inverse3Controller(
            "ws://127.0.0.1:10001",
            websocket_factory=lambda _uri, _timeout: socket,
        )
        with self.assertRaisesRegex(RuntimeError, "多个 VerseGrip"):
            controller.connect(timeout=0.5)
        self.assertEqual(socket.sent, [])

        sleeping = self.frame()
        sleeping["wireless_verse_grip"][0]["status"]["awake"] = False
        socket = self.FakeWebSocket([sleeping])
        controller = Inverse3Controller(
            "ws://127.0.0.1:10001",
            websocket_factory=lambda _uri, _timeout: socket,
        )
        with self.assertRaisesRegex(RuntimeError, "awake"):
            controller.connect(timeout=0.5)
        self.assertEqual(socket.sent, [])

    def test_unexpected_basis_is_rejected_without_reconfiguration(self):
        frame = self.frame()
        frame["inverse3"][0]["config"]["basis"]["permutation"] = "XZ-Y"
        socket = self.FakeWebSocket([frame])
        controller = Inverse3Controller(
            "ws://127.0.0.1:10001",
            basis="XYZ",
            websocket_factory=lambda _uri, _timeout: socket,
        )

        with self.assertRaisesRegex(RuntimeError, "basis=XZ-Y"):
            controller.connect(timeout=0.5)
        self.assertEqual(socket.sent, [])

        frame = self.frame()
        frame["wireless_verse_grip"][0]["config"]["basis"][
            "permutation"
        ] = "XZ-Y"
        socket = self.FakeWebSocket([frame])
        controller = Inverse3Controller(
            "ws://127.0.0.1:10001",
            basis="XYZ",
            websocket_factory=lambda _uri, _timeout: socket,
        )
        with self.assertRaisesRegex(RuntimeError, "VerseGrip 会话 basis=XZ-Y"):
            controller.connect(timeout=0.5)
        self.assertEqual(socket.sent, [])

    def test_relative_quaternion_converts_to_nrc_intrinsic_xyz_radians(self):
        quaternion = (
            0.9833474432563559,
            0.03427079855048211,
            -0.10602051106179562,
            0.14357217502739192,
        )
        a, b, c = _quaternion_to_nrc_abc(quaternion)
        self.assertAlmostEqual(a, 0.1, places=7)
        self.assertAlmostEqual(b, -0.2, places=7)
        self.assertAlmostEqual(c, 0.3, places=7)


class FakeNrcServoApi:
    def __init__(self, state=0):
        self.state = int(state)
        self.mode = 0
        self.calls = []
        self.clear_state_after_call = 3
        self.poweron_result = 0
        self.connection_status = {11: 0, 22: 0}
        self.running_states = [0]

    def get_connection_status(self, socket_fd):
        self.calls.append(("get_connection_status", socket_fd))
        return self.connection_status[socket_fd]

    def get_servo_state_robot(self, socket_fd, robot_num, _status):
        self.calls.append(("get_servo_state_robot", socket_fd, robot_num))
        return (0, self.state)

    def get_current_mode_robot(self, socket_fd, robot_num, _mode):
        self.calls.append(("get_current_mode_robot", socket_fd, robot_num))
        return (0, self.mode)

    def set_current_mode_robot(self, socket_fd, robot_num, mode):
        self.calls.append(("set_current_mode_robot", socket_fd, robot_num, mode))
        self.mode = int(mode)
        return 0

    def get_robot_running_state_robot(self, socket_fd, robot_num, _status):
        self.calls.append(
            ("get_robot_running_state_robot", socket_fd, robot_num)
        )
        if len(self.running_states) > 1:
            state = self.running_states.pop(0)
        else:
            state = self.running_states[0]
        return (0, state)

    def clear_error_robot(self, socket_fd, robot_num):
        self.calls.append(("clear_error_robot", socket_fd, robot_num))
        if self.state == 2:
            self.state = int(self.clear_state_after_call)
        return 0

    def set_servo_state_robot(self, socket_fd, robot_num, state):
        self.calls.append(("set_servo_state_robot", socket_fd, robot_num, state))
        if self.state not in (0, 1) or int(state) != 1:
            return -4
        self.state = 1
        return 0

    def set_servo_poweron_robot(self, socket_fd, robot_num):
        self.calls.append(("set_servo_poweron_robot", socket_fd, robot_num))
        if self.poweron_result != 0:
            return self.poweron_result
        if self.state != 1:
            return -4
        self.state = 3
        return 0

    def set_servo_poweroff_robot(self, socket_fd, robot_num):
        self.calls.append(("set_servo_poweroff_robot", socket_fd, robot_num))
        if self.state != 3:
            return -4
        self.state = 1
        return 0


class NrcRobotAdapterTests(unittest.TestCase):
    def test_power_on_from_stopped_runs_ready_then_poweron(self):
        api = FakeNrcServoApi(state=0)
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        transition = adapter.power_on(timeout=0.2)

        self.assertEqual((transition.initial_state, transition.final_state), (0, 3))
        self.assertIn(("set_servo_state_robot", 11, 1, 1), api.calls)
        self.assertIn(("set_servo_poweron_robot", 11, 1), api.calls)
        self.assertLess(
            api.calls.index(("set_servo_state_robot", 11, 1, 1)),
            api.calls.index(("set_servo_poweron_robot", 11, 1)),
        )

    def test_power_on_from_alarm_clears_and_releases_before_ready(self):
        api = FakeNrcServoApi(state=2)
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        transition = adapter.power_on(timeout=0.2)

        self.assertEqual((transition.initial_state, transition.final_state), (2, 3))
        self.assertEqual(
            [call[0] for call in api.calls],
            [
                "get_connection_status",
                "get_servo_state_robot",
                "clear_error_robot",
                "get_servo_state_robot",
                "set_servo_poweroff_robot",
                "get_servo_state_robot",
                "set_servo_state_robot",
                "get_servo_state_robot",
                "get_current_mode_robot",
                "get_servo_state_robot",
                "set_current_mode_robot",
                "get_current_mode_robot",
                "set_servo_poweron_robot",
                "get_servo_state_robot",
            ],
        )

    def test_power_on_from_alarm_already_stopped_runs_full_sequence(self):
        api = FakeNrcServoApi(state=2)
        api.clear_state_after_call = 0
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        transition = adapter.power_on(timeout=0.2)

        self.assertEqual((transition.initial_state, transition.final_state), (2, 3))
        expected = [
            ("clear_error_robot", 11, 1),
            ("set_servo_poweroff_robot", 11, 1),
            ("set_servo_state_robot", 11, 1, 1),
            ("set_servo_poweron_robot", 11, 1),
        ]
        positions = [api.calls.index(call) for call in expected]
        self.assertEqual(positions, sorted(positions))

    def test_power_on_stops_when_alarm_remains_after_clear(self):
        api = FakeNrcServoApi(state=2)
        api.clear_state_after_call = 2
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        with self.assertRaisesRegex(RuntimeError, "清错后仍处于报警状态"):
            adapter.power_on(timeout=0.2)

        self.assertNotIn(("set_servo_poweroff_robot", 11, 1), api.calls)
        self.assertNotIn(("set_servo_poweron_robot", 11, 1), api.calls)

    def test_power_on_stops_when_6001_is_disconnected(self):
        api = FakeNrcServoApi(state=2)
        api.connection_status[11] = -2
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        with self.assertRaisesRegex(RuntimeError, "6001 示教端口未连接"):
            adapter.power_on(timeout=0.2)

        self.assertNotIn(("get_servo_state_robot", 11, 1), api.calls)
        self.assertNotIn(("clear_error_robot", 11, 1), api.calls)

    def test_power_on_reports_nrc_result_meaning(self):
        api = FakeNrcServoApi(state=1)
        api.poweron_result = -1
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        with self.assertRaisesRegex(RuntimeError, "接收控制柜响应失败"):
            adapter.power_on(timeout=0.2)

    def test_clear_servo_error_uses_command_port_and_robot_number(self):
        api = FakeNrcServoApi(state=2)
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        transition = adapter.clear_servo_error(timeout=0.2)

        self.assertEqual((transition.initial_state, transition.final_state), (2, 3))
        self.assertIn(("clear_error_robot", 11, 1), api.calls)
        self.assertIn(("set_servo_poweroff_robot", 11, 1), api.calls)
        self.assertNotIn(("clear_error_robot", 22, 1), api.calls)
        self.assertLess(
            api.calls.index(("clear_error_robot", 11, 1)),
            api.calls.index(("set_servo_poweroff_robot", 11, 1)),
        )
        self.assertIn(("set_servo_state_robot", 11, 1, 1), api.calls)
        self.assertIn(("set_servo_poweron_robot", 11, 1), api.calls)

    def test_clear_servo_error_accepts_poweroff_when_already_stopped(self):
        api = FakeNrcServoApi(state=2)
        api.clear_state_after_call = 0
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        transition = adapter.clear_servo_error(timeout=0.2)

        self.assertEqual((transition.initial_state, transition.final_state), (2, 3))
        self.assertIn(("set_servo_poweroff_robot", 11, 1), api.calls)
        self.assertIn("伺服已经处于下电状态", transition.actions)

    def test_power_off_verifies_ready_state(self):
        api = FakeNrcServoApi(state=3)
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        transition = adapter.power_off(timeout=0.2)

        self.assertEqual((transition.initial_state, transition.final_state), (3, 1))

    def test_wait_connections_ready_checks_both_ports(self):
        api = FakeNrcServoApi(state=0)
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        adapter.wait_connections_ready(timeout=0.2)

        self.assertIn(("get_connection_status", 11), api.calls)
        self.assertIn(("get_connection_status", 22), api.calls)

    def test_connection_statuses_reports_both_ports(self):
        api = FakeNrcServoApi(state=0)
        api.connection_status = {11: 0, 22: -2}
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        self.assertEqual(adapter.connection_statuses(), {"6001": 0, "7000": -2})
        self.assertFalse(adapter.connections_ready())

    def test_wait_motion_stopped_requires_stable_nonmoving_state(self):
        api = FakeNrcServoApi(state=3)
        api.running_states = [2, 0, 0, 0]
        adapter = NrcRobotAdapter(api, command_fd=11, servo_fd=22, robot_num=1)

        stopped = adapter.wait_until_motion_stopped(timeout=0.5)

        self.assertTrue(stopped)
        running_calls = [
            call
            for call in api.calls
            if call[0] == "get_robot_running_state_robot"
        ]
        self.assertEqual(len(running_calls), 4)
        self.assertTrue(all(call[2] == 1 for call in running_calls))


class O6ControllerTests(unittest.TestCase):
    def test_timed_out_recovery_marks_queued_request_cancelled(self):
        class AliveThread:
            def is_alive(self):
                return True

        controller = O6Controller("/dev/unused", Path("/unused"))
        controller._thread = AliveThread()

        with self.assertRaisesRegex(TimeoutError, "恢复命令超时"):
            controller.recover_motor(3, 20, 30, timeout=0.01)

        request = controller._recovery_requests.get_nowait()
        self.assertTrue(request.cancelled.is_set())

        class Hand:
            def __init__(self):
                self.writes = []

            def get_state(self):
                return [100] * 6

            def get_fault(self):
                return [0] * 6

            def __getattr__(self, name):
                if name.startswith("set_"):
                    return lambda value: self.writes.append((name, value))
                raise AttributeError(name)

        hand = Hand()
        controller._process_recovery(hand, request)

        self.assertEqual(hand.writes, [])


class LeRobotRecorderContractTests(unittest.TestCase):
    def test_openpi_state_and_action_shapes_are_enforced(self):
        recorder = LeRobotEpisodeRecorder(
            sample_provider=lambda: {},
            event_callback=lambda *_: None,
            worker_python="/unused",
            repo_prefix="test/cr5_o6",
        )
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        sample = {
            "timestamp": time.monotonic(),
            "image_base_rgb": image,
            "image_wrist_rgb": image,
            "image_roi_rgb": image,
            "observation_state": [0.0] * 18,
            "action": [0.0] * 12,
        }

        recorder._validate_sample(sample)
        features = _dataset_features(224, 224)
        self.assertEqual(features["observation.state"]["shape"], (18,))
        self.assertEqual(features["action"]["shape"], (12,))
        self.assertEqual(
            features["action"]["names"][:6],
            [
                "cr5.delta_tcp.x.m",
                "cr5.delta_tcp.y.m",
                "cr5.delta_tcp.z.m",
                "cr5.delta_tcp.roll.rad",
                "cr5.delta_tcp.pitch.rad",
                "cr5.delta_tcp.yaw.rad",
            ],
        )


class FakeRoArm:
    def __init__(self):
        self.value = feedback()
        self.error = ""
        self.connected = True
        self.released = False
        self.held = False

    def latest(self):
        return self.value

    def release_torque(self):
        self.released = True

    def hold_current(self):
        self.held = True

    def move_joints(self, joints, duration_s=3.0):
        self.value = feedback(
            self.value.x,
            self.value.y,
            self.value.z,
            *[float(value) for value in joints],
        )

    def close(self, hold=True):
        self.connected = False


class FakeInverse3:
    master_type = "inverse3"
    passive = True

    def __init__(self, value=None):
        self.value = value or inverse_feedback()
        self.error = ""
        self.connected = True

    def latest(self):
        return self.value

    def close(self, hold=False):
        self.connected = False


class FakeGello:
    master_type = "gello"
    passive = True

    def __init__(self):
        self.value = GelloFeedback(
            time.monotonic(), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        )
        self.error = ""
        self.connected = True

    def latest(self):
        return self.value

    def close(self, hold=False):
        del hold
        self.connected = False


class FakeO6:
    def __init__(self):
        self.position = (253, 253, 253, 253, 253, 253)
        self.fault = (0, 0, 0, 0, 0, 0)
        self.error = ""
        self.phase = ""
        self.connected = True
        self.targets = []
        self.recoveries = []
        self.profiles = []

    def latest(self):
        return self.position, self.fault, time.monotonic()

    def set_target(self, target):
        self.position = tuple(int(value) for value in target)
        self.targets.append(self.position)

    def set_profile(self, speed, torque):
        self.profiles.append((tuple(speed), tuple(torque)))

    def wait_until_position(self, target, tolerance=6, timeout=15.0, stop_event=None):
        if stop_event is not None and stop_event.is_set():
            return False
        return max(abs(a - b) for a, b in zip(self.position, target)) <= tolerance

    def hold_current(self):
        self.set_target(self.position)

    def recover_motor(
        self, motor_number, speed, torque, timeout=4.0, stop_event=None
    ):
        index = int(motor_number) - 1
        before_fault = self.fault
        after_fault = list(self.fault)
        after_fault[index] = 0
        self.fault = tuple(after_fault)
        self.recoveries.append((motor_number, speed, torque))
        names = (
            "大拇指弯曲",
            "大拇指侧摆",
            "食指弯曲",
            "中指弯曲",
            "无名指弯曲",
            "小拇指弯曲",
        )
        return {
            "motor_number": motor_number,
            "motor_name": names[index],
            "position_before": self.position[index],
            "position_after": self.position[index],
            "fault_before": before_fault,
            "fault_after": self.fault,
        }

    def close(self):
        self.connected = False


class FakeRobot:
    def __init__(self):
        self.tcp = [500.0, 10.0, 300.0, 1.1, 2.2, 3.3, 0.0]
        self.joints = [0.0] * 7
        self.ik_targets = []
        self.servoj_open = False
        self.servoj_events = []
        self.servoj_targets = []
        self.movej_targets = []
        self.motion_stop_waits = 0
        self.stopped = False
        self.controller_job_warning = ""

    def servo_state(self):
        return 3

    def current_mode(self):
        return 1

    def tcp_position(self):
        return list(self.tcp)

    def joint_position(self):
        return list(self.joints)

    def open_servoj(self, vmax, amax, jmax):
        self.servoj_events.append("open")
        self.servoj_open = True

    def stop_servoj(self):
        self.servoj_events.append("stop")
        self.servoj_open = False

    def inverse_kinematics(self, target):
        self.ik_targets.append(list(target))
        return [0.0] * 7

    def forward_kinematics(self, joints):
        # A minimal deterministic wrist model is enough to verify that replay
        # generates a non-stale rotational delta action.
        target = list(self.tcp)
        target[3] += math.radians(float(joints[5]))
        return target

    def send_servoj(self, joints):
        self.servoj_events.append("send")
        self.joints = list(joints)
        self.servoj_targets.append(list(joints))

    def clear_movej_busy_if_stopped(self):
        return True

    def movej(self, joints, velocity, acc, dec):
        self.joints = list(joints)
        self.movej_targets.append(list(joints))

    def wait_until_motion_stopped(self, timeout, stop_event=None, stable_samples=3):
        self.motion_stop_waits += 1
        return stop_event is None or not stop_event.is_set()

    def stop_motion(self):
        self.stopped = True
        self.stop_servoj()


class TeleopEngineTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.store = TeleopConfigStore(Path(self.tempdir.name) / "teleop.json")
        self.store.data["teleop"].update(
            {
                "scale_xyz": [2.0, 2.0, 2.0],
                "max_delta_xyz_mm": [100.0, 100.0, 100.0],
                "max_tcp_speed_mm_s": 10000.0,
                "period_s": 0.01,
            }
        )
        self.store.data["replay"].update(
            {
                "relative_to_current": True,
                "servoj_period_s": 0.01,
                "mode_switch_settle_s": 0.0,
            }
        )
        self.master = FakeRoArm()
        self.hand = FakeO6()
        self.robot = FakeRobot()
        self.engine = TeleopEngine(self.store, self.master, self.hand)
        self.engine.attach_robot(self.robot)

    def tearDown(self):
        self.engine._follow_stop.set()
        thread = self.engine._follow_thread
        if thread is not None:
            thread.join(timeout=1.0)
        self.tempdir.cleanup()

    def test_xyz_delta_is_scaled_and_rpy_is_locked(self):
        self.engine.start_follow()
        self.master.value = feedback(x=110.0, y=195.0, z=302.0, t=3.1447)
        time.sleep(0.08)
        self.engine.stop_follow("test")

        target = self.robot.ik_targets[-1]
        self.assertAlmostEqual(target[0], 520.0, places=3)
        self.assertAlmostEqual(target[1], 0.0, places=3)
        self.assertAlmostEqual(target[2], 304.0, places=3)
        self.assertEqual(target[3:7], [1.1, 2.2, 3.3, 0.0])
        self.assertEqual(self.hand.targets[-1], (90, 99, 50, 50, 50, 50))
        self.assertTrue(self.master.released)
        self.assertTrue(self.master.held)

    def test_gello_xyz_j6_tcp_ik_mode_uses_fk_and_maps_tcp_c(self):
        gello = FakeGello()
        self.master.connected = False
        self.engine.set_master(gello)
        self.store.data["gello"]["control_mode"] = "tcp_6d"
        self.store.data["gello"]["startup_settle_s"] = 0.0
        self.store.data["robot"]["safety_max_command_speed_rad_s"] = 100.0
        self.store.data["robot"]["safety_max_tracking_error_rad"] = 10.0
        self.store.data["teleop"]["max_master_jump_mm"] = 1000.0
        self.store.data["teleop"]["max_tcp_speed_mm_s"] = 10000.0

        self.engine.start_follow()
        time.sleep(0.08)
        gello.value = GelloFeedback(
            time.monotonic(), (0.12, 0.0, 0.0, 0.0, 0.0, 0.10, 0.0)
        )
        deadline = time.monotonic() + 0.8
        while not self.robot.ik_targets and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(self.robot.ik_targets)
        target = self.robot.ik_targets[-1]
        self.assertNotEqual(target[:3], [500.0, 10.0, 300.0])
        self.assertGreater(
            sum(abs(value - reference) for value, reference in zip(target[3:6], [1.1, 2.2, 3.3])),
            1e-6,
        )
        self.assertTrue(self.robot.servoj_targets)
        self.assertAlmostEqual(self.robot.servoj_targets[-1][5], 0.0, places=6)
        self.assertTrue(
            np.isfinite(
                GelloKinematics(self.store.data["gello"]["kinematics_urdf"])
                .position(gello.value.arm_joints_rad)
            ).all()
        )
        self.engine.stop_follow("test")

    def test_gello_joint_mode_maps_relative_joint_targets(self):
        gello = FakeGello()
        self.master.connected = False
        self.engine.set_master(gello)
        self.store.data["gello"]["control_mode"] = "joint"
        self.store.data["gello"]["startup_settle_s"] = 0.0
        self.store.data["robot"]["safety_max_command_speed_rad_s"] = 100.0
        self.store.data["robot"]["safety_max_tracking_error_rad"] = 10.0

        self.engine.start_follow()
        time.sleep(0.08)
        gello.value = GelloFeedback(
            time.monotonic(), (0.10, -0.20, 0.05, 0.0, 0.0, 0.15, 0.0)
        )
        expected = [math.degrees(value) for value in gello.value.arm_joints_rad]
        deadline = time.monotonic() + 0.8
        while (
            not any(
                abs(target[0] - expected[0]) < 1e-6
                for target in self.robot.servoj_targets
            )
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)
        target = next(
            target
            for target in self.robot.servoj_targets
            if abs(target[0] - expected[0]) < 1e-6
        )
        for actual, wanted in zip(target[:6], expected):
            self.assertAlmostEqual(actual, wanted, places=6)
        self.assertEqual(target[6], 0.0)
        self.engine.stop_follow("test")

    def test_gello_follow_opens_servoj_before_first_command(self):
        gello = FakeGello()
        self.master.connected = False
        self.engine.set_master(gello)
        self.store.data["gello"]["control_mode"] = "joint"
        self.store.data["gello"]["startup_settle_s"] = 0.0
        self.store.data["robot"]["safety_max_command_speed_rad_s"] = 100.0
        self.store.data["robot"]["safety_max_tracking_error_rad"] = 10.0

        self.engine.start_follow()
        deadline = time.monotonic() + 0.8
        while "send" not in self.robot.servoj_events and time.monotonic() < deadline:
            time.sleep(0.005)

        self.assertIn("send", self.robot.servoj_events)
        self.assertLess(
            self.robot.servoj_events.index("open"),
            self.robot.servoj_events.index("send"),
        )
        self.engine.stop_follow("test")

    def test_inverse3_follow_updates_xyz_and_abc_in_native_radians(self):
        q_origin = (
            0.7071067811865476,
            0.0,
            0.0,
            0.7071067811865475,
        )
        q_current = (
            0.5938107868374524,
            -0.050734708264935056,
            -0.09920093636838305,
            0.7968525039405936,
        )
        inverse = FakeInverse3(inverse_feedback(orientation=q_origin))
        self.master.connected = False
        self.engine.set_master(inverse)
        self.store.data["teleop"].update(
            {
                "scale_rpy": [1.0, 1.0, 1.0],
                "max_delta_rpy_rad": [1.0, 1.0, 1.0],
                "max_tcp_angular_speed_rad_s": 100.0,
                "max_master_angular_jump_rad": 1.0,
            }
        )
        self.robot.tcp = [500.0, 10.0, 300.0, 3.10, -3.10, 3.05, 0.0]

        self.engine.start_follow()
        inverse.value = inverse_feedback(
            x=110.0,
            y=195.0,
            z=302.0,
            orientation=q_current,
        )
        expected = [
            520.0,
            0.0,
            304.0,
            -3.069890964451087,
            -2.8896455007655124,
            -2.938663011008991,
            0.0,
        ]
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if self.robot.ik_targets and all(
                math.isclose(actual, wanted, abs_tol=1e-6)
                for actual, wanted in zip(self.robot.ik_targets[-1], expected)
            ):
                break
            time.sleep(0.005)
        self.engine.stop_follow("test")

        for actual, wanted in zip(self.robot.ik_targets[-1], expected):
            self.assertAlmostEqual(actual, wanted, places=6)
        self.assertEqual(self.hand.targets, [])

    def test_inverse3_without_grip_cannot_start_six_d_follow(self):
        inverse = FakeInverse3(inverse_feedback(orientation=None))
        self.master.connected = False
        self.engine.set_master(inverse)

        with self.assertRaisesRegex(RuntimeError, "VerseGrip"):
            self.engine.start_follow()

        self.assertFalse(self.robot.servoj_open)
        self.assertEqual(self.engine.state, "idle")

    def test_joint_dataset_records_feedback_and_sent_absolute_targets_in_radians(self):
        self.store.data["dataset"]["recording_mode"] = "joint"
        self.store.data["gello"]["control_mode"] = "joint"
        self.master.master_type = "gello"
        self.engine._set_state("following")
        now = time.monotonic()
        actual = [0, 90, -90, 180, -180, 45]
        target = [10, 100, -80, 190, -170, 55, 0]
        self.engine._gello_telemetry = {
            "sent_count": 1, "feedback_at": now, "command_at": now,
            "tracking_error_deg": 10, "actual_deg": actual,
            "target_full_deg": target,
        }
        sample = self.engine.dataset_sample()
        self.assertEqual(sample["recording_mode"], "joint")
        self.assertEqual(len(sample["observation_state"]), 12)
        for result, degrees in zip(sample["observation_state"][:6], actual):
            self.assertAlmostEqual(result, math.radians(degrees))
        for result, degrees in zip(sample["action"][:6], target):
            self.assertAlmostEqual(result, math.radians(degrees))
        self.engine._set_state("idle")
        with self.assertRaisesRegex(RuntimeError, "已下发目标"):
            self.engine.dataset_sample()

    def test_dataset_tcp_rpy_uses_native_rad_and_shortest_delta(self):
        self.robot.tcp = [500.0, 10.0, 300.0, 3.10, -3.10, 0.25, 0.0]
        self.engine._set_dataset_target_tcp(
            [500.0, 10.0, 300.0, -3.10, 3.10, 0.35, 0.0]
        )
        self.engine._set_state("following")

        sample = self.engine.dataset_sample()

        self.assertEqual(sample["observation_state"][9:12], [3.10, -3.10, 0.25])
        expected_delta = [
            0.08318530717958605,
            -0.08318530717958694,
            0.1,
        ]
        for actual, wanted in zip(sample["action"][3:6], expected_delta):
            self.assertAlmostEqual(actual, wanted, places=9)
        self.engine._set_state("idle")

    def test_xyz_scale_change_rebases_without_target_jump(self):
        self.engine.start_follow()
        self.master.value = feedback(x=110.0)
        deadline = time.monotonic() + 0.5
        while (
            not self.robot.ik_targets
            or not math.isclose(self.robot.ik_targets[-1][0], 520.0, abs_tol=0.01)
        ) and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertAlmostEqual(self.robot.ik_targets[-1][0], 520.0, places=2)

        self.store.update_runtime({"teleop": {"scale_xyz": [4.0, 2.0, 2.0]}})
        time.sleep(0.04)
        self.assertAlmostEqual(self.robot.ik_targets[-1][0], 520.0, places=2)

        self.master.value = feedback(x=111.0)
        deadline = time.monotonic() + 0.5
        while not math.isclose(
            self.robot.ik_targets[-1][0], 524.0, abs_tol=0.01
        ) and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertAlmostEqual(self.robot.ik_targets[-1][0], 524.0, places=2)
        self.engine.stop_follow("test")

    def test_relative_limit_causes_fault_and_stops_servoj(self):
        self.store.data["teleop"]["max_delta_xyz_mm"] = [5.0, 5.0, 5.0]
        self.engine.start_follow()
        self.master.value = feedback(x=104.0)
        deadline = time.monotonic() + 1.0
        while self.engine.state != "fault" and time.monotonic() < deadline:
            time.sleep(0.01)

        self.assertEqual(self.engine.state, "fault")
        self.assertIn("X 相对位移", self.engine.last_error)
        self.assertFalse(self.robot.servoj_open)
        self.assertTrue(self.master.held)

    def test_software_estop_during_follow_start_cannot_publish_following(self):
        def open_then_estop(vmax, amax, jmax):
            self.robot.servoj_open = True
            self.engine.emergency_stop("启动阶段测试急停")

        self.robot.open_servoj = open_then_estop

        with self.assertRaisesRegex(RuntimeError, "启动已被软件紧急停止"):
            self.engine.start_follow()

        self.assertEqual(self.engine.state, "fault")
        self.assertFalse(self.robot.servoj_open)
        self.assertIsNone(self.engine._follow_thread)

    def test_shutdown_cancels_inflight_master_connection(self):
        connect_started = threading.Event()
        connect_released = threading.Event()

        class SlowMaster(FakeRoArm):
            def __init__(self):
                super().__init__()
                self.connected = False

            def connect(self):
                connect_started.set()
                connect_released.wait(1.0)
                self.connected = True
                return self.value

            def close(self, hold=True):
                del hold
                self.connected = False
                connect_released.set()

        slow_master = SlowMaster()
        engine = TeleopEngine(self.store, slow_master, self.hand)
        failures = []

        def connect_devices():
            try:
                engine.connect_devices()
            except Exception as exc:
                failures.append(str(exc))

        thread = threading.Thread(target=connect_devices)
        thread.start()
        self.assertTrue(connect_started.wait(0.5))
        engine.shutdown()
        thread.join(timeout=1.0)

        self.assertFalse(thread.is_alive())
        self.assertFalse(slow_master.connected)
        self.assertEqual(engine.state, "closed")
        self.assertTrue(any("退出" in failure for failure in failures))

    def test_shutdown_cancels_inflight_follow_start(self):
        servo_check_started = threading.Event()
        servo_check_released = threading.Event()
        failures = []

        def blocking_servo_state():
            servo_check_started.set()
            servo_check_released.wait(1.0)
            return 3

        self.robot.servo_state = blocking_servo_state

        def start_follow():
            try:
                self.engine.start_follow()
            except Exception as exc:
                failures.append(str(exc))

        thread = threading.Thread(target=start_follow)
        thread.start()
        self.assertTrue(servo_check_started.wait(0.5))
        self.engine.shutdown()
        servo_check_released.set()
        thread.join(timeout=1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(self.engine.state, "closed")
        self.assertFalse(self.robot.servoj_open)
        self.assertTrue(any("退出" in failure for failure in failures))

    def test_preset_persists_all_three_devices(self):
        self.robot.joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0]
        self.robot.tcp = [450.0, 20.0, 350.0, 1.0, 2.0, 3.0, 0.0]
        saved = self.engine.save_preset("A")
        loaded = TeleopConfigStore(self.store.path).data["presets"]["A"]

        self.assertEqual(saved["robot_joints_deg"], self.robot.joints)
        self.assertEqual(loaded["o6_position"], list(self.hand.position))
        self.assertEqual(len(loaded["roarm_joints_rad"]), 4)
        self.assertEqual(len(loaded["robot_tcp"]), 7)

    def test_preset_recall_moves_all_devices_and_finishes(self):
        self.robot.joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0]
        self.master.value = feedback(b=0.4, s=0.5, e=0.6, t=1.2)
        self.hand.position = (240, 230, 220, 210, 200, 190)
        self.engine.save_preset("B")
        self.robot.joints = [0.0] * 7
        self.master.value = feedback()
        self.hand.position = (253, 253, 253, 253, 253, 253)

        self.engine.recall_preset("B", resume_follow=False)
        self.engine._preset_thread.join(timeout=1.0)

        self.assertEqual(self.engine.state, "idle")
        self.assertEqual(self.robot.joints, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0])
        self.assertEqual(self.master.value.joints, (0.4, 0.5, 0.6, 1.2))
        self.assertEqual(self.hand.position, (240, 230, 220, 210, 200, 190))
        self.assertTrue(self.robot.stopped)
        self.assertEqual(self.robot.motion_stop_waits, 1)

    def test_preset_ends_movej_ownership_before_resuming_xyz_follow(self):
        self.engine.save_preset("A")
        self.engine.start_follow()

        def stopped_only_after_movej_is_ended(timeout, stop_event=None, stable_samples=3):
            self.robot.motion_stop_waits += 1
            return self.robot.stopped

        self.robot.wait_until_motion_stopped = stopped_only_after_movej_is_ended
        self.engine.recall_preset("A", resume_follow=True)
        self.engine._preset_thread.join(timeout=1.0)

        self.assertFalse(self.engine._preset_thread.is_alive())
        self.assertTrue(self.robot.stopped)
        self.assertEqual(self.engine.state, "following")
        self.engine.stop_follow("test")

    def test_idle_preset_recall_never_starts_xyz_follow(self):
        self.engine.save_preset("B")

        self.engine.recall_preset("B", resume_follow=True)
        self.engine._preset_thread.join(timeout=1.0)

        self.assertFalse(self.engine._preset_thread.is_alive())
        self.assertEqual(self.engine.state, "idle")
        self.assertFalse(self.robot.servoj_open)

    def test_preset_recall_preempts_xyz_follow_then_resumes_from_new_origin(self):
        self.robot.joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0]
        self.master.value = feedback(b=0.4, s=0.5, e=0.6, t=1.2)
        self.hand.position = (240, 230, 220, 210, 200, 190)
        self.engine.save_preset("A")
        self.robot.joints = [0.0] * 7
        self.master.value = feedback()
        self.hand.position = (253, 253, 253, 253, 253, 253)
        self.engine.start_follow()

        self.engine.recall_preset("A", resume_follow=True)
        self.engine._preset_thread.join(timeout=1.0)

        self.assertFalse(self.engine._preset_thread.is_alive())
        self.assertEqual(self.engine.state, "following")
        self.assertIn(
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0], self.robot.movej_targets
        )
        self.engine.stop_follow("test")

    def test_episode_sample_continues_during_preset_recall_and_follow_resumes(self):
        self.robot.joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0]
        self.robot.tcp = [450.0, 20.0, 350.0, 1.0, 2.0, 3.0, 0.0]
        self.master.value = feedback(b=0.4, s=0.5, e=0.6, t=1.2)
        self.hand.position = (240, 230, 220, 210, 200, 190)
        self.engine.save_preset("A")

        self.robot.joints = [0.0] * 7
        self.robot.tcp = [500.0, 20.0, 350.0, 1.0, 2.0, 3.0, 0.0]
        self.master.value = feedback()
        self.hand.position = (253, 253, 253, 253, 253, 253)
        self.engine.start_follow()

        movej_entered = threading.Event()
        release_movej = threading.Event()

        def blocking_movej(joints, velocity, acc, dec):
            movej_entered.set()
            release_movej.wait(0.5)
            self.robot.joints = list(joints)
            self.robot.movej_targets.append(list(joints))

        self.robot.movej = blocking_movej
        self.engine.recall_preset("A", resume_follow=True)
        self.assertTrue(movej_entered.wait(0.5))
        self.assertEqual(self.engine.state, "preset")

        sample = self.engine.dataset_sample()
        self.assertAlmostEqual(sample["action"][0], -0.05, places=6)
        self.assertEqual(len(sample["observation_state"]), 18)
        self.assertEqual(len(sample["action"]), 12)

        release_movej.set()
        self.engine._preset_thread.join(timeout=1.0)
        self.assertFalse(self.engine._preset_thread.is_alive())
        self.assertEqual(self.engine.state, "following")
        self.engine.stop_follow("test")

    def test_preset_state_rejects_a_second_xyz_follow_start(self):
        self.engine._set_state("preset")

        with self.assertRaisesRegex(RuntimeError, "恢复或轨迹重放"):
            self.engine.start_follow()

        self.assertEqual(self.engine.state, "preset")

    def test_stop_follow_timeout_never_publishes_idle(self):
        class StuckThread:
            def join(self, timeout=None):
                return None

            def is_alive(self):
                return True

        self.engine._follow_thread = StuckThread()
        self.engine._set_state("following")

        with self.assertRaisesRegex(TimeoutError, "未能在 2 秒内停止"):
            self.engine.stop_follow("test")

        self.assertEqual(self.engine.state, "fault")
        self.engine._follow_thread = None

    def test_cancelled_preset_worker_never_commands_motion(self):
        self.engine.save_preset("A")
        preset = self.store.data["presets"]["A"]
        self.robot.movej_targets.clear()
        self.engine._preset_stop.set()
        self.engine._set_state("fault", "测试急停")

        self.engine._recall_worker("A", preset, resume_follow=False)

        self.assertEqual(self.robot.movej_targets, [])
        self.assertEqual(self.engine.state, "fault")

    def test_master_free_mode_holds_slave_and_releases_master(self):
        self.engine.release_master_hold_slave()

        self.assertEqual(self.engine.state, "master_free")
        self.assertTrue(self.robot.stopped)
        self.assertTrue(self.master.released)
        self.assertEqual(self.hand.targets[-1], self.hand.position)

        self.engine.stop_follow("退出主臂自由模式")
        self.assertEqual(self.engine.state, "idle")
        self.assertTrue(self.master.held)

    def test_robot_disconnect_detaches_adapter_and_stops_follow(self):
        self.engine.start_follow()

        detached = self.engine.detach_robot(self.robot, "6001=-2, 7000=0")
        thread = self.engine._follow_thread
        if thread is not None:
            thread.join(timeout=0.5)

        self.assertTrue(detached)
        self.assertIsNone(self.engine.robot)
        self.assertEqual(self.engine.state, "fault")
        self.assertIn("CR5 连接断开", self.engine.last_error)
        self.assertTrue(self.master.held)

    def test_home_is_a_dedicated_persistent_preset(self):
        saved = self.engine.save_preset("HOME")
        self.assertIn("HOME", self.store.data["presets"])
        self.assertEqual(saved["robot_tcp"], self.robot.tcp)

    def test_home_returns_o6_control_to_m5_after_quick_action(self):
        self.engine.save_preset("HOME")
        self.engine.execute_o6_action("抓取")
        self.assertTrue(self.engine.o6_manual_override)

        self.engine.recall_preset("HOME", resume_follow=False)
        self.engine._preset_thread.join(timeout=1.0)

        self.assertFalse(self.engine._preset_thread.is_alive())
        self.assertEqual(self.engine.state, "idle")
        self.assertFalse(self.engine.o6_manual_override)
        self.engine.start_follow()
        self.master.value = feedback(t=3.1447)
        time.sleep(0.05)
        self.assertEqual(self.hand.targets[-1], (90, 99, 50, 50, 50, 50))

    def test_o6_motor_three_recovery_uses_its_profile_and_clears_fault_state(self):
        self.hand.fault = (0, 0, 1, 0, 0, 0)
        self.engine._set_state("fault", "O6 故障")

        result = self.engine.recover_o6_motor(3)

        self.assertEqual(self.hand.recoveries, [(3, 20, 30)])
        self.assertEqual(result["motor_name"], "食指弯曲")
        self.assertEqual(self.hand.fault, (0, 0, 0, 0, 0, 0))
        self.assertEqual(self.engine.state, "idle")

    def test_o6_recovery_refuses_overtemperature_fault(self):
        self.hand.fault = (0, 0, 2, 0, 0, 0)

        with self.assertRaisesRegex(RuntimeError, "温度过高"):
            self.engine.recover_o6_motor(3)

        self.assertEqual(self.hand.recoveries, [])

    def test_o6_recovery_cannot_overwrite_software_estop(self):
        entered = threading.Event()

        def wait_for_estop(
            motor_number, speed, torque, timeout=4.0, stop_event=None
        ):
            entered.set()
            while stop_event is not None and not stop_event.wait(0.005):
                pass
            raise RuntimeError("恢复已停止")

        self.hand.recover_motor = wait_for_estop
        errors = []

        def recover():
            try:
                self.engine.recover_o6_motor(3)
            except Exception as exc:
                errors.append(str(exc))

        thread = threading.Thread(target=recover)
        thread.start()
        self.assertTrue(entered.wait(0.5))
        self.engine.emergency_stop("测试急停")
        thread.join(timeout=0.5)

        self.assertFalse(thread.is_alive())
        self.assertTrue(errors)
        self.assertEqual(self.engine.state, "fault")

    def test_o6_quick_action_uses_configured_target_and_profile(self):
        target = self.engine.execute_o6_action("中指")

        self.assertEqual(target, (91, 132, 0, 250, 0, 0))
        self.assertEqual(self.hand.targets[-1], target)
        self.assertEqual(self.hand.profiles[-1], ((20,) * 6, (30,) * 6))
        self.assertEqual(self.engine.state, "idle")
        self.assertTrue(self.engine.o6_manual_override)

    def test_o6_quick_action_rejects_unknown_name(self):
        with self.assertRaisesRegex(ValueError, "未配置 O6 快捷动作"):
            self.engine.execute_o6_action("不存在")

    def test_software_estop_cancels_o6_quick_action(self):
        errors = []

        def wait_for_estop(target, tolerance=6, timeout=15.0, stop_event=None):
            while stop_event is not None and not stop_event.is_set():
                time.sleep(0.005)
            return False

        self.hand.wait_until_position = wait_for_estop

        def run_action():
            try:
                self.engine.execute_o6_action("抓取")
            except Exception as exc:
                errors.append(str(exc))

        thread = threading.Thread(target=run_action)
        thread.start()
        deadline = time.monotonic() + 0.5
        while not self.engine.o6_action_active and time.monotonic() < deadline:
            time.sleep(0.005)
        self.engine.emergency_stop("测试急停")
        thread.join(timeout=0.5)

        self.assertFalse(thread.is_alive())
        self.assertTrue(any("软件紧急停止" in message for message in errors))
        self.assertEqual(self.engine.state, "fault")

    def test_o6_action_can_run_during_xyz_follow_and_m5_stays_suppressed(self):
        self.engine.start_follow()
        target = self.engine.execute_o6_action("轻微抓取")
        self.master.value = feedback(x=102.0, t=3.1447)
        time.sleep(0.05)

        self.assertEqual(self.engine.state, "following")
        self.assertTrue(self.robot.servoj_open)
        self.assertEqual(self.hand.targets[-1], target)

        self.engine.enable_m5_o6_control()
        time.sleep(0.05)
        self.assertFalse(self.engine.o6_manual_override)
        self.assertEqual(self.hand.targets[-1], (90, 99, 50, 50, 50, 50))

    def test_joint_record_and_replay_synchronizes_cr5_and_o6(self):
        self.store.data["replay"]["sample_period_s"] = 0.01
        self.store.data["replay"]["max_joint_speed_deg_s"] = 10000.0
        first_hand = (200, 201, 202, 203, 204, 205)
        final_hand = (120, 121, 122, 123, 124, 125)
        self.hand.position = first_hand
        self.engine.start_joint_recording()
        self.robot.joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0]
        time.sleep(0.025)
        self.robot.joints = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 0.0]
        self.hand.position = final_hand
        time.sleep(0.025)
        path = self.engine.stop_joint_recording()

        payload = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(payload["version"], 3)
        self.assertGreaterEqual(len(payload["frames"]), 2)
        self.assertIn("o6_position", payload["frames"][0])
        recorded_final_hand = tuple(payload["frames"][-1]["o6_position"])
        self.assertEqual(recorded_final_hand, final_hand)
        self.assertTrue(self.engine.snapshot()["replay_available"])

        self.hand.position = (253, 253, 253, 253, 253, 253)
        self.hand.targets.clear()
        self.robot.joints = [0.0] * 7
        self.engine.start_joint_replay()
        self.engine._replay_thread.join(timeout=1.0)

        self.assertFalse(self.engine._replay_thread.is_alive())
        self.assertEqual(self.engine.state, "idle")
        self.assertTrue(self.robot.servoj_targets)
        self.assertEqual(self.hand.targets[-1], recorded_final_hand)
        self.assertTrue(self.engine.o6_manual_override)

    def test_three_replay_slots_use_independent_files(self):
        self.store.data["replay"]["max_joint_speed_deg_s"] = 10000.0
        self.engine._record_joint_indices = (6,)
        expected_paths = []
        for slot in (1, 2, 3):
            self.engine._record_frames = [
                {
                    "t": 0.0,
                    "robot_joints_deg": [0.0] * 7,
                    "o6_position": [220] * 6,
                    "roarm_joints_rad": [0.0] * 4,
                },
                {
                    "t": 0.05,
                    "robot_joints_deg": [0.0] * 5 + [float(slot), 0.0],
                    "o6_position": [220 - slot] * 6,
                    "roarm_joints_rad": [0.0] * 4,
                },
            ]
            expected_paths.append(self.engine._save_joint_recording(slot))

        self.assertEqual(expected_paths[0].name, "joint_replay.json")
        self.assertEqual(expected_paths[1].name, "joint_replay_2.json")
        self.assertEqual(expected_paths[2].name, "joint_replay_3.json")
        for slot in (1, 2, 3):
            frames, joint_indices = self.engine._load_joint_recording(slot)
            self.assertEqual(joint_indices, (6,))
            self.assertEqual(frames[-1]["robot_joints_deg"][5], float(slot))

        snapshot = self.engine.snapshot()
        self.assertTrue(all(item["available"] for item in snapshot["replay_slots"]))
        self.engine.start_joint_replay(slot=3)
        self.engine._replay_thread.join(timeout=1.0)

        self.assertFalse(self.engine._replay_thread.is_alive())
        self.assertEqual(round(self.robot.servoj_targets[-1][5]), 3)
        self.assertEqual(self.hand.targets[-1], (217,) * 6)

    def test_cancelled_replay_worker_never_sends_servoj(self):
        frames = [
            {
                "t": 0.0,
                "robot_joints_deg": [0.0] * 7,
                "o6_position": [200] * 6,
            },
            {
                "t": 0.05,
                "robot_joints_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                "o6_position": [190] * 6,
            },
        ]
        self.engine._replay_stop.set()
        self.engine._set_state("fault", "测试急停")

        self.engine._joint_replay_worker(frames, (6,), insert_mode=False)

        self.assertEqual(self.robot.servoj_targets, [])
        self.assertEqual(self.engine.state, "fault")

    def test_episode_insert_replay_keeps_sampling_and_freezes_unselected_joints(self):
        self.store.data["replay"]["robot_joint_indices"] = [6]
        self.store.data["replay"]["max_joint_speed_deg_s"] = 10000.0
        self.engine._record_joint_indices = (6,)
        self.engine._record_frames = [
            {
                "t": 0.0,
                "robot_joints_deg": [90.0, 90.0, 90.0, 90.0, 90.0, 15.0, 0.0],
                "o6_position": [230] * 6,
                "roarm_joints_rad": [0.0] * 4,
            },
            {
                "t": 0.15,
                "robot_joints_deg": [80.0, 80.0, 80.0, 80.0, 80.0, 25.0, 0.0],
                "o6_position": [180] * 6,
                "roarm_joints_rad": [0.0] * 4,
            },
        ]
        self.engine._save_joint_recording()
        self.robot.joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0]
        self.engine._set_dataset_target_tcp(self.robot.tcp_position())
        self.engine._set_state("following")

        self.engine.start_joint_replay(insert_into_episode=True)
        samples = []
        deadline = time.monotonic() + 1.0
        while self.engine.replaying and time.monotonic() < deadline:
            samples.append(self.engine.dataset_sample())
            time.sleep(0.005)
        self.engine._replay_thread.join(timeout=1.0)

        self.assertFalse(self.engine._replay_thread.is_alive())
        self.assertEqual(self.engine.state, "following")
        self.assertTrue(samples)
        self.assertTrue(
            all(len(sample["observation_state"]) == 18 for sample in samples)
        )
        self.assertTrue(all(len(sample["action"]) == 12 for sample in samples))
        self.assertTrue(
            any(
                math.isclose(sample["action"][3], math.radians(6.0), abs_tol=1e-6)
                or math.isclose(
                    sample["action"][3], math.radians(16.0), abs_tol=1e-6
                )
                for sample in samples
            )
        )
        self.assertTrue(
            any(
                math.isclose(
                    sample["observation_state"][0], math.radians(1.0), abs_tol=1e-6
                )
                for sample in samples
            )
        )
        self.assertTrue(
            all(
                math.isclose(sample["observation_state"][6], 0.5, abs_tol=1e-6)
                for sample in samples
            )
        )
        replay_targets = [
            target
            for target in self.robot.servoj_targets
            if round(target[5]) in (6, 16)
        ]
        self.assertEqual(round(replay_targets[0][5]), 6)
        self.assertEqual(round(replay_targets[-1][5]), 16)
        self.assertTrue(
            all(target[:5] == [1.0, 2.0, 3.0, 4.0, 5.0] for target in replay_targets[:2])
        )
        self.engine.stop_follow("test")

    def test_normal_replay_preempts_xyz_follow_then_resumes(self):
        self.store.data["replay"]["max_joint_speed_deg_s"] = 10000.0
        self.engine._record_joint_indices = (6,)
        self.engine._record_frames = [
            {
                "t": 0.0,
                "robot_joints_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0],
                "o6_position": [220] * 6,
                "roarm_joints_rad": [0.0] * 4,
            },
            {
                "t": 0.05,
                "robot_joints_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 0.0],
                "o6_position": [180] * 6,
                "roarm_joints_rad": [0.0] * 4,
            },
        ]
        self.engine._save_joint_recording()
        self.engine.start_follow()

        self.engine.start_joint_replay()
        self.engine._replay_thread.join(timeout=1.0)

        self.assertFalse(self.engine._replay_thread.is_alive())
        self.assertEqual(self.engine.state, "following")
        self.assertEqual(self.hand.position, (180,) * 6)
        self.assertTrue(
            any(round(target[5]) == 10 for target in self.robot.servoj_targets)
        )
        self.engine.stop_follow("test")

    def test_relative_replay_from_preset_does_not_jump_to_recorded_origin(self):
        self.store.data["replay"]["max_joint_speed_deg_s"] = 10000.0
        self.engine._record_joint_indices = (6,)
        self.engine._record_frames = [
            {
                "t": 0.0,
                "robot_joints_deg": [2.17, -7.75, -7.64, 30.58, -1.75, -173.40, 0.0],
                "o6_position": [220] * 6,
                "roarm_joints_rad": [0.0] * 4,
            },
            {
                "t": 0.05,
                "robot_joints_deg": [2.17, -7.75, -7.64, 30.58, -1.75, 9.10, 0.0],
                "o6_position": [180] * 6,
                "roarm_joints_rad": [0.0] * 4,
            },
        ]
        self.engine._save_joint_recording()
        preset_joints = [2.96, -16.54, -22.42, 97.59, -4.59, -5.75, 0.0]
        self.robot.joints = list(preset_joints)

        self.engine.start_joint_replay()
        self.engine._replay_thread.join(timeout=1.0)

        self.assertFalse(self.engine._replay_thread.is_alive())
        self.assertEqual(self.engine.state, "idle")
        self.assertEqual(self.robot.movej_targets, [])
        self.assertAlmostEqual(self.robot.servoj_targets[0][5], -5.75, places=3)
        self.assertAlmostEqual(
            self.robot.servoj_targets[-1][5],
            -5.75 + (9.10 - (-173.40)),
            places=3,
        )
        self.assertTrue(
            all(target[:5] == preset_joints[:5] for target in self.robot.servoj_targets)
        )

    def test_old_joint_recording_without_o6_is_rejected(self):
        path = self.engine._replay_path()
        path.write_text(
            json.dumps(
                {
                    "version": 1,
                    "frames": [
                        {"t": 0.0, "robot_joints_deg": [0.0] * 7},
                        {"t": 0.1, "robot_joints_deg": [0.0] * 7},
                    ],
                }
            ),
            encoding="utf-8",
        )

        with self.assertRaisesRegex(ValueError, "不包含 O6"):
            self.engine._load_joint_recording()
        snapshot = self.engine.snapshot()
        self.assertFalse(snapshot["replay_available"])
        self.assertIn("缺少 O6", snapshot["replay_file_status"])

        self.engine._record_frames = [
            {
                "t": 0.0,
                "robot_joints_deg": [0.0] * 7,
                "o6_position": [200] * 6,
                "roarm_joints_rad": [0.0] * 4,
            },
            {
                "t": 0.1,
                "robot_joints_deg": [0.1] * 7,
                "o6_position": [150] * 6,
                "roarm_joints_rad": [0.0] * 4,
            },
        ]
        self.engine._save_joint_recording()
        backups = list(path.parent.glob("joint_replay.v1-backup-*.json"))
        self.assertEqual(len(backups), 1)
        self.assertEqual(json.loads(backups[0].read_text())["version"], 1)


if __name__ == "__main__":
    unittest.main()

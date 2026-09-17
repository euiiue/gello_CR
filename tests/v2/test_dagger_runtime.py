"""Exercise DAgger ownership with the real engine and simulated devices."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from teleop_runtime import TeleopConfigStore, TeleopEngine
from test_teleop_runtime import FakeO6, FakeRoArm, FakeRobot


@pytest.fixture
def engine(tmp_path):
    store = TeleopConfigStore(tmp_path / "teleop.json")
    result = TeleopEngine(store, FakeRoArm(), FakeO6())
    result.attach_robot(FakeRobot())
    yield result
    result.stop_follow("test finished")


def controller():
    started = threading.Event()

    def run(*, stop_event):
        started.set()
        assert stop_event.wait(2)

    return SimpleNamespace(
        robot=SimpleNamespace(start_control=Mock()),
        run=run,
        request_stop=Mock(),
        stop=Mock(),
        snapshot=lambda: {"root": "/tmp/raw", "expert_frames": 12},
        started=started,
    )


def test_dagger_reserves_one_motion_producer_and_finalizes_on_stop(engine):
    first = controller()
    engine.start_external_control(first, engine._stop_generation)
    assert first.started.wait(1) and engine.state == "dagger"
    producer = engine._follow_thread
    engine.start_follow()  # Existing start is idempotent while its slot is occupied.
    assert engine._follow_thread is producer
    second = controller()
    with pytest.raises(RuntimeError):
        engine.start_external_control(second, engine._stop_generation)
    second.robot.start_control.assert_not_called()
    engine.stop_follow("stop DAgger")
    first.request_stop.assert_called()
    first.stop.assert_called_once()
    assert not producer.is_alive() and engine.external_control is None
    assert engine.snapshot()["dagger"]["expert_frames"] == 12


def test_estop_generation_cancels_late_dagger_start_before_servoj(engine):
    generation = engine._stop_generation
    engine.emergency_stop("cancel startup")
    pending = controller()
    with pytest.raises(RuntimeError):
        engine.start_external_control(pending, generation)
    pending.robot.start_control.assert_not_called()


def test_home_stops_dagger_and_finalizes_before_starting_home(engine):
    engine.store.data["presets"]["HOME"] = {"robot_joints_deg": [1.0] * 6 + [0.0]}
    running = controller()
    engine.start_external_control(running, engine._stop_generation)
    assert running.started.wait(1)
    original_movej = engine.robot.movej

    def home_movej(*args):
        assert not engine._follow_thread.is_alive()
        running.stop.assert_called_once()
        original_movej(*args)

    engine.robot.movej = home_movej
    ended = Mock()
    engine.return_home(ended)
    assert engine.external_control is None
    assert engine.robot.joints == [1.0] * 6 + [0.0]
    assert engine.o6.position == tuple(
        engine.store.data["o6"]["actions"][engine.store.data["o6"]["open_action"]]
    )
    ended.assert_called_once()

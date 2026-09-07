
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from gello_cr.bootstrap.operator_app import build_operator_application
from gello_cr.core.state_machine import WorkflowState


class Teleop:
    state = "idle"
    last_error = ""

    def snapshot(self):
        return {"state": self.state, "last_error": self.last_error}

    def start_follow(self):
        pass

    def stop_follow(self, reason):
        pass

    def emergency_stop(self, reason):
        pass


class Recorder:
    def close(self):
        pass

    def snapshot(self):
        return {
            "session_active": False,
            "episode_active": False,
            "buffered_frames": 0,
            "saved_episodes": 0,
            "error": "",
            "quality": {"needs_review": False},
        }

    def stop_episode(self):
        pass


class Camera:
    connected = False
    error = ""

    def connect(self, timeout=5.0):
        raise AssertionError("camera must not auto-connect")

    def latest(self):
        return None

    def close(self):
        pass


class SampleSource:
    def clear_wrist(self):
        pass

    def clear_base(self):
        pass


class Lifecycle:
    def callbacks(self):
        from gello_cr.app import RuntimeLifecycleCallbacks
        return RuntimeLifecycleCallbacks(
            connect=lambda: None,
            disconnect=lambda: None,
            power_on=lambda: None,
            power_off=lambda: None,
            reset_fault=lambda: None,
            reset_estop=lambda: None,
        )


@dataclass
class Runtime:
    teleop_engine: object
    recorder: object
    wrist_camera: object
    base_camera: object
    sample_source: object
    cr3a_lifecycle: object
    store: object

    def close(self):
        pass

    @property
    def lifecycle(self):
        return self.cr3a_lifecycle.callbacks()


class Store:
    data = {
        "master": {"type": "gello"},
        "dataset": {
            "base_roi_norm": [0.1, 0.1, 0.9, 0.9],
            "task": "task",
            "root": "/tmp/data",
        },
    }


class Factory:
    def __init__(self):
        self.build_calls = 0
        self.runtime = Runtime(
            teleop_engine=Teleop(),
            recorder=Recorder(),
            wrist_camera=Camera(),
            base_camera=Camera(),
            sample_source=SampleSource(),
            cr3a_lifecycle=Lifecycle(),
            store=Store(),
        )

    def build(self):
        self.build_calls += 1
        return self.runtime


def test_operator_application_build_starts_offline(tmp_path) -> None:
    factory = Factory()

    app = build_operator_application(tmp_path, factory=factory)

    assert factory.build_calls == 1
    assert app.backend.service.state is WorkflowState.OFFLINE
    assert not app.cameras.running
    app.close()


def test_operator_application_build_does_not_connect_cameras(tmp_path) -> None:
    factory = Factory()

    app = build_operator_application(tmp_path, factory=factory)

    assert not factory.runtime.wrist_camera.connected
    assert not factory.runtime.base_camera.connected
    app.close()


def test_operator_application_close_is_idempotent(tmp_path) -> None:
    app = build_operator_application(tmp_path, factory=Factory())

    app.close()
    app.close()


def test_operator_application_retains_runtime_object(tmp_path) -> None:
    factory = Factory()
    app = build_operator_application(tmp_path, factory=factory)

    assert app.runtime is factory.runtime
    app.close()

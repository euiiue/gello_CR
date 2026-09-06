
from __future__ import annotations

from pathlib import Path

from gello_cr.bootstrap.runtime_factory import (
    ConcreteRuntimeFactory,
    RuntimeConstructors,
    RuntimeFactoryPaths,
)


CFG = {
    'master': {'type': 'gello'},
    'robot': {
        'ip': '192.168.1.245',
        'command_port': 6001,
        'servo_port': 7000,
        'robot_num': 1,
        'movej_velocity': 10.0,
        'movej_acc': 20.0,
        'movej_dec': 20.0,
        'movej_period_s': 0.1,
        'movej_low_latency': False,
        'servoj_vmax': 80.0,
        'servoj_amax': 200.0,
        'servoj_jmax': 150.0,
    },
    'roarm': {'port': 'ro', 'baudrate': 1, 'feedback_hz': 20.0},
    'inverse3': {
        'uri': 'ws://x', 'device_id': '', 'grip_device_id': '',
        'feedback_hz': 20.0, 'basis': 'XYZ',
    },
    'gello': {
        'port': 'ge', 'software_root': '/tmp/gello',
        'joint_ids': [1,2,3,4,5,6], 'joint_offsets': [0]*6,
        'joint_signs': [1]*6, 'gripper_config': [7, 1, 2],
        'baudrate': 57600, 'feedback_hz': 100.0,
        'feedback_timeout_s': 0.5,
    },
    'o6': {'port': 'o6', 'hand_id': 39, 'baudrate': 115200},
    'dataset': {
        'worker_python': '/tmp/python', 'repo_prefix': 'local/test',
        'fps': 20, 'image_size': [224,224],
        'wrist_camera_serial': 'wrist', 'base_camera_serial': 'base',
        'camera_max_age_s': 0.5, 'camera_max_skew_s': 0.1,
    },
}


class Store:
    def __init__(self, path):
        self.path = path
        self.data = CFG


class Passive:
    connected = False
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class Engine:
    def __init__(self, store, master, o6):
        self.store = store
        self.roarm = master
        self.o6 = o6
        self.events = type('Events', (), {'put': lambda self, value: None})()
        self.attached = []
    def dataset_sample(self): return {'quality': {}}
    def attach_robot(self, session): self.attached.append(session)
    def detach_robot(self, session, reason): return True


class Recorder(Passive):
    def snapshot(self): return {}


class Camera(Passive):
    pass


class Cr3a(Passive):
    connected = False


def constructors():
    return RuntimeConstructors(
        TeleopConfigStore=Store,
        RoArmSerialController=Passive,
        Inverse3Controller=Passive,
        GelloController=Passive,
        O6Controller=Passive,
        TeleopEngine=Engine,
        LeRobotEpisodeRecorder=Recorder,
        Cr3aDevice=Cr3a,
        RealSenseRgbDevice=Camera,
    )


def test_factory_builds_gello_runtime_without_connecting_hardware(tmp_path) -> None:
    paths = RuntimeFactoryPaths(
        repo_root=tmp_path,
        config_path=tmp_path / 'config.json',
        linker_hand_sdk_root=tmp_path / 'hand',
        nrc_sdk_root=None,
    )
    runtime = ConcreteRuntimeFactory(
        paths,
        constructors=constructors(),
    ).build()

    assert runtime.master_controller is runtime.gello_controller
    assert runtime.teleop_engine.attached == []
    assert runtime.cr3a_lifecycle.device is None
    assert runtime.cr3a_lifecycle.session is None
    assert runtime.wrist_camera.args
    assert runtime.base_camera.args


def test_factory_does_not_call_connect_power_or_start_by_source() -> None:
    source = Path(
        'src/gello_cr/bootstrap/runtime_factory.py'
    ).read_text(encoding='utf-8')
    build = source.split('    def build(self) -> ConcreteRuntime:', 1)[1]
    build = build.split('\n        return ConcreteRuntime(', 1)[0]

    for forbidden in (
        '.connect(', '.power_on(', '.start_follow(', '.start_episode('
    ):
        assert forbidden not in build


def test_factory_selects_master_from_config(tmp_path) -> None:
    paths = RuntimeFactoryPaths(
        repo_root=tmp_path,
        config_path=tmp_path / 'config.json',
        linker_hand_sdk_root=tmp_path / 'hand',
        nrc_sdk_root=None,
    )
    runtime = ConcreteRuntimeFactory(
        paths,
        constructors=constructors(),
    ).build()
    assert runtime.master_controller is runtime.gello_controller


def test_runtime_exposes_application_lifecycle_callbacks(tmp_path) -> None:
    paths = RuntimeFactoryPaths(
        repo_root=tmp_path,
        config_path=tmp_path / 'config.json',
        linker_hand_sdk_root=tmp_path / 'hand',
        nrc_sdk_root=None,
    )
    runtime = ConcreteRuntimeFactory(
        paths,
        constructors=constructors(),
    ).build()
    callbacks = runtime.lifecycle

    assert callable(callbacks.connect)
    assert callable(callbacks.disconnect)
    assert callable(callbacks.power_on)
    assert callable(callbacks.power_off)
    assert callable(callbacks.reset_fault)
    assert callable(callbacks.reset_estop)

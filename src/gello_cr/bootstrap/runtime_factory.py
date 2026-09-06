
"""Concrete construction of the current GELLO/CR3A/O6 collection runtime.

Construction is intentionally side-effect free with respect to hardware.  The
factory instantiates controller/device objects but never calls connect(),
power_on(), start_follow(), camera start, or recorder start_episode().
"""

from __future__ import annotations

import importlib
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from gello_cr.app import RuntimeLifecycleCallbacks
from gello_cr.devices.cr3a import Cr3aConfig, Cr3aDevice
from gello_cr.devices.realsense import RealSenseRgbConfig, RealSenseRgbDevice

from .sample_source import RecordingSampleSource

EventCallback = Callable[[str, str], None]


@dataclass(frozen=True, slots=True)
class RuntimeFactoryPaths:
    repo_root: Path
    config_path: Path
    linker_hand_sdk_root: Path
    nrc_sdk_root: Path | None

    @classmethod
    def from_repo(
        cls,
        repo_root: str | Path,
        *,
        config_path: str | Path | None = None,
    ) -> 'RuntimeFactoryPaths':
        root = Path(repo_root).expanduser().resolve()
        config = (
            Path(config_path).expanduser().resolve()
            if config_path is not None
            else root / 'config' / 'roarm_cr5_teleop.json'
        )
        nrc_root = root / 'TESTRobot_INEXBOT'
        return cls(
            repo_root=root,
            config_path=config,
            linker_hand_sdk_root=(
                root
                / 'TESTHand_LINKERBOT'
                / 'linker_hand_python_sdk'
                / 'LinkerHand'
            ),
            nrc_sdk_root=nrc_root if nrc_root.is_dir() else None,
        )


@dataclass(frozen=True, slots=True)
class RuntimeConstructors:
    TeleopConfigStore: Any
    RoArmSerialController: Any
    Inverse3Controller: Any
    GelloController: Any
    O6Controller: Any
    TeleopEngine: Any
    LeRobotEpisodeRecorder: Any
    Cr3aDevice: Any = Cr3aDevice
    RealSenseRgbDevice: Any = RealSenseRgbDevice

    @classmethod
    def load_defaults(cls) -> 'RuntimeConstructors':
        teleop = importlib.import_module('teleop_runtime')
        recorder = importlib.import_module('lerobot_recorder')
        return cls(
            TeleopConfigStore=teleop.TeleopConfigStore,
            RoArmSerialController=teleop.RoArmSerialController,
            Inverse3Controller=teleop.Inverse3Controller,
            GelloController=teleop.GelloController,
            O6Controller=teleop.O6Controller,
            TeleopEngine=teleop.TeleopEngine,
            LeRobotEpisodeRecorder=recorder.LeRobotEpisodeRecorder,
        )


class Cr3aLifecycle:
    """Explicit CR3A lifecycle callbacks used by ApplicationService."""

    def __init__(
        self,
        *,
        store: Any,
        teleop_engine: Any,
        paths: RuntimeFactoryPaths,
        device_constructor: Any = Cr3aDevice,
        event_callback: EventCallback | None = None,
    ) -> None:
        self._store = store
        self._teleop_engine = teleop_engine
        self._paths = paths
        self._device_constructor = device_constructor
        self._event_callback = event_callback
        self.device: Any | None = None
        self.session: Any | None = None

    def _event(self, level: str, message: str) -> None:
        if self._event_callback is not None:
            self._event_callback(level, message)

    def _robot_config(self) -> Cr3aConfig:
        cfg = self._store.data['robot']
        return Cr3aConfig(
            ip=str(cfg['ip']),
            command_port=int(cfg.get('command_port', 6001)),
            servo_port=int(cfg.get('servo_port', 7000)),
            robot_num=int(cfg.get('robot_num', 1)),
            motion_mode='servoj',
            movej_velocity=float(cfg['movej_velocity']),
            movej_acc=float(cfg['movej_acc']),
            movej_dec=float(cfg['movej_dec']),
            movej_period_s=float(cfg['movej_period_s']),
            movej_low_latency=bool(cfg['movej_low_latency']),
            servoj_vmax=float(cfg['servoj_vmax']),
            servoj_amax=float(cfg['servoj_amax']),
            servoj_jmax=float(cfg['servoj_jmax']),
            sdk_root=(
                str(self._paths.nrc_sdk_root)
                if self._paths.nrc_sdk_root is not None
                else None
            ),
        )

    def connect(self) -> Any:
        if self.device is not None and getattr(self.device, 'connected', False):
            return self.session

        device = self._device_constructor(
            self._robot_config(),
            event_callback=self._event_callback,
        )
        try:
            device.connect(timeout=10.0)
            session = device.session
            if session is None:
                raise RuntimeError('CR3A connected without NRC session')
            self._teleop_engine.attach_robot(session)
        except Exception:
            try:
                device.close()
            finally:
                pass
            raise

        self.device = device
        self.session = session
        self._event('info', 'CR3A 已连接；未上使能，未启动 ServoJ')
        return session

    def _require_session(self) -> Any:
        session = self.session
        if session is None:
            raise RuntimeError('CR3A 尚未连接')
        return session

    def power_on(self) -> Any:
        session = self._require_session()
        transition = session.power_on()
        state = int(session.servo_state())
        if state != 3:
            raise RuntimeError(f'CR3A 上使能确认失败，servo_state={state}')
        return transition

    def power_off(self) -> Any:
        session = self._require_session()
        self._teleop_engine.emergency_stop('下使能前停止当前动作')
        transition = session.power_off()
        state = int(session.servo_state())
        if state not in (0, 1):
            raise RuntimeError(f'CR3A 下使能确认失败，servo_state={state}')
        return transition

    def _reset(self, label: str) -> Any:
        session = self._require_session()
        transition = session.power_on()
        state = int(session.servo_state())
        if state != 3:
            raise RuntimeError(f'{label}失败，servo_state={state}')
        self._teleop_engine.acknowledge_fault()
        self._event('info', f'{label}完成；保持 idle，不自动恢复跟随')
        return transition

    def reset_fault(self) -> Any:
        return self._reset('FAULT 复位')

    def reset_estop(self) -> Any:
        return self._reset('ESTOP 复位')

    def disconnect(self) -> bool:
        device = self.device
        session = self.session
        if session is not None:
            self._teleop_engine.detach_robot(
                session,
                'ApplicationService 断开 CR3A',
            )
        if device is not None:
            device.close()
        self.device = None
        self.session = None
        self._event('info', 'CR3A 已断开；不会自动重连')
        return True

    def callbacks(self) -> RuntimeLifecycleCallbacks:
        return RuntimeLifecycleCallbacks(
            connect=self.connect,
            disconnect=self.disconnect,
            power_on=self.power_on,
            power_off=self.power_off,
            reset_fault=self.reset_fault,
            reset_estop=self.reset_estop,
        )


@dataclass(slots=True)
class ConcreteRuntime:
    paths: RuntimeFactoryPaths
    store: Any
    roarm_controller: Any
    inverse3_controller: Any
    gello_controller: Any
    master_controller: Any
    o6_controller: Any
    teleop_engine: Any
    sample_source: RecordingSampleSource
    recorder: Any
    wrist_camera: Any
    base_camera: Any
    cr3a_lifecycle: Cr3aLifecycle

    @property
    def lifecycle(self) -> RuntimeLifecycleCallbacks:
        return self.cr3a_lifecycle.callbacks()


class ConcreteRuntimeFactory:
    """Build current production runtime objects without opening hardware."""

    def __init__(
        self,
        paths: RuntimeFactoryPaths,
        *,
        constructors: RuntimeConstructors | None = None,
        event_callback: EventCallback | None = None,
    ) -> None:
        self.paths = paths
        self.constructors = constructors or RuntimeConstructors.load_defaults()
        self.event_callback = event_callback

    def build(self) -> ConcreteRuntime:
        c = self.constructors
        store = c.TeleopConfigStore(self.paths.config_path)
        cfg = store.data

        roarm = c.RoArmSerialController(
            cfg['roarm']['port'],
            cfg['roarm']['baudrate'],
            cfg['roarm']['feedback_hz'],
        )
        inverse3 = c.Inverse3Controller(
            cfg['inverse3']['uri'],
            cfg['inverse3']['device_id'],
            cfg['inverse3']['grip_device_id'],
            cfg['inverse3']['feedback_hz'],
            basis=cfg['inverse3']['basis'],
        )
        gello_cfg = cfg['gello']
        gello = c.GelloController(
            port=gello_cfg['port'],
            software_root=gello_cfg['software_root'],
            joint_ids=gello_cfg['joint_ids'],
            joint_offsets=gello_cfg['joint_offsets'],
            joint_signs=gello_cfg['joint_signs'],
            gripper_config=gello_cfg['gripper_config'],
            baudrate=gello_cfg['baudrate'],
            feedback_hz=gello_cfg['feedback_hz'],
            feedback_timeout_s=gello_cfg['feedback_timeout_s'],
        )

        master_type = str(cfg['master']['type'])
        masters = {
            'roarm': roarm,
            'inverse3': inverse3,
            'gello': gello,
        }
        if master_type not in masters:
            raise ValueError(f'unsupported master type: {master_type}')
        master = masters[master_type]

        o6 = c.O6Controller(
            cfg['o6']['port'],
            self.paths.linker_hand_sdk_root,
            cfg['o6']['hand_id'],
            cfg['o6']['baudrate'],
        )
        engine = c.TeleopEngine(store, master, o6)

        def runtime_event(level: str, message: str) -> None:
            engine.events.put((level, message))
            if self.event_callback is not None:
                self.event_callback(level, message)

        dataset_cfg = cfg['dataset']
        sample_source = RecordingSampleSource(engine, dataset_cfg)
        recorder = c.LeRobotEpisodeRecorder(
            sample_provider=sample_source,
            event_callback=runtime_event,
            worker_python=dataset_cfg['worker_python'],
            repo_prefix=dataset_cfg['repo_prefix'],
            fps=int(dataset_cfg['fps']),
            image_size=tuple(dataset_cfg['image_size']),
        )

        wrist_camera = c.RealSenseRgbDevice(
            RealSenseRgbConfig(
                serial=str(dataset_cfg['wrist_camera_serial']),
                width=640,
                height=480,
                fps=30,
            )
        )
        base_camera = c.RealSenseRgbDevice(
            RealSenseRgbConfig(
                serial=str(dataset_cfg['base_camera_serial']),
                width=640,
                height=480,
                fps=30,
            )
        )

        lifecycle = Cr3aLifecycle(
            store=store,
            teleop_engine=engine,
            paths=self.paths,
            device_constructor=c.Cr3aDevice,
            event_callback=runtime_event,
        )

        return ConcreteRuntime(
            paths=self.paths,
            store=store,
            roarm_controller=roarm,
            inverse3_controller=inverse3,
            gello_controller=gello,
            master_controller=master,
            o6_controller=o6,
            teleop_engine=engine,
            sample_source=sample_source,
            recorder=recorder,
            wrist_camera=wrist_camera,
            base_camera=base_camera,
            cr3a_lifecycle=lifecycle,
        )

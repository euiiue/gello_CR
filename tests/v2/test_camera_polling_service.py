
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import pytest

from gello_cr.bootstrap.camera_service import CameraPollingService


@dataclass
class Snapshot:
    image_rgb: np.ndarray
    timestamp: float


class Camera:
    def __init__(self, image, *, fail=False):
        self.image = image
        self.fail = fail
        self.connected = False
        self.error = ""
        self.closed = 0
        self.timestamp = time.monotonic()

    def connect(self, timeout=5.0):
        if self.fail:
            raise RuntimeError("camera connect failed")
        self.connected = True
        self.timestamp = time.monotonic()
        return Snapshot(self.image, self.timestamp)

    def latest(self):
        if not self.connected:
            return None
        return Snapshot(self.image, self.timestamp)

    def close(self):
        self.connected = False
        self.closed += 1


class Source:
    def __init__(self):
        self.wrist = None
        self.base = None
        self.roi = None
        self.cleared_wrist = 0
        self.cleared_base = 0

    def update_wrist(self, image, timestamp):
        self.wrist = (image, timestamp)

    def update_base(self, image, roi, timestamp):
        self.base = (image, timestamp)
        self.roi = roi

    def clear_wrist(self):
        self.wrist = None
        self.cleared_wrist += 1

    def clear_base(self):
        self.base = None
        self.roi = None
        self.cleared_base += 1


def _service(*, fail_base=False):
    wrist_image = np.zeros((4, 6, 3), dtype=np.uint8)
    base_image = np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3)
    wrist = Camera(wrist_image)
    base = Camera(base_image, fail=fail_base)
    source = Source()
    service = CameraPollingService(
        wrist_camera=wrist,
        base_camera=base,
        sample_source=source,
        base_roi_norm=(0.25, 0.25, 0.75, 0.75),
        poll_hz=50.0,
    )
    return service, wrist, base, source


def test_camera_service_constructor_does_not_connect() -> None:
    service, wrist, base, source = _service()

    assert not service.running
    assert not wrist.connected
    assert not base.connected
    assert source.wrist is None
    assert source.base is None


def test_explicit_start_connects_both_and_publishes_roi() -> None:
    service, wrist, base, source = _service()

    snapshot = service.start()

    assert snapshot.wrist_connected
    assert snapshot.base_connected
    assert source.wrist is not None
    assert source.base is not None
    assert source.roi.shape[0] > 0
    assert source.roi.shape[1] > 0
    service.stop()


def test_failed_second_camera_rolls_back_first_camera() -> None:
    service, wrist, base, source = _service(fail_base=True)

    with pytest.raises(RuntimeError, match="camera connect failed"):
        service.start()

    assert not wrist.connected
    assert not base.connected
    assert wrist.closed >= 1
    assert base.closed >= 1
    assert source.wrist is None
    assert source.base is None


def test_stop_closes_both_and_clears_sample_source() -> None:
    service, wrist, base, source = _service()
    service.start()

    service.stop()
    service.stop()

    assert not service.running
    assert not wrist.connected
    assert not base.connected
    assert source.wrist is None
    assert source.base is None


def test_camera_worker_surfaces_device_error() -> None:
    service, wrist, base, source = _service()
    service.start()

    base.error = "base stream failed"
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and not service.error:
        time.sleep(0.01)

    assert "base stream failed" in service.error
    service.stop()

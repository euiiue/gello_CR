
from __future__ import annotations

import time

import pytest

from gello_cr.bootstrap.sample_source import RecordingSampleSource


class Engine:
    def dataset_sample(self):
        return {'quality': {'existing': 1}}


def _cfg():
    return {
        'camera_max_age_s': 0.5,
        'camera_max_skew_s': 0.1,
    }


def test_sample_source_requires_both_camera_streams() -> None:
    source = RecordingSampleSource(Engine(), _cfg())

    with pytest.raises(RuntimeError, match='腕部'):
        source()

    source.update_wrist(object(), time.monotonic())
    with pytest.raises(RuntimeError, match='基座'):
        source()


def test_sample_source_combines_three_rgb_images_and_quality() -> None:
    source = RecordingSampleSource(Engine(), _cfg())
    now = time.monotonic()
    wrist = object()
    base = object()
    roi = object()

    source.update_wrist(wrist, now)
    source.update_base(base, roi, now)
    sample = source()

    assert sample['image_wrist_rgb'] is wrist
    assert sample['image_base_rgb'] is base
    assert sample['image_roi_rgb'] is roi
    assert sample['quality']['existing'] == 1
    assert sample['quality']['camera_skew_exceeded'] == 0.0


def test_snapshot_ready_respects_age_and_skew() -> None:
    source = RecordingSampleSource(Engine(), _cfg())
    now = time.monotonic()
    source.update_wrist(object(), now)
    source.update_base(object(), object(), now)
    assert source.snapshot_ready()

    source.update_wrist(object(), now - 2.0)
    assert not source.snapshot_ready()


def test_clear_camera_state_makes_source_not_ready() -> None:
    source = RecordingSampleSource(Engine(), _cfg())
    now = time.monotonic()
    source.update_wrist(object(), now)
    source.update_base(object(), object(), now)
    assert source.snapshot_ready()

    source.clear_base()
    assert not source.snapshot_ready()

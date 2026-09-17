
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


@pytest.mark.parametrize('wrist_time,base_time,reason', [
    (87.0, 100.0, '腕部'),
    (100.0, 87.0, '基座'),
    (99.7, 100.0, '不同步'),
])
def test_readiness_explains_stale_camera_and_recovers(monkeypatch, wrist_time, base_time, reason):
    monkeypatch.setattr('gello_cr.bootstrap.sample_source.time.monotonic', lambda: 100.0)
    source = RecordingSampleSource(Engine(), _cfg())
    source.update_wrist(object(), wrist_time)
    source.update_base(object(), object(), base_time)

    assert not source.snapshot_ready()
    assert reason in source.readiness_error()

    source.update_wrist(object(), 100.0)
    source.update_base(object(), object(), 100.0)
    assert source.snapshot_ready()
    assert source.readiness_error() == ''


def test_third_camera_has_independent_freshness_skew_and_preview_timestamp(monkeypatch):
    monkeypatch.setattr('gello_cr.bootstrap.sample_source.time.monotonic', lambda: 100.0)
    source = RecordingSampleSource(Engine(), _cfg())
    source.update_base(object(), object(), 100.0)
    source.update_wrist(object(), 100.0)
    source.update_stream(2, object(), 98.0)
    assert '第三路' in source.readiness_error()
    sample = source()
    assert sample['quality']['roi_age_exceeded'] == 1
    assert sample['quality']['camera_skew_s'] == 2
    source.update_stream(2, object(), 99.8)
    assert '不同步' in source.readiness_error()
    source.update_stream(2, object(), 100.0)
    assert source.snapshot_ready()
    assert source.preview_snapshot()['roi_timestamp'] == 100.0

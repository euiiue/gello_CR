"""Full recorder -> Unix IPC -> real LeRobot/MP4 -> validation, synthetic inputs."""
import sys
import time

import numpy as np
import pytest

pytest.importorskip('lerobot.datasets.lerobot_dataset')
pytest.importorskip('av')

from gello_cr.recording.episode_recorder import LeRobotEpisodeRecorder
from gello_cr.recording.validate_dataset import validate_dataset


def test_real_worker_success_failure_discard_and_finalize(tmp_path, monkeypatch):
    monkeypatch.setenv('HF_HUB_OFFLINE', '1')
    monkeypatch.setenv('HF_DATASETS_OFFLINE', '1')
    image = np.full((224,224,3), 90, dtype=np.uint8)
    def sample():
        ts = time.monotonic()
        return {'timestamp': ts, 'observation_state': [0.0]*18, 'action': [0.0]*12,
                'image_base_rgb': image, 'image_wrist_rgb': image, 'image_roi_rgb': image,
                'quality': {'wrist_timestamp': ts, 'base_timestamp': ts,
                            'wrist_age_s': 0., 'base_age_s': 0., 'camera_skew_s': 0.}}
    recorder = LeRobotEpisodeRecorder(sample_provider=sample, event_callback=lambda *args: None,
        worker_python=sys.executable, repo_prefix='local/offline-test')
    counts = []
    for outcome in ('success', 'failure', 'discard'):
        recorder.start_episode('synthetic offline test', str(tmp_path), metadata={'synthetic': True})
        deadline = time.monotonic() + 10
        while recorder.snapshot()['buffered_frames'] < 3:
            assert not recorder.snapshot()['error']
            assert time.monotonic() < deadline
            time.sleep(.01)
        recorder.stop_episode()
        count = recorder.snapshot()['buffered_frames']
        assert count >= 3 and not recorder.snapshot()['episode_active']
        assert recorder.snapshot()['saved_episodes'] == len(counts)
        if outcome == 'discard':
            recorder.discard_episode()
        else:
            recorder.save_episode(outcome)
            counts.append(count)
        assert recorder.snapshot()['buffered_frames'] == 0
    root = recorder.snapshot()['root']
    recorder.close()
    recorder.close()
    result = validate_dataset(root)
    assert result['episodes'] == 2
    assert result['frames'] == sum(counts)
    assert not recorder.snapshot()['session_active']

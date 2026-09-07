from __future__ import annotations

import math

import numpy as np
import pytest

from gello_cr.data_contract import ACTION_FIELDS, STATE_FIELDS
from gello_cr.recording.schema import (
    LEROBOT_IMAGE_FEATURE_KEYS,
    SAMPLE_IMAGE_KEYS,
    dataset_features,
    validate_recording_sample,
)


def _sample() -> dict[str, object]:
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    return {
        "timestamp": 10.0,
        "observation_state": [0.0] * 18,
        "action": [0.0] * 12,
        "image_base_rgb": image,
        "image_wrist_rgb": image,
        "image_roi_rgb": image[:100, :100],
    }


def test_sample_keys_preserve_three_stream_contract() -> None:
    assert SAMPLE_IMAGE_KEYS == (
        "image_base_rgb",
        "image_wrist_rgb",
        "image_roi_rgb",
    )


def test_valid_sample_passes() -> None:
    validate_recording_sample(_sample())


def test_sample_rejects_wrong_state_and_action_dimensions() -> None:
    sample = _sample()
    sample["observation_state"] = [0.0] * 17
    with pytest.raises(ValueError, match="18"):
        validate_recording_sample(sample)

    sample = _sample()
    sample["action"] = [0.0] * 11
    with pytest.raises(ValueError, match="12"):
        validate_recording_sample(sample)


def test_sample_rejects_nonfinite_state_or_action() -> None:
    sample = _sample()
    sample["action"] = [0.0] * 11 + [math.inf]
    with pytest.raises(ValueError, match="non-finite"):
        validate_recording_sample(sample)


def test_sample_rejects_non_rgb_shape() -> None:
    sample = _sample()
    sample["image_roi_rgb"] = np.zeros((100, 100), dtype=np.uint8)
    with pytest.raises(ValueError, match="image_roi_rgb"):
        validate_recording_sample(sample)


def test_dataset_features_use_frozen_state_and_action_names() -> None:
    features = dataset_features(224, 224)

    assert features["observation.state"]["shape"] == (18,)
    assert features["observation.state"]["names"] == list(STATE_FIELDS)
    assert features["action"]["shape"] == (12,)
    assert features["action"]["names"] == list(ACTION_FIELDS)


def test_dataset_features_preserve_existing_serialized_image_names() -> None:
    features = dataset_features(224, 224)

    assert LEROBOT_IMAGE_FEATURE_KEYS == (
        "observation.images.base_0_rgb",
        "observation.images.left_wrist_0_rgb",
        "observation.images.right_wrist_0_rgb",
    )
    for key in LEROBOT_IMAGE_FEATURE_KEYS:
        assert features[key] == {
            "dtype": "video",
            "shape": (224, 224, 3),
            "names": ["height", "width", "channel"],
        }


def test_dataset_features_reject_invalid_image_size() -> None:
    with pytest.raises(ValueError, match="positive"):
        dataset_features(0, 224)

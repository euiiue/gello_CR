"""Selectable recording schemas for the LeRobot worker boundary."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from gello_cr.data_contract import ACTION_DIM, recording_fields

SAMPLE_IMAGE_KEYS = (
    "image_base_rgb",
    "image_wrist_rgb",
    "image_roi_rgb",
)

LEROBOT_IMAGE_FEATURE_KEYS = (
    "observation.images.base_0_rgb",
    "observation.images.left_wrist_0_rgb",
    "observation.images.right_wrist_0_rgb",
)


def validate_recording_sample(sample: Mapping[str, Any]) -> None:
    state_fields, _ = recording_fields(sample.get("recording_mode", "tcp"))
    state_dim = len(state_fields)
    state = sample["observation_state"]
    action = sample["action"]

    for image_key in SAMPLE_IMAGE_KEYS:
        image = np.asarray(sample[image_key])
        if image.ndim != 3 or image.shape[2] != 3 or not image.size:
            raise ValueError(
                f"Camera RGB frame {image_key} is invalid: {image.shape}"
            )

        if not np.isfinite(image).all():
            raise ValueError(f"Camera RGB frame {image_key} contains non-finite values")

    if np.asarray(state).shape != (state_dim,):
        raise ValueError(
            f"observation.state must have {state_dim} values, got {len(state)}"
        )
    if np.asarray(action).shape != (ACTION_DIM,):
        raise ValueError(
            f"action must have {ACTION_DIM} values, got {len(action)}"
        )

    if not np.isfinite(float(sample["timestamp"])):
        raise ValueError("Sample timestamp is non-finite")
    for key, value in sample.get("quality", {}).items():
        if not np.isfinite(float(value)):
            raise ValueError(f"Quality {key} is non-finite")

    values = [float(value) for value in (*state, *action)]
    if not all(np.isfinite(values)):
        raise ValueError("State/action contains non-finite values")


def dataset_features(height: int, width: int, recording_mode: str = "tcp") -> dict[str, dict[str, Any]]:
    state_fields, action_fields = recording_fields(recording_mode)
    height_value = int(height)
    width_value = int(width)
    if height_value <= 0 or width_value <= 0:
        raise ValueError("dataset image height/width must be positive")

    features: dict[str, dict[str, Any]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(state_fields),),
            "names": list(state_fields),
        },
        "action": {
            "dtype": "float32",
            "shape": (ACTION_DIM,),
            "names": list(action_fields),
        },
    }

    for feature_key in LEROBOT_IMAGE_FEATURE_KEYS:
        features[feature_key] = {
            "dtype": "video",
            "shape": (height_value, width_value, 3),
            "names": ["height", "width", "channel"],
        }

    return features

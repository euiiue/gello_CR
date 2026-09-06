"""LeRobot episode recording bridge for the CR5/O6 Qt application.

The Qt application and LeRobot deliberately run in separate Python environments:
the hardware environment owns NRC, PyQt and RealSense, while the LeRobot worker
owns datasets/PyArrow/PyAV.  A bounded synchronous Unix socket carries one
three 224x224 RGB frames plus state/action values at a time, so raw RealSense
frames are never accumulated in memory.  The streams follow the OpenPI contract:
base camera full view, wrist camera full view, and base-camera ROI.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_V2_SRC_DIR = Path(__file__).resolve().parent / "src"
if str(_V2_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_V2_SRC_DIR))

from gello_cr.recording.frame import (
    prepare_recording_frame,
    resize_rgb_for_openpi,
)
from gello_cr.recording.protocol import (
    read_exact as _read_exact,
    receive_packet as _receive_packet,
    send_packet as _send_packet,
)
from gello_cr.recording.worker_client import (
    WorkerClient as _WorkerClient,
    WorkerClientError as LeRobotRecorderError,
)
from gello_cr.recording.schema import (
    dataset_features as v2_dataset_features,
    validate_recording_sample,
)
from gello_cr.recording.episode_recorder import LeRobotEpisodeRecorder
from gello_cr.recording.worker_service import run_worker as _run_worker_service



def _dataset_features(height: int, width: int) -> dict[str, dict[str, Any]]:
    return v2_dataset_features(height, width)



def _worker_main(fd: int) -> int:
    return _run_worker_service(fd)



def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker-fd", type=int, required=True)
    args = parser.parse_args()
    return _worker_main(args.worker_fd)


if __name__ == "__main__":
    raise SystemExit(main())

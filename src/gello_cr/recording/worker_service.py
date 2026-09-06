"""Worker-side LeRobotDataset service.

This module owns the dataset state machine used by the dedicated LeRobot
subprocess.  It deliberately imports LeRobot lazily in `run_worker()` so the
main hardware/Qt Python environment can import the V2 recording package without
requiring LeRobot/PyArrow/PyAV.
"""

from __future__ import annotations

import json
import socket
import traceback
from pathlib import Path
from typing import Any, Callable

import numpy as np

from gello_cr.data_contract import ACTION_DIM, IMAGE_SIZE, STATE_DIM

from .protocol import receive_packet, send_packet
from .schema import LEROBOT_IMAGE_FEATURE_KEYS, dataset_features


class WorkerDatasetService:
    """Stateful implementation of worker operations."""

    def __init__(
        self,
        dataset_factory: Callable[..., Any],
        encoder_factory: Callable[..., Any],
    ) -> None:
        self._dataset_factory = dataset_factory
        self._encoder_factory = encoder_factory
        self.dataset: Any = None
        self.buffered_frames = 0
        self.saved_episodes = 0
        self.image_shape = (0, 0, 3)

    def handle_request(
        self,
        request: dict[str, Any],
        raw: bytes,
    ) -> tuple[dict[str, Any], bool]:
        operation = request.get("op")

        if operation == "init":
            return self._init_dataset(request), False
        if operation == "add_frame":
            return self._add_frame(request, raw), False
        if operation == "save_episode":
            return self._save_episode(request), False
        if operation == "clear_episode":
            return self._clear_episode(), False
        if operation == "finalize":
            return self._finalize(), True

        raise ValueError(f"Unknown worker operation: {operation}")

    def _init_dataset(self, request: dict[str, Any]) -> dict[str, Any]:
        if self.dataset is not None:
            raise RuntimeError("Dataset is already initialized")

        root = Path(str(request["root"])).expanduser().resolve()
        if root.exists() and any(root.iterdir()):
            raise FileExistsError(
                f"Dataset directory is not empty: {root}"
            )

        fps = int(request["fps"])
        height = int(request["height"])
        width = int(request["width"])
        if (height, width) != tuple(IMAGE_SIZE):
            raise ValueError("PI0.5 recording image size must be 224x224")

        encoder = self._encoder_factory(
            vcodec="h264",
            pix_fmt="yuv420p",
            crf=28,
            preset="fast",
            g=fps,
        )
        self.dataset = self._dataset_factory(
            repo_id=str(request["repo_id"]),
            root=root,
            fps=fps,
            robot_type="dobot_cr5_o6",
            features=dataset_features(height, width),
            use_videos=True,
            rgb_encoder=encoder,
            streaming_encoding=True,
            encoder_queue_maxsize=max(4, fps // 2),
            encoder_threads=2,
            batch_encoding_size=1,
        )
        self.image_shape = (height, width, 3)
        return {"ok": True, "root": str(root)}

    def _add_frame(
        self,
        request: dict[str, Any],
        raw: bytes,
    ) -> dict[str, Any]:
        if self.dataset is None:
            raise RuntimeError("Dataset is not initialized")

        expected_per_image = int(np.prod(self.image_shape))
        expected = expected_per_image * len(LEROBOT_IMAGE_FEATURE_KEYS)
        if len(raw) != expected:
            raise ValueError(
                f"3-stream RGB payload is {len(raw)} bytes, "
                f"expected {expected}"
            )

        images = np.frombuffer(raw, dtype=np.uint8).reshape(
            (len(LEROBOT_IMAGE_FEATURE_KEYS), *self.image_shape)
        ).copy()
        state = np.asarray(request["state"], dtype=np.float32)
        action = np.asarray(request["action"], dtype=np.float32)

        if state.shape != (STATE_DIM,) or action.shape != (ACTION_DIM,):
            raise ValueError(
                f"Invalid state/action shapes: "
                f"{state.shape}/{action.shape}"
            )

        frame = {
            LEROBOT_IMAGE_FEATURE_KEYS[index]: images[index]
            for index in range(len(LEROBOT_IMAGE_FEATURE_KEYS))
        }
        frame.update(
            {
                "observation.state": state,
                "action": action,
                "task": str(request["task"]),
            }
        )
        self.dataset.add_frame(frame)
        self.buffered_frames += 1
        return {"ok": True, "frames": self.buffered_frames}

    def _save_episode(self, request: dict[str, Any]) -> dict[str, Any]:
        if self.dataset is None or self.buffered_frames <= 0:
            raise RuntimeError("No Episode frames to save")

        # Preserve legacy ordering: diagnostics first, dataset commit second.
        # If sidecar writing fails, dataset.save_episode() is not called.
        collection_dir = Path(self.dataset.root) / "meta" / "collection"
        collection_dir.mkdir(parents=True, exist_ok=True)
        collection = dict(request["collection"])
        collection["episode_index"] = self.saved_episodes
        (
            collection_dir
            / f"episode_{self.saved_episodes:06d}.json"
        ).write_text(
            json.dumps(
                collection,
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

        self.dataset.save_episode()
        self.saved_episodes += 1
        self.buffered_frames = 0
        return {"ok": True, "episodes": self.saved_episodes}

    def _clear_episode(self) -> dict[str, Any]:
        if self.dataset is not None and self.buffered_frames:
            self.dataset.clear_episode_buffer()
        self.buffered_frames = 0
        return {"ok": True}

    def _finalize(self) -> dict[str, Any]:
        if self.dataset is not None:
            if self.buffered_frames:
                raise RuntimeError(
                    "Unsaved Episode; save or discard explicitly"
                )
            self.dataset.finalize()
        return {"ok": True, "episodes": self.saved_episodes}

    def finalize_best_effort(self) -> None:
        """Preserve the legacy worker-finally cleanup behavior."""

        if self.dataset is not None:
            try:
                self.dataset.finalize()
            except Exception:
                traceback.print_exc()


def run_worker(
    fd: int,
    *,
    dataset_factory: Callable[..., Any] | None = None,
    encoder_factory: Callable[..., Any] | None = None,
) -> int:
    """Run the worker request loop on an inherited socket file descriptor."""

    if dataset_factory is None or encoder_factory is None:
        # Imported only inside the dedicated LeRobot environment.
        from lerobot.configs.video import RGBEncoderConfig
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        if dataset_factory is None:
            dataset_factory = LeRobotDataset.create
        if encoder_factory is None:
            encoder_factory = RGBEncoderConfig

    service = WorkerDatasetService(
        dataset_factory=dataset_factory,
        encoder_factory=encoder_factory,
    )
    sock = socket.socket(fileno=fd)

    try:
        while True:
            try:
                request, raw = receive_packet(sock)
            except EOFError:
                break

            try:
                response, should_exit = service.handle_request(
                    request,
                    raw,
                )
            except Exception as exc:
                send_packet(
                    sock,
                    {
                        "ok": False,
                        "error": f"{type(exc).__name__}: {exc}",
                    },
                )
                continue

            send_packet(sock, response)
            if should_exit:
                return 0
    finally:
        service.finalize_best_effort()
        sock.close()

    return 0

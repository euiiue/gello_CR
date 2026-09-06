from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RECORDER = ROOT / "lerobot_recorder.py"
SERVICE = ROOT / "src/gello_cr/recording/worker_service.py"


def _source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function_source(path: Path, name: str) -> str:
    source = _source(path)
    module = ast.parse(source)
    for node in ast.walk(module):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            segment = ast.get_source_segment(source, node)
            assert segment is not None
            return segment
    raise AssertionError(f"function not found: {name}")


def test_root_recorder_imports_worker_service_delegate() -> None:
    source = _source(RECORDER)

    assert (
        "from gello_cr.recording.worker_service import "
        "run_worker as _run_worker_service"
    ) in source


def test_root_worker_main_is_now_only_a_delegate() -> None:
    source = _function_source(RECORDER, "_worker_main")

    assert source == (
        "def _worker_main(fd: int) -> int:\n"
        "    return _run_worker_service(fd)"
    )


def test_worker_dataset_operations_live_in_v2_service() -> None:
    root_source = _function_source(RECORDER, "_worker_main")
    service_source = _source(SERVICE)

    assert "LeRobotDataset.create" not in root_source
    assert "RGBEncoderConfig" not in root_source
    assert '"save_episode"' in service_source
    assert '"clear_episode"' in service_source
    assert '"finalize"' in service_source


def test_cli_entrypoint_still_calls_root_worker_main() -> None:
    source = _source(RECORDER)

    assert 'parser.add_argument("--worker-fd", type=int, required=True)' in source
    assert "return _worker_main(args.worker_fd)" in source

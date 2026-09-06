from __future__ import annotations

import ast
from pathlib import Path

SOURCE_PATH = Path(__file__).resolve().parents[2] / "teleop_runtime.py"


def _source() -> str:
    return SOURCE_PATH.read_text(encoding="utf-8")


def _module() -> ast.Module:
    return ast.parse(_source())


def _method_source(name: str) -> str:
    for node in ast.walk(_module()):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            segment = ast.get_source_segment(_source(), node)
            assert segment is not None
            return segment
    raise AssertionError(f"method not found: {name}")


def test_legacy_runtime_imports_joint_safety_in_both_v2_paths() -> None:
    source = _source()
    assert source.count("LeaderSpeedViolationCounter") >= 3
    assert source.count("max_command_step_violation") >= 3
    assert source.count("max_tracking_error_violation") >= 3


def test_joint_loop_routes_three_safety_checks_through_v2_primitives() -> None:
    source = _method_source("_gello_follow_loop")
    assert "speed_guard = LeaderSpeedViolationCounter()" in source
    assert "speed_guard.reset()" in source
    assert "max_command_step_violation(" in source
    assert "speed_guard.update(" in source
    assert "max_tracking_error_violation(" in source


def test_joint_loop_no_longer_embeds_legacy_numpy_safety_math() -> None:
    source = _method_source("_gello_follow_loop")
    assert "speed_counts" not in source
    assert "target_delta_rad = np.abs(np.deg2rad" not in source
    assert "speed_over_limit = speed > speed_limit" not in source
    assert "tracking_error = np.abs(np.deg2rad" not in source


def test_joint_loop_preserves_dispatch_and_low_latency_policy_boundaries() -> None:
    source = _method_source("_gello_follow_loop")
    assert "self.robot.send_servoj(target_full)" in source
    assert "if math.radians(desired_error_deg) > tracking_limit:" in source
    assert "max_segment_deg / desired_error_deg" in source
    assert "self.robot.stop_motion()" in source

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


def test_gello_joint_loop_constructs_relative_joint_mapper_from_runtime_config() -> None:
    source = _method_source("_gello_follow_loop")

    assert "joint_mapper = RelativeJointMapper(" in source
    assert 'joint_scale=float(gello_cfg["joint_scale"])' in source
    assert 'for value in gello_cfg["locked_joints"]' in source


def test_gello_joint_loop_routes_target_and_speed_delta_through_mapper() -> None:
    source = _method_source("_gello_follow_loop")

    assert "joint_mapper.target_deg(" in source
    assert "joint_mapper.leader_delta_rad(" in source


def test_gello_joint_loop_no_longer_embeds_legacy_wrapped_mapping_formula() -> None:
    source = _method_source("_gello_follow_loop")

    assert "relative = joint_scale * np.arctan2" not in source
    assert "leader_delta_rad = joint_scale * np.abs" not in source
    assert "locked_joint_indices" not in source

from __future__ import annotations

import ast
from pathlib import Path

SOURCE_PATH = Path(__file__).resolve().parents[2] / "teleop_runtime.py"


def _source() -> str:
    return SOURCE_PATH.read_text(encoding="utf-8")


def _method_source(name: str) -> str:
    source = _source()
    module = ast.parse(source)
    for node in ast.walk(module):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            segment = ast.get_source_segment(source, node)
            assert segment is not None
            return segment
    raise AssertionError(f"method not found: {name}")


def test_runtime_imports_hand_mapping_in_both_paths() -> None:
    assert _source().count("from gello_cr.control.hand_mapping import (") == 2


def test_joint_mode_uses_binary_mapper() -> None:
    source = _method_source("_gello_follow_loop")
    assert "hand_action = binary_o6_action(" in source
    assert 'open_action=o6_cfg["open_action"]' in source
    assert 'closed_action=o6_cfg["closed_action"]' in source
    assert 'self.o6.set_target(o6_cfg["actions"][hand_action])' in source
    assert '"抓取" if float(master.gripper) >= 0.5 else "张开手"' not in source


def test_cartesian_modes_use_continuous_mapper_and_deadband() -> None:
    source = _method_source("_gello_xyz_j6_follow_loop")
    assert "hand_target = interpolate_o6_target(" in source
    assert "should_send_o6_target(" in source
    assert "self.o6.set_target(hand_target)" in source
    assert "fraction = max(0.0, min(1.0, float(master.gripper)))" not in source


def test_joint_mode_keeps_event_reporting() -> None:
    source = _method_source("_gello_follow_loop")
    assert 'f"GELLO J7 触发 O6 动作：{hand_action}；"' in source
    assert "last_hand_action = hand_action" in source

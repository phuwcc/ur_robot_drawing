"""Tests for the deterministic offline smoke-test planner."""

import json

from ur_llm_control.planner import mock_plan


def test_mock_home_command():
    assert json.loads(mock_plan("home")) == {"plan": [{"skill": "home"}]}


def test_mock_pick_place_command():
    result = json.loads(mock_plan("Đưa khối đỏ vào vùng B"))
    assert result["plan"][0] == {"skill": "pick", "object": "red_cube"}
    assert result["plan"][1] == {
        "skill": "place",
        "object": "red_cube",
        "zone": "zone_b",
    }
    assert result["plan"][2] == {"skill": "home"}

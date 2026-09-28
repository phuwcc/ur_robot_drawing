import json

import pytest

from ur_llm_control.validator import (
    PlanValidationError,
    ensure_final_home,
    parse_and_validate,
)


OBJECTS = {"red_cube", "yellow_cube", "blue_cube"}
ZONES = {"zone_a", "zone_b", "zone_c"}


def validate(document):
    return parse_and_validate(json.dumps(document), OBJECTS, ZONES)


def test_adds_required_home_step():
    plan = [
        {"skill": "pick", "object": "red_cube"},
        {"skill": "place", "object": "red_cube", "zone": "zone_b"},
    ]
    validated = validate({"plan": plan})
    assert ensure_final_home(validated)[-1] == {"skill": "home"}


def test_valid_pick_place_home():
    plan = [
        {"skill": "pick", "object": "red_cube"},
        {"skill": "place", "object": "red_cube", "zone": "zone_b"},
        {"skill": "home"},
    ]
    assert validate({"plan": plan}) == plan


@pytest.mark.parametrize("document", [
    {"plan": [{"skill": "move_joint", "joint_1": 1.0}]},
    {"plan": [{"skill": "pick", "object": "green_cube"}]},
    {"plan": [{"skill": "place", "object": "red_cube", "zone": "zone_a"}]},
    {"plan": [{"skill": "home"}, {"skill": "pick", "object": "red_cube"}]},
    {"plan": [{"skill": "home", "speed": 1.0}]},
])
def test_rejects_unsafe_or_invalid_plans(document):
    with pytest.raises(PlanValidationError):
        validate(document)


def test_rejects_markdown_wrapped_json():
    with pytest.raises(PlanValidationError):
        parse_and_validate('```json\n{"plan": []}\n```', OBJECTS, ZONES)

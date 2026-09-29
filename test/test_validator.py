import json

import pytest

from ur_llm_control.validator import (
    PlanValidationError,
    parse_and_validate,
)


OBJECTS = {"red_cube", "yellow_cube", "blue_cube"}
ZONES = {"zone_a", "zone_b", "zone_c"}


def validate(document):
    return parse_and_validate(json.dumps(document), OBJECTS, ZONES)


def test_valid_pick_place_home():
    plan = [
        {"skill": "pick", "object": "red_cube"},
        {"skill": "place", "object": "red_cube", "zone": "zone_b"},
        {"skill": "home"},
    ]
    assert validate({"plan": plan}) == plan


@pytest.mark.parametrize("raw", [
    '  {"plan":[{"skill":"home"}]}  ',
    '```json\n{"plan":[{"skill":"home"}]}\n```',
    '```\n{"plan":[{"skill":"home"}]}\n```',
    'Here is the plan:\n{"plan":[{"skill":"home"}]}\nReady to execute.',
])
def test_accepts_supported_llm_json_formatting(raw):
    assert parse_and_validate(raw, OBJECTS, ZONES) == [{"skill": "home"}]


@pytest.mark.parametrize("raw, message", [
    ('{"plan":[{"skill":"home"}]', "malformed JSON"),
    (
        '{"plan":[{"skill":"home"}]} {"plan":[{"skill":"home"}]}',
        "multiple JSON objects",
    ),
    ("There is no plan here.", "does not contain a JSON object"),
])
def test_rejects_missing_malformed_or_ambiguous_json(raw, message):
    with pytest.raises(PlanValidationError, match=message):
        parse_and_validate(raw, OBJECTS, ZONES)


@pytest.mark.parametrize("document, message", [
    ({"plan": [{"skill": "move_joint", "joint_1": 1.0}]}, "invalid skill"),
    ({"plan": [{"skill": "pick", "object": "green_cube"}]}, "invalid object"),
    (
        {"plan": [
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_x"},
            {"skill": "home"},
        ]},
        "invalid zone",
    ),
    (
        {"plan": [
            {"skill": "place", "object": "red_cube", "zone": "zone_a"},
            {"skill": "home"},
        ]},
        "currently holding",
    ),
    (
        {"plan": [
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_a"},
        ]},
        "finish with home",
    ),
    ({"plan": [{"skill": "home"}, {"skill": "pick", "object": "red_cube"}]}, "home must"),
    ({"plan": [{"skill": "home", "speed": 1.0}]}, "keys must"),
])
def test_rejects_unsafe_or_invalid_plans(document, message):
    with pytest.raises(PlanValidationError, match=message):
        validate(document)

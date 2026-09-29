"""Strict validation between the untrusted LLM and the robot."""

import json
import re


ALLOWED_SKILLS = {"pick", "place", "home"}
SKILL_KEYS = {
    "pick": {"skill", "object"},
    "place": {"skill", "object", "zone"},
    "home": {"skill"},
}


class PlanValidationError(ValueError):
    pass


_FENCED_JSON = re.compile(
    r"\A```(?:json)?[ \t]*\r?\n(?P<body>.*)\r?\n```\Z",
    re.IGNORECASE | re.DOTALL,
)


def extract_json_object(raw, max_surrounding_text=500):
    """Extract one top-level JSON object from conservative LLM formatting."""
    if not isinstance(raw, str) or not raw.strip():
        raise PlanValidationError("LLM output is empty")

    text = raw.strip()
    fence = _FENCED_JSON.fullmatch(text)
    if fence:
        text = fence.group("body").strip()
    elif "```" in text:
        raise PlanValidationError("LLM output contains an invalid Markdown fence")

    try:
        document = json.loads(text)
    except json.JSONDecodeError:
        document = None
    else:
        if not isinstance(document, dict):
            raise PlanValidationError("LLM output JSON must be an object")
        return document

    spans = []
    start = None
    depth = 0
    in_string = False
    escaped = False
    for index, character in enumerate(text):
        if in_string:
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == '"':
                in_string = False
            continue
        if character == '"':
            in_string = True
        elif character == "{":
            if depth == 0:
                start = index
            depth += 1
        elif character == "}":
            if depth == 0:
                raise PlanValidationError("LLM output contains an unmatched closing brace")
            depth -= 1
            if depth == 0:
                spans.append((start, index + 1))
                start = None

    if in_string or depth:
        raise PlanValidationError("LLM output contains a malformed JSON object")
    if not spans:
        raise PlanValidationError("LLM output does not contain a JSON object")
    if len(spans) != 1:
        raise PlanValidationError("LLM output contains multiple JSON objects")

    start, end = spans[0]
    prefix = text[:start].strip()
    suffix = text[end:].strip()
    if len(prefix) > max_surrounding_text or len(suffix) > max_surrounding_text:
        raise PlanValidationError("LLM output contains too much text outside the JSON object")
    try:
        document = json.loads(text[start:end])
    except json.JSONDecodeError as exc:
        raise PlanValidationError(f"LLM output contains malformed JSON: {exc.msg}") from exc
    if not isinstance(document, dict):
        raise PlanValidationError("LLM output JSON must be an object")
    return document


def parse_and_validate(raw, objects, zones, max_steps=10):
    document = extract_json_object(raw)

    if not isinstance(document, dict) or set(document) != {"plan"}:
        raise PlanValidationError("top level must contain exactly one key: plan")
    plan = document["plan"]
    if not isinstance(plan, list) or not 1 <= len(plan) <= max_steps:
        raise PlanValidationError(f"plan must contain between 1 and {max_steps} steps")

    held = None
    home_seen = False
    for index, step in enumerate(plan):
        where = f"plan[{index}]"
        if not isinstance(step, dict):
            raise PlanValidationError(f"{where} must be an object")
        skill = step.get("skill")
        if skill not in ALLOWED_SKILLS:
            raise PlanValidationError(f"{where} contains invalid skill: {skill!r}")
        if set(step) != SKILL_KEYS[skill]:
            raise PlanValidationError(
                f"{where} keys must be exactly {sorted(SKILL_KEYS[skill])}")
        if home_seen:
            raise PlanValidationError("home must be the final step")

        if skill == "pick":
            obj = step["object"]
            if obj not in objects:
                raise PlanValidationError(f"{where} contains invalid object: {obj!r}")
            if held is not None:
                raise PlanValidationError(f"cannot pick {obj!r} while holding {held!r}")
            held = obj
        elif skill == "place":
            obj = step["object"]
            zone = step["zone"]
            if obj not in objects:
                raise PlanValidationError(f"{where} contains invalid object: {obj!r}")
            if zone not in zones:
                raise PlanValidationError(f"{where} contains invalid zone: {zone!r}")
            if held != obj:
                raise PlanValidationError(f"cannot place {obj!r}; currently holding {held!r}")
            held = None
        else:
            home_seen = True

    if held is not None:
        raise PlanValidationError(f"plan ends while still holding {held!r}")
    if not home_seen:
        raise PlanValidationError("plan must finish with home")
    return plan

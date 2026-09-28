"""Strict validation between the untrusted LLM and the robot."""

import json


ALLOWED_SKILLS = {"pick", "place", "home"}
SKILL_KEYS = {
    "pick": {"skill", "object"},
    "place": {"skill", "object", "zone"},
    "home": {"skill"},
}


class PlanValidationError(ValueError):
    pass


def ensure_final_home(plan, max_steps=10):
    """Return a validated plan with a deterministic final home step."""
    if plan[-1]["skill"] == "home":
        return plan
    if len(plan) >= max_steps:
        raise PlanValidationError("plan has no room for the required home step")
    return [*plan, {"skill": "home"}]


def parse_and_validate(raw, objects, zones, max_steps=10):
    try:
        document = json.loads(raw)
    except (TypeError, json.JSONDecodeError) as exc:
        raise PlanValidationError(f"LLM output is not valid JSON: {exc}") from exc

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
    return plan

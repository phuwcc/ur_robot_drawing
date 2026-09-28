"""Small OpenAI-compatible client for 9Router."""

import json
import os
import re
import urllib.error
import urllib.request


class PlannerError(RuntimeError):
    pass


class NineRouterPlanner:
    def __init__(self, base_url, model, api_key_env, timeout=30.0):
        self.endpoint = base_url.rstrip("/") + "/chat/completions"
        self.model = model
        self.api_key_env = api_key_env
        self.timeout = timeout

    @staticmethod
    def system_prompt(objects, zones, student_name, student_id, mapping):
        mapping_text = ", ".join(f"{zone} -> {color}" for zone, color in mapping.items())
        return f"""You are a task planner for a UR3e robot.
Convert the user's Vietnamese or English command into a JSON skill plan.

Allowed skills and exact fields:
- {{"skill":"pick","object":"OBJECT"}}
- {{"skill":"place","object":"OBJECT","zone":"ZONE"}}
- {{"skill":"home"}}

Allowed objects: {', '.join(objects)}
Allowed zones: {', '.join(zones)}

Rules:
- Return one JSON object with exactly one top-level key named "plan".
- Return JSON only. Do not use Markdown or add explanations.
- To move an object to a zone, plan pick(object), place(object, zone), home().
- Never output coordinates, joint values, trajectories, or unlisted skills.
- Keep object and zone names exactly as listed above.

Student: {student_name} ({student_id}). Personalized mapping: {mapping_text}.
"""

    def plan(self, command, objects, zones, student_name, student_id, mapping):
        api_key = os.environ.get(self.api_key_env, "").strip()
        if not api_key:
            raise PlannerError(f"environment variable {self.api_key_env} is not set")
        payload = {
            "model": self.model,
            "temperature": 0,
            "max_tokens": 500,
            "messages": [
                {
                    "role": "system",
                    "content": self.system_prompt(objects, zones, student_name, student_id, mapping),
                },
                {"role": "user", "content": command},
            ],
        }
        request = urllib.request.Request(
            self.endpoint,
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                body = json.loads(response.read().decode("utf-8"))
            return body["choices"][0]["message"]["content"].strip()
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, KeyError, IndexError) as exc:
            raise PlannerError(f"9Router request failed: {exc}") from exc


def mock_plan(command):
    """Offline smoke-test planner. It is deliberately not used for final LLM demos."""
    text = command.lower()
    colors = {"red": "red_cube", "đỏ": "red_cube", "do": "red_cube",
              "yellow": "yellow_cube", "vàng": "yellow_cube", "vang": "yellow_cube",
              "blue": "blue_cube", "xanh": "blue_cube"}
    obj = next((value for word, value in colors.items() if word in text), None)
    match = re.search(r"(?:zone|vùng|vung|ô|o)\s*[_-]?\s*([abc])\b", text)
    if not obj and not match and text.strip() in {"home", "go home", "về home", "ve home"}:
        return json.dumps({"plan": [{"skill": "home"}]})
    if not obj or not match:
        raise PlannerError("mock mode needs a color and zone A/B/C in the command")
    return json.dumps({"plan": [
        {"skill": "pick", "object": obj},
        {"skill": "place", "object": obj, "zone": f"zone_{match.group(1)}"},
        {"skill": "home"},
    ]})

"""Small OpenAI-compatible client for 9Router."""

import json
import os
import re
import socket
import urllib.error
import urllib.request


class PlannerError(RuntimeError):
    pass


def _completion_text(body, model):
    if not isinstance(body, dict):
        raise PlannerError("9Router returned an unexpected response object")

    choices = body.get("choices")
    if not isinstance(choices, list) or not choices or not isinstance(choices[0], dict):
        raise PlannerError("9Router response did not contain a model choice")

    choice = choices[0]
    message = choice.get("message")
    if not isinstance(message, dict):
        raise PlannerError("9Router response did not contain a model message")

    if "content" not in message:
        raise PlannerError("9Router response model message did not contain content")
    content = message["content"]
    if isinstance(content, list):
        content = "".join(
            part.get("text", "")
            for part in content
            if isinstance(part, dict) and isinstance(part.get("text"), str)
        )
    if not isinstance(content, str) or not content.strip():
        finish_reason = choice.get("finish_reason", "unknown")
        raise PlannerError(
            f"9Router returned empty text from model {model!r} "
            f"(finish_reason={finish_reason!r}); check that the model supports "
            "chat completions and is available to this API key"
        )
    return content.strip()


class NineRouterPlanner:
    def __init__(self, base_url, model, api_key_env, timeout=30.0):
        self.endpoint = base_url.rstrip("/") + "/chat/completions"
        self.model = model
        self.api_key_env = api_key_env
        self.timeout = timeout

    @staticmethod
    def system_prompt(objects, zones, student_name, student_id, mapping):
        mapping_text = ", ".join(f"{zone} -> {color}" for zone, color in mapping.items())
        example_object = objects[0] if objects else "OBJECT"
        example_zone = zones[0] if zones else "ZONE"
        return f"""You are a task planner for a UR3e robot.
Convert the user's Vietnamese or English command into a JSON skill plan.

Allowed skills and exact fields:
- {{"skill":"pick","object":"OBJECT"}}
- {{"skill":"place","object":"OBJECT","zone":"ZONE"}}
- {{"skill":"home"}}

Allowed objects: {', '.join(objects)}
Allowed zones: {', '.join(zones)}

Rules:
- Return exactly one JSON object with exactly one top-level key named "plan".
- Return JSON only: no Markdown, no code fences, and no explanations.
- To move an object to a zone, plan pick(object), place(object, zone), home().
- Use only the allowed skills, objects, and zones listed above.
- Never output coordinates, joint angles, velocities, accelerations, robot
  trajectories, MoveIt configuration, or ROS commands.
- Every executable manipulation plan must finish with {{"skill":"home"}}.
- Keep object and zone names exactly as listed above.

Valid example:
{{"plan":[{{"skill":"pick","object":"{example_object}"}},{{"skill":"place","object":"{example_object}","zone":"{example_zone}"}},{{"skill":"home"}}]}}

Student: {student_name} ({student_id}). Personalized mapping: {mapping_text}.
"""

    def plan(self, command, objects, zones, student_name, student_id, mapping):
        api_key = os.environ.get(self.api_key_env, "").strip()
        if not api_key:
            raise PlannerError(f"Environment variable {self.api_key_env} is not set.")
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
                response_bytes = response.read()
        except urllib.error.HTTPError as exc:
            error_text = exc.read().decode("utf-8", errors="replace").strip()
            if api_key:
                error_text = error_text.replace(api_key, "[REDACTED]")
            if len(error_text) > 1000:
                error_text = error_text[:1000] + "..."
            detail = error_text or str(exc.reason)
            raise PlannerError(
                f"9Router returned HTTP {exc.code} for POST /v1/chat/completions: "
                f"{detail}"
            ) from exc
        except (urllib.error.URLError, TimeoutError, socket.timeout, OSError) as exc:
            detail = str(exc).replace(api_key, "[REDACTED]")
            raise PlannerError(f"9Router request failed: {detail}") from exc
        try:
            response_text = response_bytes.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise PlannerError("9Router returned a non-UTF-8 API response") from exc
        try:
            body = json.loads(response_text)
        except json.JSONDecodeError as exc:
            raise PlannerError("9Router returned a non-JSON API response") from exc
        return _completion_text(body, self.model)


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

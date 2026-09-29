"""Tests for the direct Gemini planner and offline mock."""

import json
from io import BytesIO
from urllib.error import HTTPError

import pytest

from ur_llm_control import planner
from ur_llm_control.planner import GeminiPlanner, PlannerError, mock_plan


class FakeResponse:
    def __init__(self, body):
        self.body = body

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self):
        return self.body


def gemini_response(text, finish_reason="STOP"):
    return {
        "candidates": [{
            "finishReason": finish_reason,
            "content": {"parts": [{"text": text}]},
        }]
    }


def test_mock_plans_transfer_and_home():
    transfer = json.loads(mock_plan("Đưa khối đỏ vào vùng B"))
    assert transfer == {"plan": [
        {"skill": "pick", "object": "red_cube"},
        {"skill": "place", "object": "red_cube", "zone": "zone_b"},
        {"skill": "home"},
    ]}
    assert json.loads(mock_plan("home")) == {"plan": [{"skill": "home"}]}


def test_response_text_accepts_only_complete_non_thought_text():
    body = gemini_response("")
    body["candidates"][0]["content"]["parts"] = [
        {"thought": True, "text": "internal"},
        {"text": '{"plan":[{"skill":"home"}]}'},
    ]
    assert planner._response_text(body, "test-model") == (
        '{"plan":[{"skill":"home"}]}'
    )


@pytest.mark.parametrize(
    "body, message",
    [
        ({}, "no candidate"),
        (gemini_response("partial", "MAX_TOKENS"), "did not finish normally"),
        (gemini_response(""), "empty text"),
    ],
)
def test_response_text_rejects_incomplete_responses(body, message):
    with pytest.raises(PlannerError, match=message):
        planner._response_text(body, "test-model")


def test_missing_key_and_invalid_model_are_clear(monkeypatch):
    with pytest.raises(PlannerError, match="Invalid Gemini model"):
        GeminiPlanner("../bad-model")

    monkeypatch.delenv("TEST_GEMINI_KEY", raising=False)
    client = GeminiPlanner(api_key_env="TEST_GEMINI_KEY")
    with pytest.raises(PlannerError, match="TEST_GEMINI_KEY is not set"):
        client.plan("home", (), (), "Test", "123", {})


def test_request_uses_gemini_generate_content_and_json_schema(monkeypatch):
    monkeypatch.setenv("TEST_GEMINI_KEY", "secret")
    captured = {}

    def respond(request, timeout):
        captured["url"] = request.full_url
        captured["key"] = request.get_header("X-goog-api-key")
        captured["payload"] = json.loads(request.data.decode("utf-8"))
        captured["timeout"] = timeout
        return FakeResponse(json.dumps(
            gemini_response('{"plan":[{"skill":"home"}]}')
        ).encode("utf-8"))

    monkeypatch.setattr(planner.urllib.request, "urlopen", respond)
    client = GeminiPlanner(
        model="gemini-test", api_key_env="TEST_GEMINI_KEY", timeout=12.5
    )
    result = client.plan(
        "home",
        ("red_cube",),
        ("zone_a",),
        "Test",
        "123",
        {"zone_a": "Red"},
    )

    assert result == '{"plan":[{"skill":"home"}]}'
    assert captured["url"].endswith(
        "/v1beta/models/gemini-test:generateContent"
    )
    assert captured["key"] == "secret"
    assert captured["timeout"] == 12.5
    payload = captured["payload"]
    assert payload["contents"][0]["parts"][0]["text"] == "home"
    config = payload["generationConfig"]
    assert config["responseMimeType"] == "application/json"
    assert config["responseSchema"]["properties"]["plan"]["items"][
        "properties"
    ]["skill"]["enum"] == ["pick", "place", "home"]


def test_http_error_redacts_api_key(monkeypatch):
    api_key = "test-secret-key"
    monkeypatch.setenv("TEST_GEMINI_KEY", api_key)

    def fail(*_args, **_kwargs):
        raise HTTPError(
            "https://generativelanguage.googleapis.com",
            400,
            "Bad Request",
            {},
            BytesIO(b'{"error":{"message":"bad test-secret-key"}}'),
        )

    monkeypatch.setattr(planner.urllib.request, "urlopen", fail)
    client = GeminiPlanner(api_key_env="TEST_GEMINI_KEY")
    with pytest.raises(PlannerError, match="Gemini returned HTTP 400") as error:
        client.plan("home", (), (), "Test", "123", {})
    assert api_key not in str(error.value)
    assert "[REDACTED]" in str(error.value)


def test_timeout_is_clear(monkeypatch):
    monkeypatch.setenv("TEST_GEMINI_KEY", "secret")

    def time_out(*_args, **_kwargs):
        raise TimeoutError("timed out")

    monkeypatch.setattr(planner.urllib.request, "urlopen", time_out)
    client = GeminiPlanner(api_key_env="TEST_GEMINI_KEY", timeout=2.5)
    with pytest.raises(PlannerError, match="timed out after 2.5 seconds"):
        client.plan("home", (), (), "Test", "123", {})

"""Tests for the deterministic offline smoke-test planner."""

import json
from io import BytesIO
from urllib.error import HTTPError

import pytest

from ur_llm_control import planner
from ur_llm_control.planner import PlannerError, _completion_text, mock_plan


class FakeResponse:
    def __init__(self, body):
        self.body = body

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self):
        return self.body


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


def test_completion_text_accepts_text_content_blocks():
    body = {
        "choices": [{
            "message": {
                "content": [
                    {"type": "text", "text": '{"plan": [{"skill": "home"}]}'}
                ]
            },
            "finish_reason": "stop",
        }]
    }

    assert _completion_text(body, "test-model") == '{"plan": [{"skill": "home"}]}'


def test_empty_completion_reports_model_and_finish_reason():
    body = {
        "choices": [{
            "message": {"content": ""},
            "finish_reason": "length",
        }]
    }

    with pytest.raises(
        PlannerError,
        match=r"empty text from model 'test-model' \(finish_reason='length'\)",
    ):
        _completion_text(body, "test-model")


def test_completion_without_choices_reports_router_response_error():
    with pytest.raises(PlannerError, match="did not contain a model choice"):
        _completion_text({"choices": []}, "test-model")


def test_completion_without_message_or_content_is_clear():
    with pytest.raises(PlannerError, match="model message"):
        _completion_text({"choices": [{}]}, "test-model")
    with pytest.raises(PlannerError, match="did not contain content"):
        _completion_text({"choices": [{"message": {}}]}, "test-model")


def test_missing_api_key_is_clear(monkeypatch):
    monkeypatch.delenv("TEST_ROUTER_KEY", raising=False)
    client = planner.NineRouterPlanner(
        "http://localhost:20128/v1", "test-model", "TEST_ROUTER_KEY"
    )
    with pytest.raises(
        PlannerError, match="Environment variable TEST_ROUTER_KEY is not set"
    ):
        client.plan("home", (), (), "Test", "123456", {})


def test_malformed_api_response_is_planner_error(monkeypatch):
    monkeypatch.setenv("TEST_ROUTER_KEY", "secret")
    monkeypatch.setattr(
        planner.urllib.request,
        "urlopen",
        lambda *_args, **_kwargs: FakeResponse(b"not JSON"),
    )
    client = planner.NineRouterPlanner(
        "http://localhost:20128/v1", "test-model", "TEST_ROUTER_KEY"
    )
    with pytest.raises(PlannerError, match="non-JSON API response"):
        client.plan("home", (), (), "Test", "123456", {})


def test_configured_model_reaches_http_payload(monkeypatch):
    monkeypatch.setenv("TEST_ROUTER_KEY", "secret")
    captured = {}

    def respond(request, timeout):
        captured["payload"] = json.loads(request.data.decode("utf-8"))
        captured["timeout"] = timeout
        return FakeResponse(
            b'{"choices":[{"message":{"content":"{\\"plan\\":[{\\"skill\\":\\"home\\"}]}"}}]}'
        )

    monkeypatch.setattr(planner.urllib.request, "urlopen", respond)
    client = planner.NineRouterPlanner(
        "http://localhost:20128/v1", "configured-model", "TEST_ROUTER_KEY", 12.5
    )
    assert client.plan("home", (), (), "Test", "123456", {}) == (
        '{"plan":[{"skill":"home"}]}'
    )
    assert captured == {
        "payload": {
            "model": "configured-model",
            "temperature": 0,
            "max_tokens": 500,
            "messages": captured["payload"]["messages"],
        },
        "timeout": 12.5,
    }


def test_http_error_includes_router_detail_without_exposing_api_key(monkeypatch):
    api_key = "test-secret-key"
    monkeypatch.setenv("TEST_ROUTER_KEY", api_key)

    def raise_not_found(*_args, **_kwargs):
        raise HTTPError(
            "http://localhost:20128/v1/chat/completions",
            404,
            "Not Found",
            {},
            BytesIO(b'{"error":"model route not found","key":"test-secret-key"}'),
        )

    monkeypatch.setattr(planner.urllib.request, "urlopen", raise_not_found)
    client = planner.NineRouterPlanner(
        "http://localhost:20128/v1", "test-model", "TEST_ROUTER_KEY"
    )

    with pytest.raises(
        PlannerError,
        match=r"HTTP 404 for POST /v1/chat/completions:.*model route not found",
    ) as error:
        client.plan(
            "go home",
            ("red_cube",),
            ("zone_a",),
            "Test",
            "123456",
            {"zone_a": "Red"},
        )

    assert api_key not in str(error.value)
    assert "[REDACTED]" in str(error.value)

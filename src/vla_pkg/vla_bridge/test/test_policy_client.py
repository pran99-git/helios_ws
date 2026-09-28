"""Unit tests for the policy wire protocol. Pure pytest, no ROS, no server."""

import base64
import io

import pytest
import requests
from PIL import Image
from vla_bridge.goal_resolution import Pose2D
from vla_bridge.policy_client import WIRE_SIZE, PolicyClient, encode_jpeg


class FakeResponse:
    def __init__(self, payload, status=200):
        self._payload = payload
        self.status_code = status

    def raise_for_status(self):
        if self.status_code >= 400:
            raise requests.HTTPError(f"{self.status_code}")

    def json(self):
        return self._payload


class FakeSession:
    """Captures the outgoing payload and replays a canned reply."""

    def __init__(self, reply=None, raises=None):
        self.reply = reply
        self.raises = raises
        self.payload = None

    def post(self, url, json, timeout):
        self.payload = json
        if self.raises:
            raise self.raises
        return self.reply


VALID_REPLY = FakeResponse(
    {"waypoints": [[0.12, 0.0, 1.0, 0.0]], "modality_id": 7, "latency_ms": 95.1}
)


def client(session):
    c = PolicyClient("http://127.0.0.1:8777/act", timeout=0.3)
    c._session = session
    return c


def test_encode_jpeg_downscales_to_the_wire_size():
    """Full-resolution ZED frames are wasted bandwidth: the model tops out at 224."""
    encoded = encode_jpeg(Image.new("RGB", (1280, 720), "red"))
    decoded = Image.open(io.BytesIO(base64.b64decode(encoded)))
    assert decoded.size == (WIRE_SIZE, WIRE_SIZE)


def test_successful_call_decodes_the_chunk():
    result = client(FakeSession(reply=VALID_REPLY)).act(["a"] * 6, goal_text="bin")
    assert result is not None
    assert result.modality_id == 7
    assert result.waypoints[0].x == pytest.approx(0.12)


def test_unset_goals_are_omitted_from_the_payload():
    """The server rejects unsupported modality combinations, so only send what is set."""
    session = FakeSession(reply=VALID_REPLY)
    client(session).act(["a"] * 6, goal_text="bin")
    assert "goal_text" in session.payload
    assert "goal_pose" not in session.payload
    assert "goal_image_b64" not in session.payload


def test_goal_pose_is_sent_as_x_y_yaw():
    session = FakeSession(reply=VALID_REPLY)
    client(session).act(["a"] * 6, goal_pose=Pose2D(1.0, -2.0, 0.5))
    assert session.payload["goal_pose"] == [1.0, -2.0, 0.5]


def test_timeout_returns_none_rather_than_raising():
    """A slow server must degrade to an empty path, never crash the node."""
    c = client(FakeSession(raises=requests.Timeout()))
    assert c.act(["a"] * 6, goal_text="bin") is None
    assert "timeout" in (c.last_error or "")


def test_server_error_returns_none():
    c = client(FakeSession(reply=FakeResponse({}, status=422)))
    assert c.act(["a"] * 6, goal_text="bin") is None
    assert c.last_error


def test_malformed_response_returns_none():
    c = client(FakeSession(reply=FakeResponse({"waypoints": [[1.0, 2.0]]})))
    assert c.act(["a"] * 6, goal_text="bin") is None
    assert "malformed" in (c.last_error or "")


def test_last_error_clears_after_a_good_call():
    c = client(FakeSession(raises=requests.Timeout()))
    c.act(["a"] * 6, goal_text="bin")
    c._session = FakeSession(reply=VALID_REPLY)
    assert c.act(["a"] * 6, goal_text="bin") is not None
    assert c.last_error is None

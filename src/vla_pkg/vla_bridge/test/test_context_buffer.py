"""Unit tests for the observation history window. Pure pytest, no ROS."""

import pytest
from vla_bridge.context_buffer import ContextBuffer


def buffer(size=6, stride=0.333):
    return ContextBuffer(size=size, stride=stride)


def test_frames_arriving_faster_than_the_stride_are_dropped():
    """The ZED runs at 30 Hz; the window must not fill with 0.2 s of history."""
    buf = buffer()
    assert buf.push("a", 0.0) is True
    assert buf.push("b", 0.1) is False
    assert buf.push("c", 0.4) is True
    assert buf.frames()[-1] == "c"


def test_window_pads_at_the_front_before_it_fills():
    """Padding repeats the OLDEST frame, encoding "stationary until then"."""
    buf = buffer(size=4, stride=0.0)
    buf.push("a", 0.0)
    buf.push("b", 1.0)
    assert buf.frames() == ["a", "a", "a", "b"]


def test_oldest_first_current_last():
    buf = buffer(size=3, stride=0.0)
    for i, stamp in enumerate([0.0, 1.0, 2.0]):
        buf.push(i, stamp)
    assert buf.frames() == [0, 1, 2]


def test_window_slides_once_full():
    buf = buffer(size=3, stride=0.0)
    for i in range(5):
        buf.push(i, float(i))
    assert buf.frames() == [2, 3, 4]


def test_ready_and_warm_are_distinct():
    buf = buffer(size=3, stride=0.0)
    assert not buf.ready and not buf.warm
    buf.push("a", 0.0)
    assert buf.ready and not buf.warm
    buf.push("b", 1.0)
    buf.push("c", 2.0)
    assert buf.warm


def test_empty_buffer_refuses_to_invent_frames():
    with pytest.raises(RuntimeError):
        buffer().frames()


def test_clear_resets_the_stride_clock():
    buf = buffer(size=3, stride=1.0)
    buf.push("a", 0.0)
    buf.clear()
    assert not buf.ready
    assert buf.push("b", 0.1) is True


def test_size_must_be_positive():
    with pytest.raises(ValueError):
        ContextBuffer(size=0)

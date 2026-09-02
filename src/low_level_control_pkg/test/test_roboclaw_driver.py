"""Unit tests for the RoboClaw packet-serial layer.

Pure pytest -- no serial hardware. `serial.Serial` is replaced with a fake
that records every byte written and replays canned response bytes, so these
tests assert the exact wire format rather than just "it did not crash".
"""

import pytest
from roboclaw import roboclaw_driver
from roboclaw.roboclaw_driver import (
    CMD_DUTY_M1M2,
    CMD_READ_ENCODERS,
    CMD_READ_ISPEEDS,
    CMD_READ_M1_VELOCITY_PID,
    CMD_SPEED_ACCEL_M1M2,
    UINT32_MAX,
    RoboClaw,
    SerialBus,
    crc16,
)

ADDRESS = 0x80
ACK = b"\xff"


class FakeSerial:
    """Records writes and replays a queued response, like a serial port."""

    def __init__(self) -> None:
        self.written = bytearray()
        self.to_read = bytearray()
        self.buffer_resets = 0

    def reset_input_buffer(self) -> None:
        self.buffer_resets += 1

    def write(self, data: bytes) -> int:
        self.written.extend(data)
        return len(data)

    def read(self, size: int) -> bytes:
        taken = bytes(self.to_read[:size])
        del self.to_read[:size]
        return taken

    def close(self) -> None:
        pass


@pytest.fixture
def fake_port(monkeypatch):
    """A SerialBus wired to a FakeSerial instead of a real port."""
    fake = FakeSerial()
    monkeypatch.setattr(roboclaw_driver.serial, "Serial", lambda *args, **kwargs: fake)
    bus = SerialBus("/dev/null", 115200)
    return bus, fake


def read_frame(command: int, words: list[int]) -> bytes:
    """Builds a valid read response: payload words followed by their CRC."""
    payload = b"".join(w.to_bytes(4, "big") for w in words)
    crc = crc16(bytes([ADDRESS, command]) + payload)
    return payload + crc.to_bytes(2, "big")


def test_command_numbers_match_the_protocol():
    """Pins the literals from the RoboClaw manual's packet-serial table.

    Every other test here references these symbols, so without this the whole
    suite is invariant to the numbers it depends on: a typo'd command number
    would reach the motor controller with nothing failing.
    """
    assert CMD_READ_ENCODERS == 78
    assert CMD_READ_ISPEEDS == 79
    assert CMD_DUTY_M1M2 == 34
    assert CMD_SPEED_ACCEL_M1M2 == 40
    assert CMD_READ_M1_VELOCITY_PID == 55


def test_accel_field_is_unsigned_full_width(fake_port):
    """Accel occupies the full unsigned 32-bit range, not the signed one."""
    bus, fake = fake_port
    fake.to_read.extend(ACK)
    rc = RoboClaw(bus, ADDRESS)

    rc.speed_accel_m1m2(UINT32_MAX, 0, 0)

    assert bytes(fake.written[2:6]) == b"\xff\xff\xff\xff"


def test_float_accel_and_speeds_are_rounded_not_rejected(fake_port):
    """Checkpoint 4 derives these from float physics; they must not raise."""
    bus, fake = fake_port
    fake.to_read.extend(ACK)
    rc = RoboClaw(bus, ADDRESS)

    assert rc.speed_accel_m1m2(5999.6, 1280.7, -1280.7) is True

    assert bytes(fake.written[2:6]) == (6000).to_bytes(4, "big")
    assert bytes(fake.written[6:10]) == (1281).to_bytes(4, "big", signed=True)
    assert bytes(fake.written[10:14]) == (-1281).to_bytes(4, "big", signed=True)


def test_crc16_matches_the_xmodem_check_value():
    """Pins the CRC against the published CRC-16/XMODEM check value.

    Without an external vector, every other frame assertion here would be
    circular: a wrong CRC implementation would produce wrong expected bytes
    and still pass.
    """
    assert crc16(b"123456789") == 0x31C3


def test_speed_accel_writes_the_expected_frame(fake_port):
    """Accel is unsigned, both speeds are signed, all big-endian."""
    bus, fake = fake_port
    fake.to_read.extend(ACK)
    rc = RoboClaw(bus, ADDRESS)

    assert rc.speed_accel_m1m2(6000, 1281, -1281) is True

    payload = (
        (6000).to_bytes(4, "big")
        + (1281).to_bytes(4, "big", signed=True)
        + (-1281).to_bytes(4, "big", signed=True)
    )
    packet = bytes([ADDRESS, CMD_SPEED_ACCEL_M1M2]) + payload
    expected = packet + crc16(packet).to_bytes(2, "big")
    assert bytes(fake.written) == expected
    assert len(expected) == 2 + 12 + 2


def test_speed_accel_encodes_negative_speeds_as_twos_complement(fake_port):
    """A reversing wheel must not wrap to a huge forward speed."""
    bus, fake = fake_port
    fake.to_read.extend(ACK)
    rc = RoboClaw(bus, ADDRESS)

    rc.speed_accel_m1m2(0, -1, -2)

    assert bytes(fake.written[6:10]) == b"\xff\xff\xff\xff"
    assert bytes(fake.written[10:14]) == b"\xff\xff\xff\xfe"


def test_speed_accel_floors_negative_acceleration(fake_port):
    """Accel is unsigned on the wire, so a negative value must not wrap."""
    bus, fake = fake_port
    fake.to_read.extend(ACK)
    rc = RoboClaw(bus, ADDRESS)

    rc.speed_accel_m1m2(-500, 0, 0)

    assert bytes(fake.written[2:6]) == b"\x00\x00\x00\x00"


def test_speed_accel_clamps_instead_of_raising(fake_port):
    """An absurd upstream speed must not raise inside a control callback."""
    bus, fake = fake_port
    fake.to_read.extend(ACK)
    rc = RoboClaw(bus, ADDRESS)

    assert rc.speed_accel_m1m2(0, 2**40, -(2**40)) is True
    assert bytes(fake.written[6:10]) == b"\x7f\xff\xff\xff"
    assert bytes(fake.written[10:14]) == b"\x80\x00\x00\x00"


def test_speed_accel_retries_then_reports_failure(fake_port):
    """No ack means the command was not applied; say so after retrying."""
    bus, fake = fake_port
    rc = RoboClaw(bus, ADDRESS, retries=3)

    assert rc.speed_accel_m1m2(6000, 100, 100) is False
    assert fake.buffer_resets == 3
    assert len(fake.written) == 3 * 16


def test_read_velocity_pid_descales_gains_and_returns_qpps(fake_port):
    """Gains come back as gain * 65536; QPPS is a raw count rate."""
    bus, fake = fake_port
    words = [int(1.5 * 65536), int(0.5 * 65536), int(0.25 * 65536), 44000]
    fake.to_read.extend(read_frame(CMD_READ_M1_VELOCITY_PID, words))
    rc = RoboClaw(bus, ADDRESS)

    assert rc.read_velocity_pid() == (1.5, 0.5, 0.25, 44000)


def test_read_velocity_pid_reads_sixteen_payload_bytes(fake_port):
    """Four longs, not the two that the encoder read expects."""
    bus, fake = fake_port
    frame = read_frame(CMD_READ_M1_VELOCITY_PID, [0, 0, 0, 45000])
    assert len(frame) == 18
    fake.to_read.extend(frame)
    rc = RoboClaw(bus, ADDRESS)

    assert rc.read_velocity_pid() == (0.0, 0.0, 0.0, 45000)
    assert not fake.to_read, "the whole frame should have been consumed"


def test_read_velocity_pid_does_not_sign_extend_qpps(fake_port):
    """QPPS is unsigned; a high bit set must not read as a negative rate."""
    bus, fake = fake_port
    fake.to_read.extend(read_frame(CMD_READ_M1_VELOCITY_PID, [0, 0, 0, 0x80000000]))
    rc = RoboClaw(bus, ADDRESS)

    result = rc.read_velocity_pid()
    assert result is not None
    assert result[3] == 0x80000000


def test_read_velocity_pid_returns_none_on_bad_crc(fake_port):
    """A corrupt frame must not be parsed into plausible-looking gains."""
    bus, fake = fake_port
    frame = bytearray(read_frame(CMD_READ_M1_VELOCITY_PID, [0, 0, 0, 44000]))
    frame[-1] ^= 0xFF
    fake.to_read.extend(frame)
    rc = RoboClaw(bus, ADDRESS, retries=1)

    assert rc.read_velocity_pid() is None


def test_read_encoders_still_reads_two_signed_words(fake_port):
    """Regression: generalizing the read must not disturb the encoder path."""
    bus, fake = fake_port
    fake.to_read.extend(read_frame(CMD_READ_ENCODERS, [1000, 0xFFFFFFFF]))
    rc = RoboClaw(bus, ADDRESS)

    assert rc.read_encoders() == (1000, -1)


def test_read_speeds_uses_the_instantaneous_speed_command(fake_port):
    """Command 79 is ISpeeds; 108 is the averaged variant."""
    bus, fake = fake_port
    fake.to_read.extend(read_frame(CMD_READ_ISPEEDS, [1281, 0xFFFFFAFF]))
    rc = RoboClaw(bus, ADDRESS)

    assert rc.read_speeds() == (1281, -1281)
    assert bytes(fake.written) == bytes([ADDRESS, CMD_READ_ISPEEDS])


def test_short_read_returns_none(fake_port):
    """A truncated response is a failed read, not a partial parse."""
    bus, fake = fake_port
    fake.to_read.extend(b"\x00\x00\x00\x01")
    rc = RoboClaw(bus, ADDRESS, retries=1)

    assert rc.read_encoders() is None

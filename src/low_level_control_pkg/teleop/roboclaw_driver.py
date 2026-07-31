# Copyright 2026
#
# RoboClaw packet-serial driver: quadrature encoder/speed reads, a velocity
# PID/QPPS read that establishes the speed ceiling, closed-loop speed writes,
# and a raw duty-cycle write kept for the shutdown stop.
# Owned exclusively by roboclaw_driver_node -- it is the
# only process that opens these serial ports (see that node's docstring for
# why: RoboClaw's packet-serial protocol is half-duplex on a single UART, so
# two independent OS processes issuing transactions on the same port would
# interleave/corrupt each other's packets).
#
# Protocol reference: BasicMicro RoboClaw User Manual (packet serial mode).
# Each command packet is [address, command, ...payload..., crc_hi, crc_lo].
# For READ commands we send [address, command] and read back the payload
# followed by a 2-byte CRC16, computed over every byte the host sent *and*
# every payload byte the controller returned. For WRITE commands we send
# [address, command, ...payload..., crc_hi, crc_lo] and the controller
# replies with a single 0xFF byte if the CRC matched (no reply / a timeout
# means the command was not applied).

import threading

import serial

# RoboClaw command numbers.
CMD_READ_ENCODERS = 78  # returns Enc1 (4B), Enc2 (4B), CRC (2B)
CMD_READ_ISPEEDS = 79  # returns Speed1 (4B), Speed2 (4B), CRC (2B), counts/sec
CMD_DUTY_M1M2 = 34  # payload: Duty1 (2B signed), Duty2 (2B signed)
CMD_SPEED_ACCEL_M1M2 = 40  # payload: Accel (4B unsigned), Speed1, Speed2 (4B signed)
CMD_READ_M1_VELOCITY_PID = 55  # returns P, I, D, QPPS (4x 4B), CRC (2B)

DUTY_FULL_SCALE = 32767  # signed 16-bit; +-DUTY_FULL_SCALE = +-100% duty

# Velocity PID gains are stored on the controller as fixed point, gain * 65536.
PID_FIXED_POINT_SCALE = 65536.0

# Speeds and accelerations cross the wire as 32-bit words. Commands are clamped
# to that range so a bad upstream value cannot raise OverflowError inside a
# control-loop callback; the meaningful limits are enforced by the caller.
INT32_MIN = -(2**31)
INT32_MAX = 2**31 - 1
UINT32_MAX = 2**32 - 1


def crc16(data):
    """CRC16-CCITT (poly 0x1021, init 0x0000) as used by RoboClaw."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _to_signed_32(value):
    """Interpret an unsigned 32-bit value as signed (two's complement)."""
    return value - 0x100000000 if value & 0x80000000 else value


def _clamp_int32(value: float) -> int:
    """Rounds and bounds a value to the signed 32-bit wire range."""
    return min(max(round(value), INT32_MIN), INT32_MAX)


def _clamp_uint32(value: float) -> int:
    """Rounds and bounds a value to the unsigned 32-bit wire range."""
    return min(max(round(value), 0), UINT32_MAX)


class SerialBus:
    """Owns a single serial port and serializes access from multiple RoboClaws.

    Two RoboClaws sharing one bus (different addresses) must not interleave
    their request/response exchanges, so every transaction holds a lock.
    """

    def __init__(self, port, baud, timeout=0.1):
        self.port = port
        self._serial = serial.Serial(port, baudrate=baud, timeout=timeout)
        self._lock = threading.Lock()

    def transaction(self, address, command, response_len):
        """Send a read command and return `response_len` payload bytes.

        Returns the payload (without CRC) on success, or None if the read
        timed out or the CRC did not verify.
        """
        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(bytes([address, command]))
            # response_len payload bytes + 2 CRC bytes
            raw = self._serial.read(response_len + 2)

        if len(raw) != response_len + 2:
            return None

        payload = raw[:response_len]
        received_crc = (raw[response_len] << 8) | raw[response_len + 1]
        expected_crc = crc16(bytes([address, command]) + payload)
        if received_crc != expected_crc:
            return None
        return payload

    def write_transaction(self, address, command, payload):
        """Send a write command with CRC16 appended; return True iff the
        RoboClaw acknowledged with a single 0xFF byte."""
        packet = bytes([address, command]) + payload
        crc = crc16(packet)
        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(packet + bytes([(crc >> 8) & 0xFF, crc & 0xFF]))
            ack = self._serial.read(1)
        return ack == b"\xff"

    def close(self):
        try:
            self._serial.close()
        except Exception:
            pass


class RoboClaw:
    """One RoboClaw controller (two motor channels: M1, M2) on a SerialBus."""

    def __init__(self, bus, address, retries=3):
        self._bus = bus
        self._address = address
        self._retries = retries

    def _read_words(
        self, command: int, count: int, *, signed: bool = True
    ) -> tuple[int, ...] | None:
        """Reads `count` big-endian 32-bit words, retrying on timeout or CRC.

        Args:
            command: RoboClaw read command number.
            count: How many 32-bit words the command returns.
            signed: Whether to reinterpret each word as two's complement.
                Encoder counts and speeds are signed; PID gains and QPPS
                are not.

        Returns:
            The words in wire order, or None if every retry failed.
        """
        for _ in range(self._retries):
            payload = self._bus.transaction(self._address, command, 4 * count)
            if payload is not None:
                words = tuple(
                    int.from_bytes(payload[i : i + 4], "big")
                    for i in range(0, 4 * count, 4)
                )
                return tuple(_to_signed_32(w) for w in words) if signed else words
        return None

    def read_encoders(self):
        """Return (enc_m1, enc_m2) absolute quadrature counts, or None."""
        return self._read_words(CMD_READ_ENCODERS, 2)

    def read_speeds(self):
        """Return (speed_m1, speed_m2) in counts/sec, or None.

        Instantaneous speeds (command 79), not the filtered speeds command 108
        reports. Check the manual before building a slip monitor on either --
        the filtering window is not documented in the vendored references.
        """
        return self._read_words(CMD_READ_ISPEEDS, 2)
    
    def read_velocity_pid(self) -> tuple[float, float, float, int] | None:
        """Reads M1's active velocity PID constants and QPPS ceiling.

        These are the controller's live settings, loaded from EEPROM at boot.
        They diverge from what is stored if the PID was set over serial
        without a WRITENVM, so a read-back does not prove the values survive
        a power cycle.

        QPPS is what the controller was calibrated to treat as full speed, so
        it bounds any closed-loop speed command. Only M1 is queried, matching
        the rover_control drivetrain; verify M1/M2 parity in Motion Studio.

        Returns:
            (p, i, d, qpps) with the gains descaled out of fixed point, or
            None if the read failed.
        """
        words = self._read_words(CMD_READ_M1_VELOCITY_PID, 4, signed=False)
        if words is None:
            return None
        p, i, d, qpps = words
        return (
            p / PID_FIXED_POINT_SCALE,
            i / PID_FIXED_POINT_SCALE,
            d / PID_FIXED_POINT_SCALE,
            qpps,
        )

    def speed_accel_m1m2(self, accel: float, speed_m1: float, speed_m2: float) -> bool:
        """Commands both channels to a target speed under an acceleration ramp.

        Closed-loop, unlike `duty_m1m2`: the controller's own velocity PID
        holds the commanded counts/sec, so a wheel that loses traction gets
        throttled back instead of running away. The ramp bounds the torque
        step that breaks traction in the first place.

        Args:
            accel: Acceleration limit in counts/sec^2, applied to both
                channels. Negative values are treated as zero.
            speed_m1: Target speed for M1 in counts/sec, signed.
            speed_m2: Target speed for M2 in counts/sec, signed.

        Returns:
            True if the RoboClaw acknowledged, False otherwise (treat as
            "not applied").
        """
        payload = (
            _clamp_uint32(accel).to_bytes(4, "big")
            + _clamp_int32(speed_m1).to_bytes(4, "big", signed=True)
            + _clamp_int32(speed_m2).to_bytes(4, "big", signed=True)
        )
        for _ in range(self._retries):
            if self._bus.write_transaction(
                self._address, CMD_SPEED_ACCEL_M1M2, payload
            ):
                return True
        return False

    def duty_m1m2(self, duty1, duty2):
        """Open-loop duty cycle for M1 and M2, each in [-1.0, 1.0]
        (-1.0 = full reverse, 0.0 = stop, 1.0 = full forward). This is NOT
        closed-loop speed control: duty is not speed, and a wheel that loses
        traction speeds up at a fixed duty with nothing to correct it. Use
        speed_accel_m1m2 for driving; this remains for the shutdown hard-stop
        via stop(), where cutting drive outright is what is wanted. Returns
        True if the RoboClaw acknowledged, False otherwise (treat as "not
        applied").
        """
        d1 = int(round(max(-1.0, min(1.0, duty1)) * DUTY_FULL_SCALE))
        d2 = int(round(max(-1.0, min(1.0, duty2)) * DUTY_FULL_SCALE))
        payload = d1.to_bytes(2, "big", signed=True) + d2.to_bytes(
            2, "big", signed=True
        )
        for _ in range(self._retries):
            if self._bus.write_transaction(self._address, CMD_DUTY_M1M2, payload):
                return True
        return False

    def stop(self):
        """Zero duty on both channels."""
        return self.duty_m1m2(0.0, 0.0)

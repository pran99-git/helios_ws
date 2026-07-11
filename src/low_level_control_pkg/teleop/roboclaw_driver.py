# Copyright 2026
#
# RoboClaw packet-serial driver: quadrature encoder/speed reads and open-loop
# duty-cycle writes. Owned exclusively by roboclaw_driver_node -- it is the
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
CMD_READ_ENCODERS = 78   # returns Enc1 (4B), Enc2 (4B), CRC (2B)
CMD_READ_SPEEDS = 79     # returns Speed1 (4B), Speed2 (4B), CRC (2B), counts/sec
CMD_DUTY_M1M2 = 34       # payload: Duty1 (2B signed), Duty2 (2B signed)

DUTY_FULL_SCALE = 32767  # signed 16-bit; +-DUTY_FULL_SCALE = +-100% duty


def crc16(data):
    """CRC16-CCITT (poly 0x1021, init 0x0000) as used by RoboClaw."""
    crc = 0
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _to_signed_32(value):
    """Interpret an unsigned 32-bit value as signed (two's complement)."""
    return value - 0x100000000 if value & 0x80000000 else value


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
        return ack == b'\xff'

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

    def _read(self, command):
        for _ in range(self._retries):
            payload = self._bus.transaction(self._address, command, 8)
            if payload is not None:
                m1 = _to_signed_32(int.from_bytes(payload[0:4], 'big'))
                m2 = _to_signed_32(int.from_bytes(payload[4:8], 'big'))
                return m1, m2
        return None

    def read_encoders(self):
        """Return (enc_m1, enc_m2) absolute quadrature counts, or None."""
        return self._read(CMD_READ_ENCODERS)

    def read_speeds(self):
        """Return (speed_m1, speed_m2) in counts/sec, or None."""
        return self._read(CMD_READ_SPEEDS)

    def duty_m1m2(self, duty1, duty2):
        """Open-loop duty cycle for M1 and M2, each in [-1.0, 1.0]
        (-1.0 = full reverse, 0.0 = stop, 1.0 = full forward). This is NOT
        closed-loop speed control -- the RoboClaw's velocity PID/QPPS limits
        are not configured/verified on this hardware, so there is no fixed
        real-world speed at a given duty. Returns True if the RoboClaw
        acknowledged the command, False otherwise (treat as "not applied").
        """
        d1 = int(round(max(-1.0, min(1.0, duty1)) * DUTY_FULL_SCALE))
        d2 = int(round(max(-1.0, min(1.0, duty2)) * DUTY_FULL_SCALE))
        payload = d1.to_bytes(2, 'big', signed=True) + d2.to_bytes(2, 'big', signed=True)
        for _ in range(self._retries):
            if self._bus.write_transaction(self._address, CMD_DUTY_M1M2, payload):
                return True
        return False

    def stop(self):
        """Zero duty on both channels."""
        return self.duty_m1m2(0.0, 0.0)

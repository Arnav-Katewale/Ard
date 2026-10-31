"""LewanSoul / Hiwonder LX-16A serial bus servo protocol.

All servos share one half-duplex serial line (115200 baud). Every packet is:

    0x55 0x55 | ID | LENGTH | CMD | PARAMS... | CHECKSUM

    LENGTH   = number of params + 3
    CHECKSUM = ~(ID + LENGTH + CMD + sum(PARAMS)) & 0xFF

Positions are 0..1000 units covering 0..240 degrees (0.24 deg per unit).

The serial port is passed in, so the same code talks to real hardware through
pyserial or to SimulatedLx16aPort in software validation. Hardware validation
of this driver is pending.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

HEADER = b"\x55\x55"
BROADCAST_ID = 0xFE

MOVE_TIME_WRITE = 1
MOVE_STOP = 12
ID_READ = 14
TEMP_READ = 26
VIN_READ = 27
POS_READ = 28
LOAD_OR_UNLOAD_WRITE = 31

POSITION_UNITS_MAX = 1000
ANGLE_RANGE_DEG = 240.0
DEG_PER_UNIT = ANGLE_RANGE_DEG / POSITION_UNITS_MAX
MAX_MOVE_TIME_MS = 30000


class Lx16aError(IOError):
    """Malformed, missing or unexpected response from a servo."""


class SerialPort(Protocol):
    def write(self, data: bytes) -> int | None: ...
    def read(self, size: int) -> bytes: ...
    def reset_input_buffer(self) -> None: ...


@dataclass(frozen=True)
class ServoHealth:
    servo_id: int
    position_deg: float
    voltage_v: float
    temperature_c: int


def checksum(servo_id: int, length: int, cmd: int, params: bytes) -> int:
    return (~(servo_id + length + cmd + sum(params))) & 0xFF


def encode_packet(servo_id: int, cmd: int, params: bytes = b"") -> bytes:
    if not 0 <= servo_id <= 0xFE:
        raise ValueError(f"servo id {servo_id} out of range 0..254")
    length = len(params) + 3
    return HEADER + bytes([servo_id, length, cmd]) + params + bytes([checksum(servo_id, length, cmd, params)])


def decode_packet(packet: bytes) -> tuple[int, int, bytes]:
    """Validate a complete packet and return (servo_id, cmd, params)."""
    if len(packet) < 6 or packet[:2] != HEADER:
        raise Lx16aError(f"bad packet header: {packet.hex(' ')}")
    servo_id, length, cmd = packet[2], packet[3], packet[4]
    if len(packet) != length + 3:
        raise Lx16aError(f"packet length {len(packet)} does not match LENGTH field {length}")
    params = packet[5:-1]
    if packet[-1] != checksum(servo_id, length, cmd, params):
        raise Lx16aError(f"checksum mismatch in {packet.hex(' ')}")
    return servo_id, cmd, params


def degrees_to_units(angle_deg: float) -> int:
    units = round(angle_deg / DEG_PER_UNIT)
    if not 0 <= units <= POSITION_UNITS_MAX:
        raise ValueError(f"angle {angle_deg:.2f} deg is outside the servo's 0..240 deg range")
    return units


def units_to_degrees(units: int) -> float:
    return units * DEG_PER_UNIT


class Lx16aBus:
    """Sends commands to and reads replies from servos on one bus.

    echo=True discards our own transmitted bytes, which appear on the receive
    line when the Pi UART is wired to the bus through a simple half-duplex
    circuit. A USB BusLinker board does not echo.
    """

    def __init__(self, port: SerialPort, echo: bool = False):
        self._port = port
        self._echo = echo

    # --- commands ---------------------------------------------------------------

    def move(self, servo_id: int, angle_deg: float, time_ms: int) -> None:
        if not 0 <= time_ms <= MAX_MOVE_TIME_MS:
            raise ValueError(f"move time {time_ms} ms outside 0..{MAX_MOVE_TIME_MS}")
        units = degrees_to_units(angle_deg)
        params = units.to_bytes(2, "little") + int(time_ms).to_bytes(2, "little")
        self._send(servo_id, MOVE_TIME_WRITE, params)

    def stop(self, servo_id: int) -> None:
        """Stop the servo immediately at its current position."""
        self._send(servo_id, MOVE_STOP)

    def set_torque(self, servo_id: int, enabled: bool) -> None:
        """enabled=False lets the joint move freely (motor unloaded)."""
        self._send(servo_id, LOAD_OR_UNLOAD_WRITE, bytes([1 if enabled else 0]))

    # --- reads ------------------------------------------------------------------

    def read_id(self, servo_id: int = BROADCAST_ID) -> int:
        """With BROADCAST_ID, only use when exactly one servo is connected."""
        return self._query(servo_id, ID_READ, 1)[0]

    def read_position_deg(self, servo_id: int) -> float:
        units = int.from_bytes(self._query(servo_id, POS_READ, 2), "little", signed=True)
        return units_to_degrees(units)

    def read_voltage_v(self, servo_id: int) -> float:
        return int.from_bytes(self._query(servo_id, VIN_READ, 2), "little") / 1000.0

    def read_temperature_c(self, servo_id: int) -> int:
        return self._query(servo_id, TEMP_READ, 1)[0]

    def read_health(self, servo_id: int) -> ServoHealth:
        return ServoHealth(
            servo_id=servo_id,
            position_deg=self.read_position_deg(servo_id),
            voltage_v=self.read_voltage_v(servo_id),
            temperature_c=self.read_temperature_c(servo_id),
        )

    # --- low level --------------------------------------------------------------

    def _send(self, servo_id: int, cmd: int, params: bytes = b"") -> None:
        packet = encode_packet(servo_id, cmd, params)
        self._port.reset_input_buffer()
        self._port.write(packet)
        if self._echo:
            self._read_exact(len(packet))

    def _query(self, servo_id: int, cmd: int, n_params: int) -> bytes:
        self._send(servo_id, cmd)
        reply_id, reply_cmd, params = self._read_packet()
        if reply_cmd != cmd or (servo_id != BROADCAST_ID and reply_id != servo_id):
            raise Lx16aError(f"unexpected reply id={reply_id} cmd={reply_cmd} to id={servo_id} cmd={cmd}")
        if len(params) != n_params:
            raise Lx16aError(f"expected {n_params} param bytes, got {len(params)}")
        return params

    def _read_packet(self) -> bytes:
        head = self._read_exact(4)  # 0x55 0x55 ID LENGTH
        if head[:2] != HEADER:
            raise Lx16aError(f"bad reply header: {head.hex(' ')}")
        rest = self._read_exact(head[3] - 1)  # CMD + PARAMS + CHECKSUM
        return decode_packet(head + rest)

    def _read_exact(self, size: int) -> bytes:
        data = self._port.read(size)
        if len(data) != size:
            raise Lx16aError(f"no response from servo (expected {size} bytes, got {len(data)})")
        return data


def open_serial_port(device: str, baudrate: int = 115200, timeout_s: float = 0.05):
    """Open a real serial port (requires pyserial; Raspberry Pi only in practice)."""
    import serial

    return serial.Serial(device, baudrate=baudrate, timeout=timeout_s)

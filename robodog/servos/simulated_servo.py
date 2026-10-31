"""A simulated LX-16A bus for software validation.

SimulatedLx16aPort behaves like a serial port with LX-16A servos attached: it
decodes the real protocol bytes written by Lx16aBus and replies with real
protocol packets. This exercises the protocol code without hardware.

Simplification: simulated servos reach a commanded position immediately.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from robodog.servos import lx16a


@dataclass
class SimulatedServo:
    servo_id: int
    position_units: int = 500  # 120 deg, mid-range
    voltage_mv: int = 7400
    temperature_c: int = 30
    torque_enabled: bool = True
    stopped: bool = False


@dataclass
class SimulatedLx16aPort:
    servos: dict[int, SimulatedServo] = field(default_factory=dict)
    echo: bool = False
    corrupt_next_reply: bool = False
    written: list[bytes] = field(default_factory=list)
    _rx: bytearray = field(default_factory=bytearray)

    @classmethod
    def with_servos(cls, ids, **kwargs) -> "SimulatedLx16aPort":
        return cls(servos={i: SimulatedServo(i) for i in ids}, **kwargs)

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def read(self, size: int) -> bytes:
        out = bytes(self._rx[:size])
        del self._rx[:size]
        return out

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        if self.echo:
            self._rx += data
        servo_id, cmd, params = lx16a.decode_packet(bytes(data))
        if servo_id == lx16a.BROADCAST_ID:
            targets = list(self.servos.values())
        else:
            targets = [self.servos[servo_id]] if servo_id in self.servos else []
        for servo in targets:
            self._handle(servo, cmd, params)
        return len(data)

    def _handle(self, servo: SimulatedServo, cmd: int, params: bytes) -> None:
        if cmd == lx16a.MOVE_TIME_WRITE:
            servo.position_units = int.from_bytes(params[0:2], "little")
            servo.stopped = False
        elif cmd == lx16a.MOVE_STOP:
            servo.stopped = True
        elif cmd == lx16a.LOAD_OR_UNLOAD_WRITE:
            servo.torque_enabled = bool(params[0])
        elif cmd == lx16a.ID_READ:
            self._reply(servo, cmd, bytes([servo.servo_id]))
        elif cmd == lx16a.POS_READ:
            self._reply(servo, cmd, servo.position_units.to_bytes(2, "little", signed=True))
        elif cmd == lx16a.VIN_READ:
            self._reply(servo, cmd, servo.voltage_mv.to_bytes(2, "little"))
        elif cmd == lx16a.TEMP_READ:
            self._reply(servo, cmd, bytes([servo.temperature_c]))

    def _reply(self, servo: SimulatedServo, cmd: int, params: bytes) -> None:
        packet = bytearray(lx16a.encode_packet(servo.servo_id, cmd, params))
        if self.corrupt_next_reply:
            packet[-1] ^= 0xFF
            self.corrupt_next_reply = False
        self._rx += packet

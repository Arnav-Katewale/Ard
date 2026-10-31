import pytest

from robodog.servos import lx16a
from robodog.servos.lx16a import Lx16aBus, Lx16aError
from robodog.servos.simulated_servo import SimulatedLx16aPort


# --- packet encoding (byte-exact) -------------------------------------------------


def test_move_packet_bytes():
    # Servo 1 to position 500 over 1000 ms:
    # LEN = 4 params + 3 = 7; checksum = ~(1+7+1+0xF4+0x01+0xE8+0x03) & 0xFF
    expected = bytes([0x55, 0x55, 0x01, 0x07, 0x01, 0xF4, 0x01, 0xE8, 0x03])
    body_sum = 0x01 + 0x07 + 0x01 + 0xF4 + 0x01 + 0xE8 + 0x03
    expected += bytes([(~body_sum) & 0xFF])
    params = (500).to_bytes(2, "little") + (1000).to_bytes(2, "little")
    assert lx16a.encode_packet(1, lx16a.MOVE_TIME_WRITE, params) == expected


def test_position_read_request_bytes():
    # ID 3, LEN 3, CMD 28: checksum = ~(3+3+28) = ~34 = 0xDD
    assert lx16a.encode_packet(3, lx16a.POS_READ) == bytes([0x55, 0x55, 0x03, 0x03, 0x1C, 0xDD])


def test_decode_round_trip():
    packet = lx16a.encode_packet(7, lx16a.VIN_READ, b"\x08\x1d")
    assert lx16a.decode_packet(packet) == (7, lx16a.VIN_READ, b"\x08\x1d")


def test_decode_rejects_bad_checksum():
    packet = bytearray(lx16a.encode_packet(7, lx16a.TEMP_READ, b"\x1e"))
    packet[-1] ^= 0x01
    with pytest.raises(Lx16aError, match="checksum"):
        lx16a.decode_packet(bytes(packet))


def test_invalid_servo_id_rejected():
    with pytest.raises(ValueError):
        lx16a.encode_packet(255, lx16a.MOVE_STOP)


# --- unit conversion --------------------------------------------------------------


def test_degree_unit_conversion():
    assert lx16a.degrees_to_units(120.0) == 500
    assert lx16a.units_to_degrees(1000) == pytest.approx(240.0)


@pytest.mark.parametrize("angle", [-1.0, 241.0])
def test_angle_outside_servo_range_rejected(angle):
    with pytest.raises(ValueError):
        lx16a.degrees_to_units(angle)


# --- bus behaviour against the simulated port ---------------------------------------


def test_read_health():
    port = SimulatedLx16aPort.with_servos([1])
    health = Lx16aBus(port).read_health(1)
    assert health.servo_id == 1
    assert health.position_deg == pytest.approx(120.0)
    assert health.voltage_v == pytest.approx(7.4)
    assert health.temperature_c == 30


def test_broadcast_id_read_with_single_servo():
    port = SimulatedLx16aPort.with_servos([5])
    assert Lx16aBus(port).read_id() == 5


def test_move_then_read_back_position():
    port = SimulatedLx16aPort.with_servos([2])
    bus = Lx16aBus(port)
    bus.move(2, 100.8, time_ms=500)
    assert bus.read_position_deg(2) == pytest.approx(100.8)


def test_stop_and_torque_commands_reach_servo():
    port = SimulatedLx16aPort.with_servos([2])
    bus = Lx16aBus(port)
    bus.stop(2)
    bus.set_torque(2, False)
    assert port.servos[2].stopped and not port.servos[2].torque_enabled


def test_missing_servo_raises_no_response():
    bus = Lx16aBus(SimulatedLx16aPort.with_servos([1]))
    with pytest.raises(Lx16aError, match="no response"):
        bus.read_position_deg(9)


def test_corrupted_reply_detected():
    port = SimulatedLx16aPort.with_servos([1], corrupt_next_reply=True)
    with pytest.raises(Lx16aError, match="checksum"):
        Lx16aBus(port).read_temperature_c(1)


def test_echo_mode_discards_transmitted_bytes():
    port = SimulatedLx16aPort.with_servos([1], echo=True)
    assert Lx16aBus(port, echo=True).read_temperature_c(1) == 30


def test_move_time_validated():
    bus = Lx16aBus(SimulatedLx16aPort.with_servos([1]))
    with pytest.raises(ValueError):
        bus.move(1, 120.0, time_ms=40000)

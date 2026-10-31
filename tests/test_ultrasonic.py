import dataclasses
import math

import pytest

from robodog.config import ConfigError, build, load_config, section
from robodog.sensors import SensorStatus
from robodog.sensors.simulated_sensors import SimulatedEchoBackend
from robodog.sensors.ultrasonic import (
    LgpioEchoBackend,
    UltrasonicConfig,
    UltrasonicSensor,
    echo_to_distance_cm,
)

CONFIG = UltrasonicConfig(
    min_distance_cm=2.0,
    max_distance_cm=300.0,
    timeout_s=0.025,
    min_interval_s=0.06,
    speed_of_sound_ms=343.0,
    echo_voltage_safe=False,
    gpio_chip=0,
    trigger_pin=23,
    echo_pin=24,
)


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.slept = []

    def __call__(self):
        return self.now

    def sleep(self, s):
        self.slept.append(s)
        self.now += s


def make_sensor(distance_cm=50.0, **backend_kwargs):
    backend = SimulatedEchoBackend(343.0, **backend_kwargs)
    backend.distance_cm = distance_cm
    clock = FakeClock()
    return UltrasonicSensor(backend, CONFIG, clock=clock, sleep=clock.sleep), backend, clock


def test_echo_time_to_distance():
    # 1 m away -> 2 m round trip -> 2 / 343 s
    assert echo_to_distance_cm(2.0 / 343.0, 343.0) == pytest.approx(100.0)


def test_valid_reading():
    sensor, _, _ = make_sensor(42.0)
    r = sensor.read()
    assert r.status is SensorStatus.OK and r.valid
    assert r.distance_cm == pytest.approx(42.0)


def test_timeout_reading():
    sensor, _, _ = make_sensor(dropout_probability=1.0)
    r = sensor.read()
    assert r.status is SensorStatus.TIMEOUT and not r.valid
    assert math.isnan(r.distance_cm)


def test_too_close_is_out_of_range():
    sensor, _, _ = make_sensor(1.0)
    r = sensor.read()
    assert r.status is SensorStatus.OUT_OF_RANGE and not r.valid


def test_backend_exception_is_error():
    class Broken:
        def measure_echo_s(self, timeout_s):
            raise OSError("gpio failure")

    r = UltrasonicSensor(Broken(), CONFIG).read()
    assert r.status is SensorStatus.ERROR


def test_minimum_interval_between_pings():
    sensor, _, clock = make_sensor()
    sensor.read()
    clock.now += 0.01
    sensor.read()
    assert clock.slept == [pytest.approx(0.05)]


def test_timeout_must_cover_max_distance_round_trip():
    with pytest.raises(ConfigError, match="round trip"):
        dataclasses.replace(CONFIG, timeout_s=0.001)


def test_hardware_backend_refuses_without_voltage_confirmation():
    with pytest.raises(ConfigError, match="echo_voltage_safe"):
        LgpioEchoBackend(CONFIG)


def test_repository_config_builds_and_is_safe_by_default():
    cfg = build(UltrasonicConfig, section(load_config(), "ultrasonic"), "ultrasonic")
    assert cfg.echo_voltage_safe is False

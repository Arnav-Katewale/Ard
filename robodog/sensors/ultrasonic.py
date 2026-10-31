"""Ultrasonic distance sensor interface (trigger/echo time of flight).

distance = echo_time * speed_of_sound / 2   (the pulse travels out and back)

No filtering or calibration is applied here; those belong to sensor calibration.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Protocol

from robodog.config import ConfigError, check
from robodog.sensors import SensorStatus


class EchoBackend(Protocol):
    def measure_echo_s(self, timeout_s: float) -> float | None:
        """Return the echo pulse width in seconds, or None on timeout."""
        ...


@dataclass(frozen=True)
class UltrasonicConfig:
    min_distance_cm: float
    max_distance_cm: float
    timeout_s: float
    min_interval_s: float      # wait between pings so old echoes die out
    speed_of_sound_ms: float
    # Must be true only after confirming the Echo signal reaching the Pi is
    # <= 3.3 V (3.3 V sensor, or 5 V sensor with divider / level shifter).
    echo_voltage_safe: bool
    gpio_chip: int
    trigger_pin: int
    echo_pin: int

    def __post_init__(self):
        check(0 < self.min_distance_cm < self.max_distance_cm, "ultrasonic: need 0 < min < max distance")
        check(self.speed_of_sound_ms > 0, "ultrasonic: speed_of_sound_ms must be positive")
        check(self.min_interval_s >= 0, "ultrasonic: min_interval_s must be >= 0")
        round_trip_s = 2 * self.max_distance_cm / 100.0 / self.speed_of_sound_ms
        check(
            self.timeout_s > round_trip_s,
            f"ultrasonic: timeout_s must exceed the {round_trip_s * 1000:.1f} ms round trip at max distance",
        )


@dataclass(frozen=True)
class UltrasonicReading:
    distance_cm: float  # NaN when no echo was measured
    timestamp_s: float
    status: SensorStatus

    @property
    def valid(self) -> bool:
        return self.status is SensorStatus.OK


def echo_to_distance_cm(echo_s: float, speed_of_sound_ms: float) -> float:
    return echo_s * speed_of_sound_ms / 2.0 * 100.0


class UltrasonicSensor:
    def __init__(
        self,
        backend: EchoBackend,
        config: UltrasonicConfig,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ):
        self._backend = backend
        self._config = config
        self._clock = clock
        self._sleep = sleep
        self._last_ping_s: float | None = None

    def read(self) -> UltrasonicReading:
        self._respect_min_interval()
        timestamp = self._clock()
        self._last_ping_s = timestamp
        try:
            echo_s = self._backend.measure_echo_s(self._config.timeout_s)
        except Exception:
            return UltrasonicReading(math.nan, timestamp, SensorStatus.ERROR)
        if echo_s is None:
            return UltrasonicReading(math.nan, timestamp, SensorStatus.TIMEOUT)

        distance = echo_to_distance_cm(echo_s, self._config.speed_of_sound_ms)
        in_range = self._config.min_distance_cm <= distance <= self._config.max_distance_cm
        status = SensorStatus.OK if in_range else SensorStatus.OUT_OF_RANGE
        return UltrasonicReading(distance, timestamp, status)

    def _respect_min_interval(self) -> None:
        if self._last_ping_s is None:
            return
        wait = self._config.min_interval_s - (self._clock() - self._last_ping_s)
        if wait > 0:
            self._sleep(wait)


class LgpioEchoBackend:
    """Raspberry Pi 5 trigger/echo backend using lgpio. Hardware validation pending.

    Timing is done by polling in Python, so Linux scheduling adds jitter
    (expected to be on the order of a centimetre); it will be measured during
    sensor calibration rather than assumed.
    """

    TRIGGER_PULSE_S = 10e-6

    def __init__(self, config: UltrasonicConfig):
        if not config.echo_voltage_safe:
            raise ConfigError(
                "Refusing to use the ultrasonic sensor: echo_voltage_safe is false. "
                "A 5 V Echo signal can damage the Raspberry Pi GPIO. Add a voltage divider "
                "(e.g. 1 kOhm / 2 kOhm) or level shifter, then set echo_voltage_safe = true."
            )
        import lgpio  # only available on the Pi

        self._lgpio = lgpio
        self._trig = config.trigger_pin
        self._echo = config.echo_pin
        self._handle = lgpio.gpiochip_open(config.gpio_chip)
        lgpio.gpio_claim_output(self._handle, self._trig, 0)
        lgpio.gpio_claim_input(self._handle, self._echo)

    def measure_echo_s(self, timeout_s: float) -> float | None:
        lg, h = self._lgpio, self._handle
        lg.gpio_write(h, self._trig, 1)
        end = time.perf_counter() + self.TRIGGER_PULSE_S
        while time.perf_counter() < end:
            pass
        lg.gpio_write(h, self._trig, 0)

        deadline = time.perf_counter() + timeout_s
        while lg.gpio_read(h, self._echo) == 0:
            if time.perf_counter() > deadline:
                return None
        start = time.perf_counter()
        while lg.gpio_read(h, self._echo) == 1:
            if time.perf_counter() > deadline:
                return None
        return time.perf_counter() - start

    def close(self) -> None:
        self._lgpio.gpiochip_close(self._handle)

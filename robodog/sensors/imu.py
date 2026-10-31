"""IMU interface: raw acceleration/gyro readings plus basic pitch and roll.

Pitch and roll here come ONLY from the accelerometer's measurement of gravity.
That is correct when the robot is stationary and becomes noisy when it moves
(walking adds acceleration on top of gravity). It is a basic estimate, not the
final orientation estimator.

The hardware backend for a specific IMU chip is pending until the chip model is
confirmed. Anything with a `read_raw()` method returning RawImuSample works as a
backend, which is how the simulated IMU plugs in.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Protocol

from robodog.config import check
from robodog.sensors import SensorStatus

GRAVITY_MS2 = 9.80665
_AXES = {"x": 0, "y": 1, "z": 2}


@dataclass(frozen=True)
class RawImuSample:
    """One sample in the sensor's own axes. accel in m/s^2, gyro in deg/s."""

    accel: tuple[float, float, float]
    gyro: tuple[float, float, float]


class ImuBackend(Protocol):
    def read_raw(self) -> RawImuSample: ...


@dataclass(frozen=True)
class ImuReading:
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    pitch_deg: float  # basic accelerometer-only estimate
    roll_deg: float   # basic accelerometer-only estimate
    timestamp_s: float
    status: SensorStatus

    @property
    def valid(self) -> bool:
        return self.status is SensorStatus.OK


@dataclass(frozen=True)
class ImuConfig:
    # axis_map[i] says which signed sensor axis gives body axis i (x, y, z).
    # e.g. ["-y", "+x", "+z"] means body x = -sensor y, body y = +sensor x.
    axis_map: list[str]
    max_accel_ms2: float
    max_gyro_dps: float

    def __post_init__(self):
        check(len(self.axis_map) == 3, "imu.axis_map must have 3 entries")
        for entry in self.axis_map:
            check(
                len(entry) == 2 and entry[0] in "+-" and entry[1] in _AXES,
                f"imu.axis_map entry {entry!r} must look like '+x' or '-z'",
            )
        check(
            sorted(e[1] for e in self.axis_map) == ["x", "y", "z"],
            "imu.axis_map must use each sensor axis exactly once",
        )
        check(self.max_accel_ms2 > 0 and self.max_gyro_dps > 0, "imu limits must be positive")


def accel_pitch_roll(ax: float, ay: float, az: float) -> tuple[float, float]:
    """Basic tilt from gravity, in degrees. Body frame: +x fwd, +y left, +z up.

    A stationary accelerometer measures the reaction to gravity, which points
    "up" in the world. Tilting the body changes how that vector splits across
    the body axes:
        roll  = atan2(ay, az)                 positive = left side up
        pitch = atan2(-ax, sqrt(ay^2 + az^2)) positive = nose down
    """
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return pitch, roll


def remap_axes(vec: tuple[float, float, float], axis_map: list[str]) -> tuple[float, float, float]:
    """Convert a sensor-frame vector into the body frame using axis_map."""
    out = []
    for entry in axis_map:
        sign = -1.0 if entry[0] == "-" else 1.0
        out.append(sign * vec[_AXES[entry[1]]])
    return out[0], out[1], out[2]


class Imu:
    """Reads an IMU backend and returns validated, timestamped body-frame readings.

    read() never raises because of a bad sample; it returns a reading whose
    status explains the problem, so higher-level code can decide what to do.

    `calibration` (optional) is an ImuCalibration from robodog.calibration;
    when set, its offsets are removed from every valid reading.
    """

    def __init__(
        self,
        backend: ImuBackend,
        config: ImuConfig,
        clock: Callable[[], float] = time.monotonic,
        calibration=None,
    ):
        self._backend = backend
        self._config = config
        self._clock = clock
        self.calibration = calibration
        self.consecutive_failures = 0

    def read(self) -> ImuReading:
        timestamp = self._clock()
        try:
            raw = self._backend.read_raw()
        except Exception:  # communication failure of any kind
            return self._failed(timestamp, SensorStatus.ERROR)

        accel = remap_axes(raw.accel, self._config.axis_map)
        gyro = remap_axes(raw.gyro, self._config.axis_map)

        if not self._plausible(accel, gyro):
            return self._failed(timestamp, SensorStatus.INVALID)

        self.consecutive_failures = 0
        if self.calibration is not None:
            accel, gyro = self.calibration.correct(accel, gyro)
        pitch, roll = accel_pitch_roll(*accel)
        if self.calibration is not None:
            pitch, roll = self.calibration.correct_angles(pitch, roll)
        return ImuReading(*accel, *gyro, pitch, roll, timestamp, SensorStatus.OK)

    def _plausible(self, accel, gyro) -> bool:
        values = (*accel, *gyro)
        if not all(math.isfinite(v) for v in values):
            return False
        if any(abs(a) > self._config.max_accel_ms2 for a in accel):
            return False
        if any(abs(g) > self._config.max_gyro_dps for g in gyro):
            return False
        # All-zero acceleration means no gravity was measured: a dead or
        # disconnected sensor, not a real reading.
        return math.hypot(*accel) > 0.1 * GRAVITY_MS2

    def _failed(self, timestamp: float, status: SensorStatus) -> ImuReading:
        self.consecutive_failures += 1
        nan = math.nan
        return ImuReading(nan, nan, nan, nan, nan, nan, nan, nan, timestamp, status)


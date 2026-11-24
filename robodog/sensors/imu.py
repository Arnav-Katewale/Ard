"""IMU interface: raw acceleration/gyro readings plus basic pitch and roll.

Pitch and roll come from a complementary filter that blends two sensors:

- The accelerometer measures gravity, which gives the true tilt with no drift,
  but only while the body is not accelerating. Every footstep jolts it.
- The gyro measures how fast the body is rotating. Adding up (integrating)
  those rates follows fast motion smoothly, but small errors pile up, so on
  its own the angle slowly drifts.

Each reading, with dt the time since the previous reading:

    angle = alpha * (previous angle + gyro rate * dt) + (1 - alpha) * accel angle

With alpha close to 1 the estimate follows the gyro over short times, and the
accelerometer slowly pulls it back to the true tilt. The pull-back time
constant is about dt * alpha / (1 - alpha): 0.98 s for alpha 0.98 at 50 Hz.
Footstep jolts last a few hundredths of a second, so they barely move it.

Simplification: gyro rates about the body x and y axes are used as the roll
and pitch rates. That is accurate for small tilts, which is the normal
walking range; the emergency stop fires long before it matters.

The filter starts from the accelerometer tilt on the first valid reading and
starts again from it after any failed reading, so it never builds on a gap.

The project IMU is an MPU6050 (GY-521); its driver for this interface has not
been written yet. Anything with a `read_raw()` method returning RawImuSample
works as a backend, which is how the simulated IMU plugs in.
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
    pitch_deg: float  # complementary-filter estimate (gyro + accelerometer)
    roll_deg: float   # complementary-filter estimate (gyro + accelerometer)
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
    # Complementary filter weight on the gyro (0 = accelerometer only, 1 = gyro only).
    complementary_alpha: float

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
        check(0.0 <= self.complementary_alpha < 1.0,
              "imu.complementary_alpha must be in [0, 1): 1 would ignore the accelerometer and drift")


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
        self._estimate: tuple[float, float, float] | None = None  # (pitch, roll, timestamp)

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
        pitch, roll = self._fuse(pitch, roll, gyro, timestamp)
        return ImuReading(*accel, *gyro, pitch, roll, timestamp, SensorStatus.OK)

    def _fuse(self, accel_pitch, accel_roll, gyro, timestamp) -> tuple[float, float]:
        """Complementary filter step (see module docstring)."""
        if self._estimate is None or timestamp <= self._estimate[2]:
            pitch, roll = accel_pitch, accel_roll  # (re)start from the accelerometer
        else:
            prev_pitch, prev_roll, prev_t = self._estimate
            dt = timestamp - prev_t
            a = self._config.complementary_alpha
            # Roll is rotation about body +x (gyro x), pitch about body +y (gyro y).
            pitch = a * (prev_pitch + gyro[1] * dt) + (1.0 - a) * accel_pitch
            roll = a * (prev_roll + gyro[0] * dt) + (1.0 - a) * accel_roll
        self._estimate = (pitch, roll, timestamp)
        return pitch, roll

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
        self._estimate = None  # never integrate across a gap
        nan = math.nan
        return ImuReading(nan, nan, nan, nan, nan, nan, nan, nan, timestamp, status)


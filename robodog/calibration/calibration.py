"""Stationary IMU calibration.

Procedure: place the robot on a level surface, keep it still, collect a few
seconds of readings, then estimate:

- gyro bias          mean gyro reading (a still robot should read 0 deg/s)
- orientation offset pitch/roll of the mean gravity vector (a level robot should read 0)
- accel offset       error in measured gravity magnitude, applied along z
- noise              standard deviation of the stationary readings

Limitation: from a single level pose an accelerometer bias in x/y looks
exactly like a slightly tilted IMU mounting. This routine treats it as
orientation offset. Separating the two needs a multi-pose (e.g. six-face)
accelerometer calibration, which is planned for the sensor calibration milestone.
"""

from __future__ import annotations

import json
import math
import statistics
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from robodog.config import check
from robodog.sensors.imu import GRAVITY_MS2, Imu, ImuReading, accel_pitch_roll

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class ImuCalibration:
    gyro_bias: dict[str, float]
    accel_offset: dict[str, float]
    orientation_offset: dict[str, float]
    noise: dict[str, float]
    sample_count: int
    source: str  # "measured" (real robot) or "simulated" (software validation)
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat(timespec="seconds"))

    @classmethod
    def identity(cls) -> "ImuCalibration":
        zero = {"x": 0.0, "y": 0.0, "z": 0.0}
        return cls(dict(zero), dict(zero), {"pitch": 0.0, "roll": 0.0},
                   {"gyro_std": 0.0, "accel_std": 0.0}, 0, "identity")

    def correct(self, accel: Vec3, gyro: Vec3) -> tuple[Vec3, Vec3]:
        a = tuple(v - self.accel_offset[k] for v, k in zip(accel, "xyz"))
        g = tuple(v - self.gyro_bias[k] for v, k in zip(gyro, "xyz"))
        return a, g  # type: ignore[return-value]

    def correct_angles(self, pitch_deg: float, roll_deg: float) -> tuple[float, float]:
        return pitch_deg - self.orientation_offset["pitch"], roll_deg - self.orientation_offset["roll"]

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "ImuCalibration":
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))


@dataclass(frozen=True)
class CalibrationThresholds:
    min_samples: int
    max_gyro_mean_dps: float
    max_level_error_deg: float
    max_gyro_std_dps: float
    max_accel_std_ms2: float

    def __post_init__(self):
        check(self.min_samples > 1, "calibration.min_samples must be > 1")


def collect_stationary_readings(
    imu: Imu, count: int, interval_s: float, sleep: Callable[[float], None]
) -> list[ImuReading]:
    """Read `count` samples; invalid readings are skipped, not averaged in."""
    readings = []
    for _ in range(count):
        r = imu.read()
        if r.valid:
            readings.append(r)
        sleep(interval_s)
    return readings


def _axis_values(readings: list[ImuReading], prefix: str) -> list[list[float]]:
    return [[getattr(r, f"{prefix}_{k}") for r in readings] for k in "xyz"]


def estimate_imu_calibration(readings: list[ImuReading], source: str) -> ImuCalibration:
    """Estimate calibration from stationary, uncalibrated, body-frame readings."""
    check(len(readings) >= 2, "need at least 2 valid readings to calibrate")
    gyro_axes = _axis_values(readings, "gyro")
    accel_axes = _axis_values(readings, "accel")

    gyro_mean = [statistics.fmean(v) for v in gyro_axes]
    accel_mean = [statistics.fmean(v) for v in accel_axes]
    gravity_error = math.hypot(*accel_mean) - GRAVITY_MS2
    # Offset angles are measured on the z-corrected vector, exactly as Imu.read()
    # will compute them, so the mean stationary pose reads as level. Subtracting
    # angles is a small-angle approximation, fine for mounting errors of a few degrees.
    corrected = (accel_mean[0], accel_mean[1], accel_mean[2] - gravity_error)
    pitch_offset, roll_offset = accel_pitch_roll(*corrected)

    return ImuCalibration(
        gyro_bias=dict(zip("xyz", gyro_mean)),
        accel_offset={"x": 0.0, "y": 0.0, "z": gravity_error},
        orientation_offset={"pitch": pitch_offset, "roll": roll_offset},
        noise={
            "gyro_std": statistics.fmean(statistics.stdev(v) for v in gyro_axes),
            "accel_std": statistics.fmean(statistics.stdev(v) for v in accel_axes),
        },
        sample_count=len(readings),
        source=source,
    )


@dataclass(frozen=True)
class CalibrationReport:
    checks: dict[str, tuple[bool, str]]

    @property
    def passed(self) -> bool:
        return all(ok for ok, _ in self.checks.values())

    def lines(self) -> list[str]:
        return [f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}" for name, (ok, detail) in self.checks.items()]


def validate_calibration(readings: list[ImuReading], t: CalibrationThresholds) -> CalibrationReport:
    """Check *calibrated* stationary readings: gyro ~0, level, low noise."""
    checks: dict[str, tuple[bool, str]] = {}
    n = len(readings)
    checks["sample count"] = (n >= t.min_samples, f"{n} valid samples (need {t.min_samples})")
    if n < 2:
        return CalibrationReport(checks)

    gyro_axes = _axis_values(readings, "gyro")
    accel_axes = _axis_values(readings, "accel")
    worst_gyro_mean = max(abs(statistics.fmean(v)) for v in gyro_axes)
    pitch = statistics.fmean(r.pitch_deg for r in readings)
    roll = statistics.fmean(r.roll_deg for r in readings)
    gyro_std = max(statistics.stdev(v) for v in gyro_axes)
    accel_std = max(statistics.stdev(v) for v in accel_axes)

    checks["gyro near zero"] = (worst_gyro_mean <= t.max_gyro_mean_dps,
                                f"max |mean| {worst_gyro_mean:.3f} deg/s (limit {t.max_gyro_mean_dps})")
    checks["pitch level"] = (abs(pitch) <= t.max_level_error_deg, f"{pitch:+.2f} deg (limit {t.max_level_error_deg})")
    checks["roll level"] = (abs(roll) <= t.max_level_error_deg, f"{roll:+.2f} deg (limit {t.max_level_error_deg})")
    checks["gyro noise"] = (gyro_std <= t.max_gyro_std_dps, f"max std {gyro_std:.3f} deg/s (limit {t.max_gyro_std_dps})")
    checks["accel noise"] = (accel_std <= t.max_accel_std_ms2,
                             f"max std {accel_std:.3f} m/s^2 (limit {t.max_accel_std_ms2})")
    return CalibrationReport(checks)

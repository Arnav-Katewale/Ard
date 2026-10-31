"""Quantitative stability metrics over a sliding time window.

Metrics (over the last `window_s` seconds of valid IMU readings):
    rms_pitch, rms_roll   sqrt(mean(angle^2))        how far the body tilts on average
    peak_tilt             max(|pitch|, |roll|)       worst single tilt
    gyro_variance         var(gyro_x) + var(gyro_y)  how much the body rocks
    accel_variance        var(|accel|)               vibration / impacts

Composite score: each metric is divided by a reference value, which is the
level considered "just acceptable", so every term is dimensionless and
1.0 means "at the acceptable limit". The weights sum to 1, so:

    score = sum(weight_i * metric_i / reference_i)
    score < marginal_score            STABLE
    marginal_score <= score < unstable_score  MARGINAL
    score >= unstable_score           UNSTABLE

The reference values are placeholders until measured walking data exists;
the structure lets them be tuned from data instead of guessed in code.
"""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass
from enum import Enum

from robodog.config import check
from robodog.sensors.imu import ImuReading

METRICS = ("rms_pitch", "rms_roll", "peak_tilt", "gyro_variance", "accel_variance")


class StabilityLevel(Enum):
    STABLE = "STABLE"
    MARGINAL = "MARGINAL"
    UNSTABLE = "UNSTABLE"
    UNKNOWN = "UNKNOWN"  # not enough valid data


@dataclass(frozen=True)
class StabilityConfig:
    window_s: float
    min_samples: int
    weights: dict[str, float]
    references: dict[str, float]
    marginal_score: float
    unstable_score: float
    emergency_tilt_deg: float  # beyond this the robot must stop

    def __post_init__(self):
        check(self.window_s > 0 and self.min_samples >= 2, "stability: window_s > 0 and min_samples >= 2")
        check(set(self.weights) == set(METRICS), f"stability.weights must have exactly {METRICS}")
        check(set(self.references) == set(METRICS), f"stability.references must have exactly {METRICS}")
        check(all(w >= 0 for w in self.weights.values()), "stability weights must be >= 0")
        check(abs(sum(self.weights.values()) - 1.0) < 1e-6, "stability weights must sum to 1")
        check(all(r > 0 for r in self.references.values()), "stability references must be positive")
        check(0 < self.marginal_score < self.unstable_score, "stability: need 0 < marginal < unstable")
        check(self.emergency_tilt_deg > 0, "stability.emergency_tilt_deg must be positive")


@dataclass(frozen=True)
class StabilityMetrics:
    rms_pitch: float
    rms_roll: float
    peak_tilt: float
    gyro_variance: float
    accel_variance: float
    score: float
    level: StabilityLevel
    sample_count: int


def _rms(values) -> float:
    return math.sqrt(statistics.fmean(v * v for v in values))


class StabilityMonitor:
    def __init__(self, config: StabilityConfig):
        self.config = config
        self._window: deque[ImuReading] = deque()

    def update(self, reading: ImuReading) -> None:
        if not reading.valid:
            return  # invalid readings must not count as "stable" data
        self._window.append(reading)
        while self._window and reading.timestamp_s - self._window[0].timestamp_s > self.config.window_s:
            self._window.popleft()

    def clear(self) -> None:
        self._window.clear()

    def is_tilt_unsafe(self, reading: ImuReading) -> bool:
        limit = self.config.emergency_tilt_deg
        return reading.valid and (abs(reading.pitch_deg) > limit or abs(reading.roll_deg) > limit)

    def metrics(self) -> StabilityMetrics:
        w = list(self._window)
        n = len(w)
        if n < self.config.min_samples:
            nan = math.nan
            return StabilityMetrics(nan, nan, nan, nan, nan, nan, StabilityLevel.UNKNOWN, n)

        values = {
            "rms_pitch": _rms(r.pitch_deg for r in w),
            "rms_roll": _rms(r.roll_deg for r in w),
            "peak_tilt": max(max(abs(r.pitch_deg), abs(r.roll_deg)) for r in w),
            "gyro_variance": statistics.pvariance([r.gyro_x for r in w])
            + statistics.pvariance([r.gyro_y for r in w]),
            "accel_variance": statistics.pvariance([math.hypot(r.accel_x, r.accel_y, r.accel_z) for r in w]),
        }
        score = sum(self.config.weights[k] * values[k] / self.config.references[k] for k in METRICS)
        if score >= self.config.unstable_score:
            level = StabilityLevel.UNSTABLE
        elif score >= self.config.marginal_score:
            level = StabilityLevel.MARGINAL
        else:
            level = StabilityLevel.STABLE
        return StabilityMetrics(**values, score=score, level=level, sample_count=n)

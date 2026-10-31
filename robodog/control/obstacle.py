"""Obstacle state from ultrasonic distance, with hysteresis.

    CLEAR    distance >= caution_cm
    CAUTION  near_cm <= distance < caution_cm
    NEAR     stop_cm <= distance < near_cm
    STOP     distance < stop_cm

Hysteresis: moving to a MORE severe state happens immediately, but relaxing
to a less severe state requires the distance to exceed the threshold by
hysteresis_cm. This stops the state flickering when the distance hovers
near a threshold.

The ultrasonic sensor only gives one distance along its beam. It says
nothing about an obstacle's shape, width or height, and this code does not
pretend otherwise.

Invalid readings: a single missed echo keeps the current state. After
max_invalid_readings in a row the distance is unknown, so the state is
raised to at least CAUTION rather than assumed CLEAR.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from robodog.config import check
from robodog.sensors.ultrasonic import UltrasonicReading


class ObstacleState(IntEnum):  # ordered by severity
    CLEAR = 0
    CAUTION = 1
    NEAR = 2
    STOP = 3


@dataclass(frozen=True)
class ObstacleConfig:
    caution_cm: float
    near_cm: float
    stop_cm: float
    hysteresis_cm: float
    max_invalid_readings: int

    def __post_init__(self):
        check(0 < self.stop_cm < self.near_cm < self.caution_cm, "obstacle: need 0 < stop < near < caution")
        check(self.hysteresis_cm >= 0, "obstacle.hysteresis_cm must be >= 0")
        check(self.max_invalid_readings >= 1, "obstacle.max_invalid_readings must be >= 1")


class ObstacleMonitor:
    def __init__(self, config: ObstacleConfig):
        self.config = config
        self.state = ObstacleState.CLEAR
        self.invalid_count = 0
        self.last_distance_cm: float | None = None

    def classify(self, distance_cm: float, margin_cm: float = 0.0) -> ObstacleState:
        c = self.config
        if distance_cm < c.stop_cm + margin_cm:
            return ObstacleState.STOP
        if distance_cm < c.near_cm + margin_cm:
            return ObstacleState.NEAR
        if distance_cm < c.caution_cm + margin_cm:
            return ObstacleState.CAUTION
        return ObstacleState.CLEAR

    def update(self, reading: UltrasonicReading) -> ObstacleState:
        if not reading.valid:
            self.invalid_count += 1
            if self.invalid_count >= self.config.max_invalid_readings:
                self.state = max(self.state, ObstacleState.CAUTION)
                self.last_distance_cm = None
            return self.state

        self.invalid_count = 0
        d = reading.distance_cm
        self.last_distance_cm = d
        escalated = self.classify(d)
        if escalated > self.state:
            self.state = escalated
        else:
            relaxed = self.classify(d, self.config.hysteresis_cm)
            if relaxed < self.state:
                self.state = relaxed
        return self.state

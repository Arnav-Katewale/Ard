"""PID controller with output limits and integral anti-windup.

    u = Kp * e + Ki * integral(e) + Kd * de/dt

Design choices:
- The derivative term takes the error rate as an input instead of
  differentiating the error numerically. For body attitude the rate comes
  straight from the gyro, which avoids amplifying accelerometer noise.
- The integral is clamped to +/- integral_limit, and it stops growing while
  the output is saturated in the same direction (anti-windup). Without this
  the integral keeps growing during a long disturbance and causes overshoot.
- The output is clamped to +/- output_limit, so a controller can never ask
  for more correction than the configured maximum.
"""

from __future__ import annotations

from dataclasses import dataclass

from robodog.config import check


@dataclass(frozen=True)
class PidConfig:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float

    def __post_init__(self):
        check(min(self.kp, self.ki, self.kd) >= 0, "pid gains must be >= 0")
        check(self.output_limit > 0 and self.integral_limit >= 0, "pid limits must be positive")


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class Pid:
    def __init__(self, config: PidConfig):
        self.config = config
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def update(self, error: float, error_rate: float, dt_s: float) -> float:
        c = self.config
        previous_integral = self.integral
        self.integral = _clamp(self.integral + error * dt_s, c.integral_limit)

        unclamped = c.kp * error + c.ki * self.integral + c.kd * error_rate
        output = _clamp(unclamped, c.output_limit)

        # Anti-windup: if saturated and the error would push further into
        # saturation, undo this step's integral growth.
        if output != unclamped and (error > 0) == (unclamped > 0):
            self.integral = previous_integral
        return output

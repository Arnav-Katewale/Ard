"""Simulated sensor backends for software validation without hardware.

These produce deterministic (seeded) sensor input from a body attitude that the
caller sets. They are NOT measurements of the physical robot.
"""

from __future__ import annotations

import math
import random

from robodog.sensors.imu import GRAVITY_MS2, RawImuSample


class SimulatedImuBackend:
    """Generates accelerometer/gyro samples for a given body attitude.

    For roll phi and pitch theta, a stationary accelerometer reads
        (-g sin(theta), g sin(phi) cos(theta), g cos(phi) cos(theta)).
    Gyro x/y are approximated as the roll/pitch rates (valid for small tilts).
    Optional constant bias and Gaussian noise imitate real sensor error.
    """

    def __init__(
        self,
        accel_noise_std: float = 0.0,
        gyro_noise_std: float = 0.0,
        accel_bias: tuple[float, float, float] = (0.0, 0.0, 0.0),
        gyro_bias: tuple[float, float, float] = (0.0, 0.0, 0.0),
        seed: int = 0,
    ):
        self.accel_noise_std = accel_noise_std
        self.gyro_noise_std = gyro_noise_std
        self.accel_bias = accel_bias
        self.gyro_bias = gyro_bias
        self._rng = random.Random(seed)
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.pitch_rate_dps = 0.0
        self.roll_rate_dps = 0.0
        self.extra_accel = (0.0, 0.0, 0.0)  # e.g. vibration from footfalls
        self.fail_next = False

    def set_attitude(self, pitch_deg, roll_deg, pitch_rate_dps=0.0, roll_rate_dps=0.0):
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.pitch_rate_dps = pitch_rate_dps
        self.roll_rate_dps = roll_rate_dps

    def read_raw(self) -> RawImuSample:
        if self.fail_next:
            self.fail_next = False
            raise OSError("simulated I2C failure")
        th = math.radians(self.pitch_deg)
        ph = math.radians(self.roll_deg)
        g = GRAVITY_MS2
        ideal_accel = (
            -g * math.sin(th),
            g * math.sin(ph) * math.cos(th),
            g * math.cos(ph) * math.cos(th),
        )
        ideal_gyro = (self.roll_rate_dps, self.pitch_rate_dps, 0.0)
        accel = tuple(
            a + e + b + self._rng.gauss(0.0, self.accel_noise_std)
            for a, e, b in zip(ideal_accel, self.extra_accel, self.accel_bias)
        )
        gyro = tuple(
            w + b + self._rng.gauss(0.0, self.gyro_noise_std)
            for w, b in zip(ideal_gyro, self.gyro_bias)
        )
        return RawImuSample(accel=accel, gyro=gyro)

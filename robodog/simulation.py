"""Software validation of the control loop with simulated sensor input.

IMPORTANT: this is NOT a physics simulation of the robot. It is a small,
deliberately simple model whose only purpose is to exercise the real
control code (sensor interfaces, calibration, stability metrics, obstacle
logic, adaptive gait, kinematics, joint safety, LX-16A protocol) end to end
on a normal computer. Results from it say nothing about how the physical
robot will behave; that requires hardware experiments.

Toy body model, per step:
    achieved_correction += (commanded - achieved) * dt / response_time_s
    body_angle = scripted_disturbance(t) + achieved_correction + gait_sway
where gait_sway is a small oscillation at the gait frequency, scaled by the
current stride. Gyro rates are the change in body angle per step.

The obstacle distance is scripted (it does not depend on walking speed).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from robodog.calibration.calibration import (
    CalibrationReport,
    CalibrationThresholds,
    ImuCalibration,
    collect_stationary_readings,
    estimate_imu_calibration,
    validate_calibration,
)
from robodog.config import build, check, section
from robodog.control.control_loop import ControlLoop, StepResult
from robodog.robot import build_control_loop
from robodog.sensors.simulated_sensors import SimulatedEchoBackend, SimulatedImuBackend
from robodog.servos.simulated_servo import SimulatedLx16aPort


@dataclass(frozen=True)
class SimulationConfig:
    duration_s: float
    seed: int
    accel_noise_std: float
    gyro_noise_std: float
    gyro_bias_dps: list[float]
    mounting_pitch_deg: float
    mounting_roll_deg: float
    ultrasonic_noise_cm: float
    ultrasonic_dropout: float
    response_time_s: float
    gait_sway_deg: float
    roll_disturbance_deg: float
    roll_disturbance_start_s: float
    roll_disturbance_end_s: float
    pitch_disturbance_deg: float
    pitch_disturbance_start_s: float
    pitch_disturbance_end_s: float
    obstacle_start_cm: float
    obstacle_end_cm: float
    obstacle_approach_start_s: float
    obstacle_approach_end_s: float
    calibration_samples: int

    def __post_init__(self):
        check(self.duration_s > 0 and self.response_time_s > 0, "simulation: durations must be positive")
        check(len(self.gyro_bias_dps) == 3, "simulation.gyro_bias_dps needs 3 values")
        check(self.obstacle_approach_start_s < self.obstacle_approach_end_s, "simulation: bad obstacle timing")


def _window(t: float, start: float, end: float, ramp_s: float = 1.0) -> float:
    """0 outside [start, end], ramping smoothly to 1 inside."""
    if t <= start or t >= end:
        return 0.0
    rise = min(1.0, (t - start) / ramp_s)
    fall = min(1.0, (end - t) / ramp_s)
    return min(rise, fall)


@dataclass(frozen=True)
class Scenario:
    config: SimulationConfig

    def disturbance(self, t: float) -> tuple[float, float]:
        c = self.config
        pitch = c.pitch_disturbance_deg * _window(t, c.pitch_disturbance_start_s, c.pitch_disturbance_end_s)
        roll = c.roll_disturbance_deg * _window(t, c.roll_disturbance_start_s, c.roll_disturbance_end_s)
        return pitch, roll

    def obstacle_distance_cm(self, t: float) -> float:
        c = self.config
        s = (t - c.obstacle_approach_start_s) / (c.obstacle_approach_end_s - c.obstacle_approach_start_s)
        s = max(0.0, min(1.0, s))
        return c.obstacle_start_cm + s * (c.obstacle_end_cm - c.obstacle_start_cm)


@dataclass
class SimulationRecord:
    t: float
    true_pitch_deg: float
    true_roll_deg: float
    obstacle_true_cm: float
    result: StepResult


class SimulatedClock:
    """Simulated time: advances only when told to, so runs are repeatable."""

    def __init__(self):
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, s: float) -> None:
        self.now += s


@dataclass
class SimulatedRobot:
    loop: ControlLoop
    clock: SimulatedClock
    imu_backend: SimulatedImuBackend
    echo_backend: SimulatedEchoBackend
    servo_port: SimulatedLx16aPort
    scenario: Scenario
    config: SimulationConfig
    records: list[SimulationRecord] = field(default_factory=list)
    _achieved_pitch: float = 0.0
    _achieved_roll: float = 0.0
    _prev_pitch: float = 0.0
    _prev_roll: float = 0.0

    def calibrate(self, thresholds: CalibrationThresholds) -> tuple[ImuCalibration, CalibrationReport]:
        """Stationary calibration on simulated data, then validation with it applied."""
        imu = self.loop.imu
        imu.calibration = None
        self.imu_backend.set_attitude(self.config.mounting_pitch_deg, self.config.mounting_roll_deg)
        n = self.config.calibration_samples
        readings = collect_stationary_readings(imu, n, 0.0, lambda s: None)
        calibration = estimate_imu_calibration(readings, source="simulated")
        imu.calibration = calibration
        report = validate_calibration(collect_stationary_readings(imu, n, 0.0, lambda s: None), thresholds)
        return calibration, report

    def step(self) -> SimulationRecord:
        c, dt = self.config, self.loop.dt_s
        self.clock.now += dt
        t = self.clock.now

        adaptive = self.loop.adaptive
        alpha = min(1.0, dt / c.response_time_s)
        self._achieved_pitch += (adaptive.correction.pitch_deg - self._achieved_pitch) * alpha
        self._achieved_roll += (adaptive.correction.roll_deg - self._achieved_roll) * alpha

        stride_ratio = adaptive.params.stride_length_mm / max(adaptive.base.stride_length_mm, 1e-9)
        sway = c.gait_sway_deg * stride_ratio * math.sin(2 * math.pi * self.loop.gait.phase)
        dist_pitch, dist_roll = self.scenario.disturbance(t)
        true_pitch = dist_pitch + self._achieved_pitch + 0.5 * sway
        true_roll = dist_roll + self._achieved_roll + sway

        # The IMU sits on the body with a small mounting error, which calibration removes.
        self.imu_backend.set_attitude(
            true_pitch + c.mounting_pitch_deg,
            true_roll + c.mounting_roll_deg,
            pitch_rate_dps=(true_pitch - self._prev_pitch) / dt,
            roll_rate_dps=(true_roll - self._prev_roll) / dt,
        )
        self._prev_pitch, self._prev_roll = true_pitch, true_roll
        distance = self.scenario.obstacle_distance_cm(t)
        self.echo_backend.distance_cm = distance

        record = SimulationRecord(t, true_pitch, true_roll, distance, self.loop.step())
        self.records.append(record)
        return record


def build_simulated_robot(cfg: dict[str, Any]) -> SimulatedRobot:
    sim = build(SimulationConfig, section(cfg, "simulation"), "simulation")
    imu_backend = SimulatedImuBackend(
        accel_noise_std=sim.accel_noise_std,
        gyro_noise_std=sim.gyro_noise_std,
        gyro_bias=tuple(sim.gyro_bias_dps),
        seed=sim.seed,
    )
    echo_backend = SimulatedEchoBackend(
        cfg["ultrasonic"]["speed_of_sound_ms"],
        noise_std_cm=sim.ultrasonic_noise_cm,
        dropout_probability=sim.ultrasonic_dropout,
        seed=sim.seed + 1,
    )
    port = SimulatedLx16aPort.with_servos([j["servo_id"] for j in cfg["joints"]])
    clock = SimulatedClock()
    loop = build_control_loop(cfg, imu_backend, echo_backend, port, clock=clock, sleep=clock.sleep)
    return SimulatedRobot(loop, clock, imu_backend, echo_backend, port, Scenario(sim), sim)

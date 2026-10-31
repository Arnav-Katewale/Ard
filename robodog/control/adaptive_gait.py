"""Adaptive gait prototype: adjusts gait parameters from sensor feedback.

Inputs each control step:
    IMU reading       -> body pitch/roll corrections (two PID controllers)
    stability level   -> shorter strides when the body is rocking
    obstacle state    -> shorter/slower strides, higher steps, or standing still

Outputs: smoothed GaitParams and a BodyCorrection for the gait generator.

Attitude correction (target attitude is level, so error = 0 - measured):
    roll_correction  = PID(error = -roll,  error_rate = -gyro_x)
    pitch_correction = PID(error = -pitch, error_rate = -gyro_y)
A positive measured roll (left side up) gives a negative roll correction,
which shortens the left legs and lengthens the right legs (see
gait_controller), bringing the body back toward level.

Obstacle response (scale/offset tables come from config):
    CLEAR / CAUTION / NEAR  scale stride, add step height, scale cycle time
    STOP                    stand still: stride and step height go to 0

Safety and smoothness:
- PID outputs and gait parameters saturate at configured maxima. These are
  controller outputs, so saturating them is intended; joint commands are a
  different case and are still rejected (not clamped) by SafeJointGroup.
- Every parameter changes at a limited rate, so there are no abrupt jumps.
  Stride may shrink faster than it grows: slowing down is the safe direction.
- If the IMU reading is invalid, the controller does not keep correcting
  blindly: PIDs are reset, corrections ramp back to zero and stride is
  reduced as for an UNKNOWN stability level.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from robodog.config import check
from robodog.control.gait_controller import BodyCorrection, GaitParams
from robodog.control.obstacle import ObstacleState
from robodog.control.pid import Pid, PidConfig
from robodog.control.stability import StabilityLevel
from robodog.sensors.imu import ImuReading

MOVING_STATES = ("CLEAR", "CAUTION", "NEAR")
STABILITY_LEVELS = tuple(level.name for level in StabilityLevel)


@dataclass(frozen=True)
class AdaptiveConfig:
    roll_pid: PidConfig
    pitch_pid: PidConfig
    max_correction_rate_dps: float
    stride_rate_up_mm_s: float
    stride_rate_down_mm_s: float
    step_height_rate_mm_s: float
    cycle_time_rate_s_per_s: float
    max_stride_mm: float
    max_step_height_mm: float
    min_cycle_time_s: float
    max_cycle_time_s: float
    obstacle_stride_scale: dict[str, float]
    obstacle_step_height_add_mm: dict[str, float]
    obstacle_cycle_time_scale: dict[str, float]
    stability_stride_scale: dict[str, float]

    @classmethod
    def from_dict(cls, data: dict) -> "AdaptiveConfig":
        data = dict(data)
        data["roll_pid"] = PidConfig(**data["roll_pid"])
        data["pitch_pid"] = PidConfig(**data["pitch_pid"])
        return cls(**data)

    def __post_init__(self):
        rates = (self.max_correction_rate_dps, self.stride_rate_up_mm_s, self.stride_rate_down_mm_s,
                 self.step_height_rate_mm_s, self.cycle_time_rate_s_per_s)
        check(all(r > 0 for r in rates), "adaptive: all rate limits must be positive")
        check(self.max_stride_mm > 0 and self.max_step_height_mm > 0, "adaptive: maxima must be positive")
        check(0 < self.min_cycle_time_s < self.max_cycle_time_s, "adaptive: need 0 < min < max cycle time")
        for name in ("obstacle_stride_scale", "obstacle_step_height_add_mm", "obstacle_cycle_time_scale"):
            check(set(getattr(self, name)) == set(MOVING_STATES), f"adaptive.{name} must define {MOVING_STATES}")
        check(set(self.stability_stride_scale) == set(STABILITY_LEVELS),
              f"adaptive.stability_stride_scale must define {STABILITY_LEVELS}")
        check(all(0 <= s <= 1 for s in self.obstacle_stride_scale.values()), "stride scales must be in [0, 1]")
        check(all(0 <= s <= 1 for s in self.stability_stride_scale.values()), "stride scales must be in [0, 1]")


@dataclass(frozen=True)
class AdaptiveOutput:
    params: GaitParams
    correction: BodyCorrection
    obstacle: ObstacleState
    stability: StabilityLevel
    imu_ok: bool
    notes: list[str] = field(default_factory=list)

    @property
    def standing(self) -> bool:
        return self.params.stride_length_mm == 0 and self.params.step_height_mm == 0


def _approach(current: float, target: float, max_step: float) -> float:
    """Move current toward target by at most max_step."""
    return current + max(-max_step, min(max_step, target - current))


def _bound(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class AdaptiveGaitController:
    def __init__(self, config: AdaptiveConfig, base: GaitParams):
        self.config = config
        self.base = base
        self.params = base
        self.correction = BodyCorrection()
        self._roll_pid = Pid(config.roll_pid)
        self._pitch_pid = Pid(config.pitch_pid)

    def target_params(self, obstacle: ObstacleState, stability: StabilityLevel) -> GaitParams:
        c, b = self.config, self.base
        if obstacle is ObstacleState.STOP:
            stride, step, cycle = 0.0, 0.0, b.cycle_time_s
        else:
            stride = b.stride_length_mm * c.obstacle_stride_scale[obstacle.name] * c.stability_stride_scale[stability.name]
            step = b.step_height_mm + c.obstacle_step_height_add_mm[obstacle.name]
            cycle = b.cycle_time_s * c.obstacle_cycle_time_scale[obstacle.name]
        return GaitParams(
            stride_length_mm=_bound(stride, 0.0, c.max_stride_mm),
            step_height_mm=_bound(step, 0.0, c.max_step_height_mm),
            cycle_time_s=_bound(cycle, c.min_cycle_time_s, c.max_cycle_time_s),
            body_height_mm=b.body_height_mm,
            duty_factor=b.duty_factor,
        )

    def update(
        self, imu: ImuReading, stability: StabilityLevel, obstacle: ObstacleState, dt_s: float
    ) -> AdaptiveOutput:
        c = self.config
        notes = []
        imu_ok = imu.valid and math.isfinite(imu.pitch_deg) and math.isfinite(imu.roll_deg)

        if imu_ok:
            roll_target = self._roll_pid.update(-imu.roll_deg, -imu.gyro_x, dt_s)
            pitch_target = self._pitch_pid.update(-imu.pitch_deg, -imu.gyro_y, dt_s)
        else:
            self._roll_pid.reset()
            self._pitch_pid.reset()
            roll_target = pitch_target = 0.0
            stability = StabilityLevel.UNKNOWN
            notes.append("IMU invalid: attitude correction disabled, stride reduced")

        max_angle_step = c.max_correction_rate_dps * dt_s
        self.correction = BodyCorrection(
            pitch_deg=_approach(self.correction.pitch_deg, pitch_target, max_angle_step),
            roll_deg=_approach(self.correction.roll_deg, roll_target, max_angle_step),
        )

        target = self.target_params(obstacle, stability)
        stride_rate = c.stride_rate_up_mm_s if target.stride_length_mm > self.params.stride_length_mm \
            else c.stride_rate_down_mm_s
        self.params = GaitParams(
            stride_length_mm=_approach(self.params.stride_length_mm, target.stride_length_mm, stride_rate * dt_s),
            step_height_mm=_approach(self.params.step_height_mm, target.step_height_mm,
                                     c.step_height_rate_mm_s * dt_s),
            cycle_time_s=_approach(self.params.cycle_time_s, target.cycle_time_s, c.cycle_time_rate_s_per_s * dt_s),
            body_height_mm=target.body_height_mm,
            duty_factor=target.duty_factor,
        )

        if obstacle is ObstacleState.STOP:
            notes.append("obstacle STOP: coming to a stand")
        if stability is StabilityLevel.UNSTABLE:
            notes.append("UNSTABLE: stride reduced")
        return AdaptiveOutput(self.params, self.correction, obstacle, stability, imu_ok, notes)

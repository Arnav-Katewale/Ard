"""One step of the closed control loop.

    IMU ----------> stability metrics --\
                                         >--> adaptive gait --> gait generator
    ultrasonic ---> obstacle state -----/      (params,          (foot targets)
                                                corrections)          |
                                                                      v
    servos <------ joint safety layer <------- inverse kinematics <---/

Safety checks, in order, every step:
1. Emergency stop already latched -> do nothing.
2. Tilt beyond emergency_tilt_deg -> emergency stop.
3. IMU failed max_imu_failures times in a row -> emergency stop
   (never keep walking on stale attitude data).
4. If IK or joint limits reject the new targets, nothing is sent and the
   previous targets stay in place. Too many rejections in a row -> emergency stop.

The loop is single-threaded and runs one step per call. Timing (rate,
jitter, missed deadlines) on the Raspberry Pi has not been measured yet, so
no real-time guarantees are claimed.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from robodog.config import check
from robodog.control.adaptive_gait import AdaptiveGaitController, AdaptiveOutput
from robodog.control.gait_controller import GaitGenerator
from robodog.control.obstacle import ObstacleMonitor
from robodog.control.stability import StabilityMetrics, StabilityMonitor
from robodog.kinematics import LegGeometry, UnreachableError, joint_targets
from robodog.sensors.imu import Imu, ImuReading
from robodog.sensors.ultrasonic import UltrasonicReading, UltrasonicSensor
from robodog.servos.joint import JointLimitError, SafeJointGroup


@dataclass(frozen=True)
class ControlLoopConfig:
    rate_hz: float
    ultrasonic_every_n_steps: int
    max_imu_failures: int
    max_consecutive_rejections: int

    def __post_init__(self):
        check(self.rate_hz > 0, "control_loop.rate_hz must be positive")
        check(self.ultrasonic_every_n_steps >= 1, "control_loop.ultrasonic_every_n_steps must be >= 1")
        check(self.max_imu_failures >= 1 and self.max_consecutive_rejections >= 1,
              "control_loop failure limits must be >= 1")


@dataclass
class StepResult:
    step: int
    imu: ImuReading
    ultrasonic: UltrasonicReading | None
    metrics: StabilityMetrics | None = None
    adaptive: AdaptiveOutput | None = None
    phase: float | None = None
    joint_targets: dict[str, float] | None = None
    rejected: str | None = None
    estop_reason: str | None = None
    notes: list[str] = field(default_factory=list)


class ControlLoop:
    def __init__(
        self,
        config: ControlLoopConfig,
        imu: Imu,
        ultrasonic: UltrasonicSensor,
        stability: StabilityMonitor,
        obstacle: ObstacleMonitor,
        adaptive: AdaptiveGaitController,
        gait: GaitGenerator,
        geometry: LegGeometry,
        joints: SafeJointGroup,
    ):
        self.config = config
        self.imu = imu
        self.ultrasonic = ultrasonic
        self.stability = stability
        self.obstacle = obstacle
        self.adaptive = adaptive
        self.gait = gait
        self.geometry = geometry
        self.joints = joints
        self.step_count = 0
        self.consecutive_rejections = 0

    @property
    def dt_s(self) -> float:
        return 1.0 / self.config.rate_hz

    def step(self) -> StepResult:
        n = self.step_count
        self.step_count += 1

        reading = self.imu.read()
        sonar = self.ultrasonic.read() if n % self.config.ultrasonic_every_n_steps == 0 else None
        result = StepResult(step=n, imu=reading, ultrasonic=sonar)

        if self.joints.estop_active:
            result.estop_reason = self.joints.estop_reason
            return result

        self.stability.update(reading)
        if self.stability.is_tilt_unsafe(reading):
            return self._stop(result, f"tilt pitch={reading.pitch_deg:.1f} roll={reading.roll_deg:.1f} deg "
                                      f"exceeds {self.stability.config.emergency_tilt_deg} deg")
        if self.imu.consecutive_failures >= self.config.max_imu_failures:
            return self._stop(result, f"IMU unavailable for {self.imu.consecutive_failures} readings")

        obstacle_state = self.obstacle.update(sonar) if sonar is not None else self.obstacle.state
        result.metrics = self.stability.metrics()
        result.adaptive = self.adaptive.update(reading, result.metrics.level, obstacle_state, self.dt_s)
        result.notes.extend(result.adaptive.notes)
        result.phase = self.gait.advance(self.dt_s, result.adaptive.params)

        feet = self.gait.feet(result.adaptive.params, result.adaptive.correction)
        try:
            targets = joint_targets(feet, self.geometry)
            self.joints.command(targets)
        except (UnreachableError, JointLimitError) as exc:
            self.consecutive_rejections += 1
            result.rejected = str(exc)
            if self.consecutive_rejections >= self.config.max_consecutive_rejections:
                return self._stop(result, f"{self.consecutive_rejections} consecutive rejected joint commands")
            return result

        self.consecutive_rejections = 0
        result.joint_targets = targets
        return result

    def _stop(self, result: StepResult, reason: str) -> StepResult:
        self.joints.emergency_stop(reason)
        result.estop_reason = reason
        return result

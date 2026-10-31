"""Joint safety layer between control code and the servo bus.

Control code works in *joint angles* (0 deg = the joint's reference pose).
This layer converts them to *servo angles*, and enforces the safety rules:

1. Every target must be inside the joint's configured [min_deg, max_deg].
   Unsafe targets are REJECTED, never silently clamped, because clamping
   would hide bugs in the code that produced the target.
2. A multi-joint command is all-or-nothing: if any target is unsafe,
   nothing is sent.
3. Move time is stretched so no joint exceeds the configured max speed.
4. After emergency_stop() every command is refused until an explicit reset.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from robodog.config import ConfigError, check
from robodog.servos import lx16a


class JointLimitError(ValueError):
    """A commanded joint target is unsafe (outside limits, unknown joint, NaN)."""


class EmergencyStopActive(RuntimeError):
    """Commands are refused while the emergency stop is latched."""


@dataclass(frozen=True)
class JointConfig:
    name: str
    servo_id: int
    min_deg: float               # joint-angle limits
    max_deg: float
    servo_neutral_deg: float     # servo angle when the joint angle is 0
    direction: int               # +1 or -1: how joint angle maps onto servo angle

    def __post_init__(self):
        check(0 <= self.servo_id <= 253, f"{self.name}: servo_id must be 0..253")
        check(self.direction in (1, -1), f"{self.name}: direction must be +1 or -1")
        check(self.min_deg < self.max_deg, f"{self.name}: min_deg must be less than max_deg")
        for limit in (self.min_deg, self.max_deg):
            servo = self.joint_to_servo_deg(limit)
            check(
                0.0 <= servo <= lx16a.ANGLE_RANGE_DEG,
                f"{self.name}: joint limit {limit} deg maps to servo angle {servo} deg, outside 0..240",
            )

    def joint_to_servo_deg(self, joint_deg: float) -> float:
        return self.servo_neutral_deg + self.direction * joint_deg

    def servo_to_joint_deg(self, servo_deg: float) -> float:
        return (servo_deg - self.servo_neutral_deg) * self.direction

    def within_limits(self, joint_deg: float) -> bool:
        return math.isfinite(joint_deg) and self.min_deg <= joint_deg <= self.max_deg


@dataclass(frozen=True)
class ServoBusConfig:
    device: str
    echo: bool
    max_joint_speed_dps: float
    min_move_time_ms: int

    def __post_init__(self):
        check(self.max_joint_speed_dps > 0, "servo_bus.max_joint_speed_dps must be positive")
        check(0 <= self.min_move_time_ms <= lx16a.MAX_MOVE_TIME_MS, "servo_bus.min_move_time_ms out of range")


class SafeJointGroup:
    def __init__(self, bus: lx16a.Lx16aBus, joints: list[JointConfig], bus_config: ServoBusConfig):
        names = [j.name for j in joints]
        ids = [j.servo_id for j in joints]
        if len(set(names)) != len(names):
            raise ConfigError("duplicate joint names")
        if len(set(ids)) != len(ids):
            raise ConfigError("duplicate servo ids")
        self._bus = bus
        self._config = bus_config
        self.joints = {j.name: j for j in joints}
        self.current_deg: dict[str, float] = {}
        self.estop_reason: str | None = None

    @property
    def estop_active(self) -> bool:
        return self.estop_reason is not None

    def sync_from_servos(self) -> dict[str, float]:
        """Read every servo's actual position. Required before the first move."""
        self.current_deg = {
            name: joint.servo_to_joint_deg(self._bus.read_position_deg(joint.servo_id))
            for name, joint in self.joints.items()
        }
        return dict(self.current_deg)

    def find_problems(self, targets: dict[str, float]) -> list[str]:
        problems = []
        for name, angle in targets.items():
            joint = self.joints.get(name)
            if joint is None:
                problems.append(f"unknown joint {name!r}")
            elif not joint.within_limits(angle):
                problems.append(
                    f"{name} target {angle:.1f} deg outside limits [{joint.min_deg}, {joint.max_deg}]"
                )
        return problems

    def move_time_ms(self, targets: dict[str, float]) -> int:
        """Shortest move time that keeps every joint under the speed limit."""
        largest_step = max(abs(angle - self.current_deg[name]) for name, angle in targets.items())
        ms = math.ceil(largest_step / self._config.max_joint_speed_dps * 1000.0)
        return max(self._config.min_move_time_ms, ms)

    def command(self, targets: dict[str, float]) -> int:
        """Validate all targets, then send them. Returns the move time used (ms)."""
        if self.estop_active:
            raise EmergencyStopActive(f"emergency stop active: {self.estop_reason}")
        problems = self.find_problems(targets)
        if problems:
            raise JointLimitError("; ".join(problems))
        unknown_start = [n for n in targets if n not in self.current_deg]
        if unknown_start:
            raise JointLimitError(f"current position unknown for {unknown_start}; call sync_from_servos() first")

        time_ms = self.move_time_ms(targets)
        if time_ms > lx16a.MAX_MOVE_TIME_MS:
            raise JointLimitError(f"move would take {time_ms} ms, above the servo maximum")
        for name, angle in targets.items():
            joint = self.joints[name]
            self._bus.move(joint.servo_id, joint.joint_to_servo_deg(angle), time_ms)
            self.current_deg[name] = angle
        return time_ms

    def emergency_stop(self, reason: str) -> list[str]:
        """Latch the stop and halt every servo where it is.

        Keeps going even if some servos fail to respond, and returns the names
        of joints that could not be stopped.
        """
        self.estop_reason = reason
        failed = []
        for name, joint in self.joints.items():
            try:
                self._bus.stop(joint.servo_id)
            except Exception:
                failed.append(name)
        return failed

    def reset_emergency_stop(self) -> None:
        self.estop_reason = None

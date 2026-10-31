"""Leg geometry and 3-DOF leg kinematics.

Note: this is the current software model. The physical robot's legs have two
servos each (hip and knee), so this module is to be adapted; step 2 below
(the hip pitch / knee law-of-cosines solution) is the part that carries over.

The software model gives each leg three joints:
    hip   - abduction: rotates the whole leg about the body x axis
    upper - hip pitch: 0 = upper leg pointing straight down, + = swings forward
    knee  - knee bend: 0 = straight leg, + = lower leg folds backward

Leg frame: origin at the hip abduction axis, axes parallel to the body frame
(+x forward, +y left, +z up). The upper leg is offset sideways from the
abduction axis by `hip_offset_mm`, outward (+y on left legs, -y on right legs).

Inverse kinematics, step by step:
1. Abduction. Looking along x, the foot's (y, z) must be reached by the
   sideways offset L1 plus the leg's length r in its own plane:
       r = sqrt(y^2 + z^2 - L1^2)
       hip = atan2(z, y) - atan2(-r, side * L1)
2. In the leg plane the foot is at (x, -r), distance d = sqrt(x^2 + r^2).
   The law of cosines on the upper/lower leg triangle gives
       knee  = 180 deg - acos((L2^2 + L3^2 - d^2) / (2 L2 L3))
       upper = atan2(x, r) + acos((L2^2 + d^2 - L3^2) / (2 L2 d))
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from robodog.config import check

LEGS = ("FL", "FR", "RL", "RR")
LEG_SIDE = {"FL": 1, "FR": -1, "RL": 1, "RR": -1}   # +1 = left
LEG_END = {"FL": 1, "FR": 1, "RL": -1, "RR": -1}    # +1 = front


class UnreachableError(ValueError):
    """The requested foot position cannot be reached by the leg."""


@dataclass(frozen=True)
class LegGeometry:
    body_length_mm: float  # front-to-rear hip spacing
    body_width_mm: float   # left-to-right hip spacing
    hip_offset_mm: float   # L1
    upper_leg_mm: float    # L2
    lower_leg_mm: float    # L3

    def __post_init__(self):
        for name in ("body_length_mm", "body_width_mm", "upper_leg_mm", "lower_leg_mm"):
            check(getattr(self, name) > 0, f"geometry.{name} must be positive")
        check(self.hip_offset_mm >= 0, "geometry.hip_offset_mm must be >= 0")

    def hip_position(self, leg: str) -> tuple[float, float]:
        """(x, y) of a leg's hip in the body frame."""
        return LEG_END[leg] * self.body_length_mm / 2, LEG_SIDE[leg] * self.body_width_mm / 2


def leg_ik(x: float, y: float, z: float, side: int, g: LegGeometry) -> tuple[float, float, float]:
    """Foot position (leg frame, mm) -> (hip, upper, knee) joint angles in degrees."""
    L1, L2, L3 = g.hip_offset_mm, g.upper_leg_mm, g.lower_leg_mm
    r_squared = y * y + z * z - L1 * L1
    if r_squared <= 0:
        raise UnreachableError(f"foot ({x:.1f}, {y:.1f}, {z:.1f}) is inside the hip offset")
    r = math.sqrt(r_squared)
    hip = math.atan2(z, y) - math.atan2(-r, side * L1)
    hip = math.atan2(math.sin(hip), math.cos(hip))  # wrap to (-pi, pi]

    d = math.hypot(x, r)
    if not abs(L2 - L3) < d <= L2 + L3:
        raise UnreachableError(f"foot ({x:.1f}, {y:.1f}, {z:.1f}) is {d:.1f} mm from the hip, "
                               f"reachable range is ({abs(L2 - L3):.1f}, {L2 + L3:.1f}]")
    cos_knee = (L2 * L2 + L3 * L3 - d * d) / (2 * L2 * L3)
    cos_alpha = (L2 * L2 + d * d - L3 * L3) / (2 * L2 * d)
    knee = math.pi - math.acos(max(-1.0, min(1.0, cos_knee)))
    upper = math.atan2(x, r) + math.acos(max(-1.0, min(1.0, cos_alpha)))
    return math.degrees(hip), math.degrees(upper), math.degrees(knee)


def leg_fk(hip_deg: float, upper_deg: float, knee_deg: float, side: int, g: LegGeometry) -> tuple[float, float, float]:
    """Joint angles (degrees) -> foot position in the leg frame (mm)."""
    a, h, k = (math.radians(v) for v in (hip_deg, upper_deg, knee_deg))
    L1, L2, L3 = g.hip_offset_mm, g.upper_leg_mm, g.lower_leg_mm
    # Foot in the leg plane (before abduction)
    x = L2 * math.sin(h) + L3 * math.sin(h - k)
    r = L2 * math.cos(h) + L3 * math.cos(h - k)
    # Rotate the (side * L1, -r) point about the x axis by the abduction angle
    y = side * L1 * math.cos(a) + r * math.sin(a)
    z = side * L1 * math.sin(a) - r * math.cos(a)
    return x, y, z


def joint_targets(feet: dict[str, tuple[float, float, float]], g: LegGeometry) -> dict[str, float]:
    """Foot positions per leg -> joint-angle targets named like 'FL_knee'.

    Raises UnreachableError naming the leg if any foot cannot be reached.
    """
    targets = {}
    for leg, (x, y, z) in feet.items():
        try:
            hip, upper, knee = leg_ik(x, y, z, LEG_SIDE[leg], g)
        except UnreachableError as exc:
            raise UnreachableError(f"{leg}: {exc}") from exc
        targets[f"{leg}_hip"] = hip
        targets[f"{leg}_upper"] = upper
        targets[f"{leg}_knee"] = knee
    return targets

"""Open-loop trot gait: foot positions as a function of gait phase.

Gait phase runs 0 -> 1 once per cycle. Each leg has a phase offset; in a
trot the diagonal pairs (FL+RR, FR+RL) move together, half a cycle apart.

For one leg, with leg_phase = (phase + offset) mod 1:
    stance (leg_phase < duty_factor): foot on the ground, sliding backward
        from +stride/2 to -stride/2 (this pushes the body forward)
    swing  (otherwise): foot lifted along a half-sine arc of height
        step_height, moving forward from -stride/2 back to +stride/2

Body attitude corrections (from the adaptive layer) are applied by changing
each leg's extension. Tilting the body by angle c about x raises a hip at
lateral position y by y*sin(c); pitching by c about y lowers a hip at
forward position x by x*sin(c). So each hip's height above its foot is

    h = body_height + y_hip * sin(roll_correction) - x_hip * sin(pitch_correction)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from robodog.config import check
from robodog.kinematics import LEG_SIDE, LEGS, LegGeometry


@dataclass(frozen=True)
class GaitParams:
    stride_length_mm: float
    step_height_mm: float
    cycle_time_s: float
    body_height_mm: float
    duty_factor: float

    def __post_init__(self):
        check(self.stride_length_mm >= 0 and self.step_height_mm >= 0, "gait: stride/step must be >= 0")
        check(self.cycle_time_s > 0 and self.body_height_mm > 0, "gait: cycle_time_s and body_height_mm must be > 0")
        check(0.5 <= self.duty_factor < 1.0, "gait: duty_factor must be in [0.5, 1)")


@dataclass(frozen=True)
class BodyCorrection:
    pitch_deg: float = 0.0
    roll_deg: float = 0.0


@dataclass(frozen=True)
class GaitConfig:
    base: GaitParams
    phase_offsets: dict[str, float]

    def __post_init__(self):
        check(set(self.phase_offsets) == set(LEGS), f"gait.phase_offsets must define {LEGS}")


def leg_phase(phase: float, offset: float) -> float:
    return (phase + offset) % 1.0


def foot_position(
    leg: str, phase: float, offset: float, p: GaitParams, body: BodyCorrection, g: LegGeometry
) -> tuple[float, float, float]:
    """Foot target for one leg, in that leg's frame (mm)."""
    lp = leg_phase(phase, offset)
    if lp < p.duty_factor:
        s = lp / p.duty_factor
        x = p.stride_length_mm / 2 - s * p.stride_length_mm
        lift = 0.0
    else:
        s = (lp - p.duty_factor) / (1.0 - p.duty_factor)
        x = -p.stride_length_mm / 2 + s * p.stride_length_mm
        lift = p.step_height_mm * math.sin(math.pi * s)

    x_hip, y_hip = g.hip_position(leg)
    hip_height = (
        p.body_height_mm
        + y_hip * math.sin(math.radians(body.roll_deg))
        - x_hip * math.sin(math.radians(body.pitch_deg))
    )
    y = LEG_SIDE[leg] * g.hip_offset_mm  # foot directly below the upper leg
    return x, y, -hip_height + lift


def in_stance(phase: float, offset: float, p: GaitParams) -> bool:
    return leg_phase(phase, offset) < p.duty_factor


class GaitGenerator:
    """Keeps track of gait phase and produces foot targets for all four legs."""

    def __init__(self, config: GaitConfig, geometry: LegGeometry):
        self.config = config
        self.geometry = geometry
        self.phase = 0.0

    def advance(self, dt_s: float, params: GaitParams) -> float:
        self.phase = (self.phase + dt_s / params.cycle_time_s) % 1.0
        return self.phase

    def feet(self, params: GaitParams, body: BodyCorrection) -> dict[str, tuple[float, float, float]]:
        return {
            leg: foot_position(leg, self.phase, self.config.phase_offsets[leg], params, body, self.geometry)
            for leg in LEGS
        }

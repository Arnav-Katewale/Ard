"""Test terrains for the MuJoCo model: the ground the simulated robot walks on.

Each terrain is a few MuJoCo geoms that replace the plain floor. The robot
always starts at the origin facing +x, so features are placed ahead of it
along +x.

    flat        level floor
    side_slope  the whole floor tilted sideways (about the x axis), left side up
    ramp        level floor, then an incline rising ahead of the robot
    bumps       level floor with a row of low strips across the path
    wall        level floor with a wall ahead (for the sonar and obstacle logic)

Slope and ramp use tilted MuJoCo planes. A plane collides as a half-space
(everything below its surface counts as inside it), so a tilted plane through
the line x = RAMP_START_M rises out of the floor there and stays below the
floor behind it: together they form a ramp that starts at RAMP_START_M.

All sizes and angles below are PLACEHOLDERS chosen to be gentle for a small
robot. They are test cases, not measurements of any real surface.
"""

from __future__ import annotations

import math

SIDE_SLOPE_DEG = 5.0     # PLACEHOLDER
RAMP_DEG = 5.0           # PLACEHOLDER: the simulated trot stalls on 8 deg
RAMP_START_M = 0.15      # PLACEHOLDER: how far ahead of the start the incline begins
BUMP_HEIGHT_M = 0.008    # PLACEHOLDER: under the gait's step height
BUMP_DEPTH_M = 0.02      # PLACEHOLDER: size along the walking direction
BUMP_START_M = 0.12      # PLACEHOLDER
BUMP_SPACING_M = 0.10    # PLACEHOLDER
BUMP_COUNT = 6
WALL_X_M = 0.5           # PLACEHOLDER: wall front face, metres ahead of the start
WALL_THICKNESS_M = 0.04

FLOOR_RGBA = "0.35 0.4 0.45 1"
FEATURE_RGBA = "0.55 0.45 0.35 1"


def _plane(name: str, pos: str, euler_deg: str = "0 0 0") -> str:
    return f'<geom name="{name}" type="plane" size="10 10 0.1" pos="{pos}" euler="{euler_deg}" rgba="{FLOOR_RGBA}"/>'


def _flat() -> list[str]:
    return [_plane("floor", "0 0 0")]


def _side_slope() -> list[str]:
    # Positive rotation about +x lifts +y (the robot's left side).
    return [_plane("floor", "0 0 0", f"{SIDE_SLOPE_DEG} 0 0")]


def _ramp() -> list[str]:
    # Negative rotation about +y lifts +x: the incline rises ahead.
    return _flat() + [
        f'<geom name="ramp" type="plane" size="10 10 0.1" pos="{RAMP_START_M} 0 0" '
        f'euler="0 {-RAMP_DEG} 0" rgba="{FEATURE_RGBA}"/>'
    ]


def _bumps() -> list[str]:
    geoms = _flat()
    for i in range(BUMP_COUNT):
        x = BUMP_START_M + i * BUMP_SPACING_M
        geoms.append(
            f'<geom name="bump{i}" type="box" pos="{x:.4f} 0 {BUMP_HEIGHT_M / 2:.4f}" '
            f'size="{BUMP_DEPTH_M / 2:.4f} 0.5 {BUMP_HEIGHT_M / 2:.4f}" rgba="{FEATURE_RGBA}"/>'
        )
    return geoms


def _wall() -> list[str]:
    half = WALL_THICKNESS_M / 2
    return _flat() + [
        f'<geom name="wall" type="box" pos="{WALL_X_M + half:.4f} 0 0.2" '
        f'size="{half:.4f} 1 0.2" rgba="{FEATURE_RGBA}"/>'
    ]


TERRAINS = {
    "flat": _flat,
    "side_slope": _side_slope,
    "ramp": _ramp,
    "bumps": _bumps,
    "wall": _wall,
}


def terrain_geoms(name: str) -> str:
    """MJCF geoms (worldbody children) for the named terrain."""
    if name not in TERRAINS:
        raise ValueError(f"unknown terrain {name!r}; choose one of {sorted(TERRAINS)}")
    return "\n    ".join(TERRAINS[name]())


def ramp_height_m(x_m: float) -> float:
    """Ground height of the ramp terrain at distance x_m ahead of the start."""
    return max(0.0, (x_m - RAMP_START_M) * math.tan(math.radians(RAMP_DEG)))

import itertools

import pytest

from robodog.config import ConfigError, build, load_config, section
from robodog.kinematics import (
    LEG_SIDE,
    LegGeometry,
    UnreachableError,
    joint_targets,
    leg_fk,
    leg_ik,
)

G = LegGeometry(body_length_mm=200, body_width_mm=110, hip_offset_mm=30, upper_leg_mm=80, lower_leg_mm=80)


@pytest.mark.parametrize("side", [1, -1])
def test_foot_straight_below_has_zero_abduction(side):
    hip, upper, knee = leg_ik(0.0, side * 30.0, -120.0, side, G)
    assert hip == pytest.approx(0.0, abs=1e-9)
    assert knee > 0


def test_fully_straight_leg():
    hip, upper, knee = leg_ik(0.0, 30.0, -160.0, 1, G)
    assert (hip, upper, knee) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_foot_forward_increases_upper_angle():
    _, upper_center, _ = leg_ik(0.0, 30.0, -120.0, 1, G)
    _, upper_fwd, _ = leg_ik(30.0, 30.0, -120.0, 1, G)
    assert upper_fwd > upper_center


def test_shorter_leg_bends_knee_more():
    _, _, knee_tall = leg_ik(0.0, 30.0, -140.0, 1, G)
    _, _, knee_low = leg_ik(0.0, 30.0, -100.0, 1, G)
    assert knee_low > knee_tall


@pytest.mark.parametrize(
    "x,y_out,z,side",
    [(x, y, z, s) for x, y, z, s in itertools.product([-40, 0, 35], [20, 30, 50], [-90, -120, -145], [1, -1])],
)
def test_ik_then_fk_round_trip(x, y_out, z, side):
    y = side * y_out
    angles = leg_ik(x, y, z, side, G)
    assert leg_fk(*angles, side, G) == pytest.approx((x, y, z), abs=1e-6)


@pytest.mark.parametrize("foot", [(0.0, 30.0, -170.0), (150.0, 30.0, -100.0), (0.0, 10.0, -10.0), (0.0, 0.0, 0.0)])
def test_unreachable_positions_rejected(foot):
    with pytest.raises(UnreachableError):
        leg_ik(*foot, 1, G)


def test_joint_targets_names_and_unreachable_leg_identified():
    feet = {leg: (0.0, LEG_SIDE[leg] * 30.0, -120.0) for leg in ("FL", "FR", "RL", "RR")}
    targets = joint_targets(feet, G)
    assert len(targets) == 12 and "RR_knee" in targets
    feet["RL"] = (0.0, 30.0, -500.0)
    with pytest.raises(UnreachableError, match="RL"):
        joint_targets(feet, G)


def test_hip_positions():
    assert G.hip_position("FL") == (100.0, 55.0)
    assert G.hip_position("RR") == (-100.0, -55.0)


def test_invalid_geometry_rejected():
    with pytest.raises(ConfigError):
        LegGeometry(body_length_mm=200, body_width_mm=110, hip_offset_mm=30, upper_leg_mm=0, lower_leg_mm=80)


def test_repository_geometry_builds():
    build(LegGeometry, section(load_config(), "geometry"), "geometry")

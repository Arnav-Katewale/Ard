import pytest

from robodog.config import ConfigError, load_config, section
from robodog.control.gait_controller import (
    BodyCorrection,
    GaitConfig,
    GaitGenerator,
    GaitParams,
    foot_position,
    in_stance,
)
from robodog.kinematics import LegGeometry, joint_targets

G = LegGeometry(body_length_mm=200, body_width_mm=110, hip_offset_mm=30, upper_leg_mm=80, lower_leg_mm=80)
P = GaitParams(stride_length_mm=60, step_height_mm=25, cycle_time_s=0.8, body_height_mm=120, duty_factor=0.6)
TROT = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
LEVEL = BodyCorrection()


def load_gait_config():
    gait = section(load_config(), "gait")
    return GaitConfig(base=GaitParams(**gait["base"]), phase_offsets=gait["phase_offsets"])


def test_stance_starts_forward_and_ends_backward():
    x_start, _, z_start = foot_position("FL", 0.0, 0.0, P, LEVEL, G)
    x_end, _, _ = foot_position("FL", 0.0, 0.5999, P, LEVEL, G)
    assert x_start == pytest.approx(30.0)
    assert x_end == pytest.approx(-30.0, abs=0.1)
    assert z_start == pytest.approx(-120.0)


def test_swing_apex_is_step_height_above_ground():
    # Swing is phase 0.6..1.0, so its midpoint is 0.8
    x, _, z = foot_position("FL", 0.8, 0.0, P, LEVEL, G)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(-120.0 + 25.0)


def test_trot_diagonal_pairs_in_sync():
    for phase in [0.0, 0.25, 0.55, 0.8]:
        assert in_stance(phase, TROT["FL"], P) == in_stance(phase, TROT["RR"], P)
        assert in_stance(phase, TROT["FR"], P) == in_stance(phase, TROT["RL"], P)


def test_with_duty_factor_over_half_some_legs_always_on_ground():
    for i in range(100):
        phase = i / 100
        assert sum(in_stance(phase, off, P) for off in TROT.values()) >= 2


def test_roll_correction_changes_left_and_right_extension_oppositely():
    # Negative roll correction (rotate left side down) shortens left legs.
    body = BodyCorrection(roll_deg=-5.0)
    _, _, z_left = foot_position("FL", 0.0, 0.0, P, body, G)
    _, _, z_right = foot_position("FR", 0.0, 0.0, P, body, G)
    assert z_left > -120.0 > z_right


def test_pitch_correction_changes_front_and_rear_extension_oppositely():
    # Negative pitch correction (nose up) lengthens front legs.
    body = BodyCorrection(pitch_deg=-5.0)
    _, _, z_front = foot_position("FL", 0.0, 0.0, P, body, G)
    _, _, z_rear = foot_position("RL", 0.0, 0.0, P, body, G)
    assert z_front < -120.0 < z_rear


def test_phase_advances_and_wraps():
    gen = GaitGenerator(GaitConfig(P, TROT), G)
    gen.advance(0.4, P)
    assert gen.phase == pytest.approx(0.5)
    gen.advance(0.6, P)
    assert gen.phase == pytest.approx(0.25)


def test_full_cycle_is_reachable_by_ik():
    gen = GaitGenerator(GaitConfig(P, TROT), G)
    for _ in range(80):
        gen.advance(0.01, P)
        assert len(joint_targets(gen.feet(P, LEVEL), G)) == 12


def test_invalid_duty_factor_rejected():
    with pytest.raises(ConfigError):
        GaitParams(60, 25, 0.8, 120, duty_factor=0.3)


def test_repository_gait_config_builds():
    assert load_gait_config().base.duty_factor == 0.6

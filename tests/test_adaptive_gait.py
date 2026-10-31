import math

import pytest

from robodog.config import load_config, section
from robodog.control.adaptive_gait import AdaptiveConfig, AdaptiveGaitController
from robodog.control.gait_controller import GaitParams
from robodog.control.obstacle import ObstacleState
from robodog.control.stability import StabilityLevel
from robodog.sensors import SensorStatus
from robodog.sensors.imu import GRAVITY_MS2, ImuReading

CFG = AdaptiveConfig.from_dict(section(load_config(), "adaptive"))
BASE = GaitParams(**section(load_config(), "gait.base"))
DT = 0.02
CLEAR, STABLE = ObstacleState.CLEAR, StabilityLevel.STABLE


def imu(pitch=0.0, roll=0.0, gyro_x=0.0, gyro_y=0.0, status=SensorStatus.OK):
    return ImuReading(0.0, 0.0, GRAVITY_MS2, gyro_x, gyro_y, 0.0, pitch, roll, 0.0, status)


def run(ctrl, steps, reading=None, stability=STABLE, obstacle=CLEAR):
    out = None
    for _ in range(steps):
        out = ctrl.update(reading or imu(), stability, obstacle, DT)
    return out


def test_level_and_clear_keeps_base_gait():
    out = run(AdaptiveGaitController(CFG, BASE), 50)
    assert out.params == BASE
    assert out.correction.roll_deg == 0.0 and out.correction.pitch_deg == 0.0


# --- correction direction ---------------------------------------------------------


def test_positive_roll_gives_negative_roll_correction():
    out = run(AdaptiveGaitController(CFG, BASE), 25, imu(roll=5.0))
    assert out.correction.roll_deg < 0


def test_negative_roll_gives_positive_roll_correction():
    out = run(AdaptiveGaitController(CFG, BASE), 25, imu(roll=-5.0))
    assert out.correction.roll_deg > 0


def test_nose_down_pitch_gives_nose_up_correction():
    out = run(AdaptiveGaitController(CFG, BASE), 25, imu(pitch=4.0))
    assert out.correction.pitch_deg < 0


def test_correction_never_exceeds_limit():
    out = run(AdaptiveGaitController(CFG, BASE), 500, imu(roll=40.0, pitch=-40.0))
    assert abs(out.correction.roll_deg) <= CFG.roll_pid.output_limit + 1e-9
    assert abs(out.correction.pitch_deg) <= CFG.pitch_pid.output_limit + 1e-9


def test_correction_is_rate_limited():
    ctrl = AdaptiveGaitController(CFG, BASE)
    out = ctrl.update(imu(roll=30.0), STABLE, CLEAR, DT)
    assert abs(out.correction.roll_deg) <= CFG.max_correction_rate_dps * DT + 1e-9


# --- obstacle response ------------------------------------------------------------


def test_near_obstacle_shortens_stride_raises_step_and_slows():
    out = run(AdaptiveGaitController(CFG, BASE), 200, obstacle=ObstacleState.NEAR)
    assert out.params.stride_length_mm == pytest.approx(BASE.stride_length_mm * 0.4)
    assert out.params.step_height_mm == pytest.approx(BASE.step_height_mm + 8.0)
    assert out.params.cycle_time_s == pytest.approx(BASE.cycle_time_s * 1.25)


def test_stop_comes_to_a_stand_gradually():
    ctrl = AdaptiveGaitController(CFG, BASE)
    first = ctrl.update(imu(), STABLE, ObstacleState.STOP, DT)
    assert 0 < first.params.stride_length_mm < BASE.stride_length_mm  # no abrupt jump
    out = run(ctrl, 200, obstacle=ObstacleState.STOP)
    assert out.standing


def test_stride_never_jumps_more_than_rate_limit():
    ctrl = AdaptiveGaitController(CFG, BASE)
    previous = BASE.stride_length_mm
    for obstacle in [ObstacleState.STOP] * 20 + [CLEAR] * 60:
        stride = ctrl.update(imu(), STABLE, obstacle, DT).params.stride_length_mm
        assert abs(stride - previous) <= CFG.stride_rate_down_mm_s * DT + 1e-9
        previous = stride


# --- stability / safety -------------------------------------------------------------


def test_unstable_reduces_stride():
    out = run(AdaptiveGaitController(CFG, BASE), 200, stability=StabilityLevel.UNSTABLE)
    assert out.params.stride_length_mm == pytest.approx(BASE.stride_length_mm * 0.5)


def test_invalid_imu_disables_correction_and_reduces_stride():
    ctrl = AdaptiveGaitController(CFG, BASE)
    run(ctrl, 50, imu(roll=6.0))
    bad = ImuReading(*([math.nan] * 8), 0.0, SensorStatus.ERROR)
    out = run(ctrl, 200, bad)
    assert not out.imu_ok
    assert out.correction.roll_deg == pytest.approx(0.0)
    assert out.params.stride_length_mm < BASE.stride_length_mm
    assert any("IMU invalid" in n for n in out.notes)

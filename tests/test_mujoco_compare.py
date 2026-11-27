"""Fixed vs adaptive comparison helpers (simulation only)."""

import math

import pytest

from robodog.config import load_config
from robodog.control.adaptive_gait import AdaptiveConfig, AdaptiveGaitController
from robodog.control.gait_controller import GaitParams
from robodog.control.obstacle import ObstacleState
from robodog.control.stability import StabilityLevel
from robodog.mujoco_sim.compare import MODES, Sample, Trial, fixed_gait_config, mode_config, summarise
from robodog.sensors import SensorStatus
from robodog.sensors.imu import GRAVITY_MS2, ImuReading

CFG = load_config()
BASE = GaitParams(**CFG["gait"]["base"])


def _tilted_imu(deg):
    return ImuReading(0.0, 0.0, GRAVITY_MS2, 0.0, 0.0, 0.0, deg, deg, 0.0, SensorStatus.OK)


def test_fixed_config_never_changes_the_gait():
    ctrl = AdaptiveGaitController(AdaptiveConfig.from_dict(fixed_gait_config(CFG)["adaptive"]), BASE)
    for obstacle in (ObstacleState.CLEAR, ObstacleState.CAUTION, ObstacleState.NEAR):
        for stability in StabilityLevel:
            for _ in range(100):
                out = ctrl.update(_tilted_imu(10.0), stability, obstacle, 0.02)
            assert out.params == BASE
            assert (out.correction.pitch_deg, out.correction.roll_deg) == (0.0, 0.0)


def test_fixed_config_keeps_the_obstacle_stop():
    ctrl = AdaptiveGaitController(AdaptiveConfig.from_dict(fixed_gait_config(CFG)["adaptive"]), BASE)
    for _ in range(200):
        out = ctrl.update(_tilted_imu(0.0), StabilityLevel.STABLE, ObstacleState.STOP, 0.02)
    assert out.standing


def test_fixed_config_leaves_the_original_untouched():
    before = CFG["adaptive"]["roll_pid"]["kp"]
    fixed_gait_config(CFG)
    assert CFG["adaptive"]["roll_pid"]["kp"] == before != 0


def test_half_stride_and_no_balance_modes():
    half = AdaptiveConfig.from_dict(mode_config(CFG, "fixed_half_stride")["adaptive"])
    assert set(half.stability_stride_scale.values()) == {0.5}
    nob = AdaptiveConfig.from_dict(mode_config(CFG, "adaptive_no_balance")["adaptive"])
    assert nob.roll_pid.kp == nob.pitch_pid.ki == 0.0
    assert nob.stability_stride_scale == AdaptiveConfig.from_dict(CFG["adaptive"]).stability_stride_scale
    with pytest.raises(ValueError):
        mode_config(CFG, "turbo")


def test_summary_by_hand():
    s = [Sample(0.0, 3.0, -4.0, 0, 0, 1.0, 60), Sample(0.1, 0.0, 0.0, 0, 0, 1.2, 60),
         Sample(0.2, -2.0, 1.0, 0, 0, 1.5, 60)]
    out = summarise(Trial("flat", "fixed", s, None))
    # Tilt per sample = max(|pitch|, |roll|) = 4, 0, 2; RMS = sqrt((16 + 0 + 4) / 3).
    assert out.rms_tilt_deg == pytest.approx(math.sqrt(20 / 3))
    assert out.peak_tilt_deg == 4.0
    assert out.distance_m == pytest.approx(0.5)
    assert out.estop_reason is None


def test_summary_of_an_immediate_estop():
    out = summarise(Trial("flat", "fixed", [], "tilt"))
    assert math.isnan(out.rms_tilt_deg) and out.distance_m == 0.0 and out.estop_reason == "tilt"


def test_short_trial_and_plot(tmp_path):
    pytest.importorskip("mujoco")
    pytest.importorskip("matplotlib")
    from robodog.mujoco_sim.compare import plot_trials, run_trial

    trials = [run_trial(CFG, "flat", mode, seconds=1.0) for mode in MODES]
    for t in trials:
        assert len(t.samples) == 50 and t.estop_reason is None
    fixed = next(t for t in trials if t.mode == "fixed")
    assert {s.stride_mm for s in fixed.samples} == {BASE.stride_length_mm}
    path = tmp_path / "plot.png"
    plot_trials(trials, str(path))
    assert path.stat().st_size > 10_000

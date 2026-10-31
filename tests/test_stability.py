import math

import pytest

from robodog.config import ConfigError, build, load_config, section
from robodog.control.stability import METRICS, StabilityConfig, StabilityLevel, StabilityMonitor
from robodog.sensors import SensorStatus
from robodog.sensors.imu import GRAVITY_MS2, ImuReading

EQUAL = {k: 0.2 for k in METRICS}
REFS = {"rms_pitch": 5.0, "rms_roll": 5.0, "peak_tilt": 12.0, "gyro_variance": 400.0, "accel_variance": 4.0}
CONFIG = StabilityConfig(
    window_s=2.0, min_samples=5, weights=EQUAL, references=REFS,
    marginal_score=0.5, unstable_score=1.0, emergency_tilt_deg=30.0,
)


def reading(t, pitch=0.0, roll=0.0, gyro_x=0.0, gyro_y=0.0, status=SensorStatus.OK):
    return ImuReading(0.0, 0.0, GRAVITY_MS2, gyro_x, gyro_y, 0.0, pitch, roll, t, status)


def feed(monitor, angles, dt=0.01, **kw):
    for i, (p, r) in enumerate(angles):
        monitor.update(reading(i * dt, p, r, **kw))


def test_unknown_until_enough_samples():
    m = StabilityMonitor(CONFIG)
    feed(m, [(0, 0)] * 3)
    assert m.metrics().level is StabilityLevel.UNKNOWN


def test_level_still_body_is_stable_with_zero_score():
    m = StabilityMonitor(CONFIG)
    feed(m, [(0, 0)] * 50)
    metrics = m.metrics()
    assert metrics.level is StabilityLevel.STABLE
    assert metrics.score == pytest.approx(0.0)


def test_rms_and_peak_values():
    m = StabilityMonitor(CONFIG)
    feed(m, [(3.0, -4.0), (-3.0, 4.0)] * 10)
    metrics = m.metrics()
    assert metrics.rms_pitch == pytest.approx(3.0)
    assert metrics.rms_roll == pytest.approx(4.0)
    assert metrics.peak_tilt == pytest.approx(4.0)


def sway(pitch_amp, roll_amp):
    return [(pitch_amp * math.sin(i / 3), roll_amp * math.cos(i / 3)) for i in range(100)]


def test_moderate_sway_is_marginal():
    m = StabilityMonitor(CONFIG)
    feed(m, sway(10.0, 8.0))
    assert m.metrics().level is StabilityLevel.MARGINAL


def test_large_sway_is_unstable():
    m = StabilityMonitor(CONFIG)
    feed(m, sway(20.0, 15.0))
    assert m.metrics().level is StabilityLevel.UNSTABLE


def test_score_increases_with_tilt():
    small, large = StabilityMonitor(CONFIG), StabilityMonitor(CONFIG)
    feed(small, [(1.0, 1.0), (-1.0, -1.0)] * 20)
    feed(large, [(4.0, 4.0), (-4.0, -4.0)] * 20)
    assert large.metrics().score > small.metrics().score


def test_old_samples_leave_the_window():
    m = StabilityMonitor(CONFIG)
    for i in range(50):
        m.update(reading(i * 0.01, pitch=20.0))
    for i in range(300):  # 3 s of level readings afterwards
        m.update(reading(0.5 + i * 0.01))
    assert m.metrics().peak_tilt == pytest.approx(0.0)


def test_invalid_readings_ignored():
    m = StabilityMonitor(CONFIG)
    feed(m, [(0, 0)] * 10, status=SensorStatus.ERROR)
    assert m.metrics().sample_count == 0


def test_emergency_tilt_detection():
    m = StabilityMonitor(CONFIG)
    assert m.is_tilt_unsafe(reading(0, roll=-31.0))
    assert not m.is_tilt_unsafe(reading(0, roll=-29.0))


def test_weights_must_sum_to_one():
    with pytest.raises(ConfigError, match="sum to 1"):
        StabilityConfig(2.0, 5, {k: 0.5 for k in METRICS}, REFS, 0.5, 1.0, 30.0)


def test_repository_config_builds():
    build(StabilityConfig, section(load_config(), "stability"), "stability")

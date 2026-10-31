import math

import pytest

from robodog.config import ConfigError, build, load_config, section
from robodog.control.obstacle import ObstacleConfig, ObstacleMonitor, ObstacleState
from robodog.sensors import SensorStatus
from robodog.sensors.ultrasonic import UltrasonicReading

CONFIG = ObstacleConfig(caution_cm=80, near_cm=30, stop_cm=15, hysteresis_cm=5, max_invalid_readings=3)


def ok(d):
    return UltrasonicReading(d, 0.0, SensorStatus.OK)


TIMEOUT = UltrasonicReading(math.nan, 0.0, SensorStatus.TIMEOUT)


@pytest.mark.parametrize(
    "distance,state",
    [(120, ObstacleState.CLEAR), (60, ObstacleState.CAUTION), (25, ObstacleState.NEAR), (10, ObstacleState.STOP)],
)
def test_thresholds(distance, state):
    assert ObstacleMonitor(CONFIG).update(ok(distance)) is state


def test_approaching_obstacle_passes_through_every_state():
    m = ObstacleMonitor(CONFIG)
    states = [m.update(ok(d)) for d in range(120, 5, -5)]
    assert states[0] is ObstacleState.CLEAR and states[-1] is ObstacleState.STOP
    assert states == sorted(states)  # severity never decreases while approaching


def test_hysteresis_prevents_flicker_at_threshold():
    m = ObstacleMonitor(CONFIG)
    m.update(ok(79))  # CAUTION
    for d in [81, 79, 82, 78, 84]:  # jitter around 80 cm, all < 80 + 5
        assert m.update(ok(d)) is ObstacleState.CAUTION
    assert m.update(ok(86)) is ObstacleState.CLEAR


def test_escalation_is_immediate():
    m = ObstacleMonitor(CONFIG)
    m.update(ok(100))
    assert m.update(ok(12)) is ObstacleState.STOP


def test_single_timeout_keeps_state_repeated_timeouts_raise_caution():
    m = ObstacleMonitor(CONFIG)
    m.update(ok(150))
    assert m.update(TIMEOUT) is ObstacleState.CLEAR
    assert m.update(TIMEOUT) is ObstacleState.CLEAR
    assert m.update(TIMEOUT) is ObstacleState.CAUTION
    assert m.last_distance_cm is None


def test_repeated_timeouts_never_lower_severity():
    m = ObstacleMonitor(CONFIG)
    m.update(ok(10))
    for _ in range(5):
        assert m.update(TIMEOUT) is ObstacleState.STOP


def test_thresholds_must_be_ordered():
    with pytest.raises(ConfigError):
        ObstacleConfig(caution_cm=20, near_cm=30, stop_cm=15, hysteresis_cm=5, max_invalid_readings=3)


def test_repository_config_builds():
    build(ObstacleConfig, section(load_config(), "obstacle"), "obstacle")

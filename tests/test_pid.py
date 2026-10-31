import pytest

from robodog.config import ConfigError
from robodog.control.pid import Pid, PidConfig


def make(kp=1.0, ki=0.0, kd=0.0, output_limit=10.0, integral_limit=100.0):
    return Pid(PidConfig(kp, ki, kd, output_limit, integral_limit))


def test_proportional_only():
    assert make(kp=2.0).update(error=3.0, error_rate=0.0, dt_s=0.01) == pytest.approx(6.0)


def test_derivative_uses_supplied_rate():
    assert make(kp=0.0, kd=0.5).update(error=0.0, error_rate=4.0, dt_s=0.01) == pytest.approx(2.0)


def test_integral_accumulates_constant_error():
    pid = make(kp=0.0, ki=1.0)
    for _ in range(100):
        out = pid.update(error=2.0, error_rate=0.0, dt_s=0.01)
    assert out == pytest.approx(2.0)  # integral of 2 over 1 s


def test_output_is_bounded():
    pid = make(kp=100.0, output_limit=8.0)
    assert pid.update(5.0, 0.0, 0.01) == 8.0
    assert pid.update(-5.0, 0.0, 0.01) == -8.0


def test_known_error_sequence_stays_bounded():
    pid = make(kp=3.0, ki=5.0, kd=0.5, output_limit=8.0, integral_limit=2.0)
    for e in [10, -10, 25, 0, -40, 5, 5, 5, -3]:
        assert abs(pid.update(e, e * 10, 0.02)) <= 8.0
    assert abs(pid.integral) <= 2.0


def test_anti_windup_stops_integral_growth_while_saturated():
    pid = make(kp=1.0, ki=1.0, output_limit=5.0)
    for _ in range(1000):
        pid.update(error=20.0, error_rate=0.0, dt_s=0.01)  # saturated for 10 s
    assert pid.integral < 0.1
    # When the error reverses, the output responds immediately (no stored windup).
    assert pid.update(error=-1.0, error_rate=0.0, dt_s=0.01) < 0


def test_reset_clears_integral():
    pid = make(kp=0.0, ki=1.0)
    pid.update(1.0, 0.0, 1.0)
    pid.reset()
    assert pid.integral == 0.0


def test_negative_gain_rejected():
    with pytest.raises(ConfigError):
        PidConfig(kp=-1.0, ki=0.0, kd=0.0, output_limit=1.0, integral_limit=1.0)

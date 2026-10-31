import math

import pytest

from robodog.config import ConfigError, build, load_config, section
from robodog.sensors import SensorStatus
from robodog.sensors.imu import (
    GRAVITY_MS2,
    Imu,
    ImuConfig,
    RawImuSample,
    accel_pitch_roll,
    remap_axes,
)
from robodog.sensors.simulated_sensors import SimulatedImuBackend

G = GRAVITY_MS2
CONFIG = ImuConfig(axis_map=["+x", "+y", "+z"], max_accel_ms2=156.9, max_gyro_dps=2000.0)


class FixedBackend:
    def __init__(self, accel, gyro=(0.0, 0.0, 0.0)):
        self.sample = RawImuSample(accel=accel, gyro=gyro)

    def read_raw(self):
        return self.sample


# --- accelerometer tilt math ----------------------------------------------------


def test_level_body_is_zero_pitch_and_roll():
    pitch, roll = accel_pitch_roll(0.0, 0.0, G)
    assert pitch == pytest.approx(0.0) and roll == pytest.approx(0.0)


@pytest.mark.parametrize("angle", [-30.0, -10.0, 10.0, 30.0])
def test_roll_from_gravity(angle):
    a = math.radians(angle)
    pitch, roll = accel_pitch_roll(0.0, G * math.sin(a), G * math.cos(a))
    assert roll == pytest.approx(angle)
    assert pitch == pytest.approx(0.0)


@pytest.mark.parametrize("angle", [-30.0, -10.0, 10.0, 30.0])
def test_pitch_from_gravity(angle):
    a = math.radians(angle)
    pitch, roll = accel_pitch_roll(-G * math.sin(a), 0.0, G * math.cos(a))
    assert pitch == pytest.approx(angle)
    assert roll == pytest.approx(0.0)


def test_nose_vertical_down_is_plus_90_pitch():
    pitch, _ = accel_pitch_roll(-G, 0.0, 0.0)
    assert pitch == pytest.approx(90.0)


# --- axis mapping -----------------------------------------------------------------


def test_identity_axis_map():
    assert remap_axes((1.0, 2.0, 3.0), ["+x", "+y", "+z"]) == (1.0, 2.0, 3.0)


def test_rotated_mounting_axis_map():
    # Sensor mounted rotated 180 deg about z: its +x points backwards.
    assert remap_axes((1.0, 2.0, 3.0), ["-x", "-y", "+z"]) == (-1.0, -2.0, 3.0)


@pytest.mark.parametrize("bad", [["+x", "+y"], ["+x", "+x", "+z"], ["x", "+y", "+z"], ["+x", "+y", "+w"]])
def test_invalid_axis_map_rejected(bad):
    with pytest.raises(ConfigError):
        ImuConfig(axis_map=bad, max_accel_ms2=150.0, max_gyro_dps=2000.0)


def test_repository_config_builds():
    cfg = build(ImuConfig, section(load_config(), "imu"), "imu")
    assert len(cfg.axis_map) == 3


# --- Imu reader -------------------------------------------------------------------


def test_reading_has_timestamp_and_angles():
    backend = SimulatedImuBackend()
    backend.set_attitude(pitch_deg=5.0, roll_deg=-3.0)
    imu = Imu(backend, CONFIG, clock=lambda: 12.5)
    r = imu.read()
    assert r.status is SensorStatus.OK and r.valid
    assert r.timestamp_s == 12.5
    assert r.pitch_deg == pytest.approx(5.0)
    assert r.roll_deg == pytest.approx(-3.0)


def test_nan_reading_is_invalid():
    imu = Imu(FixedBackend((math.nan, 0.0, G)), CONFIG)
    r = imu.read()
    assert r.status is SensorStatus.INVALID and not r.valid
    assert imu.consecutive_failures == 1


def test_out_of_range_reading_is_invalid():
    imu = Imu(FixedBackend((0.0, 0.0, G), gyro=(5000.0, 0.0, 0.0)), CONFIG)
    assert imu.read().status is SensorStatus.INVALID


def test_all_zero_accel_is_invalid():
    imu = Imu(FixedBackend((0.0, 0.0, 0.0)), CONFIG)
    assert imu.read().status is SensorStatus.INVALID


def test_backend_exception_becomes_error_status_and_recovers():
    backend = SimulatedImuBackend()
    imu = Imu(backend, CONFIG)
    backend.fail_next = True
    assert imu.read().status is SensorStatus.ERROR
    assert imu.consecutive_failures == 1
    assert imu.read().status is SensorStatus.OK
    assert imu.consecutive_failures == 0


def test_simulated_noise_is_deterministic_for_a_seed():
    a = SimulatedImuBackend(accel_noise_std=0.2, seed=7).read_raw()
    b = SimulatedImuBackend(accel_noise_std=0.2, seed=7).read_raw()
    assert a == b

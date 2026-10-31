import pytest

from robodog.calibration.calibration import (
    CalibrationThresholds,
    ImuCalibration,
    collect_stationary_readings,
    estimate_imu_calibration,
    validate_calibration,
)
from robodog.config import build, load_config, section
from robodog.sensors.imu import Imu, ImuConfig
from robodog.sensors.simulated_sensors import SimulatedImuBackend

CONFIG = ImuConfig(axis_map=["+x", "+y", "+z"], max_accel_ms2=156.9, max_gyro_dps=2000.0)
THRESHOLDS = CalibrationThresholds(
    min_samples=200, max_gyro_mean_dps=0.2, max_level_error_deg=0.5, max_gyro_std_dps=0.5, max_accel_std_ms2=0.1
)


def stationary_backend(**kwargs):
    defaults = dict(accel_noise_std=0.03, gyro_noise_std=0.1, gyro_bias=(1.5, -0.8, 0.4), seed=1)
    defaults.update(kwargs)
    return SimulatedImuBackend(**defaults)


def collect(imu, n=400):
    return collect_stationary_readings(imu, n, interval_s=0.0, sleep=lambda s: None)


def test_gyro_bias_is_recovered():
    cal = estimate_imu_calibration(collect(Imu(stationary_backend(), CONFIG)), source="simulated")
    assert cal.gyro_bias["x"] == pytest.approx(1.5, abs=0.03)
    assert cal.gyro_bias["y"] == pytest.approx(-0.8, abs=0.03)
    assert cal.gyro_bias["z"] == pytest.approx(0.4, abs=0.03)
    assert cal.noise["gyro_std"] == pytest.approx(0.1, abs=0.02)
    assert cal.sample_count == 400 and cal.source == "simulated"


def test_tilted_mounting_becomes_orientation_offset():
    backend = stationary_backend()
    backend.set_attitude(pitch_deg=2.0, roll_deg=-1.5)  # IMU mounted slightly crooked
    cal = estimate_imu_calibration(collect(Imu(backend, CONFIG)), source="simulated")
    assert cal.orientation_offset["pitch"] == pytest.approx(2.0, abs=0.05)
    assert cal.orientation_offset["roll"] == pytest.approx(-1.5, abs=0.05)


def test_uncalibrated_fails_validation_and_calibrated_passes():
    backend = stationary_backend()
    backend.set_attitude(pitch_deg=2.0, roll_deg=-1.5)
    imu = Imu(backend, CONFIG)
    before = collect(imu)
    assert not validate_calibration(before, THRESHOLDS).passed

    imu.calibration = estimate_imu_calibration(before, source="simulated")
    report = validate_calibration(collect(imu), THRESHOLDS)
    assert report.passed, report.lines()


def test_too_few_samples_fails_validation():
    report = validate_calibration(collect(Imu(stationary_backend(), CONFIG), n=10), THRESHOLDS)
    assert not report.passed
    assert "sample count" in report.lines()[0]


def test_invalid_readings_are_skipped_during_collection():
    backend = stationary_backend()
    imu = Imu(backend, CONFIG)
    backend.fail_next = True
    assert len(collect(imu, n=10)) == 9


def test_save_and_load_round_trip(tmp_path):
    cal = estimate_imu_calibration(collect(Imu(stationary_backend(), CONFIG), n=50), source="simulated")
    path = tmp_path / "imu_calibration.json"
    cal.save(path)
    assert ImuCalibration.load(path) == cal


def test_identity_calibration_changes_nothing():
    cal = ImuCalibration.identity()
    assert cal.correct((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)) == ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0))
    assert cal.correct_angles(3.0, -2.0) == (3.0, -2.0)


def test_repository_thresholds_build():
    build(CalibrationThresholds, section(load_config(), "calibration"), "calibration")

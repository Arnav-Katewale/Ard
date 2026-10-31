import pytest

from robodog.calibration.calibration import CalibrationThresholds
from robodog.config import build, load_config, section
from robodog.control.obstacle import ObstacleState
from robodog.simulation import build_simulated_robot


@pytest.fixture(scope="module")
def finished_run():
    cfg = load_config()
    robot = build_simulated_robot(cfg)
    thresholds = build(CalibrationThresholds, section(cfg, "calibration"), "calibration")
    calibration, report = robot.calibrate(thresholds)
    steps = round(robot.config.duration_s * robot.loop.config.rate_hz)
    for _ in range(steps):
        robot.step()
    return robot, calibration, report


def test_calibration_recovers_simulated_bias_and_mounting(finished_run):
    robot, calibration, report = finished_run
    assert report.passed, report.lines()
    assert calibration.source == "simulated"
    assert calibration.gyro_bias["x"] == pytest.approx(1.2, abs=0.05)
    assert calibration.orientation_offset["pitch"] == pytest.approx(1.5, abs=0.1)


def test_run_completes_without_emergency_stop_or_rejections(finished_run):
    robot, _, _ = finished_run
    results = [r.result for r in robot.records]
    assert all(r.estop_reason is None for r in results)
    assert all(r.rejected is None for r in results)


def test_roll_disturbance_is_opposed(finished_run):
    robot, _, _ = finished_run
    mid_disturbance = [r for r in robot.records if 4.0 < r.t < 6.0]
    assert all(r.result.adaptive.correction.roll_deg < 0 for r in mid_disturbance)


def test_obstacle_approach_ends_standing_in_stop(finished_run):
    robot, _, _ = finished_run
    last = robot.records[-1].result.adaptive
    assert last.obstacle is ObstacleState.STOP
    assert last.standing


def test_obstacle_states_progress_in_order(finished_run):
    robot, _, _ = finished_run
    seen = []
    for r in robot.records:
        state = r.result.adaptive.obstacle
        if not seen or seen[-1] is not state:
            seen.append(state)
    assert seen == [ObstacleState.CLEAR, ObstacleState.CAUTION, ObstacleState.NEAR, ObstacleState.STOP]


def test_run_is_deterministic():
    def final_targets():
        robot = build_simulated_robot(load_config())
        for _ in range(100):
            robot.step()
        return robot.records[-1].result.joint_targets

    assert final_targets() == final_targets()

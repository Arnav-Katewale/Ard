import math

import pytest

from robodog.config import load_config
from robodog.control.obstacle import ObstacleState
from robodog.robot import build_control_loop
from robodog.sensors.simulated_sensors import SimulatedEchoBackend, SimulatedImuBackend
from robodog.servos import lx16a
from robodog.servos.simulated_servo import SimulatedLx16aPort


class SimClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, s):
        self.now += s


def make_loop():
    cfg = load_config()
    imu = SimulatedImuBackend()
    echo = SimulatedEchoBackend(cfg["ultrasonic"]["speed_of_sound_ms"])
    port = SimulatedLx16aPort.with_servos([j["servo_id"] for j in cfg["joints"]])
    clock = SimClock()
    loop = build_control_loop(cfg, imu, echo, port, clock=clock, sleep=clock.sleep)
    return loop, imu, echo, port, clock


def run(loop, clock, steps):
    results = []
    for _ in range(steps):
        clock.now += loop.dt_s
        results.append(loop.step())
    return results


def test_level_robot_walks_with_valid_joint_targets():
    loop, _, _, port, clock = make_loop()
    results = run(loop, clock, 100)
    assert all(r.estop_reason is None and r.rejected is None for r in results)
    assert len(results[-1].joint_targets) == 12
    assert len(port.written) > 100  # commands reached the simulated servo bus


def test_obstacle_approach_reaches_stop_and_stands():
    loop, _, echo, _, clock = make_loop()
    echo.distance_cm = 10.0
    results = run(loop, clock, 150)
    assert results[-1].adaptive.obstacle is ObstacleState.STOP
    assert results[-1].adaptive.standing


def test_excessive_tilt_triggers_emergency_stop():
    loop, imu, _, port, clock = make_loop()
    run(loop, clock, 10)
    imu.set_attitude(pitch_deg=0.0, roll_deg=35.0)
    result = run(loop, clock, 1)[0]
    assert "tilt" in result.estop_reason
    assert loop.joints.estop_active
    assert all(s.stopped for s in port.servos.values())
    sent = len(port.written)
    run(loop, clock, 5)
    assert len(port.written) == sent  # no further servo commands after emergency stop


def test_imu_loss_triggers_emergency_stop():
    loop, imu, _, _, clock = make_loop()
    imu.read_raw = lambda: (_ for _ in ()).throw(OSError("I2C lost"))
    results = run(loop, clock, 6)
    assert results[-1].estop_reason and "IMU unavailable" in results[-1].estop_reason


def test_rejected_targets_are_not_sent_and_repeated_rejection_stops():
    loop, _, _, port, clock = make_loop()
    run(loop, clock, 5)
    # Simulate an impossible gait request: body far higher than the legs can reach.
    loop.adaptive.base = loop.adaptive.params = type(loop.adaptive.params)(
        stride_length_mm=0, step_height_mm=0, cycle_time_s=0.8, body_height_mm=400, duty_factor=0.6
    )
    sent = len(port.written)
    results = run(loop, clock, 10)
    assert results[0].rejected and "reachable" in results[0].rejected
    new_packets = port.written[sent:]
    assert not any(p[4] == lx16a.MOVE_TIME_WRITE for p in new_packets)  # no move commands
    assert results[-1].estop_reason and "rejected" in results[-1].estop_reason
    assert all(p[4] == lx16a.MOVE_STOP for p in new_packets)  # only the emergency stop


def test_ultrasonic_sampled_at_lower_rate():
    loop, _, _, _, clock = make_loop()
    results = run(loop, clock, 20)
    sampled = [r for r in results if r.ultrasonic is not None]
    assert len(sampled) == 4


def test_roll_disturbance_produces_opposing_correction():
    loop, imu, _, _, clock = make_loop()
    imu.set_attitude(pitch_deg=0.0, roll_deg=6.0)
    result = run(loop, clock, 30)[-1]
    assert result.adaptive.correction.roll_deg < 0
    assert not math.isnan(result.metrics.rms_roll)
    assert result.metrics.rms_roll == pytest.approx(6.0, abs=0.01)

"""The real control-loop code running on the MuJoCo model (pip install -r requirements-sim.txt)."""

import math

import pytest

mujoco = pytest.importorskip("mujoco")

from robodog.config import build, load_config, section  # noqa: E402
from robodog.kinematics import LEG_END, LEG_SIDE, LEGS, LegGeometry, joint_targets  # noqa: E402
from robodog.mujoco_sim.backends import MujocoImuBackend, MujocoSim  # noqa: E402
from robodog.mujoco_sim.model import build_mjcf  # noqa: E402
from robodog.mujoco_sim.runner import build_sim_control_loop, run  # noqa: E402
from robodog.sensors.imu import Imu, ImuConfig  # noqa: E402
from robodog.servos import lx16a  # noqa: E402
from robodog.servos.joint import JointConfig  # noqa: E402

CFG = load_config()
GEOMETRY = build(LegGeometry, section(CFG, "geometry"), "geometry")


def _leaning_sim(front_drop_mm=0.0, left_drop_mm=0.0) -> MujocoSim:
    """A robot standing still with some legs shorter, so the body leans; settled for 1.5 s."""
    height = section(CFG, "gait")["base"]["body_height_mm"]
    feet = {
        leg: (0.0, -(height - front_drop_mm * (LEG_END[leg] > 0) - left_drop_mm * (LEG_SIDE[leg] > 0)))
        for leg in LEGS
    }
    sim = MujocoSim(CFG)
    sim.servo_port.set_initial_pose(joint_targets(feet, GEOMETRY))
    sim.sleep(1.5)
    return sim


@pytest.mark.parametrize("front_drop_mm,left_drop_mm", [(0.0, 0.0), (30.0, 0.0), (0.0, 25.0)])
def test_imu_tilt_matches_the_simulated_body(front_drop_mm, left_drop_mm):
    sim = _leaning_sim(front_drop_mm, left_drop_mm)
    reading = Imu(MujocoImuBackend(sim), build(ImuConfig, section(CFG, "imu"), "imu"), clock=sim.clock).read()
    pitch, roll = sim.body_pitch_roll_deg()
    assert reading.valid
    assert (reading.pitch_deg, reading.roll_deg) == pytest.approx((pitch, roll), abs=0.5)
    # The lean is real and in the documented direction: shorter front legs = nose down.
    if front_drop_mm:
        assert pitch > 5.0
    if left_drop_mm:
        assert roll < -5.0  # shorter left legs = left side down = negative roll


def test_servo_position_reads_return_the_simulated_joint_angles():
    sim = _leaning_sim(front_drop_mm=30.0)
    bus = lx16a.Lx16aBus(sim.servo_port)
    for j in (build(JointConfig, j, "joints") for j in CFG["joints"]):
        joint_deg = j.servo_to_joint_deg(bus.read_position_deg(j.servo_id))
        true_deg = math.degrees(sim.data.qpos[sim.model.joint(j.name).qposadr[0]])
        assert joint_deg == pytest.approx(true_deg, abs=lx16a.DEG_PER_UNIT)


def test_sonar_measures_the_distance_to_a_wall():
    wall_x_m, wall_half_thickness_m = 0.6, 0.02
    xml = build_mjcf(CFG).replace(
        '<body name="torso"',
        f'<geom name="wall" type="box" pos="{wall_x_m} 0 0.2" size="{wall_half_thickness_m} 1 0.2"/>\n'
        '    <body name="torso"',
    )
    sim = MujocoSim(CFG, model=mujoco.MjModel.from_xml_string(xml))
    loop = build_sim_control_loop(CFG, sim)
    first = next(run(loop, sim, seconds=loop.dt_s))  # step 0 pings the sonar
    sonar_x_m = float(sim.data.site("sonar").xpos[0])
    expected_cm = (wall_x_m - wall_half_thickness_m - sonar_x_m) * 100.0
    assert first.ultrasonic.valid
    assert first.ultrasonic.distance_cm == pytest.approx(expected_cm, abs=0.5)


def test_sonar_reports_no_echo_in_open_space():
    sim = MujocoSim(CFG)
    loop = build_sim_control_loop(CFG, sim)
    first = next(run(loop, sim, seconds=loop.dt_s))
    assert not first.ultrasonic.valid


def test_robot_trots_five_seconds_without_falling():
    """The unchanged control loop walks the simulated robot: no stop, small tilt, forward progress."""
    sim = MujocoSim(CFG)  # already settled standing
    loop = build_sim_control_loop(CFG, sim)
    start_x = sim.torso_position_m()[0]
    worst_tilt = worst_error = 0.0
    for result in run(loop, sim, seconds=5.0):
        assert result.estop_reason is None, result.estop_reason
        assert result.rejected is None, result.rejected
        assert result.imu.valid
        pitch, roll = sim.body_pitch_roll_deg()
        worst_tilt = max(worst_tilt, abs(pitch), abs(roll))
        worst_error = max(worst_error, abs(result.imu.pitch_deg - pitch), abs(result.imu.roll_deg - roll))
    assert worst_tilt < 15.0          # true body tilt, from the simulator
    assert worst_error < 5.0          # complementary filter vs. true tilt
    assert sim.torso_position_m()[0] - start_x > 0.1   # it actually walked forward
    assert sim.torso_position_m()[2] > 0.08            # torso still well off the floor

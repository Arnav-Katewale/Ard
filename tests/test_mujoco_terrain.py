"""The real control loop on each simulated terrain (pip install -r requirements-sim.txt).

These are simulation results with placeholder physics, not hardware results.
"""

import math
import statistics

import pytest

mujoco = pytest.importorskip("mujoco")

from robodog.config import build, load_config, section  # noqa: E402
from robodog.control.obstacle import ObstacleState  # noqa: E402
from robodog.mujoco_sim import terrain  # noqa: E402
from robodog.mujoco_sim.backends import MujocoImuBackend, MujocoSim  # noqa: E402
from robodog.mujoco_sim.model import build_mjcf, load_model  # noqa: E402
from robodog.mujoco_sim.runner import build_sim_control_loop, run  # noqa: E402
from robodog.sensors.imu import Imu, ImuConfig  # noqa: E402

CFG = load_config()


@pytest.mark.parametrize("name", sorted(terrain.TERRAINS))
def test_every_terrain_builds_a_model(name):
    model = load_model(CFG, name)
    geoms = {model.geom(i).name for i in range(model.ngeom)}
    assert "floor" in geoms and "torso" in geoms


def test_unknown_terrain_is_rejected():
    with pytest.raises(ValueError, match="unknown terrain"):
        build_mjcf(CFG, "lava")


def test_imu_reads_the_side_slope_while_standing():
    sim = MujocoSim(CFG, terrain="side_slope")
    reading = Imu(MujocoImuBackend(sim), build(ImuConfig, section(CFG, "imu"), "imu"), clock=sim.clock).read()
    pitch, roll = sim.body_pitch_roll_deg()
    assert reading.valid
    assert reading.roll_deg == pytest.approx(roll, abs=0.5)
    # Left side up = positive roll, about the slope angle (the legs are not levelled).
    assert reading.roll_deg == pytest.approx(terrain.SIDE_SLOPE_DEG, abs=1.5)


def _walk(name, seconds):
    """Trot on a terrain; return the sim and per-step (result, true pitch, true roll, torso x)."""
    sim = MujocoSim(CFG, terrain=name)
    loop = build_sim_control_loop(CFG, sim)
    steps = []
    for result in run(loop, sim, seconds):
        pitch, roll = sim.body_pitch_roll_deg()
        steps.append((result, pitch, roll, sim.torso_position_m()[0]))
    return sim, steps


@pytest.mark.parametrize("name", ["flat", "side_slope", "ramp", "bumps"])
def test_robot_trots_across_terrain(name):
    sim, steps = _walk(name, seconds=8.0)
    for result, pitch, roll, _ in steps:
        assert result.estop_reason is None, result.estop_reason
        assert result.rejected is None, result.rejected
        assert result.imu.valid
        assert max(abs(pitch), abs(roll)) < 25.0                          # true tilt
        assert abs(result.imu.pitch_deg - pitch) < 7.0                    # filter vs truth
        assert abs(result.imu.roll_deg - roll) < 7.0
    assert steps[-1][3] - steps[0][3] > 0.2                               # made progress
    if name == "ramp":
        x, _, z = sim.torso_position_m()
        start_z = (section(CFG, "gait")["base"]["body_height_mm"] + section(CFG, "mujoco_model")["foot_radius_mm"]) / 1000
        assert x > terrain.RAMP_START_M + 0.15                            # well onto the incline
        assert z - start_z > 0.5 * terrain.ramp_height_m(x)               # and climbed it
        on_incline = [r for r, _, _, sx in steps if sx > terrain.RAMP_START_M + 0.1]
        # The IMU sees nose up and the adaptive controller pitches the body
        # against it (on flat ground the mean correction is about 0).
        assert statistics.fmean(r.imu.pitch_deg for r in on_incline) < -0.5
        assert statistics.fmean(r.adaptive.correction.pitch_deg for r in on_incline) > 3.0


def test_robot_stops_in_front_of_the_wall():
    sim, steps = _walk("wall", seconds=20.0)
    states = set()
    for result, pitch, roll, _ in steps:
        assert result.estop_reason is None, result.estop_reason
        assert max(abs(pitch), abs(roll)) < 25.0
        states.add(result.adaptive.obstacle)
    assert {ObstacleState.CAUTION, ObstacleState.NEAR, ObstacleState.STOP} <= states
    last = steps[-1][0]
    assert last.adaptive.standing                                         # came to a stand
    gap_m = terrain.WALL_X_M - float(sim.data.site("sonar").xpos[0])
    assert gap_m > 0.10                                                   # never reached the wall
    readings = [r.ultrasonic.distance_cm for r, *_ in steps[-50:] if r.ultrasonic and r.ultrasonic.valid]
    assert readings and readings[-1] == pytest.approx(gap_m * 100.0, abs=1.0)


def test_ramp_height_helper():
    assert terrain.ramp_height_m(0.0) == 0.0
    assert terrain.ramp_height_m(terrain.RAMP_START_M + 1.0) == pytest.approx(
        math.tan(math.radians(terrain.RAMP_DEG))
    )

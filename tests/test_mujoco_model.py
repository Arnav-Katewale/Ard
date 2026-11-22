import math

import pytest

from robodog.config import ConfigError, build, load_config, section
from robodog.kinematics import LEG_END, LEG_SIDE, LEGS, LegGeometry, leg_fk_2dof
from robodog.mujoco_sim.model import MujocoModelConfig, build_mjcf, standing_targets

CFG = load_config()


def test_mjcf_has_every_leg_joint_and_sensor():
    xml = build_mjcf(CFG)
    for leg in LEGS:
        assert f'joint name="{leg}_hip"' in xml and f'joint name="{leg}_knee"' in xml
    for sensor in ("imu_accel", "imu_gyro", "torso_quat", "sonar"):
        assert f'name="{sensor}"' in xml


@pytest.mark.parametrize("change", [{"servo_torque_limit_nm": 0.0}, {"foot_radius_mm": 5.0}])
def test_invalid_model_config_rejected(change):
    data = dict(section(CFG, "mujoco_model"), **change)
    with pytest.raises(ConfigError):
        build(MujocoModelConfig, data, "mujoco_model")


def test_standing_targets_put_feet_below_hips():
    g = build(LegGeometry, section(CFG, "geometry"), "geometry")
    targets = standing_targets(CFG)
    height = section(CFG, "gait")["base"]["body_height_mm"]
    for leg in LEGS:
        x, z = leg_fk_2dof(targets[f"{leg}_hip"], targets[f"{leg}_knee"], g)
        assert (x, z) == pytest.approx((0.0, -height), abs=1e-6)


# --- tests that need MuJoCo (pip install -r requirements-sim.txt) -------------

mujoco = pytest.importorskip("mujoco")
from robodog.mujoco_sim.model import load_model  # noqa: E402


def _set_joints(model, data, angles_deg):
    for name, deg in angles_deg.items():
        data.qpos[model.joint(name).qposadr[0]] = math.radians(deg)
        data.ctrl[model.actuator(name).id] = math.radians(deg)


def test_model_loads_with_one_servo_per_configured_joint():
    model = load_model(CFG)
    configured = {j["name"] for j in CFG["joints"]}
    assert model.nu == len(configured) == 8
    assert {model.actuator(i).name for i in range(model.nu)} == configured


def test_joint_ranges_match_config():
    model = load_model(CFG)
    for j in CFG["joints"]:
        lo, hi = model.joint(j["name"]).range
        assert (math.degrees(lo), math.degrees(hi)) == pytest.approx((j["min_deg"], j["max_deg"]))


@pytest.mark.parametrize("hip,knee", [(0.0, 20.0), (30.0, 60.0), (-40.0, 100.0)])
def test_simulated_leg_matches_forward_kinematics(hip, knee):
    """Joint axes and lengths in the model must agree with leg_fk_2dof."""
    model = load_model(CFG)
    data = mujoco.MjData(model)
    data.qpos[0:3] = (0.0, 0.0, 1.0)  # torso floating at 1 m, level, facing +x
    _set_joints(model, data, {f"{leg}_{p}": a for leg in LEGS for p, a in (("hip", hip), ("knee", knee))})
    mujoco.mj_kinematics(model, data)
    g = build(LegGeometry, section(CFG, "geometry"), "geometry")
    fx, fz = leg_fk_2dof(hip, knee, g)
    for leg in LEGS:
        hip_x = LEG_END[leg] * g.body_length_mm / 2
        hip_y = LEG_SIDE[leg] * g.body_width_mm / 2
        foot = data.geom(f"{leg}_foot").xpos * 1000.0  # mm
        assert foot == pytest.approx((hip_x + fx, hip_y, 1000.0 + fz), abs=1e-6)


def test_robot_stands_at_gait_height_under_gravity():
    model = load_model(CFG)
    data = mujoco.MjData(model)
    _set_joints(model, data, standing_targets(CFG))
    heights = []
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        heights.append(data.qpos[2])

    m = section(CFG, "mujoco_model")
    expected_z = (section(CFG, "gait")["base"]["body_height_mm"] + m["foot_radius_mm"]) / 1000.0
    w, x, y, z = data.qpos[3:7]
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
    assert data.qpos[2] == pytest.approx(expected_z, abs=0.01)  # within 1 cm
    assert tilt_deg < 2.0
    last_half_second = heights[-int(0.5 / model.opt.timestep):]
    assert max(last_half_second) - min(last_half_second) < 0.001  # settled: < 1 mm movement

    # A level, still IMU measures gravity's reaction: about +9.81 m/s^2 up.
    accel = data.sensor("imu_accel").data
    assert accel == pytest.approx((0.0, 0.0, 9.81), abs=0.2)
    # Only the feet touch the floor, and the robot is not creeping.
    touching = {model.geom(data.contact[i].geom2).name for i in range(data.ncon)}
    assert touching == {f"{leg}_foot" for leg in LEGS}
    assert abs(data.qpos[0]) < 0.005 and abs(data.qpos[1]) < 0.005
    # Nothing in front of the robot: the rangefinder reports -1 (no hit).
    assert data.sensor("sonar").data[0] == -1

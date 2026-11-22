"""Build a MuJoCo model of RoboDog from config/robot.toml.

The model is generated, not hand-written, so leg lengths, hip spacing and
joint limits always match what the control code uses. Only the physical
properties that the control code does not need (masses, servo strength,
friction) come from the [mujoco_model] section, and those are placeholders.

Model layout (MuJoCo units: metres, kilograms, seconds; angles in degrees):

    torso (free-floating box)
      imu site       accelerometer, gyro and orientation sensors
      sonar site     rangefinder looking forward (+x), like the HC-SR04
      4 x leg:  hip joint -> upper leg -> knee joint -> lower leg -> foot

Joint axes are chosen to match robodog/kinematics.py exactly:
    hip  rotates about -y, so a positive angle swings the foot forward
    knee rotates about +y, so a positive angle folds the lower leg backward
With these axes the foot of a leg is at
    x = L2 sin(hip) + L3 sin(hip - knee),  z = -(L2 cos(hip) + L3 cos(hip - knee))
which is leg_fk_2dof.

Each joint is driven by a position servo (proportional torque towards the
commanded angle) whose torque is capped at the LX-16A's rated value. Joint
armature stands in for the gearbox: the motor turning through a high gear
ratio makes each joint feel heavier than the leg alone. The head servo is not
modelled: the gait does not use it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from robodog.config import build, check, section
from robodog.control.gait_controller import GaitParams
from robodog.kinematics import LEG_END, LEG_SIDE, LEGS, LegGeometry, joint_targets
from robodog.servos.joint import JointConfig

MM = 0.001


@dataclass(frozen=True)
class MujocoModelConfig:
    torso_mass_kg: float
    torso_height_mm: float
    upper_leg_mass_kg: float
    lower_leg_mass_kg: float
    leg_radius_mm: float
    foot_radius_mm: float
    foot_friction: float
    servo_torque_limit_nm: float
    servo_kp_nm_per_rad: float
    joint_damping: float
    joint_armature_kgm2: float
    timestep_s: float

    def __post_init__(self):
        for f in ("torso_mass_kg", "torso_height_mm", "upper_leg_mass_kg", "lower_leg_mass_kg",
                  "leg_radius_mm", "foot_radius_mm", "foot_friction", "servo_torque_limit_nm",
                  "servo_kp_nm_per_rad", "timestep_s"):
            check(getattr(self, f) > 0, f"mujoco_model.{f} must be positive")
        check(self.joint_damping >= 0, "mujoco_model.joint_damping must be >= 0")
        check(self.joint_armature_kgm2 >= 0, "mujoco_model.joint_armature_kgm2 must be >= 0")
        check(self.foot_radius_mm > self.leg_radius_mm,
              "mujoco_model.foot_radius_mm must exceed leg_radius_mm so only the feet touch the floor")


def standing_targets(cfg: dict[str, Any]) -> dict[str, float]:
    """Joint angles (deg) that put every foot straight below its hip at gait body height."""
    g = build(LegGeometry, section(cfg, "geometry"), "geometry")
    base = build(GaitParams, section(cfg, "gait")["base"], "gait.base")
    return joint_targets({leg: (0.0, -base.body_height_mm) for leg in LEGS}, g)


def build_mjcf(cfg: dict[str, Any]) -> str:
    """Return the MJCF (MuJoCo XML) text for the robot described by `cfg`."""
    g = build(LegGeometry, section(cfg, "geometry"), "geometry")
    base = build(GaitParams, section(cfg, "gait")["base"], "gait.base")
    m = build(MujocoModelConfig, section(cfg, "mujoco_model"), "mujoco_model")
    joints = {j.name: j for j in (build(JointConfig, j, "joints") for j in cfg["joints"])}
    for leg in LEGS:
        for part in ("hip", "knee"):
            check(f"{leg}_{part}" in joints, f"joints: missing {leg}_{part} for the MuJoCo model")

    L2, L3 = g.upper_leg_mm * MM, g.lower_leg_mm * MM
    leg_r, foot_r = m.leg_radius_mm * MM, m.foot_radius_mm * MM
    half_len = g.body_length_mm * MM / 2 + leg_r
    half_wid = g.body_width_mm * MM / 2 - leg_r
    half_h = m.torso_height_mm * MM / 2
    start_z = base.body_height_mm * MM + foot_r + 0.01  # drop from just above standing height

    legs_xml = []
    for leg in LEGS:
        hx, hy = LEG_END[leg] * g.body_length_mm * MM / 2, LEG_SIDE[leg] * g.body_width_mm * MM / 2
        hip, knee = joints[f"{leg}_hip"], joints[f"{leg}_knee"]
        legs_xml.append(f"""
      <body name="{leg}_upper" pos="{hx:.4f} {hy:.4f} 0">
        <joint name="{leg}_hip" axis="0 -1 0" range="{hip.min_deg} {hip.max_deg}"/>
        <geom name="{leg}_upper" type="capsule" fromto="0 0 0 0 0 {-L2:.4f}" size="{leg_r:.4f}"
              mass="{m.upper_leg_mass_kg}"/>
        <body name="{leg}_lower" pos="0 0 {-L2:.4f}">
          <joint name="{leg}_knee" axis="0 1 0" range="{knee.min_deg} {knee.max_deg}"/>
          <geom name="{leg}_lower" type="capsule" fromto="0 0 0 0 0 {-L3:.4f}" size="{leg_r:.4f}"
                mass="{m.lower_leg_mass_kg}"/>
          <geom name="{leg}_foot" type="sphere" pos="0 0 {-L3:.4f}" size="{foot_r:.4f}"
                mass="0.001" friction="{m.foot_friction} 0.005 0.0001"/>
        </body>
      </body>""")

    # Joint ranges above are converted to radians by angle="degree", but actuator
    # controls are not: servo commands (data.ctrl) are always in radians.
    actuators = "\n".join(
        f'    <position name="{name}" joint="{name}" kp="{m.servo_kp_nm_per_rad}" '
        f'ctrlrange="{math.radians(j.min_deg):.5f} {math.radians(j.max_deg):.5f}" '
        f'forcerange="{-m.servo_torque_limit_nm} '
        f'{m.servo_torque_limit_nm}" forcelimited="true"/>'
        for name, j in joints.items()
    )

    return f"""<mujoco model="robodog">
  <!-- Generated by robodog/mujoco_sim/model.py from config/robot.toml. Do not edit by hand. -->
  <compiler angle="degree" autolimits="true"/>
  <option timestep="{m.timestep_s}" integrator="implicitfast"/>
  <default>
    <joint damping="{m.joint_damping}" armature="{m.joint_armature_kgm2}" limited="true"/>
    <geom contype="1" conaffinity="1" condim="3" rgba="0.75 0.75 0.8 1"/>
  </default>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.35 0.4 0.45 1"/>
    <body name="torso" pos="0 0 {start_z:.4f}">
      <freejoint name="root"/>
      <geom name="torso" type="box" size="{half_len:.4f} {half_wid:.4f} {half_h:.4f}"
            mass="{m.torso_mass_kg}" rgba="0.2 0.45 0.8 1"/>
      <site name="imu" pos="0 0 0" size="0.005"/>
      <site name="sonar" pos="{half_len:.4f} 0 0" zaxis="1 0 0" size="0.005"/>{"".join(legs_xml)}
    </body>
  </worldbody>
  <actuator>
{actuators}
  </actuator>
  <sensor>
    <accelerometer name="imu_accel" site="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <framequat name="torso_quat" objtype="site" objname="imu"/>
    <rangefinder name="sonar" site="sonar"/>
  </sensor>
</mujoco>
"""


def load_model(cfg: dict[str, Any]):
    """Compile the generated MJCF into a mujoco.MjModel (imports mujoco lazily)."""
    import mujoco

    return mujoco.MjModel.from_xml_string(build_mjcf(cfg))

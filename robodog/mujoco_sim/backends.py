"""MuJoCo stand-ins for the robot's hardware, so the real control loop can drive the model.

build_control_loop() takes three backends: an IMU, an ultrasonic echo timer and
a serial port to the LX-16A servo bus. This module provides all three, backed
by one MuJoCo simulation instead of hardware:

    MujocoImuBackend     reads the simulated accelerometer and gyro
    MujocoEchoBackend    turns the simulated rangefinder distance into an echo time
    MujocoServoPort      decodes the same LX-16A packets the joint-safety layer
                         sends and moves the simulated joints

Nothing in the control code changes: it cannot tell these from hardware.

Simulation time is the clock. MujocoSim.clock() returns simulated seconds,
and MujocoSim.sleep() runs physics forward instead of waiting, so the
ultrasonic sensor's "wait between pings" also advances the simulation.

Simplifications (this is a model, not the robot):
- Sensors are ideal: no noise, bias or dropouts.
- A servo moves its setpoint in a straight line from its previous setpoint to
  the new target over the commanded move time, like the LX-16A's own
  interpolation, and the MuJoCo position actuator then follows that setpoint
  with limited torque.
- Voltage and temperature replies are fixed nominal values.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from robodog.config import build, section
from robodog.mujoco_sim.model import load_model, standing_targets
from robodog.sensors.imu import RawImuSample
from robodog.servos import lx16a
from robodog.servos.joint import JointConfig

NOMINAL_VOLTAGE_MV = 7400
NOMINAL_TEMPERATURE_C = 30


class MujocoSim:
    """One MuJoCo model + state, with a simulated clock.

    The robot starts standing: every joint at standing_targets(cfg) and the
    torso resting at gait body height.
    """

    def __init__(self, cfg: dict[str, Any], model=None):
        import mujoco

        self._mj = mujoco
        self.model = model if model is not None else load_model(cfg)
        self.data = mujoco.MjData(self.model)
        self.servo_port = MujocoServoPort(self, [build(JointConfig, j, "joints") for j in cfg["joints"]])
        self.servo_port.set_initial_pose(standing_targets(cfg))
        # Feet resting on the floor: hip height = body height, plus the foot sphere's radius.
        m = section(cfg, "mujoco_model")
        self.data.qpos[2] = (section(cfg, "gait")["base"]["body_height_mm"] + m["foot_radius_mm"]) / 1000.0
        mujoco.mj_forward(self.model, self.data)

    def clock(self) -> float:
        return float(self.data.time)

    def sleep(self, seconds: float) -> None:
        self.run_until(self.data.time + seconds)

    def run_until(self, t_s: float) -> None:
        """Step physics until simulated time reaches t_s."""
        # Half a timestep of tolerance so floating-point drift never adds a step.
        while self.data.time < t_s - 0.5 * self.model.opt.timestep:
            self.servo_port.update_setpoints(self.data.time)
            self._mj.mj_step(self.model, self.data)

    def sensor(self, name: str):
        return self.data.sensor(name).data

    # --- ground truth for tests and plots (the controller never sees these) ----

    def body_pitch_roll_deg(self) -> tuple[float, float]:
        """Torso pitch (positive = nose down) and roll (positive = left up) from its orientation."""
        w, x, y, z = self.data.qpos[3:7]
        # Gravity's "up" direction expressed in the body frame, from the quaternion.
        up = (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
        roll = math.degrees(math.atan2(up[1], up[2]))
        pitch = math.degrees(math.atan2(-up[0], math.hypot(up[1], up[2])))
        return pitch, roll

    def torso_position_m(self) -> tuple[float, float, float]:
        return tuple(float(v) for v in self.data.qpos[0:3])


class MujocoImuBackend:
    """IMU backend: MuJoCo's accelerometer (m/s^2) and gyro (rad/s -> deg/s).

    Both sensors sit at the torso's "imu" site, whose axes are the body axes
    (+x forward, +y left, +z up), so config [imu] axis_map should be the
    identity for the simulation.
    """

    def __init__(self, sim: MujocoSim):
        self._sim = sim

    def read_raw(self) -> RawImuSample:
        accel = tuple(float(a) for a in self._sim.sensor("imu_accel"))
        gyro = tuple(math.degrees(float(w)) for w in self._sim.sensor("imu_gyro"))
        return RawImuSample(accel=accel, gyro=gyro)


class MujocoEchoBackend:
    """Echo backend: rangefinder distance -> HC-SR04 echo time (out and back).

    MuJoCo reports -1 when the ray hits nothing; that, or an echo longer than
    the timeout, is reported as a missed echo (None), like the real sensor.
    """

    def __init__(self, sim: MujocoSim, speed_of_sound_ms: float):
        self._sim = sim
        self._speed = speed_of_sound_ms

    def measure_echo_s(self, timeout_s: float) -> float | None:
        distance_m = float(self._sim.sensor("sonar")[0])
        if distance_m < 0:
            return None
        echo_s = 2.0 * distance_m / self._speed
        return echo_s if echo_s <= timeout_s else None


@dataclass
class _ServoState:
    joint: JointConfig
    actuator_id: int
    qpos_adr: int
    start_rad: float = 0.0
    target_rad: float = 0.0
    move_start_s: float = 0.0
    move_time_s: float = 0.0

    def setpoint(self, t_s: float) -> float:
        if self.move_time_s <= 0:
            return self.target_rad
        frac = min(1.0, max(0.0, (t_s - self.move_start_s) / self.move_time_s))
        return self.start_rad + frac * (self.target_rad - self.start_rad)


class MujocoServoPort:
    """A serial port with the simulated robot's servos on the other end.

    Accepts the real LX-16A packets written by Lx16aBus and replies with real
    packets, exactly like robodog.servos.simulated_servo.SimulatedLx16aPort,
    except that positions are the MuJoCo joints' actual angles.
    """

    def __init__(self, sim: MujocoSim, joints: list[JointConfig]):
        self._sim = sim
        model = sim.model
        self.servos = {
            j.servo_id: _ServoState(j, model.actuator(j.name).id, model.joint(j.name).qposadr[0])
            for j in joints
        }
        self._rx = bytearray()

    def set_initial_pose(self, joint_deg: dict[str, float]) -> None:
        """Place every joint (and its servo setpoint) at the given joint angles."""
        data = self._sim.data
        for s in self.servos.values():
            rad = math.radians(joint_deg[s.joint.name])
            data.qpos[s.qpos_adr] = rad
            s.start_rad = s.target_rad = rad
            s.move_time_s = 0.0
            data.ctrl[s.actuator_id] = rad

    def update_setpoints(self, t_s: float) -> None:
        for s in self.servos.values():
            self._sim.data.ctrl[s.actuator_id] = s.setpoint(t_s)

    # --- SerialPort protocol -----------------------------------------------------

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def read(self, size: int) -> bytes:
        out = bytes(self._rx[:size])
        del self._rx[:size]
        return out

    def write(self, data: bytes) -> int:
        servo_id, cmd, params = lx16a.decode_packet(bytes(data))
        if servo_id == lx16a.BROADCAST_ID:
            targets = list(self.servos.values())
        else:
            targets = [self.servos[servo_id]] if servo_id in self.servos else []
        for servo in targets:
            self._handle(servo, cmd, params)
        return len(data)

    # --- packet handling -----------------------------------------------------------

    def _actual_joint_deg(self, s: _ServoState) -> float:
        return math.degrees(self._sim.data.qpos[s.qpos_adr])

    def _handle(self, s: _ServoState, cmd: int, params: bytes) -> None:
        now = self._sim.clock()
        if cmd == lx16a.MOVE_TIME_WRITE:
            servo_deg = lx16a.units_to_degrees(int.from_bytes(params[0:2], "little"))
            s.start_rad = s.setpoint(now)
            s.target_rad = math.radians(s.joint.servo_to_joint_deg(servo_deg))
            s.move_start_s = now
            s.move_time_s = int.from_bytes(params[2:4], "little") / 1000.0
        elif cmd == lx16a.MOVE_STOP:
            # Hold wherever the joint actually is right now.
            s.start_rad = s.target_rad = math.radians(self._actual_joint_deg(s))
            s.move_time_s = 0.0
        elif cmd == lx16a.POS_READ:
            servo_deg = s.joint.joint_to_servo_deg(self._actual_joint_deg(s))
            units = round(servo_deg / lx16a.DEG_PER_UNIT)
            self._reply(s, cmd, units.to_bytes(2, "little", signed=True))
        elif cmd == lx16a.ID_READ:
            self._reply(s, cmd, bytes([s.joint.servo_id]))
        elif cmd == lx16a.VIN_READ:
            self._reply(s, cmd, NOMINAL_VOLTAGE_MV.to_bytes(2, "little"))
        elif cmd == lx16a.TEMP_READ:
            self._reply(s, cmd, bytes([NOMINAL_TEMPERATURE_C]))

    def _reply(self, s: _ServoState, cmd: int, params: bytes) -> None:
        self._rx += lx16a.encode_packet(s.joint.servo_id, cmd, params)

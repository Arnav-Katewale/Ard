"""Assemble the control loop from the configuration file and a set of backends.

The same function is used with simulated backends (software validation) and
will be used with real hardware backends (IMU driver, LgpioEchoBackend,
pyserial port) during hardware integration.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from robodog.calibration.calibration import ImuCalibration
from robodog.config import build, section
from robodog.control.adaptive_gait import AdaptiveConfig, AdaptiveGaitController
from robodog.control.control_loop import ControlLoop, ControlLoopConfig
from robodog.control.gait_controller import GaitConfig, GaitGenerator, GaitParams
from robodog.control.obstacle import ObstacleConfig, ObstacleMonitor
from robodog.control.stability import StabilityConfig, StabilityMonitor
from robodog.kinematics import LegGeometry
from robodog.sensors.imu import Imu, ImuBackend, ImuConfig
from robodog.sensors.ultrasonic import EchoBackend, UltrasonicConfig, UltrasonicSensor
from robodog.servos.joint import JointConfig, SafeJointGroup, ServoBusConfig
from robodog.servos.lx16a import Lx16aBus, SerialPort


def build_control_loop(
    cfg: dict[str, Any],
    imu_backend: ImuBackend,
    echo_backend: EchoBackend,
    serial_port: SerialPort,
    calibration: ImuCalibration | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> ControlLoop:
    geometry = build(LegGeometry, section(cfg, "geometry"), "geometry")
    gait_cfg = section(cfg, "gait")
    base = build(GaitParams, gait_cfg["base"], "gait.base")
    bus_config = build(ServoBusConfig, section(cfg, "servo_bus"), "servo_bus")
    joints = [build(JointConfig, j, "joints") for j in cfg["joints"]]

    group = SafeJointGroup(Lx16aBus(serial_port, echo=bus_config.echo), joints, bus_config)
    group.sync_from_servos()

    return ControlLoop(
        config=build(ControlLoopConfig, section(cfg, "control_loop"), "control_loop"),
        imu=Imu(imu_backend, build(ImuConfig, section(cfg, "imu"), "imu"), clock=clock, calibration=calibration),
        ultrasonic=UltrasonicSensor(
            echo_backend, build(UltrasonicConfig, section(cfg, "ultrasonic"), "ultrasonic"), clock=clock, sleep=sleep
        ),
        stability=StabilityMonitor(build(StabilityConfig, section(cfg, "stability"), "stability")),
        obstacle=ObstacleMonitor(build(ObstacleConfig, section(cfg, "obstacle"), "obstacle")),
        adaptive=AdaptiveGaitController(AdaptiveConfig.from_dict(section(cfg, "adaptive")), base),
        gait=GaitGenerator(GaitConfig(base=base, phase_offsets=gait_cfg["phase_offsets"]), geometry),
        geometry=geometry,
        joints=group,
    )

"""Run the real control loop on the MuJoCo model.

    sim = MujocoSim(cfg)
    loop = build_sim_control_loop(cfg, sim)
    for result in run(loop, sim, seconds=5.0):
        ...

build_sim_control_loop() calls the same build_control_loop() the robot will
use, with the MuJoCo backends plugged in. run() then alternates:

    1. one control step (read IMU and sonar, decide, send servo packets)
    2. physics until the next control tick (1 / control_loop.rate_hz later)

so the controller sees the simulated world at exactly its configured rate.
"""

from __future__ import annotations

from typing import Any, Iterator

from robodog.calibration.calibration import ImuCalibration
from robodog.config import section
from robodog.control.control_loop import ControlLoop, StepResult
from robodog.mujoco_sim.backends import MujocoEchoBackend, MujocoImuBackend, MujocoSim
from robodog.robot import build_control_loop


def build_sim_control_loop(
    cfg: dict[str, Any], sim: MujocoSim, calibration: ImuCalibration | None = None
) -> ControlLoop:
    return build_control_loop(
        cfg,
        imu_backend=MujocoImuBackend(sim),
        echo_backend=MujocoEchoBackend(sim, section(cfg, "ultrasonic")["speed_of_sound_ms"]),
        serial_port=sim.servo_port,
        calibration=calibration,
        clock=sim.clock,
        sleep=sim.sleep,
    )


def run(loop: ControlLoop, sim: MujocoSim, seconds: float) -> Iterator[StepResult]:
    """Yield one StepResult per control step for `seconds` of simulated time."""
    start = sim.clock()
    for n in range(round(seconds / loop.dt_s)):
        result = loop.step()
        # Run physics to the next tick. If the ultrasonic sensor already slept
        # past it, this does nothing and the loop catches up next step.
        sim.run_until(start + (n + 1) * loop.dt_s)
        yield result

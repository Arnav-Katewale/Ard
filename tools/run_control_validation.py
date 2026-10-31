"""Control-loop software validation with simulated sensor input.

Runs the real RoboDog control code (calibration, stability metrics, obstacle
logic, adaptive gait, kinematics, joint safety, LX-16A protocol) against
simulated sensors and a simulated servo bus. No hardware is used.

This validates software behaviour only. It is not a measurement of the
physical robot; hardware integration testing is planned separately.

Usage:
    python tools/run_control_validation.py
    python tools/run_control_validation.py --every 0.25 --csv data/runs/validation.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robodog.calibration.calibration import CalibrationThresholds  # noqa: E402
from robodog.config import DEFAULT_CONFIG_PATH, build, load_config, section  # noqa: E402
from robodog.kinematics import LEGS  # noqa: E402
from robodog.servos import lx16a  # noqa: E402
from robodog.simulation import SimulationRecord, build_simulated_robot  # noqa: E402

BANNER = """\
==========================================================================
 RoboDog control-loop software validation
 Sensor input: SIMULATED   Servo bus: SIMULATED   Body model: toy model
 This is not a hardware result. Hardware integration testing is pending.
=========================================================================="""


def fmt(value: float, unit: str = "", digits: int = 1) -> str:
    return f"{value:+.{digits}f}{unit}" if value == value else "n/a"  # NaN check


def print_step(rec: SimulationRecord, last_distance: float | None) -> None:
    r = rec.result
    print(f"\nt = {rec.t:5.2f} s")
    if r.estop_reason:
        print(f"  EMERGENCY STOP: {r.estop_reason}")
        return
    a = r.adaptive
    distance = f"{last_distance:.0f} cm" if last_distance is not None else "no valid echo"
    print(f"  Pitch: {fmt(r.imu.pitch_deg, '°')}   Roll: {fmt(r.imu.roll_deg, '°')}"
          f"   (calibrated IMU, accelerometer tilt)")
    print(f"  Obstacle: {distance} -> {a.obstacle.name}")
    print(f"  Stability: {a.stability.name}"
          + (f" (score {r.metrics.score:.2f})" if r.metrics.score == r.metrics.score else ""))
    print("  Gait:")
    print(f"    Stride length:         {a.params.stride_length_mm:5.1f} mm")
    print(f"    Step height:           {a.params.step_height_mm:5.1f} mm")
    print(f"    Cycle time:            {a.params.cycle_time_s:5.2f} s")
    print(f"    Body pitch correction: {fmt(a.correction.pitch_deg, '°')}")
    print(f"    Body roll correction:  {fmt(a.correction.roll_deg, '°')}")
    if r.joint_targets:
        print("  Joint targets (hip / upper / knee):")
        for leg in LEGS:
            hip, upper, knee = (r.joint_targets[f"{leg}_{j}"] for j in ("hip", "upper", "knee"))
            print(f"    {leg}: {hip:+6.1f}°  {upper:+6.1f}°  {knee:+6.1f}°")
    if r.rejected:
        print(f"  REJECTED (not sent): {r.rejected}")
    for note in r.notes:
        print(f"  note: {note}")


def write_csv(path: Path, records: list[SimulationRecord]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    joint_names = [f"{leg}_{j}" for leg in LEGS for j in ("hip", "upper", "knee")]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "true_pitch_deg", "true_roll_deg", "imu_pitch_deg", "imu_roll_deg",
                    "obstacle_true_cm", "obstacle_state", "stability", "stability_score",
                    "stride_mm", "step_height_mm", "cycle_time_s", "pitch_corr_deg", "roll_corr_deg",
                    "gait_phase", *joint_names, "rejected", "estop"])
        for rec in records:
            r, a = rec.result, rec.result.adaptive
            joints = [r.joint_targets[n] if r.joint_targets else "" for n in joint_names]
            w.writerow([f"{rec.t:.3f}", rec.true_pitch_deg, rec.true_roll_deg, r.imu.pitch_deg, r.imu.roll_deg,
                        rec.obstacle_true_cm, a.obstacle.name if a else "", a.stability.name if a else "",
                        r.metrics.score if r.metrics else "",
                        a.params.stride_length_mm if a else "", a.params.step_height_mm if a else "",
                        a.params.cycle_time_s if a else "", a.correction.pitch_deg if a else "",
                        a.correction.roll_deg if a else "", r.phase if r.phase is not None else "",
                        *joints, r.rejected or "", r.estop_reason or ""])


def print_summary(robot, records: list[SimulationRecord]) -> None:
    results = [rec.result for rec in records]
    moves = sum(1 for p in robot.servo_port.written if p[4] == lx16a.MOVE_TIME_WRITE)
    print("\n--------------------------------------------------------------------------")
    print(" Summary (simulated run)")
    print(f"  Control steps:               {len(results)} at {robot.loop.config.rate_hz:.0f} Hz")
    print(f"  Joint commands accepted:     {sum(1 for r in results if r.joint_targets)}")
    print(f"  Joint commands rejected:     {sum(1 for r in results if r.rejected)}")
    print(f"  Emergency stops:             {sum(1 for r in results if r.estop_reason)}")
    print(f"  LX-16A move packets encoded: {moves}")
    print("  Obstacle state changes:")
    previous = None
    for rec in records:
        a = rec.result.adaptive
        if a and a.obstacle is not previous:
            print(f"    t = {rec.t:5.2f} s  {a.obstacle.name}")
            previous = a.obstacle
    print("--------------------------------------------------------------------------")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--duration", type=float, help="seconds of simulated time (default: from config)")
    parser.add_argument("--every", type=float, default=1.0, help="print a status block every N simulated seconds")
    parser.add_argument("--csv", type=Path, help="also write every step to this CSV file")
    args = parser.parse_args()

    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    cfg = load_config(args.config)
    robot = build_simulated_robot(cfg)
    print(BANNER)

    thresholds = build(CalibrationThresholds, section(cfg, "calibration"), "calibration")
    calibration, report = robot.calibrate(thresholds)
    print("\nIMU calibration (simulated stationary data):")
    print(f"  gyro bias (deg/s):  x={calibration.gyro_bias['x']:+.3f}  y={calibration.gyro_bias['y']:+.3f}"
          f"  z={calibration.gyro_bias['z']:+.3f}")
    print(f"  orientation offset: pitch={calibration.orientation_offset['pitch']:+.2f}°"
          f"  roll={calibration.orientation_offset['roll']:+.2f}°")
    for line in report.lines():
        print(f"  {line}")
    if not report.passed:
        print("Calibration validation failed; not starting the control loop.")
        return 1

    duration = args.duration if args.duration is not None else robot.config.duration_s
    rate = robot.loop.config.rate_hz
    print_every_steps = max(1, round(args.every * rate))
    for i in range(round(duration * rate)):
        rec = robot.step()
        if (i + 1) % print_every_steps == 0 or rec.result.estop_reason:
            print_step(rec, robot.loop.obstacle.last_distance_cm)
        if rec.result.estop_reason:
            break

    print_summary(robot, robot.records)
    if args.csv:
        write_csv(args.csv, robot.records)
        print(f"Wrote {len(robot.records)} rows to {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

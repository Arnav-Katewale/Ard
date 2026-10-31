# RoboDog

A quadruped robot dog built around a Raspberry Pi 5, an IMU, an ultrasonic distance
sensor and LX-16A serial bus servos.

**Project question:** Can a low-cost quadruped robot recognize when its gait is unstable
and change its own movement in real time using onboard sensors and Raspberry Pi 5
computation?

## Current status

| Area | Code implemented | Validated in software (simulated input) | Validated on hardware |
|---|---|---|---|
| IMU interface + basic pitch/roll | ✅ | ✅ | ⏳ pending (IMU chip not yet confirmed; hardware driver not written) |
| Ultrasonic interface | ✅ incl. Pi 5 `lgpio` backend | ✅ | ⏳ pending (Echo voltage protection must be confirmed first) |
| LX-16A servo protocol | ✅ | ✅ byte-exact packet tests | ⏳ pending |
| Joint limits + emergency stop | ✅ | ✅ | ⏳ pending |
| IMU calibration structure | ✅ | ✅ | ⏳ pending (no real calibration data yet) |
| Leg kinematics (IK/FK) | ✅ | ✅ | ⏳ pending (geometry not measured) |
| Stability metrics | ✅ | ✅ | ⏳ pending (reference values need walking data) |
| Obstacle states | ✅ | ✅ | ⏳ pending |
| Adaptive gait prototype | ✅ | ✅ | ⏳ pending |

**No hardware test results exist yet.** Every number in `config/robot.toml` marked
`PLACEHOLDER` is a starting value, not a measurement. Hardware integration testing is
planned as a separate milestone.

## Hardware

| Component | Status |
|---|---|
| Raspberry Pi 5 | Confirmed |
| LewanSoul / Hiwonder LX-16A serial bus servos | Confirmed from the servo reference files; count and controller board to be confirmed (config assumes 12 servos, 3 per leg) |
| IMU | Model to be confirmed |
| Ultrasonic sensor | Model and supply voltage to be confirmed (HC-SR04-style trigger/echo assumed) |
| Servo power / shared ground | To be confirmed |

Mechanical reference files:
- `Lewansoul lx-16a BUS Servo CAD file/`: LX-16A servo models, tracked in this repository.
- Raspberry Pi 5 CAD reference files are used locally for mechanical layout but are
  **not included in the repository** (third-party material, ~130 MB).

## Software architecture

```
 IMU ──► calibration ──► pitch/roll ──► stability metrics ──┐
                                                             ├─► adaptive gait ──► trot gait ──► inverse    ──► joint safety ──► LX-16A
 ultrasonic ──► distance ──► obstacle state (hysteresis) ───┘    (PID + rate       generator     kinematics      layer             servo bus
                                                                   limits)                                                          │
       ▲                                                                                                                            │
       └──────────────────────────────────── robot body moves, sensors measure the result ◄────────────────────────────────────────┘
```

| Module | Responsibility |
|---|---|
| `robodog/sensors/imu.py` | Timestamped raw accel/gyro, axis remapping, validation, accelerometer pitch/roll |
| `robodog/sensors/ultrasonic.py` | Time-of-flight distance, timeouts, range checks, Pi 5 `lgpio` backend |
| `robodog/servos/lx16a.py` | LX-16A packet protocol: move, stop, torque, read position/voltage/temperature |
| `robodog/servos/joint.py` | Joint limits, all-or-nothing commands, speed limit, latched emergency stop |
| `robodog/calibration/calibration.py` | Stationary IMU calibration, JSON save/load, PASS/FAIL validation report |
| `robodog/kinematics.py` | 3-DOF leg forward and inverse kinematics |
| `robodog/control/stability.py` | Sliding-window stability metrics and composite score |
| `robodog/control/obstacle.py` | CLEAR / CAUTION / NEAR / STOP with hysteresis |
| `robodog/control/pid.py` | PID with output limits and anti-windup |
| `robodog/control/gait_controller.py` | Phase-based trot foot trajectories with body attitude offsets |
| `robodog/control/adaptive_gait.py` | Combines IMU, stability and obstacle input into smoothed gait parameters |
| `robodog/control/control_loop.py` | One closed-loop step plus emergency-stop triggers |
| `robodog/robot.py` | Builds the loop from config with any set of backends (simulated or hardware) |
| `robodog/simulation.py` | Simulated sensor input and a toy body model for software validation |
| `config/robot.toml` | Every tunable value (no magic numbers in code) |

Hardware access is isolated behind small backend interfaces (`read_raw()`,
`measure_echo_s()`, a serial port object), so the same control code runs against
simulated backends now and real ones later.

### Conventions

Body frame: **+x forward, +y left, +z up**. Roll is rotation about +x (positive = left side
up). Pitch is rotation about +y (positive = nose down, by the right-hand rule).

## Key algorithms

**Basic pitch/roll (accelerometer only).** A stationary accelerometer measures gravity:
`roll = atan2(ay, az)`, `pitch = atan2(-ax, sqrt(ay² + az²))`. This is correct when the
robot is still and noisy while walking. It is a baseline, not the final orientation
estimator; a complementary filter with the gyro is the planned next step.

**IMU calibration.** On a level surface: gyro bias = mean gyro reading; orientation
offset = tilt of the mean gravity vector; noise = standard deviation. Validation then
checks that calibrated readings show near-zero gyro, level pitch/roll and noise below
thresholds. Limitation: from one pose, an x/y accelerometer bias cannot be told apart
from a tilted mounting; a multi-pose calibration is planned.

**Inverse kinematics.** Hip abduction comes from the triangle formed by the hip offset
and the leg's length when viewed from the front. Hip pitch and knee bend then come from
the law of cosines on the upper/lower leg triangle. Unreachable positions raise an
error instead of producing a bad angle. Forward kinematics is used in tests to confirm
IK round-trips.

**Trot gait.** Each leg cycles through stance (foot slides backward on the ground) and
swing (foot lifts on a half-sine arc). Diagonal legs move together.

**Stability score.** Over a 2 s window: RMS pitch, RMS roll, peak tilt, gyro variance and
acceleration variance. Each is divided by a reference "just acceptable" value and
weighted (weights sum to 1), so a score of 1.0 means "at the acceptable limit". The
reference values are placeholders until real walking data is recorded.

**Attitude feedback.** Two PID controllers turn measured roll and pitch into body
corrections. The derivative term uses the gyro rate directly instead of differentiating
noisy angles. Corrections are applied by lengthening and shortening individual legs:
`hip height = body_height + y_hip·sin(roll_corr) − x_hip·sin(pitch_corr)`.

**Obstacle response.** CAUTION shortens the stride. NEAR shortens it further, raises the
steps and slows the cycle. STOP brings the robot to a stand. Hysteresis prevents state
flicker. The sensor reports only one distance, so no obstacle shape is inferred.

## Safety design

- Joint targets outside configured limits are **rejected, never silently clamped**.
  A multi-joint command is all-or-nothing.
- Move time is stretched so no joint exceeds the configured speed.
- Emergency stop (servo `MOVE_STOP`) is latched until explicitly reset. It is triggered
  by excessive tilt, loss of IMU data, or repeated rejected commands.
- If IMU data is invalid, attitude corrections are disabled and ramp to zero instead
  of being applied blindly.
- Every gait parameter change is rate-limited. Stride shrinks faster than it grows.
- The ultrasonic hardware backend refuses to start until `echo_voltage_safe = true`
  confirms a 5 V Echo signal is divided down to ≤ 3.3 V for the Pi.
- Before any physical servo testing: the robot lifted or the legs unloaded, small
  movements only, and the safe movement direction confirmed from the joint geometry.

## Running

Requires Python 3.11+. The core code uses only the standard library.

```bash
python -m venv .venv
.venv/Scripts/activate              # Windows  (Linux / Pi: source .venv/bin/activate)
pip install -r requirements.txt     # pytest
python -m pytest                    # 211 tests, no hardware needed

python tools/run_control_validation.py
python tools/run_control_validation.py --every 0.5 --csv data/runs/validation.csv
```

On the Raspberry Pi: `pip install -r requirements-pi.txt` (pyserial, lgpio, smbus2).

### What the validation runner shows, and what it does not

`run_control_validation.py` runs the real control code against **simulated sensor
input**: a seeded IMU with bias, noise and mounting error; a scripted roll and pitch
disturbance; an obstacle approaching from 150 cm to 10 cm; and a simulated LX-16A bus
that decodes the real protocol bytes. It shows that the software pipeline behaves as
designed: calibration removes the simulated bias, corrections oppose the disturbance,
obstacle states progress with hysteresis, gait changes are smooth, and all joint
commands pass the safety layer.

The body response comes from a **deliberately simple toy model**, not robot physics.
The run therefore says nothing about whether adaptive control improves stability on the
real robot. That question can only be answered by hardware experiments comparing fixed
and adaptive gait.

## Limitations

- No hardware validation yet; all dimensions, limits, gains and thresholds are placeholders.
- Pitch/roll are accelerometer-only and will be noisy during walking.
- The control loop is single-threaded; timing and jitter on the Pi 5 have not been
  measured, and no real-time guarantees are claimed.
- The simulation is a toy model used to exercise code paths, not to predict performance.

## Next steps

1. Hardware bring-up: confirm the IMU, ultrasonic sensor and servo board; read real
   sensors; move one joint a few degrees in a confirmed safe direction.
2. Sensor calibration with real data, plus ultrasonic accuracy at known distances.
3. Measure the real geometry and joint limits; validate IK on the robot.
4. Open-loop walking with logged stability metrics.
5. Complementary filter, then fixed-vs-adaptive gait experiments on hardware.

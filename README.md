# RoboDog

A student engineering project by Arnav Katewale: a low-cost quadruped robot dog,
controlled by a Raspberry Pi 5, that is designed to notice when its own walking becomes
unstable and adjust how it walks in response.

> **Engineering question:** Can a low-cost quadruped robot recognize when its gait is
> unstable and change its own movement in real time using onboard sensors and
> Raspberry Pi 5 computation?

**Where it stands today:** the control software is written and tested, and the full
sensing-to-servo control loop runs on **simulated** sensor input. It has **not yet run on
the physical robot**. Hardware integration is the next milestone.

| | |
|---|---|
| **Hardware** | Raspberry Pi 5 · IMU (tilt and rotation) · ultrasonic distance sensor · LX-16A serial bus servos (12 planned, 3 per leg) |
| **Software** | Python 3.11+, standard library only · 211 automated tests |
| **Working today** | IMU → stability measurement → adaptive gait → inverse kinematics → safe servo commands, running against simulated sensors |
| **Not yet shown** | Any result on physical hardware |

## Run it (no hardware needed)

```bash
python -m venv .venv
.venv/Scripts/activate              # Windows  (Linux / macOS / Pi: source .venv/bin/activate)
pip install -r requirements.txt     # only pytest; the robot code needs no extra packages

python -m pytest                    # 211 tests
python tools/run_control_validation.py
```

The validation runner first calibrates a simulated IMU and prints a PASS/FAIL report.
It then runs 14 seconds of simulated time: the body is tilted by scripted roll and pitch
disturbances while a simulated obstacle approaches from 150 cm to 10 cm. Every second it
prints the measured tilt, the obstacle state, the stability score, the current gait
parameters, the body corrections and the joint angles sent to each leg. It ends with a
summary (commands accepted or rejected, emergency stops, obstacle state changes). Add
`--csv data/runs/validation.csv` to save every control step.

## Project status

### Implemented and validated in software

"Validated in software" means unit tests with known inputs, plus an end-to-end run of the
real control code against simulated sensors and a simulated servo bus.

- **IMU interface:** timestamped acceleration and rotation readings, rejection of bad
  readings, tilt (pitch/roll) from gravity
- **IMU calibration:** gyro bias, mounting tilt and noise from stationary data, with a
  PASS/FAIL validation report
- **Ultrasonic interface:** time-of-flight distance, timeouts, range checks
- **LX-16A servo protocol:** move, stop, torque, and read position/voltage/temperature,
  checked byte-for-byte against the protocol
- **Joint safety:** limits, all-or-nothing commands, speed limit, emergency stop
- **Leg kinematics:** inverse and forward kinematics for a 3-joint leg
- **Trot gait generator:** foot paths as a function of gait phase
- **Stability metrics:** RMS tilt, peak tilt, rocking and vibration, combined score
- **Obstacle states:** CLEAR / CAUTION / NEAR / STOP with hysteresis
- **Adaptive gait prototype:** PID tilt correction plus stride, step height and speed
  changes, all rate-limited
- **Control loop:** wires everything together, with emergency-stop triggers

### Implemented, waiting for hardware validation

- Raspberry Pi 5 ultrasonic backend (`lgpio`), not yet run on a Pi
- LX-16A communication over a real serial port
- Calibration with real IMU data
- Every value in `config/robot.toml` marked `PLACEHOLDER`: dimensions, joint limits,
  servo IDs, PID gains and thresholds are starting values, not measurements

### Not built yet

- IMU chip driver (IMU model still being finalized)
- Gyro + accelerometer filtering (complementary filter) for tilt while walking
- Timing measurements of the control loop on the Pi 5
- Logged experiments and plots
- Fixed-gait vs adaptive-gait comparison on the real robot
- Automated gait parameter search

**No hardware test results exist yet**, and none are claimed here.

## What's next

1. **Hardware bring-up:** confirm the exact IMU, ultrasonic sensor and servo board; read
   the real sensors; move one joint a few degrees in a direction confirmed safe from the
   joint geometry, with the robot lifted.
2. **Sensor calibration:** calibrate the real IMU; measure ultrasonic accuracy at known
   distances.
3. **Geometry:** measure the real leg dimensions and joint limits, and check the
   kinematics on the robot.
4. **Baseline walking:** open-loop trot with logged stability metrics.
5. **The core experiment:** add gyro filtering, then compare fixed and adaptive gait on
   the same tests and see whether adaptive control measurably reduces tilt.

## How it works

```
 IMU ─► calibration ─► pitch / roll ─► stability metrics ─┐
                                                          ├─► adaptive gait
 ultrasonic ─► distance ─► obstacle state ────────────────┘   (PID + rate limits)
                                                                     │ gait parameters,
                                                                     ▼ body corrections
                                                             trot gait generator
                                                                     │ foot positions
                                                                     ▼
                                                             inverse kinematics
                                                                     │ joint angles
                                                                     ▼
                                                             joint safety layer ─► LX-16A servos
                                                                                        │
              sensors measure how the body actually moved  ◄────────────────────────────┘
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

Hardware access sits behind small backend interfaces (`read_raw()`, `measure_echo_s()`,
a serial port object). The same control code runs against simulated backends now and
will run against real ones later.

**Conventions.** Body frame: +x forward, +y left, +z up. Roll is rotation about +x
(positive = left side up). Pitch is rotation about +y (positive = nose down, by the
right-hand rule).

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
the law of cosines on the upper/lower leg triangle. Unreachable positions raise an error
instead of producing a bad angle. Forward kinematics is used in tests to confirm IK
round-trips.

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
- If IMU data is invalid, attitude corrections are disabled and ramp to zero instead of
  being applied blindly.
- Every gait parameter change is rate-limited. Stride shrinks faster than it grows.
- The ultrasonic hardware backend refuses to start until `echo_voltage_safe = true`
  confirms a 5 V Echo signal is divided down to ≤ 3.3 V for the Pi.
- Before any physical servo testing: the robot lifted or the legs unloaded, small
  movements only, and the safe movement direction confirmed from the joint geometry.

## What the simulation shows, and what it does not

The validation runner uses **simulated sensor input**: a seeded IMU with bias, noise and
mounting error; scripted roll and pitch disturbances; a scripted obstacle approach; and a
simulated LX-16A bus that decodes the real protocol bytes. It shows that the software
behaves as designed: calibration removes the simulated bias, corrections oppose the
disturbance, obstacle states progress with hysteresis, gait changes are smooth, and
joint commands pass through the safety layer.

The body's response comes from a **deliberately simple toy model**, not robot physics.
The run therefore says nothing about whether adaptive control improves stability on the
real robot. Only hardware experiments comparing fixed and adaptive gait can answer that.

## Hardware notes

| Component | Status |
|---|---|
| Raspberry Pi 5 | Main computer |
| LX-16A serial bus servos | Servo model the design is based on (reference CAD included); servo count and controller board still to be finalized |
| IMU | Model still to be finalized; chip-specific driver not yet written |
| Ultrasonic sensor | HC-SR04-style trigger/echo assumed; model and supply voltage still to be finalized |
| Servo power and grounding | Still to be finalized |

On the Raspberry Pi, install `pip install -r requirements-pi.txt` (pyserial, lgpio,
smbus2).

Mechanical reference files:
- `Lewansoul lx-16a BUS Servo CAD file/`: LX-16A servo models, tracked in this repository.
- Raspberry Pi 5 CAD reference files are used locally for mechanical layout but are
  **not included in the repository** (third-party material, ~130 MB).

## Limitations

- No hardware validation yet; all dimensions, limits, gains and thresholds are placeholders.
- Pitch/roll are accelerometer-only and will be noisy during walking.
- The control loop is single-threaded. Its timing and jitter on the Pi 5 have not been
  measured, and no real-time guarantees are claimed.
- The simulation is a toy model used to exercise code paths, not to predict performance.

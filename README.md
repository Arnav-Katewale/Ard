# RoboDog: adaptive-control software

A student engineering project by Arnav Katewale: a low-cost quadruped robot dog,
controlled by a Raspberry Pi 5, that is designed to notice when its own walking becomes
unstable and adjust how it walks in response.

> **Engineering question:** Can a low-cost quadruped robot recognize when its gait is
> unstable and change its own movement in real time using onboard sensors and
> Raspberry Pi 5 computation?

You are looking at the **`adaptive-control-software` branch**. The project so far:

1. **Existing RoboDog hardware work** ([`main` branch](https://github.com/Arnav-Katewale/robot-dog/tree/main)).
   The Raspberry Pi 5 and the servo system were brought up and physically tested:
   servos driven from the Pi through a USB HID servo controller, measured movement
   tests, servo ID work, wiring documentation and hardware troubleshooting, all recorded
   in the [DEVLOG](https://github.com/Arnav-Katewale/robot-dog/blob/main/DEVLOG.md).
2. **This branch: the next step.** Control software that uses the IMU and the
   ultrasonic sensor to adjust the gait while walking: calibration, stability
   measurement, PID tilt correction, obstacle response and adaptive gait. It is
   written, tested and validated in simulation, but it **has not yet been integrated
   with the physical robot**.
3. **Next integration step.** Adapt this software's leg model to the robot's real
   2-servo-per-leg geometry, and connect it to the existing USB HID servo interface from
   `main`.

| | |
|---|---|
| **Robot hardware** | Raspberry Pi 5 · 9 LX-16A bus servos (hip + knee on each of 4 legs, plus a head servo) · Hiwonder USB HID servo controller · MPU6050 (GY-521) IMU · HC-SR04 ultrasonic sensor |
| **Physically tested so far** (on `main`) | Pi-to-servo control over USB HID, measured servo movement tests, servo ID setup |
| **This branch** | Python 3.11+, standard library only · 211 automated tests · full control loop validated on simulated sensor input |
| **Not yet done** | Running this branch's adaptive control on the physical robot |

## Run it (no hardware needed)

Create and activate a virtual environment (Python 3.11 or newer):

```bash
# Windows (PowerShell)
python -m venv .venv
.venv\Scripts\Activate.ps1

# macOS / Linux / Raspberry Pi
python3 -m venv .venv
source .venv/bin/activate
```

Then, in the activated environment:

```bash
pip install -r requirements.txt     # only pytest; the robot code needs no extra packages
python -m pytest                    # 211 tests
python tools/run_control_validation.py
```

The validation runner first calibrates a simulated IMU and prints a PASS/FAIL report.
It then runs 14 seconds of simulated time: the body is tilted by scripted roll and pitch
disturbances while a simulated obstacle approaches from 150 cm to 10 cm. Every second it
prints the tilt reported by the simulated IMU, the obstacle state, the stability score,
the current gait parameters, the body corrections and the joint angles sent to each leg.
It ends with a summary (commands accepted or rejected, emergency stops, obstacle state
changes). Add `--csv data/runs/validation.csv` to save every control step.

## Software model vs. the physical robot

This branch was developed as the next control-system iteration. Some of its software
model, especially the 3-joint leg kinematics and the 12-joint configuration, was
developed before being reconciled with the robot's current 2-servo-per-leg geometry.
Bringing the two together is the next integration step:

| Part of this branch | Software model today | Physical robot | Integration work |
|---|---|---|---|
| Leg kinematics (`robodog/kinematics.py`) | 3 joints per leg (hip abduction, hip pitch, knee) | 2 servos per leg (hip, knee) | Derive 2-joint IK for the real leg; the hip pitch / knee part of the current law-of-cosines solution carries over |
| Joint configuration (`config/robot.toml`) | 12 leg joints | 8 leg servos + 1 head servo | Use the real servo IDs (FL 1/2, FR 3/4, RL 5/6, RR 7/8, head 9) and measured limits |
| Roll correction | Per-leg height changes | Still possible with hip + knee | Re-check the correction range once the 2-joint IK exists |
| Servo communication (`robodog/servos/lx16a.py`) | LX-16A servo-level serial packets (the protocol a BusLinker passes straight through) | The robot uses the Hiwonder **USB HID** controller, which has its own command framing | Add a backend that drives the existing HID `Controller` from `main`, behind the same joint-safety layer |
| Emergency stop | Servo `MOVE_STOP` command | HID controller offers position and unload commands | Map the stop behaviour onto the HID controller's commands |

The control layers above the leg model (calibration, stability metrics, PID, obstacle
states, adaptive gait, safety checks) do not depend on the number of joints and are
expected to carry over. The 3-joint code stays in place as a tested reference until the
2-joint version replaces it.

## Project status

### Physical work already completed (on `main`)

- Servo bring-up from the Raspberry Pi through the USB HID servo controller
- Measured servo movement tests with position read-back (up to ~120° of travel, within
  about 0.5°), recorded in the DEVLOG
- Servo ID work, including resolving a duplicate-ID clash with the BusLinker
- Raspberry Pi 5 setup and troubleshooting
- Wiring documentation: power plan, Pi pin map, HC-SR04 Echo voltage divider

At the time of the latest DEVLOG entry, five of the nine servos had been bench-tested,
and the MPU6050 and HC-SR04 were not yet wired.

### Implemented and validated in software (this branch)

"Validated in software" means only this: exercised by automated tests with known inputs,
and by an end-to-end run of the control code against simulated sensors and a simulated
servo bus. None of this branch's code has been run on the physical robot.

- **IMU interface:** timestamped acceleration and rotation readings, rejection of bad
  readings, accelerometer-based pitch/roll
- **Ultrasonic interface:** time-of-flight distance, timeouts, range checks
- **LX-16A servo protocol:** packet encoding and parsing for move, stop, torque and
  position/voltage/temperature reads, checked byte-for-byte against the protocol
- **Joint safety:** limits, all-or-nothing commands, speed limit, emergency-stop logic
- **IMU calibration algorithms:** gyro bias, mounting tilt and noise from stationary
  data, with a PASS/FAIL validation report (validated on simulated data only)
- **Leg kinematics:** inverse and forward kinematics for a 3-joint leg (see the table
  above)
- **Trot gait generator:** foot paths as a function of gait phase
- **Stability metrics:** RMS tilt, peak tilt, rocking and vibration, combined score
- **Obstacle-state logic:** CLEAR / CAUTION / NEAR / STOP with hysteresis
- **PID controller:** output limits and anti-windup
- **Adaptive gait controller:** PID tilt correction plus stride, step height and speed
  changes, all rate-limited
- **Closed-loop control flow:** wires everything together, with emergency-stop triggers
- **Simulation-based validation:** `tools/run_control_validation.py`
- **Automated tests:** 211 tests, no hardware needed

### Hardware-facing code in this branch; physical validation pending

- **Raspberry Pi 5 HC-SR04 backend** (`lgpio`; Trig on GPIO23, Echo on GPIO24 through
  the divider, matching the wiring map on `main`); physical wiring and sensor
  validation pending
- **LX-16A serial backend** (pyserial); suited to a serial link such as the BusLinker,
  not to the robot's HID controller; not yet run against servos
- **IMU calibration routine:** works with any IMU backend; physical calibration needs
  the MPU6050 driver listed below
- **Configuration values:** every value in `config/robot.toml` marked `PLACEHOLDER`
  (dimensions, joint limits, servo IDs, PID gains, thresholds) is a starting value,
  not a measurement

### Not built or measured yet

- MPU6050 driver for this branch's IMU interface
- 2-joint leg kinematics for the physical leg geometry
- Servo backend connecting this branch to the existing USB HID controller code
- Ultrasonic calibration against known distances
- Gyro-based orientation filtering / sensor fusion (e.g. complementary filter)
- Raspberry Pi 5 control-loop timing and latency measurements
- Physical walking gait tuning
- Logged walking experiments and plots
- Fixed-gait vs adaptive-gait comparison on the physical robot
- Automated gait parameter optimization

The servo tests on `main` are the project's physical measurements so far. This branch
adds no hardware results and claims none.

## What's next

1. **Integrate with the real robot:** 2-joint leg kinematics, the real servo IDs, an
   HID servo backend behind the joint-safety layer, and an MPU6050 driver.
2. **Sensor bring-up and calibration:** wire the MPU6050 and HC-SR04, calibrate the IMU
   on the robot, and measure ultrasonic accuracy at known distances.
3. **Geometry:** measure the real leg dimensions and joint limits, and check the
   kinematics on the robot with it lifted.
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
| `robodog/servos/lx16a.py` | LX-16A servo-level serial protocol: move, stop, torque, read position/voltage/temperature |
| `robodog/servos/joint.py` | Joint limits, all-or-nothing commands, speed limit, latched emergency stop |
| `robodog/calibration/calibration.py` | Stationary IMU calibration, JSON save/load, PASS/FAIL validation report |
| `robodog/kinematics.py` | 3-joint leg forward and inverse kinematics (to be adapted to the 2-joint leg) |
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
a serial port object). The same control code runs against simulated backends now; the
HID servo backend and MPU6050 driver will plug in at the same points.

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

**Inverse kinematics (current 3-joint model).** Hip abduction comes from the triangle
formed by the hip offset and the leg's length when viewed from the front. Hip pitch and
knee bend then come from the law of cosines on the upper/lower leg triangle. That second
step is exactly the 2-joint problem the physical leg needs. Unreachable positions raise
an error instead of producing a bad angle. Forward kinematics is used in tests to
confirm IK round-trips.

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
- Emergency stop is latched until explicitly reset. It is triggered by excessive tilt,
  loss of IMU data, or repeated rejected commands.
- If IMU data is invalid, attitude corrections are disabled and ramp to zero instead of
  being applied blindly.
- Every gait parameter change is rate-limited. Stride shrinks faster than it grows.
- The ultrasonic hardware backend refuses to start until `echo_voltage_safe = true`
  confirms the HC-SR04's 5 V Echo signal reaches the Pi through the divider at ≤ 3.3 V.
- Before this branch drives physical servos: the robot lifted or the legs unloaded,
  small movements only, and the safe movement direction confirmed from the joint
  geometry.

## What the simulation shows, and what it does not

The validation runner uses **simulated sensor input**: a seeded IMU with bias, noise and
mounting error; scripted roll and pitch disturbances; a scripted obstacle approach; and a
simulated LX-16A bus that decodes the same servo-level protocol bytes. It shows that the
software behaves as designed: calibration removes the simulated bias, corrections oppose
the disturbance, obstacle states progress with hysteresis, gait changes are smooth, and
joint commands pass through the safety layer.

The body's response comes from a **deliberately simple toy model**, not robot physics,
and it uses the 3-joint software model. The run therefore says nothing about whether
adaptive control improves stability on the real robot. Only hardware experiments
comparing fixed and adaptive gait can answer that.

## Hardware notes

| Component | Details |
|---|---|
| Raspberry Pi 5 | Main computer |
| LX-16A bus servos ×9 | Hip + knee on each leg (IDs FL 1/2, FR 3/4, RL 5/6, RR 7/8), head 9 |
| Hiwonder bus servo controller | USB HID device used on the robot; driver and tests on `main` |
| Hiwonder BusLinker | USB-serial adapter, used for servo ID setup |
| MPU6050 (GY-521) | I²C on GPIO2/3, 3.3 V; driver for this branch not yet written |
| HC-SR04 | Trig on GPIO23; Echo on GPIO24 through a 1 kΩ / 2 kΩ divider (5 V → 3.3 V) |
| Power | 2S LiPo for the servos; a 5 V UBEC for the Pi (see the power plan on `main`) |

Full wiring, Pi pin map and parts list: [`docs/wiring-map.html` on `main`](https://github.com/Arnav-Katewale/robot-dog/blob/main/docs/wiring-map.html).

On the Raspberry Pi, `pip install -r requirements-pi.txt` installs this branch's
hardware libraries (pyserial, lgpio, smbus2).

Mechanical reference files:
- `Lewansoul lx-16a BUS Servo CAD file/`: LX-16A servo models, tracked in this repository.
- Raspberry Pi 5 CAD reference files are used locally for mechanical layout but are
  **not included in the repository** (third-party material, ~130 MB).

## Limitations

- This branch's control software has not been run on the physical robot. Its dimensions,
  limits, gains and thresholds are placeholders.
- The leg model is 3-joint while the robot's legs are 2-joint (see "Software model vs.
  the physical robot").
- Pitch/roll are accelerometer-only and will be noisy during walking.
- The control loop is single-threaded. Its timing and jitter on the Pi 5 have not been
  measured, and no real-time guarantees are claimed.
- The simulation is a toy model used to exercise code paths, not to predict performance.

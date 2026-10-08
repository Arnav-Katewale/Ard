# Architecture

## Who does what

```
                 ┌──────────────────────────── Raspberry Pi 5 ────────────────────────────┐
                 │                                                                        │
 HC-SR04  ─GPIO─►│  sensors ──► behaviour ──► gait engine ──► kinematics ──► servo driver ├─USB─► servo controller ─bus─► 9× LX-16A
 MPU6050  ─I²C──►│     │          (what to do)   (where feet go)  (angles)     (robodog/    │                                  │
                 │     └──────► balance correction ─────────┘                   servo.py)  │◄─USB── positions, battery voltage ┘
                 │                                                                        │
                 └──────────── Wi-Fi: SSH for development, later a remote control ─────────┘
```

### Raspberry Pi 5: the brain

The Pi runs one Python program that decides everything. It doesn't drive the motors electrically; it decides **where every joint should be**, many times a second.

1. **Read the sensors**
   - **MPU6050 (I²C):** body tilt (pitch and roll) and how fast it is rotating. Used to tell whether the robot is level, leaning or falling.
   - **HC-SR04 (GPIO):** the Pi sends a trigger pulse and times the echo, which gives distance to the obstacle ahead.
   - **Servo controller (USB):** current joint positions and battery voltage.
2. **Decide what to do (behaviour).** A state machine: stand → walk forward → obstacle ahead → stop → scan with the head → turn toward the clearest direction → walk. Plus sit, lie down and "battery low: sit down and stop".
3. **Plan the steps (gait engine).** For walking, decide where each foot should be at each moment: which legs are lifted, how far each foot swings, and the timing between legs (for example a trot, where diagonal legs move together).
4. **Turn foot positions into joint angles (kinematics).** Given the leg segment lengths from the CAD, work out the hip and knee angles that put a foot at a given point, then convert the angles to servo units (0–1000 ≈ 0–240°).
5. **Correct for balance (IMU feedback).** If the body tilts, adjust the leg heights to level it. This is the "closed-loop stabilisation" goal.
6. **Send the targets.** One USB command moves all 9 servos together (`Controller.move({...}, time_ms)`).
7. **Stay safe.** Clamp every joint to its calibrated safe range, watch the battery, and make all servos go limp on errors or Ctrl-C.

This loop will run at roughly 20–50 updates per second. The real rate depends on how fast the controller answers over USB, which will be measured once all 9 servos are connected.

### Servo controller: the translator

It receives "servo N go to position P over T ms" from the Pi and passes it to the servos over the single bus wire. It also reads positions and battery voltage back. It doesn't make decisions.

### LX-16A servos: the muscles

Each servo has its own position sensor and control loop. Once told a target, it drives itself there and holds it. Each one knows its ID (1–9) and only responds to commands for that ID.

### Battery and UBEC: power

The 2S LiPo powers the controller and servos directly (7.4 V). The UBEC steps it down to 5 V for the Pi. See [`wiring-map.html`](wiring-map.html).

## Code layout (current and planned)

| Module | Status | Job |
|---|---|---|
| `robodog/servo.py` | ✅ done | USB protocol: move, read positions, battery, go limp |
| `robodog/calibration` | planned | Per-servo centre offsets and safe limits |
| `robodog/kinematics` | planned | Foot position ↔ hip/knee angles |
| `robodog/gait` | planned | Step timing and foot trajectories |
| `robodog/imu` | planned | MPU6050 reading and tilt estimate |
| `robodog/sonar` | planned | HC-SR04 distance |
| `robodog/behaviour` | planned | State machine: stand, walk, avoid, sit |
| `tools/servo_gui.py` | ✅ done | Desktop test panel |

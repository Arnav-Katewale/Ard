# robot-dog

A small quadruped robot dog: nine LewanSoul LX-16A bus servos (hip + knee on each leg, plus a head pan), a Raspberry Pi 5, a 3D-printed PETG body, an HC-SR04 ultrasonic sensor and an MPU6050 IMU.

Goals: stable walking, turning, obstacle avoidance, and later closed-loop balance from the IMU.

- **[DEVLOG.md](DEVLOG.md):** every step so far, with problems and fixes
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md):** what each part does, including the Pi's job
- **[docs/wiring-map.html](docs/wiring-map.html):** wiring diagram, Pi pin map, parts list

## Status

- [x] All four servos on hand bench-tested (IDs 1–4)
- [x] Raspberry Pi 5 set up (Raspberry Pi OS Lite 64-bit, SSH over Wi-Fi)
- [x] Pi controls the servos over USB
- [ ] Leg CAD design
- [ ] Remaining five servos (ID 5 set; 6–9 to go)
- [ ] Sensors wired (HC-SR04, MPU6050)
- [ ] Walking gait

## Hardware

| Part | Notes |
|---|---|
| LX-16A serial bus servos ×9 | 6–8.4 V, positions 0–1000 ≈ 0–240° |
| Hiwonder bus servo controller | USB HID device `0483:5750`, no COM port |
| Raspberry Pi 5 | Talks to the controller over USB |
| 2S LiPo (7.4 V) | Powers the servos directly; a 5 V 5 A UBEC powers the Pi |
| HC-SR04 | Echo goes through a 1 kΩ / 2 kΩ divider to GPIO24 |
| MPU6050 (GY-521) | I²C on GPIO2/3, 3.3 V |

Full wiring, Pi pin map and servo ID plan: open [`docs/wiring-map.html`](docs/wiring-map.html) in a browser.

### Servo IDs

| Leg | Hip | Knee |
|---|---|---|
| Front left | 1 | 2 |
| Front right | 3 | 4 |
| Rear left | 5 | 6 |
| Rear right | 7 | 8 |
| Head | 9 | |

## Repo layout

```
robodog/servo.py         Controller class: move, read position, battery, go limp
tools/servo_gui.py       Desktop test panel (Windows/macOS/Linux with a display)
tools/servo_check.py     Health check: scan IDs, move each servo ~48 degrees and back, verify
tools/buslinker.py       Set servo IDs / read voltage & temperature through a BusLinker (one servo at a time)
pi-setup/setup_usb.sh    One-time: lets non-root users open the servo controller on the Pi
pi-setup/reset_pi_password.py  Reset the Pi password + add an SSH key via the SD card's cloud-init files
docs/wiring-map.html     Wiring diagram, pin map, parts list
docs/ARCHITECTURE.md     System design and planned modules
DEVLOG.md                Dated build log
```

## Using it

### On a PC (servo controller plugged into the PC)

```
pip install -r requirements.txt
python tools/servo_gui.py
```

### On the Raspberry Pi

```
git clone https://github.com/Arnav-Katewale/robot-dog.git
cd robot-dog
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
sudo bash pi-setup/setup_usb.sh        # once; then replug the controller
.venv/bin/python -m robodog.servo      # read-only check: battery + servo positions
.venv/bin/python tools/servo_check.py  # move each servo ~12 degrees and back, verify
```

## Protocol notes

The controller takes packets `55 55 <len> <cmd> <params…>` inside a 64-byte HID report. Commands used here:

| Cmd | Name | Params |
|---|---|---|
| `0x03` | Move servos | count, time (ms, LE16), then per servo: id, position (LE16) |
| `0x0F` | Battery voltage | none; reply is millivolts (LE16) |
| `0x14` | Unload (go limp) | count, ids… |
| `0x15` | Read positions | count, ids…; reply is id + position (signed LE16) per servo |

Motor (continuous-rotation) mode and servo ID changes aren't in the controller's published command set. Avoid guessing command numbers: `0x06` runs a stored action group.

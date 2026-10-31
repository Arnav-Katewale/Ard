# RoboDog

A quadruped robot dog built around a Raspberry Pi 5, an IMU, an ultrasonic distance
sensor and LX-16A serial bus servos.

**Project question:** Can a low-cost quadruped robot recognize when its gait is unstable
and change its own movement in real time using onboard sensors and Raspberry Pi 5
computation?

## Setup

Requires Python 3.11 or newer.

```bash
python -m venv .venv
.venv/Scripts/activate        # Windows  (Linux / Pi: source .venv/bin/activate)
pip install -r requirements.txt
python -m pytest
```

On the Raspberry Pi, additionally: `pip install -r requirements-pi.txt`.

## Mechanical reference files

- `Lewansoul lx-16a BUS Servo CAD file/` — LX-16A servo models, tracked in this repository.
- Raspberry Pi 5 CAD reference files are used locally for mechanical layout but are
  **not included in the repository** (third-party material, ~130 MB).

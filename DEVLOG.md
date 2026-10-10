# Dev log

Newest entries at the bottom. Each entry: what was done, why, what went wrong, how it was fixed.

---

## 2026-10-07 — Servo controller bench test

**Done:** Connected the Hiwonder (LewanSoul) bus servo controller to the PC and confirmed it responds.

- The controller is a **USB HID device** (VID `0483`, PID `5750`, MM32F103RB chip), not a serial/COM port. No driver needed; talking to it from Python needs the `hidapi` package.
- Packets: `55 55 <len> <cmd> <params…>` inside a 64-byte HID report.
- On USB power alone the board reported 4.69 V and no servos answered. With the 2S LiPo connected: 8.59 V, servos answered.

**Problem:** only servos 1 and 2 answered, though four were connected. Two servos also had blinking LEDs.
**Fix:** the servos had duplicate / unset IDs. IDs were set to 1–4 with Hiwonder's software. Afterwards all four answered on every read and the blinking stopped.

**Tool built:** `tools/servo_gui.py`, a desktop panel with a slider per servo, move-time control, "go limp" and a live battery readout. Sliders start at each servo's current position so nothing jumps on startup.

**Result:** all four servos moved correctly from the panel.

---

## 2026-10-07 — Motor mode investigation

The controller's published command set (move, action groups, battery, unload, read positions) has **no motor (continuous rotation) mode** and no ID-setting command. Hiwonder's own PC software does both through this board using unpublished commands.

**Decision:** don't guess command numbers: `0x06` runs stored action groups, so a wrong guess could move servos unexpectedly. Motor mode turned out not to be needed (see next entry).

---

## 2026-10-07 — Design change: 2 servos per leg

**Original plan:** 5 servos (1 per leg + head).
**Problem:** a 1-DOF leg can only swing forward and back. It can't lift, so the robot would shuffle rather than walk, turning would rely on scraping the feet, and the IMU would have almost nothing to correct with. That conflicts with the goals of stable walking and gait stabilisation.

**Options considered:**
- 2 servos per leg (hip + knee), 9 servos total ← **chosen**
- Keep 1 per leg with continuously rotating "wheel-legs" (RHex style). Cheaper, but needs motor mode, and the LX-16A can only sense position over ~240° of the rotation, so phasing the legs is hard.

**Servo ID plan:** FL 1/2, FR 3/4, RL 5/6, RR 7/8 (hip/knee), head 9.

---

## 2026-10-07 — Parts selection

Bought: 5× LX-16A, 5 V 5 A UBEC (the 5 A option, since the Pi 5 needs it), HC-SR04 2-pack, MPU6050 GY-521 3-pack, resistor kit, M3 heat-set insert kit with screws, M2.5 × 10 mm screws and M2.5 nuts.

Already owned: Pi 5 + microSD, Pi Active Cooler, 2S LiPo (HRB 2200 mAh 50C) + charger, Dupont wires, soldering iron, PETG.

Deliberately **not** bought:
- Logic level shifter: a resistor divider (three 1 kΩ) drops the HC-SR04's 5 V Echo signal to 3.33 V.
- Standoff kit: printed posts plus M2.5 screws do the job.
- BusLinker board: only needed for motor mode.
- Battery splitter: the controller has a 2-screw power terminal that the UBEC input can share.

Full parts list and wiring: [`docs/wiring-map.html`](docs/wiring-map.html).

---

## 2026-10-07 — Raspberry Pi 5 setup

**Done:** Flashed Raspberry Pi OS Lite (64-bit, Debian 13 "trixie") with Raspberry Pi Imager. Hostname, user, Wi-Fi and SSH were configured in Imager. The Pi runs headless and is controlled from the PC over SSH.

**Problem 1: "Access denied" writing the SD card.** Windows ransomware protection was off and the disk wasn't read-only. The card's old exFAT partition was most likely held open by Windows.
**Fix:** wiped the card with `diskpart clean`, using a guarded script that only touched a single USB disk of 100–130 GB. Imager then wrote successfully.

**Problem 2: Pi never joined Wi-Fi.** The network name contains a curly apostrophe (`’`, U+2019). Imager 2.0 wrote it into cloud-init's `network-config` as `"…\xe2\x80\x99…"`. In YAML double-quoted strings `\xNN` is a *character*, not a byte, so the Pi looked for a network called `…â\x80\x99…`, which doesn't exist.
**Fix:** edited `network-config` on the SD card's boot partition to use `’`, and changed the cloud-init instance ID (in `cmdline.txt` and `meta-data`) so setup re-ran on the next boot. The Pi joined Wi-Fi.

**Lesson:** if a Wi-Fi name has non-ASCII characters, check `network-config` on the boot partition before anything else.

---

## 2026-10-08 — Remote access (SSH key)

**Problem:** the Pi password set in Imager was forgotten.
**Fix:** `pi-setup/reset_pi_password.py` edits the SD card's cloud-init `user-data` to add the PC's SSH public key and set a new password. The password is hashed locally (`openssl passwd -6`) and only the hash is written to the card. The instance ID is bumped so it applies on the next boot.

**Result:** passwordless SSH login from the PC with an ed25519 key. `sudo` still asks for the password.

---

## 2026-10-08 — Pi ↔ servo controller over USB

**Done:**
- Python venv on the Pi with `hidapi`.
- `pi-setup/setup_usb.sh` installs a udev rule so non-root users can open the controller (USB `0483:5750`, both `usb` and `hidraw` subsystems).
- `robodog/servo.py`: a `Controller` class shared by the Pi code and the desktop panel.

**Problem: the Pi didn't detect the controller at all.** There was no USB plug-in event in the kernel log.
**Cause:** the first USB cable was **charge-only** (no data wires).
**Fix:** used a data cable. The Pi detected the controller immediately.

**Side issue:** the Pi shut down once (solid red LED) while cables were being changed. Hardware flags showed no USB over-current and no under-voltage, so the most likely cause is the Pi 5's power button being pressed by accident.

**Result:** read-only test from the Pi gave battery 8.60 V and servo positions 443 / 461 / 206 / 327. A movement test from the Pi moved each servo +50 units over 1 s and back:

| Servo | Start | Target | Reached | Returned |
|---|---|---|---|---|
| 1 | 443 | 493 | 491 | 443 |
| 2 | 461 | 511 | 509 | 462 |
| 3 | 207 | 257 | 255 | 208 |
| 4 | 327 | 377 | 374 | 328 |

Errors of 1–3 units (~0.5°) are normal servo precision.

**Decision:** keep USB (not UART via Dupont wires) between the Pi and the controller on the finished robot. It's tested and needs one cable. Use a short 10–20 cm data cable inside the body.

---

## 2026-10-08 — Power plan

Battery: HRB 2S LiPo, 7.4 V nominal (8.4 V full), 2200 mAh, 50C.

```
Battery ──► controller screw terminal ──┬──► controller ──► servos (bus)
                                        └──► 5 V UBEC ──► Pi pins 2 (+5 V) & 6 (GND)
Pi USB ──► controller (data only)
```

- The controller's power input is a blue 2-screw terminal, so the UBEC input wires share it. No splitter is needed.
- The controller's slide switch only cuts the controller, not the Pi. Unplug the battery to power everything off. A main switch is optional later.
- Don't discharge the LiPo below ~6.4 V (3.2 V per cell).
- Never connect the Pi's USB-C supply and the UBEC at the same time.

**Status:** waiting for the UBEC to arrive.

---

## 2026-10-08 — GitHub repo

Created this repo. The servo code was moved into `robodog/servo.py`; the desktop panel imports it instead of keeping its own copy. The Pi now runs from a `git clone` of this repo (`~/robot-dog`).

---

## 2026-10-10 — Servo health check

**Done:** Re-tested all servos from the Pi and added `tools/servo_check.py`, a reusable health check. It scans IDs, then moves each servo 50 units (~12°) over 1 s and back, and verifies the position read-back.

| Servo | Start | Target | Reached | Returned | Result |
|---|---|---|---|---|---|
| 1 | 443 | 493 | 491 | 443 | OK |
| 2 | 462 | 512 | 510 | 462 | OK |
| 3 | 207 | 257 | 255 | 208 | OK |
| 4 | 328 | 378 | 375 | 329 | OK |

Battery 8.62–8.64 V. IDs 5–12: no response (the new servos aren't connected yet).

**Note:** right after boot, `arnavk-RPI5.local` may not resolve for the first minute or so even though the Pi is up and SSH works by IP. Wait a moment, or connect by IP.

---

## 2026-10-10 — First new servo: duplicate ID, set to 5 with the BusLinker

**Problem:** with the first new servo daisy-chained after servo 2 (controller → servo 2 → new servo), servo 2 stopped answering and servo 3 briefly read servo 2's position (463 instead of 208). IDs 1, 3 and 4 otherwise stayed steady.
**Cause:** the new servo shipped as **ID 2**, the same as old servo 2. Both replied to every ID-2 query and the replies collided on the bus.

**Fix:** set the new servo's ID to **5** with a Hiwonder BusLinker (CH340 USB-serial, COM4, 115200 baud). Unlike the USB HID controller, the BusLinker passes the servos' own protocol straight through, including ID read/write (cmd 14/13), voltage (27), temperature (26) and position (28). New tool: `tools/buslinker.py` (`info`, `set-id N`). It refuses to continue unless exactly one servo answers consistently.

```
ID 2: position -1, 8.428 V, 35 C
ID 2 -> 5: done
ID 5: position -1, 8.428 V, 35 C
```

**Also:** `tools/servo_check.py` now moves each servo ~48° (200 units) by default so it is easy to see, and reads each ID three times, flagging readings that jump or drop out (duplicate ID or loose cable) instead of moving them.

**Gotcha:** the HID controller reports "no answer" as position −1, but a servo can genuinely sit at −1 (just past its 0 end). Servo 5 is there now, so it may look missing on the controller until it's moved.

**Procedure for the remaining new servos:** connect one at a time, alone, to the BusLinker → `python tools/buslinker.py set-id N` (6, 7, 8, 9) → label it.

---

## 2026-10-10 — Five servos on the bus

Reconnected all five servos to the USB controller (servo 5 chained after servo 2) with the Pi in charge. Read-only check: IDs 1–4 steady at 444 / 462 / 208 / 329, servo 2 answering again, so the clash is gone. Servo 5 read `[None, -2, None]`: the −1 ambiguity noted above, because it was parked just past its 0 end. Moving it to 500 over 2 s fixed it; it now reads 499 consistently. Battery 8.63 V.

---

## 2026-10-10 — Five-servo movement test (~48°)

`tools/servo_check.py --ids 1-9`: each servo turned 200 units (~48°) over 1.5 s and back, one at a time.

| Servo | Start | Target | Reached | Returned | Result |
|---|---|---|---|---|---|
| 1 | 443 | 643 | 642 | 443 | OK |
| 2 | 462 | 662 | 662 | 462 | OK |
| 3 | 208 | 408 | 407 | 209 | OK |
| 4 | 329 | 529 | 527 | 329 | OK |
| 5 | 499 | 699 | 697 | 499 | OK |

All within 3 units (~0.7°). Battery 8.61–8.62 V.

---

## Next

1. UBEC install: measure 5 V output, wire into the terminal, set the Pi's power-supply setting, first run on battery only.
2. Leg CAD, using a servo for measurements.

"""Servo health check: find which servo IDs answer, then move each one a little and back.

Run on the Pi from the repo root:  .venv/bin/python tools/servo_check.py
Add --read-only to skip the movement test.
Each servo moves 50 units (~12 degrees) over 1 s and returns. Detach anything that could collide first.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from robodog.servo import Controller  # noqa: E402

STEP = 50          # servo units (~12 degrees)
MOVE_MS = 1000
TOLERANCE = 10     # units; ~2.4 degrees


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default="1-12", help="ID range to scan, e.g. 1-9")
    ap.add_argument("--read-only", action="store_true", help="only report positions, don't move")
    args = ap.parse_args()
    lo, hi = (int(x) for x in args.ids.split("-"))

    c = Controller()
    try:
        print("battery: %.2f V" % c.battery())
        found = {sid: p for sid in range(lo, hi + 1) if (p := c.position(sid)) is not None}
        print("servos answering:", ", ".join(f"{sid} ({p})" for sid, p in found.items()) or "none")
        if args.read_only or not found:
            return

        failed = []
        for sid, start in found.items():
            target = start + STEP if start <= 1000 - STEP else start - STEP
            c.move({sid: target}, MOVE_MS)
            time.sleep(MOVE_MS / 1000 + 0.3)
            mid = c.position(sid)
            c.move({sid: start}, MOVE_MS)
            time.sleep(MOVE_MS / 1000 + 0.3)
            end = c.position(sid)
            ok = mid is not None and end is not None \
                and abs(mid - target) <= TOLERANCE and abs(end - start) <= TOLERANCE
            if not ok:
                failed.append(sid)
            print(f"servo {sid}: start {start}, target {target}, reached {mid}, returned {end}"
                  f"  -> {'OK' if ok else 'CHECK'}")
            time.sleep(0.5)
        print("battery: %.2f V" % c.battery())
        print("ALL OK" if not failed else f"CHECK servos: {failed}")
    finally:
        c.close()


if __name__ == "__main__":
    main()

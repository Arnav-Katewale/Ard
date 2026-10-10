"""Servo health check: find which servo IDs answer, then move each one and back.

Run on the Pi from the repo root:  .venv/bin/python tools/servo_check.py
Add --read-only to skip the movement test.
Each servo moves 200 units (~48 degrees) over 1.5 s and returns (change with --step / --time).
Detach anything that could collide first.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from robodog.servo import Controller  # noqa: E402

STEP = 200         # servo units (~48 degrees)
MOVE_MS = 1500
TOLERANCE = 10     # units; ~2.4 degrees


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default="1-12", help="ID range to scan, e.g. 1-9")
    ap.add_argument("--read-only", action="store_true", help="only report positions, don't move")
    ap.add_argument("--step", type=int, default=STEP, help="move size in servo units (1000 = ~240 degrees)")
    ap.add_argument("--time", type=int, default=MOVE_MS, help="move time in ms")
    args = ap.parse_args()
    lo, hi = (int(x) for x in args.ids.split("-"))

    c = Controller()
    try:
        print("battery: %.2f V" % c.battery())
        # Read every ID three times. Two servos sharing an ID reply on top of each
        # other, which shows up as readings that jump around or drop out.
        reads = {sid: [c.position(sid) for _ in range(3)] for sid in range(lo, hi + 1)}
        found, unstable = {}, {}
        for sid, r in reads.items():
            vals = [p for p in r if p is not None]
            if not vals:
                continue
            if len(vals) < len(r) or max(vals) - min(vals) > TOLERANCE:
                unstable[sid] = r
            else:
                found[sid] = vals[0]
        print("servos answering:", ", ".join(f"{sid} ({p})" for sid, p in found.items()) or "none")
        if unstable:
            print("UNSTABLE (possible duplicate ID or loose cable), not moved:",
                  "; ".join(f"{sid} {r}" for sid, r in unstable.items()))
        if args.read_only or not found:
            return

        failed = []
        for sid, start in found.items():
            target = start + args.step if start <= 1000 - args.step else start - args.step
            c.move({sid: target}, args.time)
            time.sleep(args.time / 1000 + 0.3)
            mid = c.position(sid)
            c.move({sid: start}, args.time)
            time.sleep(args.time / 1000 + 0.3)
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

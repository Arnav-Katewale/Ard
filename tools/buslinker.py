"""Talk to LX-16A servos directly through a Hiwonder BusLinker (CH340 USB-serial, 115200 baud).

Unlike the USB HID servo controller, the BusLinker passes the servos' own protocol
straight through, so it can read and set servo IDs, voltage and temperature.

Packet: 55 55 <id> <len> <cmd> <params...> <checksum>
        len = number of params + 3, checksum = ~(id + len + cmd + params) & 0xFF
ID 254 is broadcast: every servo on the bus obeys it.

Usage (only ONE servo connected when reading or setting an ID):
    python tools/buslinker.py info              # read ID, position, voltage, temperature
    python tools/buslinker.py set-id 5          # change the connected servo's ID to 5
    python tools/buslinker.py --port COM4 info
"""
import argparse
import struct
import sys
import time

import serial
from serial.tools import list_ports

BROADCAST = 254
CMD_ID_WRITE = 13
CMD_ID_READ = 14
CMD_TEMP_READ = 26
CMD_VIN_READ = 27
CMD_POS_READ = 28


def find_port():
    for p in list_ports.comports():
        if p.vid == 0x1A86 and p.pid == 0x7523:   # CH340
            return p.device
    sys.exit("BusLinker (CH340) not found. Is it plugged in?")


class Bus:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.1)

    def close(self):
        self.s.close()

    def _send(self, sid, cmd, params=b""):
        body = bytes([sid, len(params) + 3, cmd]) + bytes(params)
        self.s.write(b"\x55\x55" + body + bytes([~sum(body) & 0xFF]))

    def _query(self, sid, cmd, nparams):
        """Send a read command and return the reply's params, or None."""
        self.s.reset_input_buffer()
        self._send(sid, cmd)
        want = 6 + nparams
        data = b""
        deadline = time.monotonic() + 0.1
        while time.monotonic() < deadline and len(data) < want + 16:
            data += self.s.read(want + 16 - len(data))
            # Look for a complete, valid reply anywhere in what we have.
            i = data.find(b"\x55\x55")
            while i != -1 and len(data) - i >= want:
                pkt = data[i:i + want]
                body = pkt[2:-1]
                if body[1] == nparams + 3 and body[2] == cmd and (~sum(body) & 0xFF) == pkt[-1]:
                    return body[0], body[3:]
                i = data.find(b"\x55\x55", i + 1)
        return None

    def read_id(self, sid=BROADCAST):
        r = self._query(sid, CMD_ID_READ, 1)
        return r[1][0] if r else None

    def position(self, sid):
        r = self._query(sid, CMD_POS_READ, 2)
        return struct.unpack("<h", r[1])[0] if r else None

    def voltage(self, sid):
        r = self._query(sid, CMD_VIN_READ, 2)
        return struct.unpack("<H", r[1])[0] / 1000 if r else None

    def temperature(self, sid):
        r = self._query(sid, CMD_TEMP_READ, 1)
        return r[1][0] if r else None

    def set_id(self, old, new):
        self._send(old, CMD_ID_WRITE, [new])
        time.sleep(0.1)


def info(bus):
    ids = [bus.read_id() for _ in range(3)]
    if None in ids or len(set(ids)) != 1:
        sys.exit(f"Inconsistent ID replies {ids}: is exactly one servo connected and powered?")
    sid = ids[0]
    print(f"ID {sid}: position {bus.position(sid)}, {bus.voltage(sid)} V, {bus.temperature(sid)} C")
    return sid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="COM port (default: find the CH340)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    s = sub.add_parser("set-id")
    s.add_argument("new_id", type=int)
    args = ap.parse_args()

    bus = Bus(args.port or find_port())
    try:
        old = info(bus)
        if args.cmd == "set-id":
            if not 0 <= args.new_id <= 253:
                sys.exit("ID must be 0-253")
            bus.set_id(old, args.new_id)
            got = bus.read_id(args.new_id)
            print(f"ID {old} -> {args.new_id}: " + ("done" if got == args.new_id else f"FAILED (read back {got})"))
            info(bus)
    finally:
        bus.close()


if __name__ == "__main__":
    main()

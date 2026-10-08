"""Hiwonder / LewanSoul USB bus-servo controller (USB HID 0483:5750).

The board shows up as a USB HID device - no COM port. Packets are
0x55 0x55 <len> <cmd> <params...>, padded into a 64-byte HID report.
Positions run 0-1000 (about 240 degrees).
"""
import struct
import hid

VID, PID = 0x0483, 0x5750
CMD_SERVO_MOVE = 0x03
CMD_GET_BATTERY_VOLTAGE = 0x0F
CMD_MULT_SERVO_UNLOAD = 0x14
CMD_MULT_SERVO_POS_READ = 0x15
POS_MIN, POS_MAX = 0, 1000


class Controller:
    def __init__(self):
        self.h = hid.device()
        self.h.open(VID, PID)

    def close(self):
        try:
            self.h.close()
        except Exception:
            pass

    def _drain(self):
        self.h.set_nonblocking(1)
        while self.h.read(64):
            pass
        self.h.set_nonblocking(0)

    def _send(self, cmd, params=b''):
        pkt = bytes([0x55, 0x55, len(params) + 2, cmd]) + bytes(params)
        self.h.write(b'\x00' + pkt + b'\x00' * (64 - len(pkt)))

    def _query(self, cmd, params=b'', timeout_ms=300):
        self._drain()
        self._send(cmd, params)
        r = bytes(self.h.read(64, timeout_ms))
        if len(r) >= 4 and r[0] == 0x55 and r[1] == 0x55 and r[3] == cmd:
            return r
        return None

    def battery(self):
        r = self._query(CMD_GET_BATTERY_VOLTAGE)
        return (r[4] | r[5] << 8) / 1000 if r else None

    def position(self, sid):
        """Current position, or None if the servo did not answer."""
        r = self._query(CMD_MULT_SERVO_POS_READ, [1, sid])
        if not r or r[4] != 1 or r[5] != sid:
            return None
        p = struct.unpack('<h', r[6:8])[0]
        return None if p == -1 else p

    def move(self, targets, time_ms):
        """targets: {servo_id: position}. Servos arrive together after time_ms."""
        params = [len(targets), time_ms & 0xFF, time_ms >> 8]
        for sid, pos in targets.items():
            pos = max(POS_MIN, min(POS_MAX, int(pos)))
            params += [sid, pos & 0xFF, pos >> 8]
        self._send(CMD_SERVO_MOVE, params)

    def unload(self, ids):
        self._send(CMD_MULT_SERVO_UNLOAD, [len(ids)] + list(ids))


if __name__ == "__main__":
    # Read-only check: battery and servo 1-4 positions. Nothing moves.
    c = Controller()
    print("battery: %.2f V" % c.battery())
    for sid in range(1, 5):
        print(f"servo {sid}: {c.position(sid)}")
    c.close()

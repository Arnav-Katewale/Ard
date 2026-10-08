"""Desktop servo test panel for the Hiwonder / LewanSoul USB bus-servo controller.

Sliders for each servo, move speed, go limp, live battery reading.
Run from the repo root:  python tools/servo_gui.py   (needs:  pip install hidapi)
"""
import os
import sys
import time
import tkinter as tk
from tkinter import ttk, messagebox

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from robodog.servo import Controller, POS_MIN, POS_MAX  # noqa: E402

SCAN_IDS = range(1, 13)
SEND_INTERVAL = 0.08         # seconds between live-drag updates


class ServoRow:
    def __init__(self, app, parent, sid, pos):
        self.app, self.sid = app, sid
        self.var = tk.IntVar(value=max(POS_MIN, min(POS_MAX, pos if pos is not None else 500)))
        self.last_sent = None

        self.frame = ttk.Frame(parent, padding=(0, 4))
        ttk.Label(self.frame, text=f"Servo {sid}", width=9, font=("Segoe UI", 10, "bold")).pack(side="left")
        self.scale = ttk.Scale(self.frame, from_=POS_MIN, to=POS_MAX, variable=self.var,
                               orient="horizontal", length=420, command=self._on_drag)
        self.scale.pack(side="left", padx=6, fill="x", expand=True)
        self.scale.bind("<ButtonRelease-1>", lambda e: self.send(force=True))
        self.entry = ttk.Spinbox(self.frame, from_=POS_MIN, to=POS_MAX, width=6, increment=10,
                                 textvariable=self.var, command=lambda: self.send(force=True))
        self.entry.bind("<Return>", lambda e: self.send(force=True))
        self.entry.pack(side="left", padx=4)
        self.actual = ttk.Label(self.frame, width=14, foreground="#666")
        self.actual.pack(side="left")
        self.show_actual(pos)
        self.frame.pack(fill="x")

    def show_actual(self, pos):
        self.actual.config(text="no feedback" if pos is None else f"actual: {pos}")

    def _on_drag(self, _):
        self.var.set(int(float(self.var.get())))
        if self.app.live.get():
            self.send()

    def send(self, force=False):
        try:
            pos = int(self.var.get())
        except (tk.TclError, ValueError):
            return
        now = time.monotonic()
        if not force and now - self.app.last_send < SEND_INTERVAL:
            return
        if pos == self.last_sent and not force:
            return
        self.app.move({self.sid: pos})
        self.last_sent = pos


class App:
    def __init__(self, root):
        self.root = root
        self.ctrl = None
        self.rows = {}
        self.last_send = 0.0
        root.title("Servo Control")
        root.geometry("720x460")

        top = ttk.Frame(root, padding=10)
        top.pack(fill="x")
        self.status = ttk.Label(top, text="Not connected", font=("Segoe UI", 10))
        self.status.pack(side="left")
        self.battery = ttk.Label(top, text="", font=("Segoe UI", 10, "bold"))
        self.battery.pack(side="right")

        ctl = ttk.Frame(root, padding=(10, 0))
        ctl.pack(fill="x")
        ttk.Label(ctl, text="Move time (ms):").pack(side="left")
        self.move_time = tk.IntVar(value=500)
        ttk.Spinbox(ctl, from_=0, to=30000, increment=100, width=7,
                    textvariable=self.move_time).pack(side="left", padx=(4, 14))
        self.live = tk.BooleanVar(value=True)
        ttk.Checkbutton(ctl, text="Move while dragging", variable=self.live).pack(side="left")

        btns = ttk.Frame(root, padding=10)
        btns.pack(fill="x")
        ttk.Button(btns, text="Scan / Reconnect", command=self.connect).pack(side="left")
        ttk.Button(btns, text="Read positions", command=self.read_positions).pack(side="left", padx=6)
        ttk.Button(btns, text="All to centre (500)", command=self.centre).pack(side="left")
        ttk.Button(btns, text="Go limp (unload)", command=self.unload).pack(side="left", padx=6)

        add = ttk.Frame(btns)
        add.pack(side="right")
        ttk.Label(add, text="Add ID:").pack(side="left")
        self.add_id = tk.StringVar()
        e = ttk.Entry(add, textvariable=self.add_id, width=4)
        e.pack(side="left", padx=4)
        e.bind("<Return>", lambda ev: self.add_servo())
        ttk.Button(add, text="Add", command=self.add_servo).pack(side="left")

        ttk.Separator(root).pack(fill="x", padx=10)
        self.servo_frame = ttk.Frame(root, padding=10)
        self.servo_frame.pack(fill="both", expand=True)

        ttk.Label(root, padding=(10, 0, 10, 8), foreground="#666",
                  text="Sliders start at each servo's current position. Drag, type a value + Enter, "
                       "or use the arrows. Range 0-1000 (about 240 degrees).").pack(fill="x")

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.connect()
        self.poll_battery()

    # --- hardware actions -------------------------------------------------
    def _guard(self, fn, *a):
        if not self.ctrl:
            return None
        try:
            return fn(*a)
        except (OSError, IOError) as ex:
            self.status.config(text=f"Lost connection: {ex}", foreground="red")
            self.ctrl.close()
            self.ctrl = None
            return None

    def connect(self):
        if self.ctrl:
            self.ctrl.close()
            self.ctrl = None
        for r in self.rows.values():
            r.frame.destroy()
        self.rows.clear()
        try:
            self.ctrl = Controller()
        except (OSError, IOError):
            self.status.config(text="Controller not found - plug it in and press Scan", foreground="red")
            return
        found = []
        for sid in SCAN_IDS:
            pos = self._guard(self.ctrl.position, sid)
            if pos is not None:
                self.rows[sid] = ServoRow(self, self.servo_frame, sid, pos)
                found.append(sid)
        msg = f"Connected - servos found: {', '.join(map(str, found))}" if found \
            else "Connected - no servos answered (is the battery on?)"
        self.status.config(text=msg, foreground="green" if found else "orange")

    def move(self, targets):
        try:
            t = max(0, min(30000, int(self.move_time.get())))
        except (tk.TclError, ValueError):
            t = 500
        self._guard(self.ctrl.move, targets, t)
        self.last_send = time.monotonic()

    def read_positions(self):
        for sid, row in self.rows.items():
            pos = self._guard(self.ctrl.position, sid)
            row.show_actual(pos)
            if pos is not None:
                row.var.set(max(POS_MIN, min(POS_MAX, pos)))
                row.last_sent = None

    def centre(self):
        if not self.rows:
            return
        if not messagebox.askyesno("Centre all", "Move every servo to 500 (centre)?"):
            return
        for row in self.rows.values():
            row.var.set(500)
        self.move({sid: 500 for sid in self.rows})

    def unload(self):
        if self.rows:
            self._guard(self.ctrl.unload, list(self.rows))
            self.status.config(text="Servos limp - moving a slider powers them again", foreground="orange")

    def add_servo(self):
        try:
            sid = int(self.add_id.get())
        except ValueError:
            return
        if not 0 <= sid <= 253 or sid in self.rows or not self.ctrl:
            return
        pos = self._guard(self.ctrl.position, sid)
        if pos is None and not messagebox.askyesno(
                "No feedback", f"Servo {sid} did not answer. Add it anyway?\n"
                "Its slider starts at 500, so the first move may jump."):
            return
        self.rows[sid] = ServoRow(self, self.servo_frame, sid, pos)
        self.add_id.set("")

    def poll_battery(self):
        v = self._guard(self.ctrl.battery) if self.ctrl else None
        if v is None:
            self.battery.config(text="Battery: --", foreground="#666")
        else:
            colour = "red" if v < 6.4 else "orange" if v < 7.0 else "green"
            self.battery.config(text=f"Battery: {v:.2f} V", foreground=colour)
        self.root.after(3000, self.poll_battery)

    def on_close(self):
        if self.ctrl:
            self.ctrl.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except tk.TclError:
        pass
    App(root)
    root.mainloop()

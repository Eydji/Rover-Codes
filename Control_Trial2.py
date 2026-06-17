"""
╔══════════════════════════════════════════════════════════════╗
║         ESP32 Skid-Steer Rover — Python Controller           ║
║         WASD / Arrow Keys  +  Speed Slider                   ║
║         Windows  |  pyserial required                        ║
╚══════════════════════════════════════════════════════════════╝

Install dependencies:
    pip install pyserial

Usage:
    1. Flash the ESP32 with the Arduino sketch.
    2. Connect the ESP32 via USB.
    3. Run:  python rover_controller.py
    4. Select the correct COM port and click Connect.
    5. Click on the window first to capture keyboard input.

Controls:
    W / ↑       Forward
    S / ↓       Backward
    A / ←       Turn Left  (skid-steer)
    D / →       Turn Right (skid-steer)
    Space       Emergency Stop
    Release key → auto-stop
"""

import json
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

# ──────────────────────────────────────────────────────────────
#  SERIAL MANAGER
# ──────────────────────────────────────────────────────────────
class SerialManager:
    def __init__(self):
        self.ser       = None
        self.connected = False
        self._lock     = threading.Lock()

    def list_ports(self):
        ports = serial.tools.list_ports.comports()
        return [p.device for p in sorted(ports)]

    def connect(self, port, baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)          # wait for ESP32 reset
            self.connected = True
            return True, None
        except Exception as e:
            self.connected = False
            return False, str(e)

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False

    def send(self, cmd: dict):
        if not self.connected or not self.ser:
            return None
        try:
            with self._lock:
                payload = json.dumps(cmd) + "\n"
                self.ser.write(payload.encode("utf-8"))
                time.sleep(0.02)
                resp = self.ser.readline().decode("utf-8", errors="ignore").strip()
                return json.loads(resp) if resp else None
        except Exception:
            return None

# ──────────────────────────────────────────────────────────────
#  ROVER COMMAND LAYER
# ──────────────────────────────────────────────────────────────
class Rover:
    def __init__(self, serial_manager: SerialManager):
        self.sm            = serial_manager
        self.speed         = 200
        self.current_cmd   = None

    def _send(self, cmd):
        return self.sm.send(cmd)

    def forward(self):
        return self._send({"cmd": "forward",  "speed": self.speed})

    def backward(self):
        return self._send({"cmd": "backward", "speed": self.speed})

    def left(self):
        return self._send({"cmd": "left",     "speed": self.speed})

    def right(self):
        return self._send({"cmd": "right",    "speed": self.speed})

    def stop(self):
        return self._send({"cmd": "stop"})

    def status(self):
        return self._send({"cmd": "status"})

# ──────────────────────────────────────────────────────────────
#  GUI
# ──────────────────────────────────────────────────────────────
class RoverApp(tk.Tk):
    # ── palette ──────────────────────────────────────────────
    BG          = "#0d0f14"
    PANEL       = "#161b25"
    ACCENT      = "#00e5ff"
    ACCENT_DIM  = "#007a8a"
    DANGER      = "#ff3b5c"
    TEXT        = "#e8edf5"
    TEXT_DIM    = "#5a6478"
    BTN_IDLE    = "#1e2535"
    BTN_ACTIVE  = "#00e5ff"
    BTN_TXT_ACT = "#0d0f14"
    GREEN       = "#00ff9d"

    KEYS_FORWARD  = {"w", "Up"}
    KEYS_BACKWARD = {"s", "Down"}
    KEYS_LEFT     = {"a", "Left"}
    KEYS_RIGHT    = {"d", "Right"}
    KEYS_STOP     = {" ", "space"}

    def __init__(self):
        super().__init__()
        self.title("Rover Control")
        self.resizable(False, False)
        self.configure(bg=self.BG)

        self.sm      = SerialManager()
        self.rover   = Rover(self.sm)
        self._active_key   = None
        self._key_held     = False
        self._send_thread  = None
        self._log_lines    = []

        self._build_ui()
        self._refresh_ports()
        self.bind_all("<KeyPress>",   self._on_key_press)
        self.bind_all("<KeyRelease>", self._on_key_release)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── UI construction ───────────────────────────────────────
    def _build_ui(self):
        # title bar
        hdr = tk.Frame(self, bg=self.BG)
        hdr.pack(fill="x", padx=24, pady=(20, 0))
        tk.Label(hdr, text="◈  ROVER CONTROL", font=("Courier New", 13, "bold"),
                 bg=self.BG, fg=self.ACCENT).pack(side="left")
        self._status_dot = tk.Label(hdr, text="●", font=("Courier New", 14),
                                    bg=self.BG, fg=self.DANGER)
        self._status_dot.pack(side="right")
        self._status_lbl = tk.Label(hdr, text="DISCONNECTED",
                                    font=("Courier New", 9), bg=self.BG, fg=self.DANGER)
        self._status_lbl.pack(side="right", padx=(0, 6))

        self._sep(4)

        # ── connection panel ──────────────────────────────────
        conn = self._card()
        tk.Label(conn, text="SERIAL PORT", font=("Courier New", 8, "bold"),
                 bg=self.PANEL, fg=self.TEXT_DIM).grid(row=0, column=0, sticky="w", pady=(0,4))

        port_row = tk.Frame(conn, bg=self.PANEL)
        port_row.grid(row=1, column=0, sticky="ew")
        conn.columnconfigure(0, weight=1)

        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(port_row, textvariable=self._port_var,
                                       width=12, state="readonly",
                                       font=("Courier New", 10))
        self._port_cb.pack(side="left")

        self._btn_refresh = self._small_btn(port_row, "↻", self._refresh_ports)
        self._btn_refresh.pack(side="left", padx=(6, 0))

        self._btn_connect = self._action_btn(conn, "CONNECT", self._toggle_connect)
        self._btn_connect.grid(row=2, column=0, sticky="ew", pady=(10, 0))

        self._sep(8)

        # ── speed slider ──────────────────────────────────────
        spd = self._card()
        tk.Label(spd, text="SPEED", font=("Courier New", 8, "bold"),
                 bg=self.PANEL, fg=self.TEXT_DIM).pack(anchor="w")

        slider_row = tk.Frame(spd, bg=self.PANEL)
        slider_row.pack(fill="x", pady=(4, 0))

        self._speed_var = tk.IntVar(value=200)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Rover.Horizontal.TScale",
                         background=self.PANEL,
                         troughcolor=self.BTN_IDLE,
                         sliderthickness=16)

        self._slider = ttk.Scale(slider_row, from_=50, to=255,
                                  orient="horizontal", length=240,
                                  variable=self._speed_var,
                                  style="Rover.Horizontal.TScale",
                                  command=self._on_speed_change)
        self._slider.pack(side="left", fill="x", expand=True)

        self._speed_lbl = tk.Label(slider_row, text="200",
                                    font=("Courier New", 13, "bold"),
                                    bg=self.PANEL, fg=self.ACCENT, width=4)
        self._speed_lbl.pack(side="right")

        self._sep(8)

        # ── D-pad display ─────────────────────────────────────
        pad_frame = self._card()
        tk.Label(pad_frame, text="DIRECTION", font=("Courier New", 8, "bold"),
                 bg=self.PANEL, fg=self.TEXT_DIM).pack(anchor="w", pady=(0, 8))

        dpad = tk.Frame(pad_frame, bg=self.PANEL)
        dpad.pack()

        self._dpad_btns = {}
        arrows = {
            "forward":  ("▲", 0, 1),
            "backward": ("▼", 2, 1),
            "left":     ("◀", 1, 0),
            "right":    ("▶", 1, 2),
        }
        for name, (char, r, c) in arrows.items():
            b = tk.Label(dpad, text=char, font=("Courier New", 22),
                          bg=self.BTN_IDLE, fg=self.TEXT_DIM,
                          width=3, relief="flat", cursor="arrow")
            b.grid(row=r, column=c, padx=3, pady=3, ipadx=4, ipady=4)
            self._dpad_btns[name] = b

        # centre stop
        stop_c = tk.Label(dpad, text="■", font=("Courier New", 22),
                           bg=self.DANGER, fg=self.BG,
                           width=3, relief="flat", cursor="hand2")
        stop_c.grid(row=1, column=1, padx=3, pady=3, ipadx=4, ipady=4)
        stop_c.bind("<Button-1>", lambda e: self._emergency_stop())
        self._dpad_btns["stop"] = stop_c

        self._sep(8)

        # ── log ───────────────────────────────────────────────
        log_card = self._card()
        tk.Label(log_card, text="LOG", font=("Courier New", 8, "bold"),
                 bg=self.PANEL, fg=self.TEXT_DIM).pack(anchor="w", pady=(0, 4))

        self._log_text = tk.Text(log_card, height=6, width=38,
                                  bg="#0a0c10", fg=self.GREEN,
                                  font=("Courier New", 8),
                                  relief="flat", state="disabled",
                                  insertbackground=self.ACCENT)
        self._log_text.pack(fill="x")

        # ── keyboard hint ─────────────────────────────────────
        hint = tk.Frame(self, bg=self.BG)
        hint.pack(pady=(4, 16))
        hints = [("W/↑", "Fwd"), ("S/↓", "Back"), ("A/←", "Left"),
                 ("D/→", "Right"), ("SPC", "Stop")]
        for key, lbl in hints:
            tk.Label(hint, text=key, font=("Courier New", 7, "bold"),
                     bg=self.ACCENT_DIM, fg=self.BG, padx=4).pack(side="left", padx=2)
            tk.Label(hint, text=lbl, font=("Courier New", 7),
                     bg=self.BG, fg=self.TEXT_DIM).pack(side="left", padx=(0, 6))

    # ── helpers ───────────────────────────────────────────────
    def _sep(self, h=6):
        tk.Frame(self, bg=self.BG, height=h).pack(fill="x")

    def _card(self):
        f = tk.Frame(self, bg=self.PANEL, padx=16, pady=12)
        f.pack(fill="x", padx=16, pady=2)
        return f

    def _action_btn(self, parent, text, cmd):
        return tk.Button(parent, text=text, command=cmd,
                         font=("Courier New", 10, "bold"),
                         bg=self.BTN_IDLE, fg=self.ACCENT,
                         activebackground=self.ACCENT, activeforeground=self.BG,
                         relief="flat", cursor="hand2", pady=6)

    def _small_btn(self, parent, text, cmd):
        return tk.Button(parent, text=text, command=cmd,
                         font=("Courier New", 10, "bold"),
                         bg=self.BTN_IDLE, fg=self.ACCENT,
                         activebackground=self.ACCENT, activeforeground=self.BG,
                         relief="flat", cursor="hand2", padx=8, pady=2)

    # ── port management ───────────────────────────────────────
    def _refresh_ports(self):
        ports = self.sm.list_ports()
        self._port_cb["values"] = ports
        if ports:
            self._port_cb.current(0)
        self._log(f"Found {len(ports)} port(s): {', '.join(ports) if ports else 'none'}")

    def _toggle_connect(self):
        if self.sm.connected:
            self.sm.disconnect()
            self._set_connected(False)
            self._log("Disconnected.")
        else:
            port = self._port_var.get()
            if not port:
                messagebox.showwarning("No port", "Select a COM port first.")
                return
            self._log(f"Connecting to {port}…")
            ok, err = self.sm.connect(port)
            if ok:
                self._set_connected(True)
                self._log(f"Connected on {port} ✓")
                # read ready message
                threading.Thread(target=self._read_ready, daemon=True).start()
            else:
                self._log(f"Error: {err}")
                messagebox.showerror("Connection failed", err)

    def _read_ready(self):
        try:
            time.sleep(0.5)
            if self.sm.ser and self.sm.ser.is_open:
                line = self.sm.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self._log(f"ESP32: {line}")
        except Exception:
            pass

    def _set_connected(self, state: bool):
        if state:
            self._status_dot.config(fg=self.GREEN)
            self._status_lbl.config(text="CONNECTED",    fg=self.GREEN)
            self._btn_connect.config(text="DISCONNECT",  fg=self.DANGER)
        else:
            self._status_dot.config(fg=self.DANGER)
            self._status_lbl.config(text="DISCONNECTED", fg=self.DANGER)
            self._btn_connect.config(text="CONNECT",     fg=self.ACCENT)

    # ── speed slider ──────────────────────────────────────────
    def _on_speed_change(self, val):
        v = int(float(val))
        self._speed_lbl.config(text=str(v))
        self.rover.speed = v

    # ── key events ───────────────────────────────────────────
    def _on_key_press(self, event):
        key = event.keysym
        if key == self._active_key:
            return                   # already handling this key
        self._active_key = key
        self._dispatch_key(key, pressed=True)

    def _on_key_release(self, event):
        key = event.keysym
        if key == self._active_key:
            self._active_key = None
            self._dispatch_key(key, pressed=False)

    def _dispatch_key(self, key, pressed: bool):
        if not self.sm.connected:
            return

        if key in self.KEYS_FORWARD:
            self._highlight_dpad("forward", pressed)
            if pressed:
                self._async_cmd(self.rover.forward, "forward")
            else:
                self._async_cmd(self.rover.stop, "stop")

        elif key in self.KEYS_BACKWARD:
            self._highlight_dpad("backward", pressed)
            if pressed:
                self._async_cmd(self.rover.backward, "backward")
            else:
                self._async_cmd(self.rover.stop, "stop")

        elif key in self.KEYS_LEFT:
            self._highlight_dpad("left", pressed)
            if pressed:
                self._async_cmd(self.rover.left, "left")
            else:
                self._async_cmd(self.rover.stop, "stop")

        elif key in self.KEYS_RIGHT:
            self._highlight_dpad("right", pressed)
            if pressed:
                self._async_cmd(self.rover.right, "right")
            else:
                self._async_cmd(self.rover.stop, "stop")

        elif key in self.KEYS_STOP and pressed:
            self._emergency_stop()

    def _emergency_stop(self):
        self._highlight_dpad("stop", True)
        self._async_cmd(self.rover.stop, "STOP")
        self.after(200, lambda: self._highlight_dpad("stop", False))

    # ── async command sender ──────────────────────────────────
    def _async_cmd(self, fn, label):
        def _run():
            resp = fn()
            msg = f"→ {label}"
            if resp:
                msg += f"  ← {resp}"
            self._log(msg)
        threading.Thread(target=_run, daemon=True).start()

    # ── d-pad highlight ───────────────────────────────────────
    def _highlight_dpad(self, name, active: bool):
        btn = self._dpad_btns.get(name)
        if not btn:
            return
        if name == "stop":
            btn.config(bg=self.DANGER if not active else "#ff7090")
        else:
            btn.config(
                bg=self.BTN_ACTIVE  if active else self.BTN_IDLE,
                fg=self.BTN_TXT_ACT if active else self.TEXT_DIM,
            )

    # ── log ───────────────────────────────────────────────────
    def _log(self, msg: str):
        def _update():
            ts = time.strftime("%H:%M:%S")
            self._log_text.config(state="normal")
            self._log_text.insert("end", f"[{ts}] {msg}\n")
            self._log_text.see("end")
            # keep last 200 lines
            lines = int(self._log_text.index("end-1c").split(".")[0])
            if lines > 200:
                self._log_text.delete("1.0", f"{lines-200}.0")
            self._log_text.config(state="disabled")
        self.after(0, _update)

    # ── close ─────────────────────────────────────────────────
    def _on_close(self):
        if self.sm.connected:
            self.rover.stop()
            self.sm.disconnect()
        self.destroy()


# ──────────────────────────────────────────────────────────────
#  ENTRY POINT
# ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = RoverApp()
    app.mainloop()

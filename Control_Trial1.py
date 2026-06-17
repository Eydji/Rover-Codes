#!/usr/bin/env python3
"""
============================================================
 ESP32 Motor Controller — Keyboard Control (TUI)
 Real-time keyboard input, live dashboard in terminal
============================================================

 INSTALL:
   pip install pyserial pynput

 RUN:
   python esp32_keyboard.py
   python esp32_keyboard.py --port COM3
   python esp32_keyboard.py --port /dev/ttyUSB0

============================================================
 KEYBOARD MAP
============================================================

  ── MOTOR SELECTION ─────────────────────────────────────
   1 2 3 4 5 6     Select active motor (1–6)
   0               Select ALL motors at once

  ── MOTOR SPEED CONTROL (active motor) ──────────────────
   W / ↑           Speed up (forward)
   S / ↓           Slow down / reverse
   SPACE           Stop selected motor(s)
   BACKSPACE       Stop ALL motors

  ── SERVO SELECTION ──────────────────────────────────────
   F1 F2 F3 F4     Select active servo (1–4)
   F5              Select ALL servos at once

  ── SERVO ANGLE CONTROL (active servo) ──────────────────
   A / ←           Rotate left  (−step°)
   D / →           Rotate right (+step°)
   C               Center selected servo(s) at 90°
   SHIFT+C         Center ALL servos

  ── SPEED / STEP TUNING ──────────────────────────────────
   + / =           Increase speed step (motor: +10)
   - / _           Decrease speed step (motor: −10)
   [ / ]           Decrease / increase servo step (angle)

  ── SYSTEM ───────────────────────────────────────────────
   R               Request status from ESP32
   ESC / Q         Emergency stop + quit

============================================================
"""

import serial
import serial.tools.list_ports
import json
import time
import threading
import argparse
import sys
import os
import signal
from collections import deque

try:
    from pynput import keyboard as kb
except ImportError:
    print("Missing dependency: pip install pynput")
    sys.exit(1)


# ══════════════════════════════════════════════════════════════════════════════
#  ANSI Helpers
# ══════════════════════════════════════════════════════════════════════════════

RESET   = "\033[0m"
BOLD    = "\033[1m"
DIM     = "\033[2m"

# Colours
BLACK   = "\033[30m"
RED     = "\033[31m"
GREEN   = "\033[32m"
YELLOW  = "\033[33m"
BLUE    = "\033[34m"
MAGENTA = "\033[35m"
CYAN    = "\033[36m"
WHITE   = "\033[37m"

BG_BLACK   = "\033[40m"
BG_RED     = "\033[41m"
BG_GREEN   = "\033[42m"
BG_YELLOW  = "\033[43m"
BG_BLUE    = "\033[44m"
BG_MAGENTA = "\033[45m"
BG_CYAN    = "\033[46m"
BG_WHITE   = "\033[47m"

BRIGHT_BLACK   = "\033[90m"
BRIGHT_RED     = "\033[91m"
BRIGHT_GREEN   = "\033[92m"
BRIGHT_YELLOW  = "\033[93m"
BRIGHT_BLUE    = "\033[94m"
BRIGHT_MAGENTA = "\033[95m"
BRIGHT_CYAN    = "\033[96m"
BRIGHT_WHITE   = "\033[97m"

BG_BRIGHT_BLACK = "\033[100m"

def clr():        sys.stdout.write("\033[2J\033[H"); sys.stdout.flush()
def goto(r, c):   sys.stdout.write(f"\033[{r};{c}H"); sys.stdout.flush()
def hide_cursor(): sys.stdout.write("\033[?25l"); sys.stdout.flush()
def show_cursor(): sys.stdout.write("\033[?25h"); sys.stdout.flush()
def erase_line():  sys.stdout.write("\033[2K"); sys.stdout.flush()

def write(text): sys.stdout.write(text); sys.stdout.flush()


# ══════════════════════════════════════════════════════════════════════════════
#  Controller Core
# ══════════════════════════════════════════════════════════════════════════════

class ESP32Controller:
    NUM_MOTORS = 6
    NUM_SERVOS = 4

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self._serial = None
        self._lock   = threading.Lock()
        self._running = False
        self._rx_thread = None

        self.motor_speeds  = [0]  * self.NUM_MOTORS
        self.servo_angles  = [90] * self.NUM_SERVOS
        self.log_queue     = deque(maxlen=6)
        self.esp_status    = "Connecting…"

    def connect(self):
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2.0)
            self._serial.reset_input_buffer()
            self._running = True
            self._rx_thread = threading.Thread(target=self._reader, daemon=True)
            self._rx_thread.start()
            self.esp_status = "ONLINE"
            return True
        except serial.SerialException as e:
            self.esp_status = f"OFFLINE ({e})"
            return False

    def disconnect(self):
        self.stop_all()
        self.center_all()
        time.sleep(0.3)
        self._running = False
        if self._serial and self._serial.is_open:
            self._serial.close()

    def _send(self, payload):
        if not self._serial or not self._serial.is_open:
            return
        line = json.dumps(payload) + "\n"
        with self._lock:
            try:
                self._serial.write(line.encode())
                self._serial.flush()
            except serial.SerialException:
                pass

    def _reader(self):
        while self._running:
            try:
                if self._serial.in_waiting > 0:
                    raw = self._serial.readline().decode("utf-8", errors="replace").strip()
                    if raw:
                        self._parse(raw)
                else:
                    time.sleep(0.005)
            except (serial.SerialException, OSError):
                self.esp_status = "DISCONNECTED"
                break

    def _parse(self, raw):
        try:
            resp = json.loads(raw)
            if "motors" in resp and "servos" in resp:
                self.motor_speeds = list(resp["motors"])
                self.servo_angles = list(resp["servos"])
            elif "motor" in resp:
                idx = resp["motor"] - 1
                self.motor_speeds[idx] = resp.get("speed", 0)
            elif "servo" in resp:
                idx = resp["servo"] - 1
                self.servo_angles[idx] = resp.get("angle", 90)
            msg = resp.get("message", raw[:60])
            self.log_queue.append(f"ESP32 ▸ {msg}")
        except json.JSONDecodeError:
            self.log_queue.append(f"RAW ▸ {raw[:60]}")

    # ── Commands ─────────────────────────────────────────────────────────────

    def set_motor(self, idx, speed):
        speed = max(-255, min(255, speed))
        self.motor_speeds[idx] = speed
        self._send({"cmd": "motor", "id": idx + 1, "speed": speed})

    def set_servo(self, idx, angle):
        angle = max(0, min(180, angle))
        self.servo_angles[idx] = angle
        self._send({"cmd": "servo", "id": idx + 1, "angle": angle})

    def stop_all(self):
        for i in range(self.NUM_MOTORS):
            self.motor_speeds[i] = 0
        self._send({"cmd": "stop"})
        self.log_queue.append("▸ All motors STOPPED")

    def center_all(self):
        for i in range(self.NUM_SERVOS):
            self.servo_angles[i] = 90
        self._send({"cmd": "center"})
        self.log_queue.append("▸ All servos CENTERED")

    def get_status(self):
        self._send({"cmd": "status"})


# ══════════════════════════════════════════════════════════════════════════════
#  TUI Dashboard
# ══════════════════════════════════════════════════════════════════════════════

class Dashboard:
    """Renders the live terminal UI."""

    MOTOR_COLORS = [BRIGHT_CYAN, BRIGHT_GREEN, BRIGHT_YELLOW,
                    BRIGHT_MAGENTA, BRIGHT_RED, BRIGHT_BLUE]
    SERVO_COLORS = [CYAN, GREEN, YELLOW, MAGENTA]

    def __init__(self, ctrl: ESP32Controller):
        self.ctrl = ctrl
        self.active_motor = 1        # 1–6, or 0 = all
        self.active_servo = 1        # 1–4, or 0 = all
        self.motor_step   = 20       # speed increment
        self.servo_step   = 10       # angle increment
        self._lock        = threading.Lock()

    def _bar(self, value, width=20, max_val=255):
        """Render a signed bar for motor speed."""
        half = width // 2
        ratio = abs(value) / max_val
        filled = int(ratio * half)
        if value > 0:
            bar = " " * half + (BRIGHT_GREEN + "█" * filled + RESET) + " " * (half - filled)
        elif value < 0:
            bar = " " * (half - filled) + (BRIGHT_RED + "█" * filled + RESET) + " " * half
        else:
            bar = " " * width
        return f"[{bar}]"

    def _servo_bar(self, angle, width=18):
        """Render a 0–180 bar for servo angle."""
        filled = int((angle / 180) * width)
        bar = BRIGHT_CYAN + "█" * filled + RESET + DIM + "░" * (width - filled) + RESET
        return f"[{bar}]"

    def render(self):
        with self._lock:
            clr()
            ctrl = self.ctrl
            c = ctrl

            W = 62  # total box width (inner)

            def box_top(title=""):
                pad = (W - len(title) - 2) // 2
                write(BRIGHT_BLACK + "╔" + "═" * pad + BRIGHT_WHITE + BOLD + f" {title} " + RESET + BRIGHT_BLACK + "═" * (W - pad - len(title) - 2) + "╗" + RESET + "\n")

            def box_mid():
                write(BRIGHT_BLACK + "╠" + "═" * W + "╣" + RESET + "\n")

            def box_bot():
                write(BRIGHT_BLACK + "╚" + "═" * W + "╝" + RESET + "\n")

            def row(content=""):
                # Pad content to width W (strip ANSI for length calc)
                import re
                ansi_escape = re.compile(r'\033\[[0-9;]*m')
                plain = ansi_escape.sub('', content)
                pad = W - len(plain)
                write(BRIGHT_BLACK + "║" + RESET + content + " " * max(0, pad) + BRIGHT_BLACK + "║" + RESET + "\n")

            # ── Header ─────────────────────────────────────────────────────
            box_top("ESP32 MOTOR CONTROLLER")

            status_col = BRIGHT_GREEN if ctrl.esp_status == "ONLINE" else BRIGHT_RED
            row(f"  {DIM}Port:{RESET} {BRIGHT_WHITE}{ctrl.port}{RESET}   "
                f"{DIM}Status:{RESET} {status_col}{ctrl.esp_status}{RESET}   "
                f"{DIM}Baud:{RESET} {ctrl.baud}")
            row()

            # ── DC Motors ──────────────────────────────────────────────────
            box_mid()
            row(f"  {BOLD}{BRIGHT_WHITE}DC MOTORS{RESET}   "
                f"{DIM}Active: {RESET}"
                + (BRIGHT_YELLOW + BOLD + "ALL" + RESET if self.active_motor == 0
                   else self.MOTOR_COLORS[self.active_motor - 1] + BOLD + f"Motor {self.active_motor}" + RESET)
                + f"   {DIM}Step:{RESET} {BRIGHT_WHITE}{self.motor_step:+d}{RESET}")
            row()

            for i in range(ctrl.NUM_MOTORS):
                spd = ctrl.motor_speeds[i]
                col = self.MOTOR_COLORS[i]
                active_marker = (BRIGHT_YELLOW + "▶ " + RESET
                                 if (self.active_motor == i + 1 or self.active_motor == 0)
                                 else "  ")
                direction = (BRIGHT_GREEN + "FWD" if spd > 0
                             else BRIGHT_RED + "REV" if spd < 0
                             else DIM + "STP") + RESET
                bar = self._bar(spd)
                row(f"  {active_marker}{col}M{i+1}{RESET} {bar} {direction} {BRIGHT_WHITE}{spd:+4d}{RESET}")

            row()
            row(f"  {DIM}W/↑ faster  S/↓ slower  SPACE stop motor  BKSP stop all  1-6 select  0=all{RESET}")

            # ── Servos ─────────────────────────────────────────────────────
            box_mid()
            row(f"  {BOLD}{BRIGHT_WHITE}SERVOS{RESET}   "
                f"{DIM}Active: {RESET}"
                + (BRIGHT_YELLOW + BOLD + "ALL" + RESET if self.active_servo == 0
                   else self.SERVO_COLORS[self.active_servo - 1] + BOLD + f"Servo {self.active_servo}" + RESET)
                + f"   {DIM}Step:{RESET} {BRIGHT_WHITE}{self.servo_step}°{RESET}")
            row()

            for i in range(ctrl.NUM_SERVOS):
                ang = ctrl.servo_angles[i]
                col = self.SERVO_COLORS[i]
                active_marker = (BRIGHT_YELLOW + "▶ " + RESET
                                 if (self.active_servo == i + 1 or self.active_servo == 0)
                                 else "  ")
                bar = self._servo_bar(ang)
                row(f"  {active_marker}{col}S{i+1}{RESET} {bar} {BRIGHT_WHITE}{ang:3d}°{RESET}")

            row()
            row(f"  {DIM}A/← left  D/→ right  C center  F1-F4 select  F5=all  [/] angle step{RESET}")

            # ── Log ────────────────────────────────────────────────────────
            box_mid()
            row(f"  {BOLD}{BRIGHT_WHITE}LOG{RESET}")
            logs = list(ctrl.log_queue)
            for i in range(6):
                if i < len(logs):
                    entry = logs[-(i + 1)]
                    fade = BRIGHT_WHITE if i == 0 else (DIM if i > 2 else RESET)
                    row(f"  {fade}{entry[:W-4]}{RESET}")
                else:
                    row()

            # ── Footer ─────────────────────────────────────────────────────
            box_mid()
            row(f"  {DIM}ESC/Q — emergency stop & quit   R — refresh status   +/- — motor step{RESET}")
            box_bot()


# ══════════════════════════════════════════════════════════════════════════════
#  Keyboard Handler
# ══════════════════════════════════════════════════════════════════════════════

class KeyboardController:
    def __init__(self, ctrl: ESP32Controller, dash: Dashboard):
        self.ctrl      = ctrl
        self.dash      = dash
        self._listener = None
        self._running  = True
        self._held     = set()       # currently held keys

        # Repeat-on-hold state
        self._repeat_thread = threading.Thread(target=self._repeat_loop, daemon=True)
        self._repeat_thread.start()

    # ── Key Press ────────────────────────────────────────────────────────────

    def on_press(self, key):
        self._held.add(key)
        self._handle(key)

    def on_release(self, key):
        self._held.discard(key)

    def _handle(self, key):
        ctrl = self.ctrl
        dash = self.dash

        try:
            # ── Quit ──────────────────────────────────────────────────────
            if key == kb.Key.esc or (hasattr(key, 'char') and key.char in ('q', 'Q')):
                self._running = False
                return False  # stop listener

            # ── Motor selection: 0–6 ──────────────────────────────────────
            if hasattr(key, 'char') and key.char in '0123456':
                dash.active_motor = int(key.char)
                ctrl.log_queue.append(
                    f"▸ Motor focus: {'ALL' if dash.active_motor == 0 else f'Motor {dash.active_motor}'}"
                )

            # ── Motor speed: W/↑ faster, S/↓ slower ─────────────────────
            elif key in (kb.Key.up,) or (hasattr(key, 'char') and key.char == 'w'):
                self._adjust_motor(+dash.motor_step)

            elif key in (kb.Key.down,) or (hasattr(key, 'char') and key.char == 's'):
                self._adjust_motor(-dash.motor_step)

            # ── Stop motor(s) ─────────────────────────────────────────────
            elif key == kb.Key.space:
                if dash.active_motor == 0:
                    ctrl.stop_all()
                else:
                    ctrl.set_motor(dash.active_motor - 1, 0)
                    ctrl.log_queue.append(f"▸ Motor {dash.active_motor} stopped")

            elif key == kb.Key.backspace:
                ctrl.stop_all()

            # ── Servo selection: F1–F5 ────────────────────────────────────
            elif key == kb.Key.f1: dash.active_servo = 1; ctrl.log_queue.append("▸ Servo focus: Servo 1")
            elif key == kb.Key.f2: dash.active_servo = 2; ctrl.log_queue.append("▸ Servo focus: Servo 2")
            elif key == kb.Key.f3: dash.active_servo = 3; ctrl.log_queue.append("▸ Servo focus: Servo 3")
            elif key == kb.Key.f4: dash.active_servo = 4; ctrl.log_queue.append("▸ Servo focus: Servo 4")
            elif key == kb.Key.f5: dash.active_servo = 0; ctrl.log_queue.append("▸ Servo focus: ALL")

            # ── Servo angle: A/← left, D/→ right ─────────────────────────
            elif key == kb.Key.left or (hasattr(key, 'char') and key.char == 'a'):
                self._adjust_servo(-dash.servo_step)

            elif key == kb.Key.right or (hasattr(key, 'char') and key.char == 'd'):
                self._adjust_servo(+dash.servo_step)

            # ── Center servo(s) ───────────────────────────────────────────
            elif hasattr(key, 'char') and key.char in ('c', 'C'):
                if dash.active_servo == 0 or key.char == 'C':
                    ctrl.center_all()
                else:
                    ctrl.set_servo(dash.active_servo - 1, 90)
                    ctrl.log_queue.append(f"▸ Servo {dash.active_servo} → 90°")

            # ── Motor step size: + / - ────────────────────────────────────
            elif hasattr(key, 'char') and key.char in ('+', '='):
                dash.motor_step = min(255, dash.motor_step + 10)
                ctrl.log_queue.append(f"▸ Motor step → {dash.motor_step}")

            elif hasattr(key, 'char') and key.char in ('-', '_'):
                dash.motor_step = max(5, dash.motor_step - 10)
                ctrl.log_queue.append(f"▸ Motor step → {dash.motor_step}")

            # ── Servo step size: [ / ] ────────────────────────────────────
            elif hasattr(key, 'char') and key.char == '[':
                dash.servo_step = max(1, dash.servo_step - 5)
                ctrl.log_queue.append(f"▸ Servo step → {dash.servo_step}°")

            elif hasattr(key, 'char') and key.char == ']':
                dash.servo_step = min(45, dash.servo_step + 5)
                ctrl.log_queue.append(f"▸ Servo step → {dash.servo_step}°")

            # ── Status request ────────────────────────────────────────────
            elif hasattr(key, 'char') and key.char in ('r', 'R'):
                ctrl.get_status()
                ctrl.log_queue.append("▸ Status requested…")

        except Exception:
            pass

        dash.render()

    # ── Motor / Servo Helpers ────────────────────────────────────────────────

    def _adjust_motor(self, delta):
        ctrl = self.ctrl
        dash = self.dash
        if dash.active_motor == 0:
            for i in range(ctrl.NUM_MOTORS):
                new_spd = max(-255, min(255, ctrl.motor_speeds[i] + delta))
                ctrl.set_motor(i, new_spd)
            ctrl.log_queue.append(f"▸ All motors {'▲' if delta > 0 else '▼'} {delta:+d}")
        else:
            idx = dash.active_motor - 1
            new_spd = max(-255, min(255, ctrl.motor_speeds[idx] + delta))
            ctrl.set_motor(idx, new_spd)
            ctrl.log_queue.append(
                f"▸ Motor {dash.active_motor} → {new_spd:+d}"
            )

    def _adjust_servo(self, delta):
        ctrl = self.ctrl
        dash = self.dash
        if dash.active_servo == 0:
            for i in range(ctrl.NUM_SERVOS):
                new_ang = max(0, min(180, ctrl.servo_angles[i] + delta))
                ctrl.set_servo(i, new_ang)
            ctrl.log_queue.append(f"▸ All servos {'→' if delta > 0 else '←'} {delta:+d}°")
        else:
            idx = dash.active_servo - 1
            new_ang = max(0, min(180, ctrl.servo_angles[idx] + delta))
            ctrl.set_servo(idx, new_ang)
            ctrl.log_queue.append(
                f"▸ Servo {dash.active_servo} → {new_ang}°"
            )

    # ── Key Repeat Loop (hold W/S/A/D to continuously adjust) ────────────────

    REPEAT_DELAY    = 0.08   # seconds between repeats while held
    REPEAT_INITIAL  = 0.3    # delay before repeat starts

    _REPEAT_KEYS = {
        kb.Key.up, kb.Key.down, kb.Key.left, kb.Key.right,
    }

    def _repeat_loop(self):
        """Fire held-key actions continuously while a direction key is held."""
        timers = {}  # key → time it was first pressed
        while self._running:
            now = time.time()
            for key in list(self._held):
                char = getattr(key, 'char', None)
                is_repeat_key = (key in self._REPEAT_KEYS or
                                 char in ('w', 's', 'a', 'd', 'W', 'S', 'A', 'D'))
                if not is_repeat_key:
                    timers.pop(key, None)
                    continue
                first = timers.get(key, now)
                if key not in timers:
                    timers[key] = now
                elif now - first > self.REPEAT_INITIAL:
                    self._handle(key)
            time.sleep(self.REPEAT_DELAY)

    # ── Start / Stop ─────────────────────────────────────────────────────────

    def start(self):
        self._listener = kb.Listener(on_press=self.on_press, on_release=self.on_release)
        self._listener.start()

    def wait(self):
        """Block until the user quits."""
        while self._running:
            time.sleep(0.05)

    def stop(self):
        self._running = False
        if self._listener:
            self._listener.stop()


# ══════════════════════════════════════════════════════════════════════════════
#  Port Detection
# ══════════════════════════════════════════════════════════════════════════════

def find_esp32_port():
    keywords = ["cp210", "ch340", "ch341", "ftdi", "usb serial", "uart", "esp32"]
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc or k in hwid for k in keywords):
            return p.device
    return ports[0].device if ports else None


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="ESP32 Keyboard Controller")
    parser.add_argument("--port", "-p", help="Serial port (e.g. COM4 or /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200)
    parser.add_argument("--mock", action="store_true",
                        help="Run without hardware (mock mode for testing UI)")
    args = parser.parse_args()

    # ── Port selection ────────────────────────────────────────────────────
    if args.mock:
        port = "MOCK"
    else:
        port = args.port or find_esp32_port()
        if not port:
            print("No serial port found. Use --port to specify one, or --mock to test the UI.")
            sys.exit(1)

    ctrl = ESP32Controller(port, args.baud)
    ctrl.log_queue.append("▸ Keyboard controller starting…")

    if not args.mock:
        ok = ctrl.connect()
        if not ok:
            print(f"Failed to connect to {port}. Use --mock to test without hardware.")
            sys.exit(1)
    else:
        ctrl.esp_status = "MOCK"
        ctrl.log_queue.append("▸ Running in MOCK mode (no hardware)")

    dash = Dashboard(ctrl)
    kbd  = KeyboardController(ctrl, dash)

    # ── Setup terminal ────────────────────────────────────────────────────
    hide_cursor()
    clr()

    # Render once immediately
    dash.render()

    # Start listener
    kbd.start()

    try:
        kbd.wait()
    finally:
        kbd.stop()
        ctrl.disconnect()
        show_cursor()
        clr()
        print("ESP32 Controller shut down. All motors stopped, servos centered.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Xbox-Controller / Tastatur als Fernsteuerung für den Roborock Qrevo Curv.

Standard: nur fahren, Sauger aus.
X bzw. V schaltet Saugen ein/aus.
"""

from __future__ import annotations

import argparse
import asyncio
import fcntl
import json
import logging
import math
import os
import select
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from roborock import RoborockCommand
from roborock.data import UserData
from roborock.devices.device_manager import UserParams, create_device_manager
from roborock.exceptions import RoborockException
from roborock.web_api import RoborockApiClient

SESSION_PATH = Path.home() / ".config" / "roborock-xbox" / "session.json"

# Xbox über evdev (USB)
ABS_LX, ABS_LY = 0x00, 0x01
ABS_LT, ABS_RT = 0x02, 0x05  # oft Z/RZ; zusätzlich GAS/BRAKE
ABS_GAS, ABS_BRAKE = 0x09, 0x0A
BTN_A, BTN_B, BTN_X, BTN_Y = 0x130, 0x131, 0x133, 0x134
BTN_LB, BTN_RB = 0x136, 0x137
BTN_SELECT = 0x13A  # View — Wischen an/aus
EV_SYN, EV_KEY, EV_ABS = 0x00, 0x01, 0x03
EVENT_STRUCT = struct.Struct("llHHi")
EVIOCGRAB = 0x40044590

# Tastatur
KEY_ESC, KEY_W, KEY_A, KEY_S, KEY_D, KEY_Q, KEY_E = 1, 17, 30, 31, 32, 16, 18
KEY_V, KEY_M, KEY_LEFTSHIFT, KEY_SPACE, KEY_ENTER = 47, 50, 42, 57, 28
KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT = 103, 108, 105, 106

# Roborock Fan- / Wasser-Codes
FAN_OFF = 105
FAN_ON = 102  # balanced
WATER_OFF = 200
WATER_ON = 201  # low

MAX_VELOCITY = 0.20
MAX_OMEGA = 1.40
SLOW_SCALE = 0.30
DEADZONE = 0.18
SEND_INTERVAL = 0.20
PROTOCOL_VELOCITY = 0.29
PROTOCOL_OMEGA = 3.0
AXIS_MAX = 32767.0


@dataclass
class Stick:
    forward: float = 0.0
    turn: float = 0.0
    slow: bool = False
    start: bool = False
    stop: bool = False
    dock: bool = False
    vacuum_toggle: bool = False
    mop_toggle: bool = False
    connected: bool = False
    source: str = ""


def shape(value: float) -> float:
    return math.copysign(value * value, value)


def apply_deadzone(value: float, deadzone: float = DEADZONE) -> float:
    magnitude = abs(value)
    if magnitude < deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return math.copysign(min(scaled, 1.0), value)


def norm_stick(value: int) -> float:
    # USB-xpad: -32768..32767, Mitte ~0
    return apply_deadzone(max(-1.0, min(1.0, value / AXIS_MAX)))


def norm_trigger(value: int, code: int) -> float:
    if code in (ABS_GAS, ABS_BRAKE):
        return max(0.0, min(1.0, value / 1023.0))
    # Z/RZ oft -32768 los bis 32767 voll
    return max(0.0, min(1.0, (value + AXIS_MAX) / (2.0 * AXIS_MAX)))


def find_xbox_event_devices() -> list[tuple[str, bool]]:
    """Gibt (event_path, is_usb) zurück. USB zuerst."""
    found: list[tuple[str, bool]] = []
    try:
        text = Path("/proc/bus/input/devices").read_text()
    except OSError:
        return []
    for block in text.split("\n\n"):
        lower = block.lower()
        if "xbox" not in lower:
            continue
        is_usb = "bus=0003" in lower
        event_path = None
        for line in block.splitlines():
            if not line.startswith("H: Handlers="):
                continue
            for part in line.split():
                if part.startswith("event"):
                    candidate = f"/dev/input/{part}"
                    if os.path.exists(candidate):
                        event_path = candidate
        if event_path:
            found.append((event_path, is_usb))
    found.sort(key=lambda item: (not item[1], item[0]))
    return found


def find_keyboard_event() -> str | None:
    try:
        text = Path("/proc/bus/input/devices").read_text()
    except OSError:
        return None
    preferred = []
    fallback = []
    for block in text.split("\n\n"):
        name = ""
        handlers = ""
        for line in block.splitlines():
            if line.startswith("N: Name="):
                name = line.split("=", 1)[1].strip().strip('"').lower()
            if line.startswith("H: Handlers="):
                handlers = line
        if "keyboard" not in name and "kbd" not in handlers:
            continue
        if "xbox" in name:
            continue
        event = None
        for part in handlers.split():
            if part.startswith("event"):
                event = f"/dev/input/{part}"
        if not event or not os.path.exists(event):
            continue
        if "mx" in name or "mchncl" in name:
            preferred.append(event)
        else:
            fallback.append(event)
    return (preferred or fallback or [None])[0]


class InputHub:
    """Xbox-Sticks plus Tastatur (WASD), falls der Pad keine HID-Daten schickt."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._axes: dict[int, int] = {}
        self._axis_live: set[int] = set()
        self._buttons: dict[int, bool] = {}
        self._keys: dict[int, bool] = {}
        self._pad_connected = False
        self._keyboard_ok = False
        self._edge_vacuum = False
        self._edge_mop = False
        self._edge_start = False
        self._prev_x = False
        self._prev_select = False
        self._prev_a = False
        self._prev_v = False
        self._prev_m = False
        self._prev_space = False
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    def start(self) -> None:
        self._threads = [
            threading.Thread(target=self._pad_loop, name="xbox", daemon=True),
            threading.Thread(target=self._keyboard_loop, name="keys", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def close(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=1.5)

    def snapshot(self, invert_turn: bool) -> Stick:
        with self._lock:
            axes = dict(self._axes)
            live = set(self._axis_live)
            buttons = dict(self._buttons)
            keys = dict(self._keys)
            pad = self._pad_connected
            keyboard_ok = self._keyboard_ok
            vacuum_edge = self._edge_vacuum
            mop_edge = self._edge_mop
            start_edge = self._edge_start
            self._edge_vacuum = False
            self._edge_mop = False
            self._edge_start = False

        forward = 0.0
        turn = 0.0
        slow = False
        source = ""

        if ABS_LY in live:
            forward = -norm_stick(axes.get(ABS_LY, 0))
            source = "pad"
        if ABS_LX in live:
            turn = norm_stick(axes.get(ABS_LX, 0))
            source = "pad"
        trigger = 0.0
        for code in (ABS_LT, ABS_RT, ABS_GAS, ABS_BRAKE):
            if code in live:
                trigger = max(trigger, norm_trigger(axes.get(code, 0), code))
        if buttons.get(BTN_LB) or buttons.get(BTN_RB) or trigger > 0.15:
            slow = True

        # Tastatur überlagert / ersetzt, wenn Pad keine Achsen liefert
        key_forward = (1.0 if keys.get(KEY_W) or keys.get(KEY_UP) else 0.0) + (
            -1.0 if keys.get(KEY_S) or keys.get(KEY_DOWN) else 0.0
        )
        key_turn = (1.0 if keys.get(KEY_D) or keys.get(KEY_RIGHT) else 0.0) + (
            -1.0 if keys.get(KEY_A) or keys.get(KEY_LEFT) else 0.0
        )
        key_turn += (0.7 if keys.get(KEY_E) else 0.0) + (-0.7 if keys.get(KEY_Q) else 0.0)
        if key_forward or key_turn:
            forward = max(-1.0, min(1.0, key_forward))
            turn = max(-1.0, min(1.0, key_turn))
            source = "tastatur"
        if keys.get(KEY_LEFTSHIFT):
            slow = True

        if invert_turn:
            turn = -turn

        stop = bool(buttons.get(BTN_B) or keys.get(KEY_ESC))
        dock = bool(buttons.get(BTN_Y))
        connected = pad or keyboard_ok
        return Stick(
            forward=forward,
            turn=turn,
            slow=slow,
            start=start_edge,
            stop=stop,
            dock=dock,
            vacuum_toggle=vacuum_edge,
            mop_toggle=mop_edge,
            connected=connected,
            source=source or ("pad" if pad else "tastatur" if keyboard_ok else ""),
        )

    def _note_edges(self) -> None:
        x = self._buttons.get(BTN_X, False)
        select = self._buttons.get(BTN_SELECT, False)
        a = self._buttons.get(BTN_A, False)
        v = self._keys.get(KEY_V, False)
        m = self._keys.get(KEY_M, False)
        space = self._keys.get(KEY_SPACE, False)
        if x and not self._prev_x:
            self._edge_vacuum = True
        if v and not self._prev_v:
            self._edge_vacuum = True
        if select and not self._prev_select:
            self._edge_mop = True
        if m and not self._prev_m:
            self._edge_mop = True
        if (a and not self._prev_a) or (space and not self._prev_space):
            self._edge_start = True
        self._prev_x = x
        self._prev_select = select
        self._prev_a = a
        self._prev_v = v
        self._prev_m = m
        self._prev_space = space

    def _pad_loop(self) -> None:
        while not self._stop.is_set():
            devices = find_xbox_event_devices()
            usb = [item for item in devices if item[1]]
            devices = usb or devices
            if not devices:
                with self._lock:
                    self._pad_connected = False
                    self._axes = {}
                    self._axis_live = set()
                self._stop.wait(0.4)
                continue
            try:
                self._read_pad_event(devices[0][0], devices[0][1])
            except OSError as exc:
                print(f"Controller-Lesefehler: {exc}")
                with self._lock:
                    self._pad_connected = False
                self._stop.wait(0.4)

    def _read_pad_event(self, path: str, is_usb: bool) -> None:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        grabbed = False
        saw_stick = False
        try:
            fcntl.ioctl(fd, EVIOCGRAB, 1)
            grabbed = True
            print(f"Controller übernommen ({'USB' if is_usb else 'Bluetooth'}): {path}")
            with self._lock:
                self._pad_connected = True
                self._axes = {}
                self._axis_live = set()
                self._buttons = {}
            interesting_abs = {ABS_LX, ABS_LY, ABS_LT, ABS_RT, ABS_GAS, ABS_BRAKE}
            interesting_btn = {BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, BTN_SELECT}
            while not self._stop.is_set():
                ready, _, _ = select.select([fd], [], [], 0.3)
                if not ready:
                    if not os.path.exists(path):
                        raise OSError("Controller getrennt")
                    # Grab ggf. zurückholen, falls Brave ihn klaut
                    try:
                        fcntl.ioctl(fd, EVIOCGRAB, 1)
                        grabbed = True
                    except OSError:
                        pass
                    continue
                try:
                    chunk = os.read(fd, EVENT_STRUCT.size * 64)
                except BlockingIOError:
                    continue
                if not chunk:
                    raise OSError("Controller getrennt")
                for offset in range(0, len(chunk) // EVENT_STRUCT.size * EVENT_STRUCT.size, EVENT_STRUCT.size):
                    _s, _u, ev_type, code, value = EVENT_STRUCT.unpack_from(chunk, offset)
                    with self._lock:
                        if ev_type == EV_ABS and code in interesting_abs:
                            self._axes[code] = value
                            self._axis_live.add(code)
                            if not saw_stick and code in (ABS_LX, ABS_LY) and abs(value) > 4000:
                                saw_stick = True
                                print("Stick-Signal OK — Fahren aktiv.")
                        elif ev_type == EV_KEY and code in interesting_btn:
                            self._buttons[code] = value != 0
                            self._note_edges()
        finally:
            if grabbed:
                try:
                    fcntl.ioctl(fd, EVIOCGRAB, 0)
                except OSError:
                    pass
            os.close(fd)
            with self._lock:
                self._pad_connected = False

    def _keyboard_loop(self) -> None:
        while not self._stop.is_set():
            path = find_keyboard_event()
            if path is None:
                with self._lock:
                    self._keyboard_ok = False
                self._stop.wait(0.5)
                continue
            try:
                self._read_keyboard(path)
            except OSError:
                with self._lock:
                    self._keyboard_ok = False
                self._stop.wait(0.4)

    def _read_keyboard(self, path: str) -> None:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        interesting = {
            KEY_ESC,
            KEY_W,
            KEY_A,
            KEY_S,
            KEY_D,
            KEY_Q,
            KEY_E,
            KEY_V,
            KEY_M,
            KEY_LEFTSHIFT,
            KEY_SPACE,
            KEY_ENTER,
            KEY_UP,
            KEY_DOWN,
            KEY_LEFT,
            KEY_RIGHT,
        }
        try:
            with self._lock:
                self._keyboard_ok = True
            while not self._stop.is_set():
                ready, _, _ = select.select([fd], [], [], 0.3)
                if not ready:
                    if not os.path.exists(path):
                        raise OSError("Tastatur weg")
                    continue
                try:
                    chunk = os.read(fd, EVENT_STRUCT.size * 32)
                except BlockingIOError:
                    continue
                if not chunk:
                    raise OSError("Tastatur weg")
                for offset in range(0, len(chunk) // EVENT_STRUCT.size * EVENT_STRUCT.size, EVENT_STRUCT.size):
                    _s, _u, ev_type, code, value = EVENT_STRUCT.unpack_from(chunk, offset)
                    if ev_type != EV_KEY or code not in interesting:
                        continue
                    pressed = value != 0  # 1 down, 2 repeat
                    with self._lock:
                        self._keys[code] = pressed
                        self._note_edges()
        finally:
            os.close(fd)
            with self._lock:
                self._keyboard_ok = False
                self._keys = {}


def load_session() -> tuple[str, UserData] | None:
    if not SESSION_PATH.is_file():
        return None
    try:
        raw = json.loads(SESSION_PATH.read_text())
        return raw["email"], UserData.from_dict(raw["user_data"])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


def save_session(email: str, user_data: UserData) -> None:
    SESSION_PATH.parent.mkdir(parents=True, exist_ok=True)
    SESSION_PATH.write_text(json.dumps({"email": email, "user_data": user_data.as_dict()}, indent=2))
    SESSION_PATH.chmod(0o600)


async def login(force: bool) -> tuple[str, UserData]:
    existing = None if force else load_session()
    if existing is not None:
        return existing
    email = input("Roborock-E-Mail: ").strip()
    if not email:
        raise SystemExit("Ohne E-Mail-Adresse geht die Anmeldung nicht.")
    client = RoborockApiClient(email)
    print(f"Code wird an {email} geschickt.")
    await client.request_code()
    code = input("Code aus der E-Mail: ").strip()
    user_data = await client.code_login(code)
    save_session(email, user_data)
    print("Anmeldung gespeichert.")
    return email, user_data


def choose_device(devices: list, explicit: str | None):
    vacuums = [device for device in devices if device.v1_properties is not None]
    if explicit:
        for device in vacuums:
            haystack = f"{device.name} {device.product.model}".lower()
            if explicit.lower() in haystack or explicit == device.duid:
                return device
        known = ", ".join(device.name for device in vacuums) or "keine"
        raise SystemExit(f"Kein passender Saugroboter für {explicit!r}. Gefunden: {known}")
    curv = [
        device
        for device in vacuums
        if "curv" in device.name.lower() or "curv" in device.product.model.lower()
    ]
    if len(curv) == 1:
        return curv[0]
    if len(vacuums) == 1:
        return vacuums[0]
    if not vacuums:
        raise SystemExit("Im Konto ist kein steuerbarer Roborock-Saugroboter.")
    print("Mehrere Saugroboter gefunden:")
    for index, device in enumerate(vacuums, start=1):
        print(f"  {index}. {device.name} ({device.product.model})")
    picked = input("Nummer: ").strip()
    try:
        return vacuums[int(picked) - 1]
    except (ValueError, IndexError):
        raise SystemExit("Diese Nummer gibt es nicht.") from None


def motion(stick: Stick) -> tuple[float, float]:
    scale = SLOW_SCALE if stick.slow else 1.0
    velocity = shape(stick.forward) * MAX_VELOCITY * scale
    omega = -shape(stick.turn) * MAX_OMEGA * scale
    velocity = max(-PROTOCOL_VELOCITY, min(PROTOCOL_VELOCITY, velocity))
    omega = max(-PROTOCOL_OMEGA, min(PROTOCOL_OMEGA, omega))
    return velocity, omega


class RemoteDrive:
    def __init__(self, command) -> None:
        self._command = command
        self.seq = 1
        self.nested = True
        self.active = False
        self.vacuum_on = False
        self.mop_on = False
        self._last_rtt = 0.45

    async def start(self) -> None:
        await self._command.send(RoborockCommand.APP_RC_START)
        self.active = True
        self.seq = 1
        await self.apply_vacuum(False)
        await self.apply_mop(False)

    async def apply_vacuum(self, on: bool) -> None:
        code = FAN_ON if on else FAN_OFF
        try:
            await self._command.send(RoborockCommand.SET_CUSTOM_MODE, params=[code])
            self.vacuum_on = on
            if not on:
                try:
                    await self._command.send(RoborockCommand.STOP_FAN_MOTOR_WORK)
                except RoborockException:
                    pass
            print(f"\nSaugen: {'AN' if on else 'AUS'}")
        except RoborockException as exc:
            print(f"\nSaugmodus nicht gesetzt: {exc}")

    async def toggle_vacuum(self) -> None:
        await self.apply_vacuum(not self.vacuum_on)

    async def apply_mop(self, on: bool) -> None:
        code = WATER_ON if on else WATER_OFF
        try:
            await self._command.send(RoborockCommand.SET_WATER_BOX_CUSTOM_MODE, params=[code])
            self.mop_on = on
            print(f"\nWischen: {'AN' if on else 'AUS'}")
        except RoborockException as exc:
            print(f"\nWischmodus nicht gesetzt: {exc}")

    async def toggle_mop(self) -> None:
        await self.apply_mop(not self.mop_on)

    async def move(self, velocity: float, omega: float) -> None:
        duration = int(max(350, min(1600, self._last_rtt * 1000 + 350)))
        payload = {
            "omega": round(omega, 3),
            "velocity": round(velocity, 3),
            "seqnum": self.seq,
            "duration": duration,
        }
        self.seq += 1
        started = time.monotonic()
        try:
            await self._send_move(payload)
        except RoborockException:
            if not self.nested:
                raise
            self.nested = False
            await self._send_move(payload)
        self._last_rtt = time.monotonic() - started

    async def _send_move(self, payload: dict) -> None:
        params = [[payload]] if self.nested else [payload]
        await self._command.send(RoborockCommand.APP_RC_MOVE, params=params)

    async def finish(self, dock: bool) -> None:
        if self.active:
            try:
                await self.apply_vacuum(False)
                await self.apply_mop(False)
            except RoborockException:
                pass
            for command in (RoborockCommand.APP_RC_STOP, RoborockCommand.APP_RC_END):
                try:
                    await self._command.send(command)
                except RoborockException as exc:
                    print(f"\n{command} wurde abgelehnt: {exc}")
            self.active = False
        if dock:
            await self._command.send(RoborockCommand.APP_CHARGE)


async def wait_for_input(hub: InputHub, stop: asyncio.Event) -> bool:
    announced = False
    while not stop.is_set():
        if hub.snapshot(False).connected:
            return True
        if not announced:
            print("Warte auf Controller oder Tastatur …")
            announced = True
        try:
            await asyncio.wait_for(stop.wait(), timeout=0.4)
        except TimeoutError:
            continue
    return False


def status_line(
    velocity: float,
    omega: float,
    slow: bool,
    held: bool,
    vacuum: bool,
    mop: bool,
    source: str,
) -> str:
    pace = "langsam" if slow else "normal"
    if not held:
        pace = "steht"
    modes = f"saugen {'AN' if vacuum else 'AUS'}  wischen {'AN' if mop else 'AUS'}"
    return f"  v {velocity:+.2f}  drehen {omega:+.2f}  {pace}  {modes}  ({source or '-'})    "


async def drive(device, hub: InputHub, invert_turn: bool, stop: asyncio.Event) -> None:
    remote = RemoteDrive(device.v1_properties.command)
    link = "lokal" if device.is_local_connected else "über die Cloud"
    print(f"Verbunden mit {device.name} ({link}).")
    print("Standard: nur fahren — Saugen AUS, Wischen AUS.")
    print("Pad: Stick fahren | X Saugen | View Wischen | A Start | B Programm-Ende | Y zur Station | LB langsam")
    print("Tastatur: WASD fahren | Q/E drehen | V Saugen | M Wischen | Leertaste Start | Esc Programm-Ende")
    print("Y beendet nur die Fernsteuerung und schickt ihn zur Station — danach kannst du wieder übernehmen.")
    moving = False
    dock_latched = False
    try:
        while not stop.is_set():
            stick = hub.snapshot(invert_turn)
            if not stick.connected:
                if remote.active and moving:
                    await remote.move(0.0, 0.0)
                    moving = False
                    print("\nEingabe weg — Roboter stoppt.")
                if not await wait_for_input(hub, stop):
                    break
                print("Eingabe wieder da.")
                continue
            if stick.stop:
                break
            if stick.dock:
                if not dock_latched:
                    dock_latched = True
                    moving = False
                    print()
                    await remote.finish(dock=True)
                    print("Zur Station geschickt. Stick oder A startet die Fernsteuerung wieder.")
                await asyncio.sleep(0.05)
                continue
            dock_latched = False
            if stick.vacuum_toggle or stick.mop_toggle:
                if not remote.active:
                    try:
                        await remote.start()
                        print("\nFernsteuerung an.")
                    except RoborockException as exc:
                        print(f"\nFernsteuerung abgelehnt: {exc}")
                        await asyncio.sleep(0.5)
                        continue
                if stick.vacuum_toggle:
                    await remote.toggle_vacuum()
                if stick.mop_toggle:
                    await remote.toggle_mop()
            velocity, omega = motion(stick)
            wants_move = abs(velocity) > 0.005 or abs(omega) > 0.01
            if not remote.active:
                if wants_move or stick.start:
                    try:
                        await remote.start()
                    except RoborockException as exc:
                        print(f"\nFernsteuerung abgelehnt: {exc}")
                        await asyncio.sleep(1.0)
                        continue
                    print("\nFernsteuerung an.")
                else:
                    await asyncio.sleep(0.05)
                    continue
            if wants_move or moving:
                sent = time.monotonic()
                await remote.move(velocity if wants_move else 0.0, omega if wants_move else 0.0)
                moving = wants_move
                sys.stdout.write(
                    "\r"
                    + status_line(
                        velocity,
                        omega,
                        stick.slow,
                        wants_move,
                        remote.vacuum_on,
                        remote.mop_on,
                        stick.source,
                    )
                )
                sys.stdout.flush()
                remaining = max(0.0, SEND_INTERVAL - (time.monotonic() - sent))
            else:
                remaining = 0.05
            try:
                await asyncio.wait_for(stop.wait(), timeout=remaining)
            except TimeoutError:
                pass
    finally:
        print()
        if remote.active:
            await remote.finish(dock=False)
            print("Fernsteuerung beendet.")


async def async_main(args: argparse.Namespace) -> None:
    email, user_data = await login(args.reauth)
    print("Suche den Saugroboter …")
    manager = await create_device_manager(UserParams(username=email, user_data=user_data))
    hub = InputHub()
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    try:
        devices = await manager.get_devices()
        device = choose_device(devices, args.device)
        deadline = time.monotonic() + 25
        while not device.is_connected and time.monotonic() < deadline and not stop.is_set():
            await asyncio.sleep(0.25)
        if not device.is_connected:
            raise SystemExit(f"{device.name} antwortet nicht. Er muss im selben WLAN online sein.")
        try:
            await device.v1_properties.status.refresh()
            state = device.v1_properties.status.state
            battery = device.v1_properties.status.battery
            print(f"Status: {getattr(state, 'name', state)}, Akku {battery} %.")
        except RoborockException as exc:
            print(f"Status gerade nicht lesbar: {exc}")
        hub.start()
        if not await wait_for_input(hub, stop):
            return
        print("Eingabe bereit (Controller und/oder Tastatur).")
        await drive(device, hub, args.invert_turn, stop)
    finally:
        hub.close()
        await manager.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Roborock Curv per Xbox/Tastatur fahren.")
    parser.add_argument("--device", help="Name oder Teil des Namens.")
    parser.add_argument("--reauth", action="store_true")
    parser.add_argument("--invert-turn", action="store_true")
    parser.add_argument("--debug", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.DEBUG if args.debug else logging.WARNING)
    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        print("\nBeendet.")


if __name__ == "__main__":
    main()

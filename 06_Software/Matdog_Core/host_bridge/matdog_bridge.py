#!/usr/bin/env python3
"""MATDOG host bridge: the single owner of the Controller USB serial port.

One process, Python 3 standard library only:

* owns the ESP32-S3 USB-Serial-JTAG tty (reset-safe open, exclusive, no
  modem-line ioctls, line speed preserved verbatim);
* logs every RX/TX line append-only (0600) with host UTC + monotonic stamps;
* derives one ``/api/motion/status`` object from either the ``@TEL`` stream
  (``@MOTION TELEMETRY <hz>``) or, for firmware without it, read-only legacy
  polling of ``@MOTION VALIDATION SENSORS`` / ``JOINT <n>``;
* serves the built Digital Twin viewer, status, SSE events and health over HTTP
  (same origin, no CORS);
* exposes a loopback-only line console (one writer) that sees every RX line;
* accepts exactly one HTTP write: ``POST /api/bridge/stop``, which writes the
  two de-escalation commands and nothing else.

Safety rationale for the open path (see the ESP32-S3 notes in VALIDATION.md):
the USB-Serial-JTAG peripheral maps DTR/RTS to EN/IO0. pyserial drops DTR while
RTS is still asserted (DTR=0/RTS=1 == EN reset). A plain kernel open raises
both lines together and the final close drops both together, neither of which
is the reset combination, so this module never issues TIOCMBIS/TIOCMBIC/TIOCMSET
and never changes the baud rate (a 1200-baud touch means bootloader entry).

``--fake-device`` runs the same code against an in-process pseudo-terminal
simulator. Every status it produces carries
``telemetry.source = "FAKE_DEVICE_NOT_HARDWARE"`` and ``bridge.fake_device``.
"""

from __future__ import annotations

import argparse
import collections
import datetime as _dt
import errno
import fcntl
import gzip
import http.server
import ipaddress
import json
import math
import mimetypes
import os
import queue
import random
import re
import select
import signal
import socket
import socketserver
import subprocess
import sys
import tempfile
import termios
import threading
import time
import urllib.parse
from typing import Any, Callable, Optional

BRIDGE_NAME = "matdog_bridge"
BRIDGE_VERSION = 1
DEFAULT_PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00"
DEFAULT_HTTP = "0.0.0.0:8765"
DEFAULT_CONSOLE = "127.0.0.1:8766"
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_VIEWER_DIST = os.path.normpath(
    os.path.join(HERE, "..", "viewer", "matdog_bno085_fullbody_viewer", "dist"))

STALE_AFTER_MS = 1500
SETTLE_S = 1.0               # collect/discard after open before the first TX
FLUSH_TO_SUBSCRIBE_S = 0.2   # lone "\n" first, then the first real command
TEL_DETECT_S = 3.0           # no @TEL within this window -> legacy polling
TEL_SILENCE_S = 3.0          # TEL stream silent -> resubscribe once, then legacy
LEGACY_TIMEOUT_S = 1.0       # one command in flight, 1 s timeout
LEGACY_MIN_INTERVAL_S = 0.1  # <= 10 commands/s
CONSOLE_PAUSE_S = 5.0        # legacy polling pauses after any console command
SSE_MIN_INTERVAL_S = 0.05    # <= 20 Hz per client
SSE_HEARTBEAT_S = 2.0
SSE_IDLE_RESEND_S = 1.0      # re-send current (aged) status so stale/bridge state propagates
SSE_MAX_CLIENTS = 16
MAX_LINE_BYTES = 16384
MAX_CONSOLE_LINE = 512
STOP_COMMANDS = ("@MOTION COMMISSIONING STOP", "@MOTION VALIDATION STOP")
FAKE_SOURCE = "FAKE_DEVICE_NOT_HARDWARE"
LEGACY_SOURCE = "USB_SERIAL_LEGACY_POLL"
# Slot order of the Controller arrays: LF, RF, RH, LH x (hip, upper, lower).
JOINT_NAMES = [f"{leg}_{part}_joint" for leg in ("lf", "rf", "rh", "lh")
               for part in ("hip", "upper_leg", "lower_leg")]
# Legacy round robin: SENSORS (quaternion) every four joints.
LEGACY_SEQUENCE = ("S", 0, 1, 2, 3, "S", 4, 5, 6, 7, "S", 8, 9, 10, 11)
MODEM_LINE_IOCTLS = frozenset(
    getattr(termios, name) for name in ("TIOCMBIS", "TIOCMBIC", "TIOCMSET") if hasattr(termios, name))

# Serial states reported to clients.
S_DISCONNECTED = "SERIAL_DISCONNECTED"
S_BUSY = "SERIAL_OWNER_BUSY"
S_ERROR = "SERIAL_ERROR"
S_SETTLING = "SETTLING"
S_CONNECTED = "CONNECTED"


def utc_now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="microseconds").replace("+00:00", "Z")


def utc_stamp() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


# --------------------------------------------------------------- logging ---

class SessionLog:
    """Append-only JSONL log of every serial line and bridge event (mode 0600)."""

    def __init__(self, directory: str):
        self.directory = os.path.abspath(os.path.expanduser(directory))
        if not os.path.isdir(self.directory):
            os.makedirs(self.directory, mode=0o700)
            os.chmod(self.directory, 0o700)
        self.path = os.path.join(self.directory, "bridge_serial.jsonl")
        fd = os.open(self.path, os.O_WRONLY | os.O_APPEND | os.O_CREAT | os.O_CLOEXEC, 0o600)
        os.fchmod(fd, 0o600)
        self._file = os.fdopen(fd, "a", encoding="utf-8", buffering=1)
        self._lock = threading.Lock()
        self.errors = 0
        self.records = 0

    def write(self, direction: str, line: str, **extra: Any) -> None:
        record = {"utc": utc_now(), "mono": round(time.monotonic(), 6), "dir": direction, "line": line}
        record.update(extra)
        text = json.dumps(record, ensure_ascii=False, separators=(",", ":"))
        with self._lock:
            try:
                self._file.write(text + "\n")
                self.records += 1
            except (OSError, ValueError):
                self.errors += 1

    def close(self) -> None:
        with self._lock:
            try:
                self._file.flush()
                os.fsync(self._file.fileno())
                self._file.close()
            except (OSError, ValueError):
                pass


# ------------------------------------------------------- serial ownership ---

class SerialOwnerBusy(Exception):
    """Another process (or bridge) owns the serial device."""


def lock_path_for(device: str, lock_dir: str) -> str:
    return os.path.join(lock_dir, "matdog_bridge_" + re.sub(r"[^A-Za-z0-9_.-]", "_", device) + ".lock")


def acquire_owner_lock(path: str) -> int:
    """Advisory flock on a lock file; held for the whole bridge lifetime."""
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as exc:
        holder = ""
        try:
            holder = os.pread(fd, 64, 0).decode(errors="replace").strip()
        except OSError:
            pass
        os.close(fd)
        raise SerialOwnerBusy(f"owner lock {path} is held (pid {holder or '?'})") from exc
    os.ftruncate(fd, 0)
    os.pwrite(fd, f"{os.getpid()}\n".encode(), 0)
    return fd


def other_openers(device: str) -> list[int]:
    """PIDs (other than ours) that hold the device open, best effort via /proc."""
    try:
        target = os.path.realpath(device)
    except OSError:
        return []
    found: list[int] = []
    own = os.getpid()
    try:
        pids = [int(p) for p in os.listdir("/proc") if p.isdigit()]
    except OSError:
        return []
    for pid in pids:
        if pid == own:
            continue
        fd_dir = f"/proc/{pid}/fd"
        try:
            entries = os.listdir(fd_dir)
        except OSError:
            continue
        for entry in entries:
            try:
                if os.readlink(os.path.join(fd_dir, entry)) == target:
                    found.append(pid)
                    break
            except OSError:
                continue
    return found


def make_raw(attrs: list) -> list:
    """cfmakeraw() equivalent that preserves ispeed/ospeed (and CBAUD) verbatim."""
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP | termios.INLCR |
               termios.IGNCR | termios.ICRNL | termios.IXON | termios.IXOFF | termios.IXANY | termios.INPCK)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHOE | termios.ECHOK | termios.ECHONL | termios.ICANON |
               termios.ISIG | termios.IEXTEN)
    cflag = (cflag & ~(termios.CSIZE | termios.PARENB)) | termios.CS8 | termios.CREAD
    cc = list(cc)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    return [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]


def safe_open(path: str) -> int:
    """Reset-safe exclusive open. Never touches DTR/RTS, never changes the line speed."""
    try:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
    except OSError as exc:
        if exc.errno == errno.EBUSY:
            raise SerialOwnerBusy(f"{path}: exclusive (TIOCEXCL) owner present") from exc
        raise
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        before = termios.tcgetattr(fd)
        termios.tcsetattr(fd, termios.TCSANOW, make_raw(before))
        after = termios.tcgetattr(fd)
        if after[4] != before[4] or after[5] != before[5]:
            raise OSError(errno.EIO, "line speed changed during raw-mode setup")
    except BaseException:
        os.close(fd)
        raise
    return fd


class SerialLink:
    """Owns the tty: open/reopen loop, line assembly, serialized whole-line writes."""

    def __init__(self, path: str, lock_dir: str, log: SessionLog,
                 on_open: Callable[[float], None], on_line: Callable[[str, float], None],
                 on_close: Callable[[float, str, str], None], reopen_s: float = 1.0):
        self.path = path
        self.lock_path = lock_path_for(path, lock_dir)
        self.log = log
        self.on_open, self.on_line, self.on_close = on_open, on_line, on_close
        self.reopen_s = reopen_s
        self.fd: Optional[int] = None
        self.lock_fd: Optional[int] = None
        self.state = S_DISCONNECTED
        self.detail = ""
        self.opens = 0
        self.rx_lines = 0
        self.rx_bytes = 0
        self.tx_lines = 0
        self.overflows = 0
        self._write_lock = threading.Lock()
        self._fd_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="serial-link", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)
        self._close("bridge shutdown", S_DISCONNECTED, notify=False)
        if self.lock_fd is not None:
            try:
                os.close(self.lock_fd)
            except OSError:
                pass
            self.lock_fd = None

    def _set_state(self, state: str, detail: str) -> None:
        if state != self.state or detail != self.detail:
            self.log.write("EVT", f"serial {state}", detail=detail, path=self.path)
        self.state, self.detail = state, detail

    def _try_open(self) -> bool:
        try:
            if self.lock_fd is None:
                self.lock_fd = acquire_owner_lock(self.lock_path)
            if not os.path.exists(self.path):
                self._set_state(S_DISCONNECTED, "device path absent")
                return False
            others = other_openers(self.path)
            if others:
                raise SerialOwnerBusy(f"{self.path} is held open by pid {', '.join(map(str, others))}")
            fd = safe_open(self.path)
        except SerialOwnerBusy as exc:
            self._set_state(S_BUSY, str(exc))
            return False
        except OSError as exc:
            state = S_DISCONNECTED if exc.errno in (errno.ENOENT, errno.ENODEV, errno.ENXIO) else S_ERROR
            self._set_state(state, f"{exc.__class__.__name__}: {exc}")
            return False
        with self._fd_lock:
            self.fd = fd
        self.opens += 1
        self._set_state(S_CONNECTED, "owned (TIOCEXCL + flock); raw mode, speed preserved")
        self.on_open(time.monotonic())
        return True

    def _close(self, reason: str, state: str, notify: bool = True) -> None:
        with self._fd_lock:
            fd, self.fd = self.fd, None
        if fd is not None:
            try:
                os.close(fd)  # plain close: no modem-line ioctl
            except OSError:
                pass
            self._set_state(state, reason)
            if notify:
                self.on_close(time.monotonic(), state, reason)

    def _run(self) -> None:
        buffer = bytearray()
        while not self._stop.is_set():
            if self.fd is None:
                buffer.clear()
                if not self._try_open():
                    self._stop.wait(self.reopen_s)
                    continue
            fd = self.fd
            if fd is None:
                continue
            try:
                readable, _, broken = select.select([fd], [], [fd], 0.1)
                if not readable and not broken:
                    continue
                chunk = os.read(fd, 65536)
                if not chunk:
                    raise OSError(errno.EIO, "end of file (device hung up)")
            except (OSError, ValueError) as exc:
                self._close(f"device lost: {exc}", S_DISCONNECTED)
                continue
            self.rx_bytes += len(chunk)
            buffer += chunk
            while True:
                index = buffer.find(b"\n")
                if index < 0:
                    break
                raw = bytes(buffer[:index]).rstrip(b"\r")
                del buffer[:index + 1]
                self.rx_lines += 1
                self.on_line(raw.decode("utf-8", errors="replace"), time.monotonic())
            if len(buffer) > MAX_LINE_BYTES:
                self.overflows += 1
                self.log.write("EVT", "rx line overflow discarded", bytes=len(buffer))
                buffer.clear()

    def write_line(self, text: str, origin: str) -> None:
        """Writes one complete line; concurrent writers never interleave mid-line."""
        data = (text + "\n").encode("utf-8")
        with self._write_lock:
            fd = self.fd
            if fd is None:
                raise OSError(errno.ENODEV, f"serial {self.state}")
            view = memoryview(data)
            deadline = time.monotonic() + 1.0
            try:
                while view:
                    try:
                        written = os.write(fd, view)
                        view = view[written:]
                    except BlockingIOError:
                        if time.monotonic() > deadline:
                            raise OSError(errno.ETIMEDOUT, "serial write timed out")
                        select.select([], [fd], [], 0.05)
            except OSError as exc:
                self.log.write("TX_FAIL", text, origin=origin, error=str(exc))
                raise
            self.tx_lines += 1
            self.log.write("TX", text, origin=origin)


# ------------------------------------------------------- protocol engine ---

_KV = re.compile(r"(\w+)=(\S+)")


def parse_kv(text: str) -> dict[str, str]:
    return {key: value for key, value in _KV.findall(text)}


def _int(value: Optional[str]) -> Optional[int]:
    try:
        return int(value) if value is not None else None
    except ValueError:
        return None


def _float(value: Optional[str]) -> Optional[float]:
    try:
        result = float(value) if value is not None else None
    except ValueError:
        return None
    return result if result is not None and math.isfinite(result) else None


def parse_sensors(line: str) -> Optional[dict]:
    if not line.startswith("SENSORS "):
        return None
    kv = parse_kv(line)
    uptime = _int(kv.get("uptime_ms"))
    if uptime is None:
        return None
    quaternion = None
    parts = kv.get("quaternion", "").split(",")
    if len(parts) == 4:
        values = [_float(p) for p in parts]
        if all(v is not None for v in values) and math.hypot(*values) > 1e-6:  # type: ignore[arg-type]
            quaternion = values
    return {"uptime_ms": uptime, "imu_valid": kv.get("imu_valid") == "1",
            "imu_stamp_ms": _int(kv.get("imu_stamp_ms")), "quaternion": quaternion,
            "power_valid": kv.get("bms_valid") == "1", "power_stamp_ms": _int(kv.get("bms_stamp_ms")),
            "soc_pct": _float(kv.get("soc")), "power_state": kv.get("power"),
            "fault": _int(kv.get("fault")), "boot_safe_off_pending": _int(kv.get("boot_safe_off_pending"))}


def parse_joint(line: str) -> Optional[dict]:
    if not line.startswith("JOINT "):
        return None
    kv = parse_kv(line)
    slot = _int(kv.get("slot"))
    stamp = _int(kv.get("stamp_ms"))
    if slot is None or not 0 <= slot < 12 or stamp is None:
        return None
    mapped = kv.get("mapped_valid") == "1"
    age = _int(kv.get("age_ms")) or 0
    voltage = _int(kv.get("voltage_dv"))
    return {"slot": slot, "id": _int(kv.get("id")), "unit": kv.get("configured_unit"),
            "raw": _int(kv.get("raw")), "mapped": mapped, "torque": _int(kv.get("torque")),
            "speed": _int(kv.get("speed_raw")), "goal": _int(kv.get("goal")), "output": _int(kv.get("output")),
            "temp": _int(kv.get("temp")), "volt_dv": voltage, "status": _int(kv.get("status")),
            "stamp_ms": stamp, "reply_age_ms": age, "controller_now_ms": stamp + age,
            # mapped_valid=0: calibration RESTORE not done; actual is unavailable, never 0.
            "actual_rad": _float(kv.get("actual_rad")) if mapped else None}


class TelemetryEngine:
    """Deterministic protocol state machine; every method takes the host monotonic time."""

    def __init__(self, tel_hz: int = 10, fake_device: bool = False, legacy_poll: bool = True,
                 settle_s: float = SETTLE_S, detect_s: float = TEL_DETECT_S,
                 silence_s: float = TEL_SILENCE_S, legacy_timeout_s: float = LEGACY_TIMEOUT_S,
                 legacy_interval_s: float = LEGACY_MIN_INTERVAL_S, console_pause_s: float = CONSOLE_PAUSE_S,
                 log: Optional[SessionLog] = None):
        self.tel_hz = tel_hz
        self.fake_device = fake_device
        self.legacy_poll = legacy_poll
        self.settle_s, self.detect_s, self.silence_s = settle_s, detect_s, silence_s
        self.legacy_timeout_s, self.legacy_interval_s = legacy_timeout_s, legacy_interval_s
        self.console_pause_s = console_pause_s
        self.log = log
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self.version = 0
        self.serial_state = S_DISCONNECTED
        self.serial_detail = ""
        self.counters = collections.Counter()
        self.last_error = ""
        self._rx_times: collections.deque = collections.deque()
        self._tel_times: collections.deque = collections.deque()
        self.recent_rx: collections.deque = collections.deque(maxlen=400)
        self.rx_seq = 0
        self.console_last_tx = -1e9
        self._reset_link_state(None)
        self._reset_controller_state()

    # -- state resets --
    def _reset_link_state(self, opened_at: Optional[float]) -> None:
        self.phase = "CLOSED" if opened_at is None else "SETTLING"
        self.opened_at = opened_at
        self.first_tx_done = False
        self.phase_at = opened_at or 0.0
        self.mode = "NONE"
        self.legacy_known = False
        self.last_tel = -1e9
        self.resubscribe_pending = False
        self.poll_inflight: Optional[tuple] = None
        self.poll_sent_at = -1e9
        self.poll_index = 0
        self.legacy_cycle_started = None
        self.legacy_cycle_ms: Optional[float] = None

    def _reset_controller_state(self) -> None:
        self.status: Optional[dict] = None
        self.last_sample_mono: Optional[float] = None
        self.last_sample_epoch_ms: Optional[int] = None
        self.boot_nonce: Optional[int] = None
        self.ctrl_now_ms: Optional[int] = None
        self.sensors: Optional[dict] = None
        self.joints: list[Optional[dict]] = [None] * 12

    def _evt(self, text: str, **extra: Any) -> None:
        if self.log:
            self.log.write("EVT", text, **extra)

    def _publish(self) -> None:
        self.version += 1
        self.changed.notify_all()

    # -- link events --
    def link_opened(self, now: float) -> None:
        with self.lock:
            self._reset_link_state(now)
            self.serial_state, self.serial_detail = S_SETTLING, "collecting stale ring content before first TX"
            self._publish()

    def link_closed(self, now: float, state: str, detail: str) -> None:
        with self.lock:
            self._reset_link_state(None)
            self.serial_state, self.serial_detail = state, detail
            self.counters["serial_disconnects"] += 1
            self._publish()

    def link_state(self, state: str, detail: str) -> None:
        with self.lock:
            if self.phase == "CLOSED" and (state, detail) != (self.serial_state, self.serial_detail):
                self.serial_state, self.serial_detail = state, detail
                self._publish()

    def note_tx(self, now: float, origin: str) -> None:
        with self.lock:
            if not self.first_tx_done:
                self.first_tx_done = True
                if self.serial_state == S_SETTLING:
                    self.serial_state, self.serial_detail = S_CONNECTED, "owned; first TX done"
                self._publish()
            if origin == "console":
                self.console_last_tx = now

    # -- RX --
    def line(self, text: str, now: float) -> bool:
        """Returns True when the line was treated as fresh (after the first TX)."""
        with self.lock:
            self.rx_seq += 1
            if not self.first_tx_done:
                # Stale HWCDC ring content: logged by the caller, never parsed or timed.
                self.counters["rx_pre_tx_lines"] += 1
                return False
            self.counters["rx_lines"] += 1
            self.recent_rx.append((self.rx_seq, now, text))
            self._rx_times.append(now)
            if text.startswith("@TEL "):
                self._handle_tel(text[5:], now)
            elif text.startswith("SENSORS "):
                self._handle_sensors(text, now)
            elif text.startswith("JOINT "):
                self._handle_joint(text, now)
            return True

    def _detect_reboot(self, now: float, why: str) -> None:
        self.counters["reboots_seen"] += 1
        self._evt("controller reboot detected", reason=why)
        self._reset_controller_state()
        self.resubscribe_pending = True
        self.poll_inflight = None

    def _handle_tel(self, payload: str, now: float) -> None:
        try:
            obj = json.loads(payload)
        except ValueError as exc:
            self.counters["parse_errors"] += 1
            self.last_error = f"@TEL JSON: {exc}"
            return
        if not isinstance(obj, dict) or not isinstance(obj.get("now_ms"), (int, float)):
            self.counters["parse_errors"] += 1
            self.last_error = "@TEL object without numeric now_ms"
            return
        nonce = obj.get("boot_nonce") if isinstance(obj.get("boot_nonce"), int) else None
        now_ms = int(obj["now_ms"])
        if self.status is not None:
            if nonce is not None and self.boot_nonce is not None and nonce != self.boot_nonce:
                self._detect_reboot(now, f"boot_nonce {self.boot_nonce} -> {nonce}")
            elif self.ctrl_now_ms is not None and now_ms < self.ctrl_now_ms:
                self._detect_reboot(now, f"now_ms went backwards {self.ctrl_now_ms} -> {now_ms}")
        self.boot_nonce = nonce
        self.ctrl_now_ms = now_ms
        self._tel_times.append(now)
        self.counters["tel_lines"] += 1
        self.last_tel = now
        if self.mode != "TEL":
            self._evt("telemetry source TEL", previous=self.mode)
        self.mode, self.phase, self.poll_inflight = "TEL", "TEL", None
        self.status = obj
        self._sampled(now)

    def _controller_clock(self, observed_ms: int, now: float, what: str) -> None:
        if self.ctrl_now_ms is not None and observed_ms + 250 < self.ctrl_now_ms:
            self._detect_reboot(now, f"{what} went backwards {self.ctrl_now_ms} -> {observed_ms}")
        self.ctrl_now_ms = max(observed_ms, self.ctrl_now_ms or 0)

    def _handle_sensors(self, text: str, now: float) -> None:
        record = parse_sensors(text)
        if record is None:
            self.counters["parse_errors"] += 1
            self.last_error = "malformed SENSORS line"
            return
        self._controller_clock(record["uptime_ms"], now, "uptime_ms")
        self.sensors = record
        if self.poll_inflight == ("S",):
            self.poll_inflight = None
        self.counters["legacy_sensors"] += 1
        if self.mode != "TEL":
            self._synthesize(now)

    def _handle_joint(self, text: str, now: float) -> None:
        record = parse_joint(text)
        if record is None:
            self.counters["parse_errors"] += 1
            self.last_error = "malformed JOINT line"
            return
        self._controller_clock(record["controller_now_ms"], now, "joint stamp")
        self.joints[record["slot"]] = record
        if self.poll_inflight == ("J", record["slot"]):
            self.poll_inflight = None
        self.counters["legacy_joints"] += 1
        if self.mode != "TEL":
            self._synthesize(now)

    def _synthesize(self, now: float) -> None:
        """Builds the /api/motion/status equivalent from read-only legacy replies."""
        now_ms = self.ctrl_now_ms or 0
        sensors = self.sensors
        joints = self.joints
        stamps = [j["stamp_ms"] for j in joints if j]
        if sensors and sensors.get("imu_stamp_ms") is not None:
            stamps.append(sensors["imu_stamp_ms"])
        if sensors:
            stamps.append(sensors["uptime_ms"])
        sampled = min(max(stamps), now_ms) if stamps else now_ms
        complete = all(j is not None for j in joints)
        mapped_all = complete and all(j["mapped"] for j in joints)  # type: ignore[index]
        any_unmapped = any(j is not None and not j["mapped"] for j in joints)
        actual = [j["actual_rad"] for j in joints] if mapped_all else None  # type: ignore[index]
        raw = [j["raw"] for j in joints] if complete else None  # type: ignore[index]
        restore: Optional[bool] = False if any_unmapped else (True if mapped_all else None)
        servos = []
        for slot, j in enumerate(joints):
            if j is None:
                servos.append({"slot": slot, "id": None, "raw": None, "mapped": None, "torque": None,
                               "speed": None, "goal": None, "temp": None, "volt_dv": None, "status": None,
                               "age_ms": None, "actual_rad": None, "stamp_ms": None})
            else:
                servos.append({"slot": slot, "id": j["id"], "raw": j["raw"], "mapped": j["mapped"],
                               "torque": j["torque"], "speed": j["speed"], "goal": j["goal"], "temp": j["temp"],
                               "volt_dv": j["volt_dv"], "status": j["status"],
                               "age_ms": max(0, now_ms - j["stamp_ms"]), "actual_rad": j["actual_rad"],
                               "stamp_ms": j["stamp_ms"]})
        self.status = {
            "version": 1, "now_ms": now_ms, "last_sequence": None, "owner": "unknown",
            "executor_state": "UNKNOWN", "stand_state": "UNKNOWN", "fault": "UNKNOWN",
            "reason": "RESTORE_REQUIRED" if restore is False else "LEGACY_POLL_NO_MOTION_STATUS",
            "stale": False, "tracking_error_rad": None, "completed_sequence": None,
            "telemetry": {"sampled_at_ms": sampled, "actual_rad": actual,
                          "joint_at_ms": [j["stamp_ms"] if j else None for j in joints],
                          "raw_ticks": raw, "target_rad": None,
                          "quaternion": sensors["quaternion"] if sensors and sensors["imu_valid"] else None,
                          "base_height_m": None, "source": LEGACY_SOURCE, "feet": None},
            "active_command": None,
            "readiness": {"restore": restore, "calibration_accepted": None, "limits_approved": None,
                          "permit": None, "support_observable": None, "hardware_approved": None},
            "local_session": None, "validation": None,
            "servos": servos,
            "imu": {"valid": bool(sensors and sensors["imu_valid"]),
                    "stamp_ms": sensors["imu_stamp_ms"] if sensors else None} if sensors else None,
            "power": {"state": sensors["power_state"], "fault": sensors["fault"], "soc_pct": sensors["soc_pct"],
                      "valid": sensors["power_valid"], "stamp_ms": sensors["power_stamp_ms"],
                      "boot_safe_off_pending": sensors["boot_safe_off_pending"]} if sensors else None,
            "legacy": {"joints_seen": sum(1 for j in joints if j), "cycle_ms": self.legacy_cycle_ms,
                       "poll_timeouts": self.counters["legacy_timeouts"]},
        }
        self._sampled(now)

    def _sampled(self, now: float) -> None:
        self.last_sample_mono = now
        self.last_sample_epoch_ms = int(time.time() * 1000)
        self._publish()

    # -- scheduling --
    def tick(self, now: float) -> list[str]:
        """Returns bridge-originated lines to write now (at most one poll in flight)."""
        with self.lock:
            out: list[str] = []
            phase = self.phase
            subscribe = f"@MOTION TELEMETRY {self.tel_hz}"
            if phase == "CLOSED":
                pass
            elif phase == "SETTLING":
                if now - self.phase_at >= self.settle_s:
                    out.append("")  # lone "\n" flushes the Controller line buffer
                    self.phase, self.phase_at = "FLUSHING", now
            elif phase == "FLUSHING":
                if now - self.phase_at >= FLUSH_TO_SUBSCRIBE_S:
                    out.append(subscribe)
                    self.phase, self.phase_at = "DETECTING", now
                    self.resubscribe_pending = False
            elif self.resubscribe_pending:
                self.resubscribe_pending = False
                out.append(subscribe)
                if self.mode != "TEL":
                    self.phase, self.phase_at = "DETECTING", now
            elif phase == "DETECTING":
                if now - self.phase_at >= self.detect_s:
                    self.phase, self.phase_at = "LEGACY_POLL", now
                    if self.mode != "LEGACY_POLL":
                        self._evt("no @TEL within detect window; legacy read-only polling")
                    self.mode = "LEGACY_POLL" if self.legacy_poll else "NONE"
                    self.legacy_known = True
                    self._publish()
            elif phase == "TEL":
                if now - self.last_tel >= self.silence_s:
                    self._evt("@TEL silent; resubscribing once")
                    out.append(subscribe)
                    self.phase, self.phase_at = "DETECTING", now
                    self.legacy_known = True
            if (self.phase == "LEGACY_POLL" or (self.phase == "DETECTING" and self.legacy_known)) \
                    and self.legacy_poll and self.mode != "TEL":
                command = self._next_poll(now)
                if command:
                    out.append(command)
            return out

    def _next_poll(self, now: float) -> Optional[str]:
        if now - self.console_last_tx < self.console_pause_s:
            return None
        if self.poll_inflight is not None:
            if now - self.poll_sent_at < self.legacy_timeout_s:
                return None
            self.counters["legacy_timeouts"] += 1
            self.poll_inflight = None
        if now - self.poll_sent_at < self.legacy_interval_s:
            return None
        item = LEGACY_SEQUENCE[self.poll_index]
        if self.poll_index == 0:
            if self.legacy_cycle_started is not None:
                self.legacy_cycle_ms = round((now - self.legacy_cycle_started) * 1000.0, 1)
            self.legacy_cycle_started = now
        self.poll_index = (self.poll_index + 1) % len(LEGACY_SEQUENCE)
        self.poll_sent_at = now
        if item == "S":
            self.poll_inflight = ("S",)
            return "@MOTION VALIDATION SENSORS"
        self.poll_inflight = ("J", item)
        return f"@MOTION VALIDATION JOINT {item}"

    # -- outputs --
    def _rate(self, times: collections.deque, now: float, window: float = 5.0) -> float:
        while times and now - times[0] > window:
            times.popleft()
        if len(times) < 2:
            return 0.0
        span = max(now - times[0], 1e-3)
        return round((len(times) - 1) / span, 2) if now - times[-1] < 2.0 else 0.0

    def bridge_info(self, now: float, serial_path: str = "") -> dict:
        age = None if self.last_sample_mono is None else int((now - self.last_sample_mono) * 1000)
        return {"name": BRIDGE_NAME, "bridge_version": BRIDGE_VERSION, "source": self.mode,
                "serial_state": self.serial_state, "serial_detail": self.serial_detail,
                "serial_path": serial_path, "phase": self.phase,
                "host_received_ms": self.last_sample_epoch_ms, "age_ms": age, "boot_nonce": self.boot_nonce,
                "reboots_seen": self.counters["reboots_seen"],
                "lines_per_s": self._rate(self._rx_times, now), "tel_hz": self._rate(self._tel_times, now),
                "fake_device": self.fake_device,
                "console_active": now - self.console_last_tx < self.console_pause_s,
                "legacy_cycle_ms": self.legacy_cycle_ms}

    def is_stale(self, now: float) -> bool:
        if self.status is None or self.last_sample_mono is None:
            return True
        return (bool(self.status.get("stale")) or (now - self.last_sample_mono) * 1000 > STALE_AFTER_MS
                or self.serial_state not in (S_CONNECTED, S_SETTLING))

    def status_payload(self, now: float, serial_path: str = "") -> dict:
        with self.lock:
            if self.status is None:
                payload: dict = {"version": 1, "available": False, "stale": True}
            else:
                payload = dict(self.status)
                payload["available"] = True
                payload["stale"] = self.is_stale(now)
                if self.fake_device:
                    telemetry = dict(payload.get("telemetry") or {})
                    telemetry["source"] = FAKE_SOURCE
                    payload["telemetry"] = telemetry
            payload["bridge"] = self.bridge_info(now, serial_path)
            return payload


# --------------------------------------------------------------- bridge ---

class Bridge:
    """Wires the serial link, the engine, the console fan-out and the session log."""

    def __init__(self, port: str, log: SessionLog, lock_dir: str, tel_hz: int = 10,
                 fake_device: bool = False, legacy_poll: bool = True, engine_kwargs: Optional[dict] = None):
        self.port = port
        self.log = log
        self.engine = TelemetryEngine(tel_hz=tel_hz, fake_device=fake_device, legacy_poll=legacy_poll,
                                      log=log, **(engine_kwargs or {}))
        self.link = SerialLink(port, lock_dir, log, self._on_open, self._on_line, self._on_close)
        self.console: Optional["ConsoleClient"] = None
        self.console_lock = threading.Lock()
        self._stop = threading.Event()
        self._tick_thread: Optional[threading.Thread] = None
        self.started_mono = time.monotonic()

    # link callbacks (reader thread)
    def _on_open(self, now: float) -> None:
        self.engine.link_opened(now)
        self._console_note(f"#BRIDGE SERIAL_OPEN {self.port} (settling {self.engine.settle_s:.1f}s)")

    def _on_close(self, now: float, state: str, detail: str) -> None:
        self.engine.link_closed(now, state, detail)
        self._console_note(f"#BRIDGE {state} {detail}")

    def _on_line(self, text: str, now: float) -> None:
        fresh = self.engine.line(text, now)
        self.log.write("RX" if fresh else "RX_PRE_TX", text)
        console = self.console
        if console:
            console.send(text if fresh else f"#PRE_TX {text}")

    def _console_note(self, text: str) -> None:
        self.log.write("EVT", text)
        console = self.console
        if console:
            console.send(text)

    def start(self) -> None:
        self.link.start()
        self._tick_thread = threading.Thread(target=self._tick_loop, name="engine-tick", daemon=True)
        self._tick_thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._tick_thread:
            self._tick_thread.join(timeout=2)
        self.link.stop()
        with self.engine.lock:
            self.engine.changed.notify_all()

    def _tick_loop(self) -> None:
        while not self._stop.wait(0.02):
            if self.engine.phase == "CLOSED":
                self.engine.link_state(self.link.state, self.link.detail)
            for line in self.engine.tick(time.monotonic()):
                try:
                    self.write(line, "bridge")
                except OSError as exc:
                    self.engine.last_error = f"bridge write failed: {exc}"

    def write(self, text: str, origin: str) -> None:
        self.link.write_line(text, origin)
        self.engine.note_tx(time.monotonic(), origin)

    def console_write(self, text: str) -> None:
        if not self.engine.first_tx_done:
            deadline = time.monotonic() + 3.0
            while not self.engine.first_tx_done and time.monotonic() < deadline and self.link.fd is not None:
                time.sleep(0.05)
        self.write(text, "console")

    def request_stop(self, origin: str = "http") -> dict:
        written: list[str] = []
        error = ""
        mark = self.engine.rx_seq
        try:
            if not self.engine.first_tx_done:
                self.write("", origin)
            for command in STOP_COMMANDS:
                self.write(command, origin)
                written.append(command)
        except OSError as exc:
            error = str(exc)
        replies: list[str] = []
        if written:
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                with self.engine.lock:
                    replies = [t for seq, _, t in self.engine.recent_rx
                               if seq > mark and ("STOP" in t or t.startswith(("VALIDATION=", "COMMISSIONING")))]
                if len(replies) >= len(STOP_COMMANDS):
                    break
                time.sleep(0.05)
        self.log.write("EVT", "stop request", origin=origin, written=written, error=error, replies=replies)
        self._console_note(f"#BRIDGE STOP via {origin}: wrote {len(written)} line(s)")
        return {"ok": len(written) == len(STOP_COMMANDS), "written": written, "replies": replies,
                "error": error, "serial_state": self.engine.serial_state, "host_utc": utc_now(),
                "note": "De-escalation request only. Controller replies are listed when they arrived within "
                        "0.6 s; this is not a power cut and does not replace the physical stop."}

    def status_payload(self) -> dict:
        return self.engine.status_payload(time.monotonic(), self.port)

    def health(self) -> dict:
        now = time.monotonic()
        engine = self.engine
        with engine.lock:
            counters = dict(engine.counters)
            info = engine.bridge_info(now, self.port)
            last_error = engine.last_error
        return {"name": BRIDGE_NAME, "bridge_version": BRIDGE_VERSION, "ok": True,
                "fake_device": engine.fake_device,
                "serial": {"path": self.port, "state": engine.serial_state, "detail": engine.serial_detail,
                           "owner_lock": self.link.lock_path, "owner_lock_held": self.link.lock_fd is not None,
                           "opens": self.link.opens, "rx_bytes": self.link.rx_bytes,
                           "tx_lines": self.link.tx_lines, "overflows": self.link.overflows},
                "mode": engine.mode, "phase": engine.phase, "tel_hz_requested": engine.tel_hz,
                "bridge": info,
                "counters": {"rx_lines": counters.get("rx_lines", 0),
                             "rx_pre_tx_lines": counters.get("rx_pre_tx_lines", 0),
                             "tel_lines": counters.get("tel_lines", 0),
                             "parse_errors": counters.get("parse_errors", 0),
                             "reboots_seen": counters.get("reboots_seen", 0),
                             "legacy_sensors": counters.get("legacy_sensors", 0),
                             "legacy_joints": counters.get("legacy_joints", 0),
                             "legacy_timeouts": counters.get("legacy_timeouts", 0),
                             "serial_disconnects": counters.get("serial_disconnects", 0)},
                "console": {"attached": self.console is not None},
                "log": {"path": self.log.path, "records": self.log.records, "errors": self.log.errors},
                "last_error": last_error, "uptime_s": round(now - self.started_mono, 1)}

    # console ownership
    def claim_console(self, client: "ConsoleClient") -> bool:
        with self.console_lock:
            if self.console is not None:
                return False
            self.console = client
            return True

    def release_console(self, client: "ConsoleClient") -> None:
        with self.console_lock:
            if self.console is client:
                self.console = None


# -------------------------------------------------------------- console ---

def is_loopback(host: str) -> bool:
    try:
        return ipaddress.ip_address(host.split("%")[0]).is_loopback
    except ValueError:
        return False


class ConsoleClient:
    """Bounded outbound queue so a slow console never blocks the serial reader."""

    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.queue: "queue.Queue[Optional[str]]" = queue.Queue(maxsize=5000)
        self.dropped = 0
        self.alive = True
        self.thread = threading.Thread(target=self._drain, name="console-out", daemon=True)
        self.thread.start()

    def send(self, line: str) -> None:
        if not self.alive:
            return
        try:
            self.queue.put_nowait(line)
        except queue.Full:
            self.dropped += 1

    def _drain(self) -> None:
        while self.alive:
            line = self.queue.get()
            if line is None:
                break
            try:
                self.sock.sendall((line + "\n").encode("utf-8", errors="replace"))
            except OSError:
                self.alive = False

    def close(self) -> None:
        self.alive = False
        try:
            self.queue.put_nowait(None)
        except queue.Full:
            pass


class ConsoleServer(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], bridge: Bridge):
        if not is_loopback(address[0]):
            raise ValueError(f"console must bind a loopback address, not {address[0]}")
        self.bridge = bridge
        self.rejected_peers = 0
        super().__init__(address, ConsoleHandler)

    def verify_request(self, request, client_address) -> bool:  # type: ignore[override]
        if not is_loopback(client_address[0]):
            self.rejected_peers += 1
            self.bridge.log.write("EVT", "console peer rejected (not loopback)", peer=client_address[0])
            return False
        return True


class ConsoleHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        server: ConsoleServer = self.server  # type: ignore[assignment]
        bridge = server.bridge
        if not is_loopback(self.client_address[0]):  # defence in depth
            return
        client = ConsoleClient(self.connection)
        if not bridge.claim_console(client):
            client.send("#BRIDGE CONSOLE_BUSY another console client is attached (one writer at a time)")
            time.sleep(0.2)
            client.close()
            return
        bridge.log.write("EVT", "console attached", peer=self.client_address[0])
        state = bridge.engine.serial_state
        client.send(f"#BRIDGE CONSOLE_ATTACHED serial={bridge.port} state={state} mode={bridge.engine.mode}")
        try:
            for raw in self.rfile:
                text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                if len(text) > MAX_CONSOLE_LINE or "\n" in text or "\r" in text:
                    client.send("#BRIDGE CONSOLE_REJECTED line too long")
                    continue
                try:
                    bridge.console_write(text)
                except OSError as exc:
                    client.send(f"#BRIDGE CONSOLE_WRITE_FAILED {exc}")
        except OSError:
            pass
        finally:
            bridge.release_console(client)
            client.close()
            bridge.log.write("EVT", "console detached", peer=self.client_address[0])


# ----------------------------------------------------------------- http ---

mimetypes.add_type("model/stl", ".stl")
mimetypes.add_type("application/xml", ".urdf")
mimetypes.add_type("application/x-ndjson", ".jsonl")
mimetypes.add_type("text/javascript", ".js")
COMPRESSIBLE = {".js", ".css", ".html", ".stl", ".urdf", ".json", ".jsonl", ".svg", ".map"}


class BridgeHTTPServer(http.server.ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], bridge: Bridge, viewer_dist: str):
        self.bridge = bridge
        self.viewer_dist = os.path.realpath(viewer_dist)
        self.stopping = threading.Event()
        self.sse_clients = 0
        self.sse_lock = threading.Lock()
        self.gzip_cache: dict[tuple[str, float], bytes] = {}
        super().__init__(address, BridgeHandler)


class BridgeHandler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "matdog_bridge/1"
    server: BridgeHTTPServer

    def log_message(self, fmt: str, *args: Any) -> None:  # quiet: the serial log is the record
        pass

    def _send(self, code: int, body: bytes, content_type: str, extra: Optional[dict] = None,
              api: bool = True) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        if api:
            self.send_header("Cache-Control", "no-store")
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, code: int, obj: Any) -> None:
        self._send(code, json.dumps(obj, separators=(",", ":")).encode(), "application/json")

    def do_GET(self) -> None:  # noqa: N802
        path = urllib.parse.urlsplit(self.path).path
        if path == "/api/motion/status":
            self._json(200, self.server.bridge.status_payload())
        elif path == "/api/bridge/health":
            self._json(200, self.server.bridge.health())
        elif path == "/api/bridge/events":
            self._events()
        elif path.startswith("/api/"):
            self._json(404, {"error": "NOT_FOUND"})
        else:
            self._static(path)

    def do_POST(self) -> None:  # noqa: N802
        path = urllib.parse.urlsplit(self.path).path
        length = _int(self.headers.get("Content-Length")) or 0
        if length > 4096:
            self._json(413, {"error": "BODY_TOO_LARGE"})
            return
        if length:
            self.rfile.read(length)
        if path != "/api/bridge/stop":
            self._json(405 if path.startswith("/api/") else 404, {"error": "NO_WRITE_ROUTE"})
            return
        origin = self.headers.get("Origin")
        if origin is not None:
            host = self.headers.get("Host", "")
            parsed = urllib.parse.urlsplit(origin)
            if parsed.netloc != host:
                self._json(403, {"error": "CROSS_ORIGIN_REJECTED"})
                return
        result = self.server.bridge.request_stop("http")
        self._json(200 if result["ok"] else 503, result)

    def _reject(self) -> None:
        self._json(405, {"error": "METHOD_NOT_ALLOWED"})

    do_PUT = do_DELETE = do_PATCH = do_OPTIONS = _reject  # type: ignore[assignment]

    def _events(self) -> None:
        server = self.server
        with server.sse_lock:
            if server.sse_clients >= SSE_MAX_CLIENTS:
                self._json(503, {"error": "TOO_MANY_EVENT_CLIENTS"})
                return
            server.sse_clients += 1
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.send_header("X-Accel-Buffering", "no")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.close_connection = True
            engine = server.bridge.engine
            self.wfile.write(b"retry: 1500\n\n")
            self.wfile.flush()
            last_version = -1
            last_sent = 0.0
            last_beat = time.monotonic()
            while not server.stopping.is_set():
                with engine.lock:
                    engine.changed.wait_for(lambda: engine.version != last_version or server.stopping.is_set(),
                                            timeout=0.25)
                    version = engine.version
                now = time.monotonic()
                if server.stopping.is_set():
                    break
                if now - last_sent < SSE_MIN_INTERVAL_S:
                    time.sleep(SSE_MIN_INTERVAL_S - (now - last_sent))
                    now = time.monotonic()
                if version != last_version or now - last_sent >= SSE_IDLE_RESEND_S:
                    payload = json.dumps(server.bridge.status_payload(), separators=(",", ":"))
                    self.wfile.write(f"id: {version}\ndata: {payload}\n\n".encode())
                    self.wfile.flush()
                    last_version, last_sent, last_beat = version, now, now
                elif now - last_beat >= SSE_HEARTBEAT_S:
                    self.wfile.write(b": heartbeat\n\n")
                    self.wfile.flush()
                    last_beat = now
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with server.sse_lock:
                server.sse_clients -= 1

    def _static(self, path: str) -> None:
        root = self.server.viewer_dist
        relative = urllib.parse.unquote(path).lstrip("/") or "index.html"
        if relative.endswith("/"):
            relative += "index.html"
        candidate = os.path.realpath(os.path.join(root, relative))
        if not (candidate == root or candidate.startswith(root + os.sep)):
            self._json(404, {"error": "NOT_FOUND"})
            return
        if not os.path.isfile(candidate):
            if relative == "index.html":
                body = (b"<!doctype html><meta charset=utf-8><title>MATDOG bridge</title>"
                        b"<p>Viewer bundle not built. Run <code>npm run build</code> in "
                        b"06_Software/Matdog_Core/viewer/matdog_bno085_fullbody_viewer, or pass --viewer-dist."
                        b"<p>API: <a href=/api/bridge/health>/api/bridge/health</a>")
                self._send(503, body, "text/html; charset=utf-8")
            else:
                self._json(404, {"error": "NOT_FOUND"})
            return
        content_type = mimetypes.guess_type(candidate)[0] or "application/octet-stream"
        if content_type.startswith("text/") or content_type in ("application/xml", "application/json"):
            content_type += "; charset=utf-8"
        stat = os.stat(candidate)
        extension = os.path.splitext(candidate)[1].lower()
        accepts_gzip = "gzip" in (self.headers.get("Accept-Encoding") or "")
        cache = "public, max-age=3600, immutable" if "/assets/" in candidate else "no-cache"
        headers = {"Cache-Control": cache, "Vary": "Accept-Encoding"}
        if accepts_gzip and extension in COMPRESSIBLE:
            key = (candidate, stat.st_mtime)
            body = self.server.gzip_cache.get(key)
            if body is None:
                with open(candidate, "rb") as handle:
                    body = gzip.compress(handle.read(), compresslevel=6, mtime=0)
                self.server.gzip_cache[key] = body
            headers["Content-Encoding"] = "gzip"
        else:
            with open(candidate, "rb") as handle:
                body = handle.read()
        self._send(200, body, content_type, headers, api=False)


# ------------------------------------------------------------ fake device ---

class FakeController:
    """In-process pty simulator for demos/tests ONLY. Never hardware.

    Speaks both firmware dialects: ``protocol="tel"`` answers ``@MOTION TELEMETRY``
    with an ``@TEL`` stream; ``protocol="legacy"`` rejects it and answers only the
    read-only ``@MOTION VALIDATION SENSORS|JOINT n`` commands, like the firmware
    installed on 2026-10-14. Also emulates the HWCDC stale ring at open, a dropped
    first line, IMU stream noise, reboots (new boot nonce) and USB unplug/replug.
    """

    BUS_IDS = [13, 12, 11, 23, 22, 21, 33, 32, 31, 43, 42, 41]

    def __init__(self, protocol: str = "tel", link_dir: Optional[str] = None, drop_first_line: bool = True,
                 unmapped_slots: tuple[int, ...] = (), seed: int = 7):
        if protocol not in ("tel", "legacy"):
            raise ValueError("protocol must be tel or legacy")
        self.protocol = protocol
        self.link_dir = link_dir or tempfile.mkdtemp(prefix="matdog_fake_")
        self.link_path = os.path.join(self.link_dir, "usb-FAKE_DEVICE_NOT_HARDWARE-if00")
        self.drop_first_line = drop_first_line
        self.unmapped_slots = set(unmapped_slots)
        self.random = random.Random(seed)
        self.received: list[str] = []
        self.lock = threading.Lock()
        self.io_lock = threading.Lock()  # fd lifecycle: never use a closed (reusable) fd number
        self.master: Optional[int] = None
        self.slave: Optional[int] = None
        self.dropped = False
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._boot()

    def _boot(self) -> None:
        self.t0 = time.monotonic()
        self.boot_nonce = self.random.randrange(1, 2**32)
        self.tel_period: Optional[float] = None
        self.tel_seq = 0
        self.next_tel = 0.0
        self.next_stream = 0.0

    def uptime_ms(self) -> int:
        return int((time.monotonic() - self.t0) * 1000)

    def plug(self) -> None:
        master, slave = os.openpty()
        os.set_blocking(master, False)
        # The simulated Controller never echoes; keep the pty line discipline out of the way.
        termios.tcsetattr(slave, termios.TCSANOW, make_raw(termios.tcgetattr(slave)))
        with self.io_lock:
            self.master, self.slave = master, slave
        with self.lock:
            self.dropped = False  # the first line written after each (re)open is lost
        tmp = self.link_path + ".tmp"
        if os.path.lexists(tmp):
            os.unlink(tmp)
        os.symlink(os.ttyname(slave), tmp)
        os.replace(tmp, self.link_path)
        # Stale HWCDC ring content, delivered at the next open with fresh host stamps.
        for index in range(3):
            self._emit(f"RV w=1.000000 x=0.000000 y=0.000000 z=0.000000 accuracy_rad=0.1 status=3 count={index} STALE_RING")
        self._emit(f"@TEL {json.dumps({'version': 1, 'now_ms': 1, 'stale_ring': True})}")

    def unplug(self) -> None:
        with self.io_lock:
            for fd in (self.master, self.slave):
                if fd is not None:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
            self.master = self.slave = None
        if os.path.lexists(self.link_path):
            os.unlink(self.link_path)

    def reboot(self) -> None:
        with self.lock:
            self._boot()
        self._emit("MATDOG Controller FAKE_DEVICE_NOT_HARDWARE boot banner")

    def start(self) -> None:
        self.plug()
        self._thread = threading.Thread(target=self._run, name="fake-controller", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self.unplug()
        try:
            os.rmdir(self.link_dir)
        except OSError:
            pass

    def _emit(self, line: str) -> None:
        with self.io_lock:
            if self.master is None:
                return
            try:
                os.write(self.master, (line + "\r\n").encode())
            except OSError:
                pass  # like HWCDC: bytes are dropped when nobody drains the port

    def _pose(self, t: float) -> tuple[list[float], list[float], list[float]]:
        actual, target = [], []
        for slot in range(12):
            leg, part = divmod(slot, 3)
            phase = t * 0.9 + leg * math.pi / 2
            if part == 0:
                q = 0.06 * math.sin(phase)
            elif part == 1:
                q = 1.30 + 0.18 * math.sin(phase)
            else:
                q = -1.40 + 0.12 * math.sin(phase + 0.6)
            actual.append(round(q, 6))
            target.append(round(q + 0.015 * math.sin(phase * 2), 6))
        half = (0.04 * math.sin(t * 0.7) / 2, 0.03 * math.sin(t * 0.5) / 2, 0.10 * math.sin(t * 0.2) / 2)
        cr, sr = math.cos(half[0]), math.sin(half[0])
        cp, sp = math.cos(half[1]), math.sin(half[1])
        cy, sy = math.cos(half[2]), math.sin(half[2])
        quaternion = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                      cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
        return actual, target, [round(v, 9) for v in quaternion]

    def _servo_raw(self, slot: int, q: float) -> int:
        return int(2048 + q * 4096 / (2 * math.pi) * (1 if slot % 2 else -1))

    def tel_object(self) -> dict:
        now = self.uptime_ms()
        actual, target, quaternion = self._pose(now / 1000.0)
        mapped = [slot not in self.unmapped_slots for slot in range(12)]
        all_mapped = all(mapped)
        tracking = max(abs(a - b) for a, b in zip(actual, target))
        self.tel_seq += 1
        servos = [{"slot": s, "id": self.BUS_IDS[s], "raw": self._servo_raw(s, actual[s]), "mapped": mapped[s],
                   "torque": 0, "speed": 0, "goal": 0, "temp": 31 + s % 4, "volt_dv": 118, "status": 0,
                   "age_ms": 20 + 7 * s} for s in range(12)]
        return {
            "version": 1, "now_ms": now, "last_sequence": 0, "owner": "FAKE_DEVICE",
            "executor_state": "OFF", "stand_state": "IDLE", "fault": False,
            "reason": "FAKE_DEVICE_NOT_HARDWARE" if all_mapped else "RESTORE_REQUIRED",
            "stale": False, "tracking_error_rad": round(tracking, 6), "completed_sequence": 0,
            "telemetry": {"sampled_at_ms": max(0, now - 4), "actual_rad": actual if all_mapped else None,
                          "joint_at_ms": [max(0, now - 20 - 7 * s) for s in range(12)],
                          "raw_ticks": [sv["raw"] for sv in servos], "target_rad": target,
                          "quaternion": quaternion, "base_height_m": None, "source": FAKE_SOURCE, "feet": None},
            "active_command": None,
            "readiness": {"restore": all_mapped, "calibration_accepted": all_mapped, "limits_approved": False,
                          "permit": False, "support_observable": False, "hardware_approved": False},
            "local_session": {"state": "OFF", "reason": "FAKE_DEVICE_NOT_HARDWARE"},
            "validation": {"capable": False, "scope": "UNQUALIFIED", "segment": 0, "profile_sha256": "",
                           "foot_load_proven": False},
            "tel": 1, "boot_nonce": self.boot_nonce, "tel_seq": self.tel_seq, "servos": servos,
            "imu": {"valid": True, "stamp_ms": max(0, now - 3)},
            "commissioning": {"mode": "DIAGNOSTIC", "pose": "PRONE_COMPATIBLE", "pose_max_dev_rad": 0.0412,
                              "pose_joint": 5, "entry": "NONE", "limits": "NONE", "approved_joints": 0,
                              "egress_mask": 0, "restore": "RESTORED" if all_mapped else "NOT_RUN",
                              "segment": 0, "phase": "FAKE_IDLE", "session": "OFF"},
        }

    def _handle(self, line: str) -> None:
        text = line.strip()
        with self.lock:
            self.received.append(text)
            if self.drop_first_line and not self.dropped:
                self.dropped = True  # emulates the first write after open being lost
                return
        upper = text.upper()
        if upper.startswith("@MOTION TELEMETRY"):
            if self.protocol != "tel":
                self._emit("ERR UNKNOWN_COMMAND")
                return
            parts = text.split()
            hz = _int(parts[2]) if len(parts) > 2 else None
            if hz is None or not 0 <= hz <= 20:
                self._emit("TELEMETRY=REFUSED REASON=RATE")
                return
            self.tel_period = 1.0 / hz if hz else None
            self.next_tel = time.monotonic()
            self._emit(f"TELEMETRY=OK RATE_HZ={hz}")
        elif upper == "@MOTION VALIDATION SENSORS":
            now = self.uptime_ms()
            _, _, q = self._pose(now / 1000.0)
            self._emit(f"SENSORS uptime_ms={now} imu_valid=1 imu_stamp_ms={max(0, now - 6)} "
                       f"quaternion={q[0]:.9f},{q[1]:.9f},{q[2]:.9f},{q[3]:.9f} bms_valid=1 "
                       f"bms_stamp_ms={max(0, now - 900)} soc=77.500 power=RUN fault=0 boot_safe_off_pending=0")
        elif upper.startswith("@MOTION VALIDATION JOINT "):
            slot = _int(text.split()[-1])
            if slot is None or not 0 <= slot < 12:
                self._emit("VALIDATION=REFUSED REASON=USAGE_OR_LIVE_GATE")
                return
            now = self.uptime_ms()
            stamp = max(0, now - 40)
            actual, _, _ = self._pose(stamp / 1000.0)
            mapped = slot not in self.unmapped_slots
            self._emit(f"JOINT slot={slot} id={self.BUS_IDS[slot]} configured_unit=M{slot:02d} "
                       f"raw={self._servo_raw(slot, actual[slot])} torque=0 speed_raw=0 goal=0 goal_speed=0 "
                       f"goal_acc=0 output=1000 temp={31 + slot % 4} voltage_dv=118 status=0 stamp_ms={stamp} "
                       f"age_ms={now - stamp} transaction_ms=1 mapped_valid={1 if mapped else 0} "
                       f"actual_rad={actual[slot] if mapped else 0.0:.9f}")
        elif upper == "@MOTION VALIDATION STOP":
            self._emit(f"VALIDATION=PASS REASON=SAFE_OFF_REQUESTED SESSION=OFF SEGMENT=0 NONCE=1 LIMITS=0 "
                       f"UPTIME_MS={self.uptime_ms()} FAKE_DEVICE_NOT_HARDWARE")
        elif upper == "@MOTION COMMISSIONING STOP":
            if self.protocol == "tel":
                self._emit("COMMISSIONING=STOPPED REASON=LOCAL_OPERATOR_STOP FAKE_DEVICE_NOT_HARDWARE")
            else:
                self._emit("ERR UNKNOWN_COMMAND")
        elif text.startswith("@"):
            self._emit(f"ERR UNKNOWN_COMMAND FAKE_DEVICE_NOT_HARDWARE {text[:40]}")

    def _run(self) -> None:
        buffer = bytearray()
        while not self._stop.is_set():
            chunk = b""
            with self.io_lock:
                master = self.master
                if master is not None:
                    try:
                        readable, _, _ = select.select([master], [], [], 0.01)
                        if readable:
                            chunk = os.read(master, 4096)
                    except OSError:
                        chunk = b""
            if master is None:
                buffer.clear()
                time.sleep(0.02)
                continue
            buffer += chunk
            while b"\n" in buffer:
                index = buffer.index(b"\n")
                line = bytes(buffer[:index]).decode(errors="replace").rstrip("\r")
                del buffer[:index + 1]
                self._handle(line)
            now = time.monotonic()
            if self.tel_period and now >= self.next_tel:
                self.next_tel = max(self.next_tel + self.tel_period, now - self.tel_period)
                self._emit("@TEL " + json.dumps(self.tel_object(), separators=(",", ":")))
            if now >= self.next_stream:
                self.next_stream = now + 0.5
                up = self.uptime_ms()
                self._emit(f"MAG x=-21.688 y=-2.312 z=-42.812 |B|=48.048 status=3 count={up // 500}")
                self._emit(f"COUNTS acc={up // 10} gyr={up // 10} mag={up // 500} game=0 rv={up // 500} runtime_resets=0")


# ------------------------------------------------------------------- cli ---

def parse_address(text: str, what: str) -> tuple[str, int]:
    host, sep, port = text.rpartition(":")
    if not sep or not port.isdigit():
        raise argparse.ArgumentTypeError(f"{what} must be HOST:PORT")
    return host.strip("[]") or "0.0.0.0", int(port)


def lan_addresses() -> list[str]:
    try:
        output = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True,
                                timeout=2, check=False).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    found = []
    for line in output.splitlines():
        fields = line.split()
        if len(fields) >= 4 and not fields[1].startswith(("lo", "docker", "br-", "veth")):
            found.append(fields[3].split("/")[0])
    return found


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial device path (default: by-id ESP32-S3 node)")
    parser.add_argument("--http", default=DEFAULT_HTTP, help="HTTP bind HOST:PORT (default 0.0.0.0:8765)")
    parser.add_argument("--console", default=DEFAULT_CONSOLE, help="loopback console HOST:PORT (default 127.0.0.1:8766)")
    parser.add_argument("--viewer-dist", default=DEFAULT_VIEWER_DIST, help="built viewer directory (vite build output)")
    parser.add_argument("--log-dir", default=None, help="session log directory (default ~/MATDOG/private-preflight/bridge-<UTC>)")
    parser.add_argument("--lock-dir", default=None, help="owner lock directory (default $XDG_RUNTIME_DIR or /tmp)")
    parser.add_argument("--tel-hz", type=int, default=10, help="@MOTION TELEMETRY rate to request (1..20, default 10)")
    parser.add_argument("--no-legacy-poll", action="store_true", help="never poll legacy read-only commands")
    parser.add_argument("--fake-device", action="store_true", help="DEMO/TEST ONLY: internal pty simulator, never hardware")
    parser.add_argument("--fake-protocol", choices=("tel", "legacy"), default="tel", help="fake firmware dialect")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if not 1 <= args.tel_hz <= 20:
        print("--tel-hz must be 1..20", file=sys.stderr)
        return 2
    http_address = parse_address(args.http, "--http")
    console_address = parse_address(args.console, "--console")
    if not is_loopback(console_address[0]):
        print("--console must be a loopback address (127.0.0.1 or ::1)", file=sys.stderr)
        return 2
    log_dir = args.log_dir or os.path.join(os.path.expanduser("~/MATDOG/private-preflight"), f"bridge-{utc_stamp()}")
    log = SessionLog(log_dir)
    lock_dir = args.lock_dir or os.environ.get("XDG_RUNTIME_DIR") or tempfile.gettempdir()
    fake: Optional[FakeController] = None
    port = args.port
    if args.fake_device:
        fake = FakeController(protocol=args.fake_protocol)
        fake.start()
        port = fake.link_path
    log.write("EVT", "bridge start", argv=sys.argv, port=port, fake_device=bool(fake), pid=os.getpid(),
              http=args.http, console=args.console, viewer_dist=args.viewer_dist)
    bridge = Bridge(port, log, lock_dir, tel_hz=args.tel_hz, fake_device=bool(fake),
                    legacy_poll=not args.no_legacy_poll)
    try:
        httpd = BridgeHTTPServer(http_address, bridge, args.viewer_dist)
        console = ConsoleServer(console_address, bridge)
    except OSError as exc:
        print(f"cannot bind: {exc}", file=sys.stderr)
        log.write("EVT", "bind failed", error=str(exc))
        log.close()
        return 1
    bridge.start()
    threads = [threading.Thread(target=httpd.serve_forever, name="http", daemon=True),
               threading.Thread(target=console.serve_forever, name="console", daemon=True)]
    for thread in threads:
        thread.start()
    banner = "FAKE DEVICE - NOT HARDWARE - " if fake else ""
    print(f"[{BRIDGE_NAME}] {banner}serial {port}")
    print(f"[{BRIDGE_NAME}] log {log.path}")
    print(f"[{BRIDGE_NAME}] viewer {args.viewer_dist}{'' if os.path.isdir(args.viewer_dist) else '  (MISSING: run npm run build)'}")
    port_number = httpd.server_address[1]
    hosts = lan_addresses() if http_address[0] in ("0.0.0.0", "") else [http_address[0]]
    for host in hosts or ["127.0.0.1"]:
        print(f"[{BRIDGE_NAME}] dashboard http://{host}:{port_number}/")
    print(f"[{BRIDGE_NAME}] console {console_address[0]}:{console.server_address[1]} (loopback, one writer)")
    sys.stdout.flush()
    done = threading.Event()

    def shutdown(signum: int, _frame: Any) -> None:
        done.set()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    try:
        while not done.wait(0.5):
            pass
    finally:
        httpd.stopping.set()
        bridge.stop()
        httpd.shutdown()
        console.shutdown()
        httpd.server_close()
        console.server_close()
        if fake:
            fake.stop()
        log.write("EVT", "bridge stop")
        log.close()
        print(f"[{BRIDGE_NAME}] stopped; log {log.path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

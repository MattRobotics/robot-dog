#!/usr/bin/env python3
"""matdog-esp32-bridge: the single Linux owner of the ESP32-S3 USB CDC port.

Prepared for the Jetson Orin Nano (dev.15). NOT installed, NOT enabled, NOT
hardware-validated. Python 3 standard library only.

* Finds the Controller by its stable udev identity
  (``/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<MAC>-if00``),
  opens it once, persistently, with the reset-safe open of ``matdog_bridge``
  (``os.open``, TIOCEXCL, raw termios, line speed preserved, never DTR/RTS).
* Handshake on every open: ``@HOSTLINK HELLO`` (protocol, Motion contract,
  firmware identity) and ``@MOTION CAPABILITIES``; an incompatible Controller
  gets no Motion command except STOP.
* One request in flight, matched to its reply line; a timeout is reported,
  never retried. STOP is written at once, ahead of any queued request.
* Semantic commands only (STAND, LIE_DOWN, BASE_VELOCITY/JOYSTICK, STOP) as
  Motion contract v1 envelopes, owner ``local`` (the USB client), issued at the
  Controller uptime estimated from its last reply. The Controller keeps all
  authority, TTLs, the 1 s joystick deadline and STOP; the bridge adds none.
* Every (re)open starts a new ``link_epoch``. Motion requests must carry the
  current epoch: a client stream from before a disconnect or a Controller
  reboot is refused (``STALE_LINK_EPOCH``), nothing is ever replayed.
* Clients use a local Unix socket, one JSON object per line
  (``{"op":"status"}``, ``{"op":"velocity","epoch":3,"vx":..}``...). No HTTP.
* SIGTERM/SIGINT closes the tty and releases the owner lock (flash, service).
"""

from __future__ import annotations

import argparse
import errno
import glob
import json
import os
import random
import select
import signal
import socket
import sys
import threading
import time
from typing import Any, Callable, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matdog_bridge import (SerialOwnerBusy, acquire_owner_lock, lock_path_for,  # noqa: E402
                           other_openers, safe_open)

HOSTLINK_PROTOCOL = 1
MOTION_CONTRACT = 1
MOTION_STACK = "MATDOG_SIMPLE_V1"
REQUIRED_COMMANDS = ("STAND", "JOYSTICK", "PRONE", "STOP")
BY_ID_PATTERN = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00"
DEFAULT_SOCKET = "/run/matdog/esp32-bridge.sock"
DEFAULT_LOCK_DIR = "/run/lock"
REPLY_TIMEOUT_S = 1.0
VELOCITY_TTL_MS = 500          # contract: BASE_VELOCITY ttl <= 1000 ms
COMMAND_TTL_MS = 1000
MAX_LINE_BYTES = 16384         # @MOTION STATUS is < 6 KiB
MAX_TX_LINE = 319              # Controller line reader: 320 B with the newline
MAX_CLIENTS = 4
REOPEN_S = 1.0

L_DISCONNECTED, L_BUSY, L_HANDSHAKE, L_READY, L_INCOMPATIBLE = (
    "DISCONNECTED", "OWNER_BUSY", "HANDSHAKE", "READY", "INCOMPATIBLE")


class BridgeError(Exception):
    """A request the bridge refuses or cannot complete; code is client-facing."""

    def __init__(self, code: str, detail: str = ""):
        super().__init__(code if not detail else f"{code}: {detail}")
        self.code, self.detail = code, detail


def find_port(pattern: str = BY_ID_PATTERN, serial: str = "") -> str:
    """Exactly one stable by-id path (optionally the given USB serial/MAC), else an error."""
    matches = sorted(p for p in glob.glob(pattern) if not serial or serial.lower() in os.path.basename(p).lower())
    if len(matches) != 1:
        raise BridgeError("PORT_NOT_UNIQUE", f"{len(matches)} match(es) for {pattern}" + (f" serial {serial}" if serial else ""))
    return matches[0]


def is_ack(line: str, ident: str) -> bool:
    return line.startswith('{"version":1,"ack":true,') and f'"id":"{ident}"' in line


def is_refusal(line: str) -> bool:
    return line.startswith(("MOTION=REJECTED", "ERROR=", "COMMAND=REFUSED", "HOSTLINK=", "MOTION_CAPABILITIES="))


class Esp32Link:
    """Owns the tty: open/handshake/reopen loop, line dispatch, one request in flight."""

    def __init__(self, port: str, lock_dir: str = DEFAULT_LOCK_DIR, expect_version: str = "",
                 expect_git: str = "", reply_timeout: float = REPLY_TIMEOUT_S, reopen_s: float = REOPEN_S,
                 status_period: float = 1.0, on_event: Callable[[str], None] = lambda text: None):
        self.port, self.lock_path = port, lock_path_for(port, lock_dir)
        self.expect_version, self.expect_git = expect_version, expect_git
        self.reply_timeout, self.reopen_s, self.status_period = reply_timeout, reopen_s, status_period
        self.on_event = on_event
        self.state, self.detail = L_DISCONNECTED, "not started"
        self.epoch = 0
        self.hello: Optional[dict] = None
        self.capabilities: Optional[dict] = None
        self.status: Optional[dict] = None
        self.status_at = 0.0
        self.counters = {"opens": 0, "rx_lines": 0, "tx_lines": 0, "unmatched_lines": 0, "timeouts": 0, "overflows": 0}
        self._fd: Optional[int] = None
        self._lock_fd: Optional[int] = None
        self._uptime_ms = 0            # Controller uptime of its last reply
        self._uptime_at = 0.0          # host monotonic time that reply was received
        self._session = f"{random.getrandbits(32):08x}"
        self._sequence = 0
        self._write_lock = threading.Lock()     # whole lines never interleave
        self._request_lock = threading.Lock()   # one request in flight (STOP excepted)
        self._waiters: list[tuple[Callable[[str], bool], list, threading.Event]] = []
        self._waiters_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # --------------------------------------------------------- lifecycle ---
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="esp32-link", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Close the tty and release the owner lock: the port is free for flash/service."""
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)
        self._close("bridge stopped", L_DISCONNECTED)
        if self._lock_fd is not None:
            os.close(self._lock_fd)
            self._lock_fd = None

    def _event(self, text: str) -> None:
        self.on_event(text)

    def _set(self, state: str, detail: str) -> None:
        if (state, detail) != (self.state, self.detail):
            self._event(f"link {state}: {detail}")
        self.state, self.detail = state, detail

    def _open(self) -> bool:
        try:
            if self._lock_fd is None:
                self._lock_fd = acquire_owner_lock(self.lock_path)
            if not os.path.exists(self.port):
                self._set(L_DISCONNECTED, "device path absent")
                return False
            others = other_openers(self.port)
            if others:
                raise SerialOwnerBusy(f"held open by pid {', '.join(map(str, others))}")
            fd = safe_open(self.port)
        except SerialOwnerBusy as exc:
            self._set(L_BUSY, str(exc))
            return False
        except OSError as exc:
            self._set(L_DISCONNECTED, f"{exc.__class__.__name__}: {exc}")
            return False
        self._fd = fd
        self.epoch += 1
        self.counters["opens"] += 1
        self.hello = self.capabilities = self.status = None
        self._set(L_HANDSHAKE, f"opened, epoch {self.epoch}")
        return True

    def _close(self, reason: str, state: str) -> None:
        fd, self._fd = self._fd, None
        if fd is not None:
            try:
                os.close(fd)  # plain close: no modem-line ioctl
            except OSError:
                pass
        with self._waiters_lock:
            waiters, self._waiters = self._waiters, []
        for _, box, done in waiters:
            box.append(None)
            done.set()
        if fd is not None or state != self.state:
            self._set(state, reason)

    def _run(self) -> None:
        buffer = bytearray()
        next_status = 0.0
        while not self._stop.is_set():
            if self._fd is None:
                buffer.clear()
                if not self._open():
                    self._stop.wait(self.reopen_s)
                    continue
                threading.Thread(target=self._handshake, args=(self.epoch,), daemon=True).start()
            fd = self._fd
            try:
                readable, _, broken = select.select([fd], [], [fd], 0.1)
                chunk = os.read(fd, 65536) if readable or broken else b""
                if (readable or broken) and not chunk:
                    raise OSError(errno.EIO, "end of file (device hung up)")
            except (OSError, ValueError, TypeError) as exc:
                self._close(f"device lost: {exc}", L_DISCONNECTED)
                continue
            buffer += chunk
            while (index := buffer.find(b"\n")) >= 0:
                line = bytes(buffer[:index]).rstrip(b"\r").decode("utf-8", errors="replace")
                del buffer[:index + 1]
                self.counters["rx_lines"] += 1
                self._dispatch(line)
            if len(buffer) > MAX_LINE_BYTES:
                self.counters["overflows"] += 1
                buffer.clear()
            now = time.monotonic()
            if self.state == L_READY and self.status_period > 0 and now >= next_status:
                next_status = now + self.status_period
                threading.Thread(target=self._poll_status, daemon=True).start()

    def _dispatch(self, line: str) -> None:
        with self._waiters_lock:
            for entry in self._waiters:
                match, box, done = entry
                if match(line):
                    self._waiters.remove(entry)
                    box.append(line)
                    done.set()
                    return
        self.counters["unmatched_lines"] += 1   # streams, boot banner, late replies

    # ----------------------------------------------------------- requests ---
    def _write(self, text: str) -> None:
        if len(text) > MAX_TX_LINE or "\n" in text:
            raise BridgeError("LINE_TOO_LONG", f"{len(text)} > {MAX_TX_LINE}")
        data = (text + "\n").encode()
        with self._write_lock:
            fd = self._fd
            if fd is None:
                raise BridgeError("DISCONNECTED", self.detail)
            view, deadline = memoryview(data), time.monotonic() + self.reply_timeout
            try:
                while view:
                    try:
                        view = view[os.write(fd, view):]
                    except BlockingIOError:
                        if time.monotonic() > deadline:
                            raise BridgeError("WRITE_TIMEOUT")
                        select.select([], [fd], [], 0.02)
            except OSError as exc:
                raise BridgeError("DISCONNECTED", str(exc)) from exc
            self.counters["tx_lines"] += 1

    def _exchange(self, text: str, match: Callable[[str], bool], epoch: Optional[int] = None,
                  refusals: bool = True) -> str:
        """Write one line and wait for its reply (or a refusal line). Never retried."""
        box: list = []
        done = threading.Event()
        entry = (lambda line: match(line) or (refusals and is_refusal(line)), box, done)
        with self._waiters_lock:
            if epoch is not None and epoch != self.epoch:
                raise BridgeError("STALE_LINK_EPOCH", f"current {self.epoch}")
            self._waiters.append(entry)
        try:
            self._write(text)
            if not done.wait(self.reply_timeout):
                self.counters["timeouts"] += 1
                raise BridgeError("TIMEOUT", text.split(" ", 2)[1] if " " in text else text)
        finally:
            with self._waiters_lock:
                if entry in self._waiters:
                    self._waiters.remove(entry)
        if not box or box[0] is None:
            raise BridgeError("DISCONNECTED", self.detail)
        if is_refusal(box[0]) and not match(box[0]):
            raise BridgeError("REFUSED", box[0])
        return box[0]

    def _request(self, text: str, match: Callable[[str], bool], epoch: Optional[int] = None) -> str:
        with self._request_lock:
            return self._exchange(text, match, epoch)

    def _clock(self, uptime_ms: int) -> None:
        self._uptime_ms, self._uptime_at = int(uptime_ms), time.monotonic()

    def controller_now_ms(self) -> int:
        """Never ahead of the Controller: its reply was sampled before we received it."""
        return (self._uptime_ms + int((time.monotonic() - self._uptime_at) * 1000)) & 0xFFFFFFFF

    def _handshake(self, epoch: int) -> None:
        try:
            self._write("")  # a lone newline ends any partial line left in the Controller reader
            hello = json.loads(self._request("@HOSTLINK HELLO", lambda l: l.startswith('{"hostlink":')))
            self._clock(hello.get("uptime_ms", 0))
            caps = json.loads(self._request("@MOTION CAPABILITIES", lambda l: l.startswith('{"version":1,"motion_stack":')))
        except (BridgeError, ValueError) as exc:
            if epoch == self.epoch and self._fd is not None:
                self._set(L_INCOMPATIBLE, f"handshake failed: {exc}")
            return
        problems = []
        if hello.get("hostlink") != HOSTLINK_PROTOCOL:
            problems.append(f"hostlink {hello.get('hostlink')} != {HOSTLINK_PROTOCOL}")
        if hello.get("contract") != MOTION_CONTRACT:
            problems.append(f"contract {hello.get('contract')} != {MOTION_CONTRACT}")
        if hello.get("motion_stack") != MOTION_STACK or caps.get("motion_stack") != MOTION_STACK:
            problems.append("motion stack")
        if hello.get("usb_owner") != "local":
            problems.append("usb owner")
        missing = [c for c in REQUIRED_COMMANDS if c not in caps.get("commands", [])]
        if missing:
            problems.append(f"commands missing {missing}")
        if self.expect_version and hello.get("version") != self.expect_version:
            problems.append(f"version {hello.get('version')} != {self.expect_version}")
        if self.expect_git and not str(hello.get("git_sha", "")).startswith(self.expect_git):
            problems.append(f"git {hello.get('git_sha')} != {self.expect_git}")
        if epoch != self.epoch:
            return
        self.hello, self.capabilities = hello, caps
        if problems:
            self._set(L_INCOMPATIBLE, "; ".join(problems))
        else:
            self._set(L_READY, f"{hello.get('version')} {str(hello.get('git_sha'))[:12]}")

    def _poll_status(self) -> None:
        try:
            self.refresh_status()
        except BridgeError:
            pass

    def refresh_status(self) -> dict:
        line = self._request("@MOTION STATUS", lambda l: l.startswith('{"version":1,"now_ms":'))
        status = json.loads(line)
        self._clock(status.get("now_ms", 0))
        self.status, self.status_at = status, time.monotonic()
        return status

    def _motion(self, epoch: Any, intent: str, params: dict, ttl_ms: int) -> dict:
        if not isinstance(epoch, int) or isinstance(epoch, bool):
            raise BridgeError("EPOCH_REQUIRED", "read link_epoch from status")
        if self.state != L_READY:
            raise BridgeError("NOT_READY", f"{self.state}: {self.detail}")
        with self._request_lock:
            self._sequence += 1
            ident = f"jb-{self._session}-{self._sequence}"
            envelope = {"version": MOTION_CONTRACT, "id": ident, "sequence": self._sequence,
                        "issued_at_ms": self.controller_now_ms(), "ttl_ms": ttl_ms, "owner": "local",
                        "type": intent, "params": params}
            text = "@MOTION JSON " + json.dumps(envelope, separators=(",", ":"), allow_nan=False)
            return json.loads(self._exchange(text, lambda l: is_ack(l, ident), epoch))

    def stand(self, epoch: Any, height_m: float) -> dict:
        return self._motion(epoch, "POSTURE.STAND", {"target_height_m": float(height_m)}, COMMAND_TTL_MS)

    def lie_down(self, epoch: Any) -> dict:
        return self._motion(epoch, "POSTURE.LIE_DOWN", {}, COMMAND_TTL_MS)

    def velocity(self, epoch: Any, vx: float, vy: float, yaw_rate: float) -> dict:
        """One JOYSTICK sample; the Controller ends the walk 1 s after the last one."""
        params = {"vx": float(vx), "vy": float(vy), "yaw_rate": float(yaw_rate), "gait": "JOYSTICK"}
        return self._motion(epoch, "BASE_VELOCITY", params, VELOCITY_TTL_MS)

    def stop_motion(self) -> dict:
        """Written at once, ahead of any queued request; any epoch, any handshake state."""
        line = self._exchange("@MOTION STOP", lambda l: l.startswith('{"version":1,"ack":true,"id":"usb-'), refusals=False)
        return json.loads(line)

    def link_info(self) -> dict:
        age = None if self.status is None else round(time.monotonic() - self.status_at, 3)
        return {"state": self.state, "detail": self.detail, "link_epoch": self.epoch, "port": self.port,
                "firmware": self.hello, "status_age_s": age, "counters": dict(self.counters)}


# ------------------------------------------------------------- clients ---
def handle_request(link: Esp32Link, request: Any) -> dict:
    """One client request -> one reply object. Never raises."""
    try:
        if not isinstance(request, dict):
            raise BridgeError("BAD_REQUEST", "a JSON object is required")
        op = request.get("op")
        if op == "link":
            return {"ok": True, "link": link.link_info()}
        if op == "status":
            fresh = request.get("fresh", False)
            reply = link.refresh_status() if fresh or link.status is None else link.status
            return {"ok": True, "reply": reply, "link": link.link_info()}
        if op == "capabilities":
            return {"ok": link.capabilities is not None, "reply": link.capabilities, "link": link.link_info()}
        if op == "stop":
            return {"ok": True, "reply": link.stop_motion(), "link": link.link_info()}
        epoch = request.get("epoch")
        if op == "stand":
            reply = link.stand(epoch, request["height_m"])
        elif op == "lie_down":
            reply = link.lie_down(epoch)
        elif op == "velocity":
            reply = link.velocity(epoch, request["vx"], request["vy"], request["yaw_rate"])
        else:
            raise BridgeError("UNKNOWN_OP", str(op))
        return {"ok": reply.get("outcome") in ("ACCEPTED", "COMPLETED"), "reply": reply, "link": link.link_info()}
    except BridgeError as exc:
        return {"ok": False, "error": exc.code, "detail": exc.detail, "link": link.link_info()}
    except (KeyError, TypeError, ValueError) as exc:
        return {"ok": False, "error": "BAD_REQUEST", "detail": str(exc), "link": link.link_info()}


class SocketServer:
    """Local Unix socket, one JSON object per line each way, at most MAX_CLIENTS clients."""

    def __init__(self, path: str, link: Esp32Link):
        self.path, self.link = path, link
        self.clients = threading.Semaphore(MAX_CLIENTS)
        if os.path.exists(path):
            os.unlink(path)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(path)
        os.chmod(path, 0o660)
        self.sock.listen(MAX_CLIENTS)

    def serve(self, stop: threading.Event) -> None:
        self.sock.settimeout(0.2)
        while not stop.is_set():
            try:
                conn, _ = self.sock.accept()
            except socket.timeout:
                continue
            if not self.clients.acquire(blocking=False):
                conn.sendall(b'{"ok":false,"error":"TOO_MANY_CLIENTS"}\n')
                conn.close()
                continue
            threading.Thread(target=self._client, args=(conn,), daemon=True).start()

    def _client(self, conn: socket.socket) -> None:
        try:
            with conn, conn.makefile("rwb") as stream:
                for raw in stream:
                    if len(raw) > 4096:
                        reply = {"ok": False, "error": "BAD_REQUEST", "detail": "request too long"}
                    else:
                        try:
                            reply = handle_request(self.link, json.loads(raw))
                        except ValueError as exc:
                            reply = {"ok": False, "error": "BAD_REQUEST", "detail": str(exc)}
                    stream.write((json.dumps(reply, separators=(",", ":")) + "\n").encode())
                    stream.flush()
        except OSError:
            pass
        finally:
            self.clients.release()

    def close(self) -> None:
        self.sock.close()
        if os.path.exists(self.path):
            os.unlink(self.path)


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="", help="tty path (default: the unique Espressif by-id path)")
    parser.add_argument("--serial", default="", help="USB serial (MAC) the by-id path must contain")
    parser.add_argument("--socket", default=DEFAULT_SOCKET)
    parser.add_argument("--lock-dir", default=DEFAULT_LOCK_DIR)
    parser.add_argument("--expect-version", default="", help="refuse Motion unless HELLO reports this version")
    parser.add_argument("--expect-git", default="", help="refuse Motion unless git_sha starts with this")
    parser.add_argument("--status-period", type=float, default=1.0, help="status poll period s (0 = on demand)")
    args = parser.parse_args(argv)
    try:
        port = args.port or find_port(serial=args.serial)
    except BridgeError as exc:
        print(f"matdog-esp32-bridge: {exc}", file=sys.stderr)
        return 2
    link = Esp32Link(port, args.lock_dir, args.expect_version, args.expect_git, status_period=args.status_period,
                     on_event=lambda text: print(f"matdog-esp32-bridge: {text}", file=sys.stderr, flush=True))
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())
    server = SocketServer(args.socket, link)
    link.start()
    try:
        server.serve(stop)
    finally:
        server.close()
        link.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

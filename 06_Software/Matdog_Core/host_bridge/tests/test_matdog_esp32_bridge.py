#!/usr/bin/env python3
"""matdog-esp32-bridge offline tests: a pseudo-terminal Controller simulator.

The simulator answers the dev.15 HostLink lines (@HOSTLINK HELLO, @MOTION
CAPABILITIES/STATUS/STOP/JSON) and applies the Motion contract checks the
bridge relies on (owner, TTL, timestamps). It is NOT the firmware: the real
router/MotionService side is covered by test_command_router_persistence.cpp.
No real device, no hardware.
"""
import fcntl
import json
import os
import select
import sys
import tempfile
import termios
import threading
import time
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import matdog_esp32_bridge as B  # noqa: E402
from matdog_bridge import MODEM_LINE_IOCTLS  # noqa: E402


class FakeController(threading.Thread):
    def __init__(self, link_path: str, protocol: int = 1, version: str = "1.0.1-dev.15", uptime0: int = 50000):
        super().__init__(daemon=True)
        self.master, slave = os.openpty()
        self.slave_path = os.ttyname(slave)
        os.close(slave)  # only the bridge opens the slave, by path
        tmp = link_path + ".tmp"
        os.symlink(self.slave_path, tmp)
        os.replace(tmp, link_path)
        self.protocol, self.version = protocol, version
        self.t0, self.uptime0 = time.monotonic(), uptime0
        self.received: list = []          # (monotonic, line)
        self.sent: list = []              # (monotonic, line)
        self.delay: dict = {}             # line prefix -> reply delay s
        self.drop: set = set()            # line prefixes never answered
        self.refuse_json = ""             # reply line for every @MOTION JSON
        self.noise = False
        self.outstanding = 0
        self.max_outstanding = 0
        self.stops = 0
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self.start()

    def now_ms(self) -> int:
        return self.uptime0 + int((time.monotonic() - self.t0) * 1000)

    def reboot(self) -> None:
        self.t0, self.uptime0 = time.monotonic(), 0

    def close(self) -> None:
        # A read blocked in another thread keeps the master alive: stop the
        # reader first, then close, so the slave really sees the hang-up.
        self._closed.set()
        self.join()
        with self._lock:
            os.close(self.master)

    def _send(self, line: str) -> None:
        with self._lock:
            if self._closed.is_set():
                return
            if self.noise:
                os.write(self.master, b"BMS pack_v=25.10 soc=80\nboot noise without newline? no\n")
            os.write(self.master, (line + "\n").encode())
            self.sent.append((time.monotonic(), line))

    def _reply_later(self, prefix: str, line: str, counted: bool) -> None:
        def later():
            time.sleep(self.delay.get(prefix, 0))
            self._send(line)
            if counted:
                with self._lock:
                    self.outstanding -= 1
        if counted:
            with self._lock:
                self.outstanding += 1
                self.max_outstanding = max(self.max_outstanding, self.outstanding)
        threading.Thread(target=later, daemon=True).start()

    def _answer(self, line: str) -> None:
        self.received.append((time.monotonic(), line))
        if not line:
            return
        prefix = " ".join(line.split(" ")[:2])
        if prefix in self.drop:
            return
        if line == "@HOSTLINK HELLO":
            reply = json.dumps({"hostlink": self.protocol, "contract": 1, "firmware": "MATDOG Controller", "version": self.version,
                                "git_sha": "928b8b73277e", "git_dirty": False, "build_id": "sim", "build_utc": "sim",
                                "profile": "ROBOT_POWERED", "motion_stack": "MATDOG_SIMPLE_V1", "usb_owner": "local",
                                "line_max": 319, "uptime_ms": self.now_ms()}, separators=(",", ":"))
        elif line == "@MOTION CAPABILITIES":
            reply = '{"version":1,"motion_stack":"MATDOG_SIMPLE_V1","ready":true,"commands":["STAND","WALK_STANDARD","JOYSTICK","WALK_STOP","PRONE","STOP"]}'
        elif line == "@MOTION STATUS":
            reply = '{"version":1,"now_ms":%d,"state":"READY","phase":"IDLE"}' % self.now_ms()
        elif line == "@MOTION STOP":
            self.stops += 1
            self._send('{"version":1,"ack":true,"id":"usb-%d","sequence":%d,"owner":"local","outcome":"ACCEPTED","reason":"STOP_REQUESTED","executor_state":"STOPPED"}' % (self.stops, self.stops))
            return
        elif line.startswith("@MOTION JSON "):
            if self.refuse_json:
                reply = self.refuse_json
            else:
                c = json.loads(line[13:])
                age = (self.now_ms() - c["issued_at_ms"]) & 0xFFFFFFFF
                reason = ("LOCAL_OWNER" if c["owner"] != "local" else "FUTURE_TIMESTAMP" if age >= 0x80000000 else
                          "TTL_RANGE" if c["type"] == "BASE_VELOCITY" and c["ttl_ms"] > 1000 else
                          "EXPIRED" if age >= c["ttl_ms"] else "COMMAND_ACCEPTED")
                if reason == "LOCAL_OWNER":
                    reply = "MOTION=REJECTED REASON=LOCAL_OWNER_REQUIRED"
                else:
                    reply = json.dumps({"version": 1, "ack": True, "id": c["id"], "sequence": c["sequence"], "owner": "local",
                                        "outcome": "ACCEPTED" if reason == "COMMAND_ACCEPTED" else "REJECTED",
                                        "reason": reason, "executor_state": "READY"}, separators=(",", ":"))
            self._reply_later(prefix, reply, counted=True)
            return
        else:
            reply = "ERROR=UNKNOWN_COMMAND"
        self._reply_later(prefix, reply, counted=False)

    def run(self) -> None:
        buf = b""
        while not self._closed.is_set():
            try:
                if not select.select([self.master], [], [], 0.05)[0]:
                    continue
                chunk = os.read(self.master, 4096)
            except OSError:            # no slave opener yet, or closed
                time.sleep(0.01)
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._answer(line.decode().rstrip("\r"))

    def lines(self, prefix: str) -> list:
        return [l for _, l in self.received if l.startswith(prefix)]


class BridgeCase(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="matdog-esp32-bridge-")
        self.port = os.path.join(self.dir, "usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00")
        self.sims = []
        self.link = None

    def tearDown(self):
        if self.link:
            self.link.stop()
        for sim in self.sims:
            if not sim._closed.is_set():
                sim.close()
        import shutil
        shutil.rmtree(self.dir, ignore_errors=True)

    def sim(self, **kw) -> FakeController:
        s = FakeController(self.port, **kw)
        self.sims.append(s)
        return s

    def bridge(self, **kw) -> B.Esp32Link:
        kw.setdefault("reply_timeout", 0.5)
        kw.setdefault("reopen_s", 0.05)
        kw.setdefault("status_period", 0)
        self.link = B.Esp32Link(self.port, lock_dir=self.dir, **kw)
        self.link.start()
        return self.link

    def wait(self, predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.01)
        self.fail(f"timeout waiting; link {self.link.link_info() if self.link else None}")

    def ready(self, epoch=1):
        self.wait(lambda: self.link.state == B.L_READY and self.link.epoch == epoch)


class Discovery(unittest.TestCase):
    def test_unique_stable_identity(self):
        with tempfile.TemporaryDirectory() as d:
            pattern = os.path.join(d, "usb-Espressif_USB_JTAG_serial_debug_unit_*-if00")
            with self.assertRaises(B.BridgeError) as none:
                B.find_port(pattern)
            self.assertEqual(none.exception.code, "PORT_NOT_UNIQUE")
            a = os.path.join(d, "usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00")
            open(a, "w").close()
            self.assertEqual(B.find_port(pattern), a)
            open(os.path.join(d, "usb-Espressif_USB_JTAG_serial_debug_unit_AA:BB:CC:DD:EE:FF-if00"), "w").close()
            with self.assertRaises(B.BridgeError):
                B.find_port(pattern)            # two Controllers: never guess
            self.assertEqual(B.find_port(pattern, serial="14:c1:9f"), a)


class Protocol(BridgeCase):
    def test_handshake_commands_and_envelopes(self):
        sim = self.sim()
        sim.noise = True                          # unsolicited stream lines interleave
        link = self.bridge()
        self.ready()
        self.assertEqual(sim.received[0][1], "")  # lone newline first
        self.assertEqual(link.hello["version"], "1.0.1-dev.15")
        self.assertEqual(B.handle_request(link, {"op": "stand", "epoch": 1, "height_m": 0.15})["reply"]["outcome"], "ACCEPTED")
        self.assertTrue(B.handle_request(link, {"op": "lie_down", "epoch": 1})["ok"])
        before = sim.now_ms()
        reply = B.handle_request(link, {"op": "velocity", "epoch": 1, "vx": 0.003, "vy": 0.001, "yaw_rate": -0.01})
        self.assertTrue(reply["ok"], reply)
        envelope = json.loads(sim.lines("@MOTION JSON")[-1][13:])
        self.assertEqual((envelope["owner"], envelope["type"], envelope["ttl_ms"], envelope["params"]["gait"]),
                         ("local", "BASE_VELOCITY", 500, "JOYSTICK"))
        self.assertLessEqual(envelope["issued_at_ms"], sim.now_ms())   # never ahead of the Controller
        self.assertGreater(envelope["issued_at_ms"], before - 200)
        self.assertLessEqual(len(sim.lines("@MOTION JSON")[-1]), 319)
        status = B.handle_request(link, {"op": "status", "fresh": True})
        self.assertEqual(status["reply"]["state"], "READY")
        self.assertEqual(status["link"]["link_epoch"], 1)
        self.assertGreater(link.counters["unmatched_lines"], 0)

    def test_incompatible_controller_gets_stop_only(self):
        sim = self.sim(protocol=2)
        link = self.bridge()
        self.wait(lambda: link.state == B.L_INCOMPATIBLE)
        self.assertIn("hostlink 2", link.detail)
        refused = B.handle_request(link, {"op": "velocity", "epoch": 1, "vx": 0.003, "vy": 0, "yaw_rate": 0})
        self.assertEqual(refused["error"], "NOT_READY")
        self.assertEqual(sim.lines("@MOTION JSON"), [])
        self.assertTrue(B.handle_request(link, {"op": "stop"})["ok"])
        self.assertEqual(sim.stops, 1)

    def test_expected_version_and_old_firmware(self):
        self.sim(version="1.0.1-dev.14")
        link = self.bridge(expect_version="1.0.1-dev.15")
        self.wait(lambda: link.state == B.L_INCOMPATIBLE)
        self.assertIn("version 1.0.1-dev.14", link.detail)

    def test_firmware_without_hostlink(self):
        sim = self.sim()
        sim.drop = set()
        orig = sim._answer
        sim._answer = lambda line: (sim.received.append((time.monotonic(), line)), sim._send("ERROR=UNKNOWN_COMMAND")) \
            if line == "@HOSTLINK HELLO" else orig(line)
        link = self.bridge()
        self.wait(lambda: link.state == B.L_INCOMPATIBLE)
        self.assertIn("REFUSED", link.detail)

    def test_bad_client_requests_and_refusal_lines(self):
        sim = self.sim()
        link = self.bridge()
        self.ready()
        for request, code in (([], "BAD_REQUEST"), ({"op": "fly"}, "UNKNOWN_OP"),
                              ({"op": "velocity", "vx": 0, "vy": 0, "yaw_rate": 0}, "EPOCH_REQUIRED"),
                              ({"op": "velocity", "epoch": True, "vx": 0, "vy": 0, "yaw_rate": 0}, "EPOCH_REQUIRED"),
                              ({"op": "stand", "epoch": 1}, "BAD_REQUEST"),
                              ({"op": "velocity", "epoch": 1, "vx": "a", "vy": 0, "yaw_rate": 0}, "BAD_REQUEST")):
            self.assertEqual(B.handle_request(link, request)["error"], code, request)
        nan = B.handle_request(link, {"op": "velocity", "epoch": 1, "vx": float("nan"), "vy": 0, "yaw_rate": 0})
        self.assertEqual(nan["error"], "BAD_REQUEST")         # never serialised as NaN
        sim.refuse_json = "MOTION=REJECTED REASON=INVALID_JSON_SCHEMA"
        refused = B.handle_request(link, {"op": "lie_down", "epoch": 1})
        self.assertEqual((refused["error"], refused["detail"]), ("REFUSED", "MOTION=REJECTED REASON=INVALID_JSON_SCHEMA"))

    def test_timeout_is_reported_never_retried(self):
        sim = self.sim()
        link = self.bridge(reply_timeout=0.3)
        self.ready()
        sim.drop = {"@MOTION JSON"}
        start = time.monotonic()
        self.assertEqual(B.handle_request(link, {"op": "stand", "epoch": 1, "height_m": 0.15})["error"], "TIMEOUT")
        self.assertLess(time.monotonic() - start, 0.6)
        time.sleep(0.4)
        self.assertEqual(len(sim.lines("@MOTION JSON")), 1)
        sim.drop = set()
        sim.delay = {"@MOTION JSON": 0.4}                      # a late reply...
        self.assertEqual(B.handle_request(link, {"op": "lie_down", "epoch": 1})["error"], "TIMEOUT")
        sim.delay = {}
        ok = B.handle_request(link, {"op": "lie_down", "epoch": 1})   # ...never answers the next request
        self.assertTrue(ok["ok"], ok)
        self.assertEqual(ok["reply"]["id"], json.loads(sim.lines("@MOTION JSON")[-1][13:])["id"])

    def test_one_request_in_flight(self):
        sim = self.sim()
        link = self.bridge()
        self.ready()
        sim.delay = {"@MOTION JSON": 0.1}
        results = []
        threads = [threading.Thread(target=lambda: results.append(
            B.handle_request(link, {"op": "velocity", "epoch": 1, "vx": 0.002, "vy": 0, "yaw_rate": 0}))) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertTrue(all(r["ok"] for r in results), results)
        self.assertEqual(sim.max_outstanding, 1)

    def test_stop_overtakes_a_pending_request(self):
        sim = self.sim()
        link = self.bridge(reply_timeout=1.0)
        self.ready()
        sim.delay = {"@MOTION JSON": 0.6}
        slow = threading.Thread(target=lambda: B.handle_request(link, {"op": "stand", "epoch": 1, "height_m": 0.12}))
        slow.start()
        self.wait(lambda: sim.lines("@MOTION JSON"))
        start = time.monotonic()
        stop = B.handle_request(link, {"op": "stop"})
        self.assertTrue(stop["ok"], stop)
        self.assertLess(time.monotonic() - start, 0.3)
        slow.join()
        stop_at = next(t for t, l in sim.received if l == "@MOTION STOP")
        stand_reply_at = next(t for t, l in sim.sent if '"ack":true' in l and '"id":"jb-' in l)
        self.assertLess(stop_at, stand_reply_at)

    def test_reconnect_starts_a_new_epoch_and_replays_nothing(self):
        sim = self.sim()
        link = self.bridge()
        self.ready()
        self.assertTrue(B.handle_request(link, {"op": "velocity", "epoch": 1, "vx": 0.003, "vy": 0, "yaw_rate": 0})["ok"])
        sim.close()                                             # USB unplug / Controller reset
        self.wait(lambda: link.state == B.L_DISCONNECTED)
        self.assertEqual(B.handle_request(link, {"op": "velocity", "epoch": 1, "vx": 0.003, "vy": 0, "yaw_rate": 0})["error"], "NOT_READY")
        sim2 = self.sim(uptime0=0)                              # re-enumerated, rebooted Controller
        self.ready(epoch=2)
        stale = B.handle_request(link, {"op": "velocity", "epoch": 1, "vx": 0.003, "vy": 0, "yaw_rate": 0})
        self.assertEqual(stale["error"], "STALE_LINK_EPOCH")
        time.sleep(0.2)
        self.assertEqual([l for _, l in sim2.received], ["", "@HOSTLINK HELLO", "@MOTION CAPABILITIES"])  # nothing replayed
        self.assertTrue(B.handle_request(link, {"op": "velocity", "epoch": 2, "vx": 0.003, "vy": 0, "yaw_rate": 0})["ok"])

    def test_controller_clock_reset_is_refused_by_the_contract(self):
        sim = self.sim()
        link = self.bridge()
        self.ready()
        sim.reboot()                                             # uptime restarts without a USB drop
        reply = B.handle_request(link, {"op": "velocity", "epoch": 1, "vx": 0.003, "vy": 0, "yaw_rate": 0})
        self.assertEqual(reply["reply"]["reason"], "FUTURE_TIMESTAMP")
        self.assertFalse(reply["ok"])
        self.assertEqual(len(sim.lines("@MOTION JSON")), 1)     # no retry
        B.handle_request(link, {"op": "status", "fresh": True})  # the next reply re-bases the clock
        self.assertTrue(B.handle_request(link, {"op": "velocity", "epoch": 1, "vx": 0.003, "vy": 0, "yaw_rate": 0})["ok"])

    def test_late_velocity_expires_at_the_controller(self):
        sim = self.sim()
        link = self.bridge(reply_timeout=1.0)
        self.ready()
        sim.delay = {}
        real = sim._answer
        def stalled(line):                                      # the line reaches the Controller 600 ms late
            if line.startswith("@MOTION JSON"):
                time.sleep(0.6)
            real(line)
        sim._answer = stalled
        reply = B.handle_request(link, {"op": "velocity", "epoch": 1, "vx": 0.003, "vy": 0, "yaw_rate": 0})
        self.assertEqual(reply["reply"]["reason"], "EXPIRED")

    def test_stop_releases_the_port_and_never_touches_modem_lines(self):
        calls = []
        real = fcntl.ioctl
        def spy(fd, request, *args):
            calls.append(request)
            return real(fd, request, *args)
        self.sim()
        with mock.patch("fcntl.ioctl", spy):
            link = self.bridge()
            self.ready()
            link.stop()
        self.link = None
        self.assertNotIn(True, [c in MODEM_LINE_IOCTLS for c in calls])
        self.assertTrue(set(calls) <= {termios.TIOCEXCL}, calls)
        target = os.path.realpath(self.port)
        held = [e for e in os.listdir("/proc/self/fd") if os.path.realpath(f"/proc/self/fd/{e}") == target]
        self.assertEqual(held, [])                               # tty closed
        fd = B.acquire_owner_lock(B.lock_path_for(self.port, self.dir))   # owner lock free
        os.close(fd)


class SocketClients(BridgeCase):
    def test_unix_socket_one_json_line_each_way(self):
        import socket as s
        self.sim()
        link = self.bridge()
        self.ready()
        path = os.path.join(self.dir, "bridge.sock")
        server = B.SocketServer(path, link)
        stop = threading.Event()
        t = threading.Thread(target=server.serve, args=(stop,), daemon=True)
        t.start()
        try:
            self.assertEqual(os.stat(path).st_mode & 0o777, 0o660)
            with s.socket(s.AF_UNIX, s.SOCK_STREAM) as c, c.makefile("rwb") as f:
                c.connect(path)
                for req in ({"op": "link"}, "not json", {"op": "velocity", "epoch": 1, "vx": 0.002, "vy": 0, "yaw_rate": 0}):
                    f.write((req if isinstance(req, str) else json.dumps(req)).encode() + b"\n")
                    f.flush()
                replies = [json.loads(f.readline()) for _ in range(3)]
            self.assertEqual(replies[0]["link"]["state"], "READY")
            self.assertEqual(replies[1]["error"], "BAD_REQUEST")
            self.assertTrue(replies[2]["ok"], replies[2])
        finally:
            stop.set()
            t.join()
            server.close()
        self.assertFalse(os.path.exists(path))


if __name__ == "__main__":
    unittest.main(verbosity=2)

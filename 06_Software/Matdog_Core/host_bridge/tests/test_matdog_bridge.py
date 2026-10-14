#!/usr/bin/env python3
"""Offline tests for matdog_bridge.py. Pseudo-terminals only: no real serial device is opened.

Run from the repository root:
    python3 -m unittest discover -s 06_Software/Matdog_Core/host_bridge/tests -v
"""

from __future__ import annotations

import fcntl
import http.client
import json
import os
import shutil
import socket
import sys
import tempfile
import termios
import threading
import time
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import matdog_bridge as mb  # noqa: E402

REAL_IOCTL = fcntl.ioctl
SENSORS_LINE = ("SENSORS uptime_ms=6424 imu_valid=1 imu_stamp_ms=6405 quaternion=0.057250977,0.010009766,"
                "0.005432129,-0.998291016 bms_valid=1 bms_stamp_ms=4817 soc=77.500 power=RUN fault=0 "
                "boot_safe_off_pending=0")


def joint_line(slot: int, mapped: bool = True, stamp: int = 6200, age: int = 200, actual: float = -0.039884) -> str:
    return (f"JOINT slot={slot} id={13 + slot} configured_unit=M22 raw={2009 + slot} torque=0 speed_raw=0 goal=0 "
            f"goal_speed=0 goal_acc=0 output=1000 temp=33 voltage_dv=115 status=0 stamp_ms={stamp} age_ms={age} "
            f"transaction_ms=1 mapped_valid={1 if mapped else 0} actual_rad={actual if mapped else 0.0:.9f}")


def wait_until(predicate, timeout: float = 6.0, step: float = 0.02) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


class IoctlRecorder:
    def __init__(self):
        self.requests: list[int] = []
        self.lock = threading.Lock()

    def __call__(self, fd, request, *args):
        with self.lock:
            self.requests.append(request)
        return REAL_IOCTL(fd, request, *args)


def tel_line(now_ms: int, nonce: int = 11, **extra) -> str:
    obj = {"version": 1, "now_ms": now_ms, "telemetry": {"sampled_at_ms": now_ms - 3, "actual_rad": None},
           "tel": 1, "boot_nonce": nonce, "tel_seq": now_ms // 100}
    obj.update(extra)
    return "@TEL " + json.dumps(obj)


# ------------------------------------------------------------- safe open ---

class SafeOpenTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = os.openpty()
        self.path = os.ttyname(self.slave)
        attrs = termios.tcgetattr(self.slave)
        attrs[4] = attrs[5] = termios.B115200  # a non-default speed must survive verbatim
        termios.tcsetattr(self.slave, termios.TCSANOW, attrs)
        self.before = termios.tcgetattr(self.slave)
        self.tmp = tempfile.mkdtemp(prefix="matdog_bridge_test_")

    def tearDown(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_safe_open_is_raw_exclusive_speed_preserving_and_never_touches_modem_lines(self):
        self.assertTrue(self.before[3] & termios.ECHO, "fresh pty starts with ECHO on")
        recorder = IoctlRecorder()
        tcset_calls = []
        real_tcsetattr = termios.tcsetattr

        def tcsetattr(fd, when, attrs):
            tcset_calls.append(list(attrs))
            return real_tcsetattr(fd, when, attrs)

        with mock.patch("fcntl.ioctl", side_effect=recorder), mock.patch("termios.tcsetattr", side_effect=tcsetattr):
            fd = mb.safe_open(self.path)
            os.close(fd)
        self.assertIn(termios.TIOCEXCL, recorder.requests)
        self.assertFalse(set(recorder.requests) & mb.MODEM_LINE_IOCTLS, recorder.requests)
        for name in ("TIOCMBIS", "TIOCMBIC", "TIOCMSET"):
            self.assertNotIn(getattr(termios, name), recorder.requests)
        self.assertEqual(len(tcset_calls), 1)
        self.assertEqual(tcset_calls[0][4:6], self.before[4:6])
        after = termios.tcgetattr(self.slave)
        self.assertEqual(after[4:6], self.before[4:6])
        self.assertEqual(after[2] & termios.CBAUD, self.before[2] & termios.CBAUD)
        for flag in (termios.ECHO, termios.ICANON, termios.ISIG, termios.IEXTEN):
            self.assertFalse(after[3] & flag)
        self.assertFalse(after[1] & termios.OPOST)
        self.assertFalse(after[0] & termios.ICRNL)

    def test_second_open_reports_owner_busy(self):
        fd = mb.safe_open(self.path)
        try:
            with self.assertRaises(mb.SerialOwnerBusy):
                mb.safe_open(self.path)  # TIOCEXCL -> EBUSY
        finally:
            os.close(fd)
        lock = mb.lock_path_for(self.path, self.tmp)
        held = mb.acquire_owner_lock(lock)
        try:
            with self.assertRaises(mb.SerialOwnerBusy):
                mb.acquire_owner_lock(lock)  # flock is per open file description
        finally:
            os.close(held)

    def test_serial_link_reports_owner_busy_state(self):
        log = mb.SessionLog(os.path.join(self.tmp, "log"))
        first = mb.SerialLink(self.path, self.tmp, log, lambda now: None, lambda line, now: None,
                              lambda now, state, detail: None)
        second = mb.SerialLink(self.path, self.tmp, log, lambda now: None, lambda line, now: None,
                               lambda now, state, detail: None)
        other_dir = os.path.join(self.tmp, "other")
        os.mkdir(other_dir)
        third = mb.SerialLink(self.path, other_dir, log, lambda now: None, lambda line, now: None,
                              lambda now, state, detail: None)
        try:
            self.assertTrue(first._try_open())
            self.assertFalse(second._try_open())
            self.assertEqual(second.state, mb.S_BUSY)  # flock held by the first bridge
            self.assertFalse(third._try_open())
            self.assertEqual(third.state, mb.S_BUSY)  # different lock dir: TIOCEXCL still refuses
        finally:
            for link in (first, second, third):
                link.stop()
            log.close()


# ---------------------------------------------------------------- engine ---

class EngineTests(unittest.TestCase):
    def make(self, **kwargs) -> mb.TelemetryEngine:
        engine = mb.TelemetryEngine(**kwargs)
        engine.link_opened(100.0)
        engine.note_tx(101.0, "bridge")
        return engine

    def test_pre_tx_ring_lines_are_never_parsed(self):
        engine = mb.TelemetryEngine()
        engine.link_opened(0.0)
        self.assertFalse(engine.line(tel_line(5000), 0.01))
        self.assertFalse(engine.line(SENSORS_LINE, 0.02))
        self.assertIsNone(engine.status)
        self.assertEqual(engine.counters["rx_pre_tx_lines"], 2)
        engine.note_tx(1.0, "bridge")
        self.assertTrue(engine.line(tel_line(6000), 1.1))
        self.assertEqual(engine.status["now_ms"], 6000)

    def test_tel_parsing_bridge_block_and_parse_errors(self):
        engine = self.make()
        engine.line(tel_line(1000, servos=[{"slot": 0}], commissioning={"mode": "DIAGNOSTIC"}), 101.1)
        payload = engine.status_payload(101.2, "/dev/fake")
        self.assertTrue(payload["available"])
        self.assertFalse(payload["stale"])
        self.assertEqual(payload["bridge"]["source"], "TEL")
        self.assertEqual(payload["bridge"]["boot_nonce"], 11)
        self.assertEqual(payload["commissioning"], {"mode": "DIAGNOSTIC"})
        engine.line("@TEL {not json", 101.3)
        engine.line("@TEL [1,2]", 101.4)
        self.assertEqual(engine.counters["parse_errors"], 2)
        self.assertEqual(engine.status["now_ms"], 1000)

    def test_noise_lines_are_ignored(self):
        engine = self.make()
        for line in ("MAG x=-21.688 y=-2.312 z=-42.812 |B|=48.048 status=3 count=13062",
                     "RV w=0.6 x=0 y=0 z=-0.7 accuracy_rad=0.06 status=3 count=1",
                     "GYR x=0 y=0 z=0 |w|=0 status=0", "SAVE_GATE ready=NO still_ms=0",
                     "COUNTS acc=1 gyr=1 mag=1 game=0 rv=1 runtime_resets=0",
                     "VALIDATION=REFUSED REASON=USAGE_OR_LIVE_GATE", "MATDOG Controller boot banner", ""):
            engine.line(line, 101.5)
        self.assertIsNone(engine.status)
        self.assertEqual(engine.counters["parse_errors"], 0)

    def test_legacy_synthesis_with_unmapped_joint_reports_null_actual(self):
        engine = self.make()
        engine.line(SENSORS_LINE, 101.1)
        for slot in range(12):
            engine.line(joint_line(slot, mapped=slot != 3, stamp=6200 + slot, actual=0.1 * slot), 101.2)
        status = engine.status
        self.assertEqual(status["telemetry"]["source"], mb.LEGACY_SOURCE)
        self.assertIsNone(status["telemetry"]["actual_rad"])  # never zeros
        self.assertIsNone(status["telemetry"]["target_rad"])
        self.assertEqual(status["readiness"]["restore"], False)
        self.assertEqual(status["reason"], "RESTORE_REQUIRED")
        self.assertIsNone(status["servos"][3]["actual_rad"])
        self.assertFalse(status["servos"][3]["mapped"])
        self.assertAlmostEqual(status["servos"][4]["actual_rad"], 0.4)
        self.assertEqual(status["telemetry"]["quaternion"], [0.057250977, 0.010009766, 0.005432129, -0.998291016])
        self.assertEqual(status["telemetry"]["raw_ticks"][0], 2009)
        self.assertEqual(status["power"]["soc_pct"], 77.5)
        self.assertEqual(status["imu"], {"valid": True, "stamp_ms": 6405})
        self.assertGreaterEqual(status["now_ms"], status["telemetry"]["sampled_at_ms"])

    def test_legacy_synthesis_all_mapped_and_exact_real_joint_line(self):
        engine = self.make()
        real = ("JOINT slot=0 id=13 configured_unit=M22 raw=2009 torque=0 speed_raw=0 goal=0 goal_speed=0 "
                "goal_acc=0 output=1000 temp=33 voltage_dv=115 status=0 stamp_ms=29212 age_ms=279 "
                "transaction_ms=1 mapped_valid=1 actual_rad=-0.039884000")
        parsed = mb.parse_joint(real)
        self.assertEqual((parsed["id"], parsed["raw"], parsed["temp"], parsed["volt_dv"]), (13, 2009, 33, 115))
        self.assertEqual(parsed["controller_now_ms"], 29491)
        self.assertAlmostEqual(parsed["actual_rad"], -0.039884)
        engine.line(real.replace("stamp_ms=29212", "stamp_ms=6000"), 101.1)
        self.assertIsNone(engine.status["telemetry"]["actual_rad"])  # incomplete channel stays null
        for slot in range(1, 12):
            engine.line(joint_line(slot, stamp=6100), 101.2)
        actual = engine.status["telemetry"]["actual_rad"]
        self.assertEqual(len(actual), 12)
        self.assertAlmostEqual(actual[0], -0.039884)
        self.assertTrue(engine.status["readiness"]["restore"])

    def test_reboot_detection_by_nonce_and_by_uptime(self):
        engine = self.make()
        engine.line(tel_line(50_000, nonce=1), 101.1)
        engine.line(tel_line(50_100, nonce=2), 101.2)
        self.assertEqual(engine.counters["reboots_seen"], 1)
        self.assertEqual(engine.status["boot_nonce"], 2)
        self.assertIn("@MOTION TELEMETRY 10", engine.tick(101.3))  # resubscribe after reboot
        engine.line(tel_line(400, nonce=None), 101.4)  # nonce absent: uptime went backwards
        self.assertEqual(engine.counters["reboots_seen"], 2)
        legacy = self.make()
        legacy.line(SENSORS_LINE, 101.1)
        legacy.line(joint_line(0), 101.2)
        legacy.line(SENSORS_LINE.replace("uptime_ms=6424", "uptime_ms=900"), 101.3)
        self.assertEqual(legacy.counters["reboots_seen"], 1)
        self.assertIsNone(legacy.status["servos"][0]["raw"])  # joint cache reset after reboot

    def test_autodetect_flush_subscribe_fallback_and_poll_pacing(self):
        engine = mb.TelemetryEngine(tel_hz=10)
        engine.link_opened(0.0)
        self.assertEqual(engine.tick(0.5), [])
        self.assertEqual(engine.tick(1.0), [""])  # lone newline first
        engine.note_tx(1.0, "bridge")
        self.assertEqual(engine.tick(1.1), [])
        self.assertEqual(engine.tick(1.21), ["@MOTION TELEMETRY 10"])
        self.assertEqual(engine.tick(4.1), [])
        self.assertEqual(engine.mode, "NONE")
        out = engine.tick(4.22)
        self.assertEqual(engine.mode, "LEGACY_POLL")
        self.assertEqual(out, ["@MOTION VALIDATION SENSORS"])
        self.assertEqual(engine.tick(4.25), [])  # one in flight
        engine.line(SENSORS_LINE, 4.26)
        self.assertEqual(engine.tick(4.27), [])  # >= 100 ms between commands
        self.assertEqual(engine.tick(4.33), ["@MOTION VALIDATION JOINT 0"])
        self.assertEqual(engine.tick(5.0), [])
        self.assertEqual(engine.tick(5.34), ["@MOTION VALIDATION JOINT 1"])  # 1 s timeout
        self.assertEqual(engine.counters["legacy_timeouts"], 1)
        engine.note_tx(5.4, "console")
        engine.line(joint_line(1), 5.41)
        self.assertEqual(engine.tick(6.0), [])
        self.assertEqual(engine.tick(10.3), [])  # paused 5 s after console activity
        self.assertEqual(engine.tick(10.45), ["@MOTION VALIDATION JOINT 2"])
        sent = []
        t = 10.45
        while t < 20.0:
            t += 0.02
            for line in engine.tick(t):
                sent.append(t)
                if "SENSORS" in line:
                    engine.line(SENSORS_LINE.replace("uptime_ms=6424", f"uptime_ms={int(t * 1000)}"), t)
                else:
                    slot = int(line.split()[-1])
                    engine.line(joint_line(slot, stamp=int(t * 1000)), t)
        gaps = [b - a for a, b in zip(sent, sent[1:])]
        self.assertGreaterEqual(min(gaps), 0.099)
        self.assertLessEqual(len(sent) / (sent[-1] - sent[0]), 10.5)
        self.assertIsNotNone(engine.legacy_cycle_ms)
        engine.line(tel_line(30_000), 20.1)  # firmware with @TEL appears: polling stops
        self.assertEqual(engine.mode, "TEL")
        self.assertEqual(engine.tick(20.5), [])

    def test_tel_silence_resubscribes_then_falls_back(self):
        engine = self.make(detect_s=3.0, silence_s=3.0)
        engine.phase = "DETECTING"
        engine.phase_at = 101.0
        engine.line(tel_line(1000), 101.5)
        self.assertEqual(engine.mode, "TEL")
        self.assertEqual(engine.tick(104.4), [])
        self.assertEqual(engine.tick(104.6), ["@MOTION TELEMETRY 10"])
        self.assertEqual(engine.tick(107.7), ["@MOTION VALIDATION SENSORS"])
        self.assertEqual((engine.phase, engine.mode), ("LEGACY_POLL", "LEGACY_POLL"))

    def test_staleness_rules(self):
        engine = self.make()
        engine.line(tel_line(1000), 101.1)
        self.assertFalse(engine.status_payload(102.5)["stale"])
        self.assertTrue(engine.status_payload(102.7)["stale"])  # > 1.5 s since last sample
        engine.line(tel_line(1100, stale=True), 102.8)
        self.assertTrue(engine.status_payload(102.8)["stale"])  # controller-declared stale is kept
        engine.line(tel_line(1200), 102.9)
        self.assertFalse(engine.status_payload(102.9)["stale"])
        engine.link_closed(103.0, mb.S_DISCONNECTED, "unplugged")
        payload = engine.status_payload(103.0)
        self.assertTrue(payload["stale"])
        self.assertEqual(payload["bridge"]["serial_state"], mb.S_DISCONNECTED)
        empty = mb.TelemetryEngine().status_payload(0.0)
        self.assertEqual((empty["available"], empty["stale"]), (False, True))

    def test_fake_mode_labels_every_status(self):
        engine = self.make(fake_device=True)
        engine.line(tel_line(1000, telemetry={"sampled_at_ms": 990, "source": "HARDWARE_SERVO_FEEDBACK"}), 101.1)
        payload = engine.status_payload(101.1)
        self.assertEqual(payload["telemetry"]["source"], mb.FAKE_SOURCE)
        self.assertTrue(payload["bridge"]["fake_device"])
        self.assertEqual(engine.status["telemetry"]["source"], "HARDWARE_SERVO_FEEDBACK")  # stored object untouched


# ----------------------------------------------------------- integration ---

class Stack:
    """Fake pty controller + bridge + HTTP + console on ephemeral loopback ports."""

    def __init__(self, protocol: str = "tel", dist: str | None = None, **fake_kwargs):
        self.tmp = tempfile.mkdtemp(prefix="matdog_bridge_it_")
        self.fake = mb.FakeController(protocol=protocol, link_dir=os.path.join(self.tmp, "by-id"), **fake_kwargs)
        os.mkdir(self.fake.link_dir)
        self.fake.start()
        self.log = mb.SessionLog(os.path.join(self.tmp, "log"))
        self.bridge = mb.Bridge(self.fake.link_path, self.log, self.tmp, fake_device=True,
                                engine_kwargs={"settle_s": 0.3, "detect_s": 0.8, "silence_s": 1.0})
        self.dist = dist or os.path.join(self.tmp, "dist")
        os.makedirs(self.dist, exist_ok=True)
        self.httpd = mb.BridgeHTTPServer(("127.0.0.1", 0), self.bridge, self.dist)
        self.console = mb.ConsoleServer(("127.0.0.1", 0), self.bridge)
        self.bridge.start()
        for server in (self.httpd, self.console):
            threading.Thread(target=server.serve_forever, daemon=True).start()
        self.port = self.httpd.server_address[1]

    def close(self):
        self.httpd.stopping.set()
        self.bridge.stop()
        self.httpd.shutdown()
        self.console.shutdown()
        self.httpd.server_close()
        self.console.server_close()
        self.fake.stop()
        self.log.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def request(self, method: str, path: str, headers: dict | None = None, body: bytes | None = None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        data = response.read()
        connection.close()
        return response, data

    def status(self) -> dict:
        return json.loads(self.request("GET", "/api/motion/status")[1])

    def fresh(self) -> bool:
        status = self.status()
        return status.get("available") is True and not status["stale"]


class IntegrationTests(unittest.TestCase):
    def stack(self, *args, **kwargs) -> Stack:
        stack = Stack(*args, **kwargs)
        self.addCleanup(stack.close)
        return stack

    def test_tel_status_health_headers_and_lifecycle_without_modem_ioctls(self):
        recorder = IoctlRecorder()
        with mock.patch("fcntl.ioctl", side_effect=recorder):
            stack = self.stack("tel")
            self.assertTrue(wait_until(stack.fresh), stack.status())
            response, body = stack.request("GET", "/api/motion/status")
            self.assertEqual(response.getheader("Cache-Control"), "no-store")
            self.assertIsNone(response.getheader("Access-Control-Allow-Origin"))
            status = json.loads(body)
            self.assertEqual(status["telemetry"]["source"], mb.FAKE_SOURCE)
            self.assertEqual(status["bridge"]["source"], "TEL")
            self.assertTrue(status["bridge"]["fake_device"])
            self.assertEqual(len(status["telemetry"]["actual_rad"]), 12)
            self.assertEqual(len(status["servos"]), 12)
            # The fake drops the first received line: it must be the lone flush newline.
            self.assertEqual(stack.fake.received[0], "")
            self.assertEqual(stack.fake.received[1], "@MOTION TELEMETRY 10")
            self.assertGreater(stack.bridge.engine.counters["rx_pre_tx_lines"], 0)
            health = json.loads(stack.request("GET", "/api/bridge/health")[1])
            self.assertEqual(health["serial"]["state"], mb.S_CONNECTED)
            self.assertTrue(health["serial"]["owner_lock_held"])
            self.assertEqual(health["mode"], "TEL")
            self.assertEqual(health["counters"]["parse_errors"], 0)
            self.assertTrue(wait_until(lambda: stack.status()["bridge"]["tel_hz"] > 5))
            # Unplug: SERIAL_DISCONNECTED + stale; replug: fresh again.
            stack.fake.unplug()
            self.assertTrue(wait_until(lambda: stack.status()["bridge"]["serial_state"] == mb.S_DISCONNECTED))
            self.assertTrue(stack.status()["stale"])
            stack.fake.plug()
            self.assertTrue(wait_until(stack.fresh, timeout=8.0))
            self.assertGreaterEqual(stack.bridge.link.opens, 2)
            # Reboot: new boot nonce and uptime restart -> counted, resubscribed, fresh again.
            nonce = stack.status()["boot_nonce"]
            stack.fake.reboot()
            self.assertTrue(wait_until(lambda: stack.status().get("boot_nonce") not in (None, nonce), timeout=8.0))
            self.assertGreaterEqual(json.loads(stack.request("GET", "/api/bridge/health")[1])["counters"]["reboots_seen"], 1)
            self.assertTrue(wait_until(stack.fresh))
        self.assertIn(termios.TIOCEXCL, recorder.requests)
        self.assertFalse(set(recorder.requests) & mb.MODEM_LINE_IOCTLS)

    def test_legacy_poll_end_to_end_with_unmapped_slot(self):
        stack = self.stack("legacy", unmapped_slots=(4,))
        self.assertTrue(wait_until(lambda: stack.status().get("bridge", {}).get("source") == "LEGACY_POLL", 6.0))
        self.assertTrue(wait_until(lambda: stack.status().get("legacy", {}).get("joints_seen") == 12, 6.0))
        status = stack.status()
        self.assertEqual(status["telemetry"]["source"], mb.FAKE_SOURCE)  # fake label wins over legacy
        self.assertIsNone(status["telemetry"]["actual_rad"])
        self.assertIsNone(status["servos"][4]["actual_rad"])
        self.assertIsNotNone(status["servos"][5]["actual_rad"])
        self.assertEqual(status["readiness"]["restore"], False)
        self.assertIsNotNone(status["telemetry"]["quaternion"])
        sent = [line for line in stack.fake.received if line.startswith("@MOTION VALIDATION")]
        self.assertTrue(all(line == "@MOTION VALIDATION SENSORS" or line.startswith("@MOTION VALIDATION JOINT ")
                            for line in sent), sent)

    def test_sse_stream_rate_and_payload(self):
        stack = self.stack("tel")
        self.assertTrue(wait_until(stack.fresh))
        sock = socket.create_connection(("127.0.0.1", stack.port), timeout=5)
        sock.sendall(b"GET /api/bridge/events HTTP/1.1\r\nHost: 127.0.0.1\r\nAccept: text/event-stream\r\n\r\n")
        data = b""
        start = time.monotonic()
        while time.monotonic() - start < 2.0:
            data += sock.recv(65536)
        sock.close()
        header, _, stream = data.partition(b"\r\n\r\n")
        self.assertIn(b"text/event-stream", header)
        self.assertIn(b"Cache-Control: no-store", header)
        events = [chunk for chunk in stream.split(b"\n\n") if b"data: " in chunk]
        self.assertGreaterEqual(len(events), 12)   # ~10 Hz TEL
        self.assertLessEqual(len(events), 41)      # <= 20 Hz cap
        payload = json.loads(events[-1].split(b"data: ", 1)[1])
        self.assertEqual(payload["bridge"]["source"], "TEL")
        self.assertLess(len(events[-1]), 8192)

    def test_stop_endpoint_is_the_only_write_and_rejects_cross_origin(self):
        stack = self.stack("tel")
        self.assertTrue(wait_until(stack.fresh))
        response, body = stack.request("POST", "/api/bridge/stop", {"Origin": "http://evil.example"})
        self.assertEqual(response.status, 403)
        response, body = stack.request("POST", "/api/bridge/stop",
                                       {"Origin": f"http://127.0.0.1:{stack.port}", "Host": f"127.0.0.1:{stack.port}"})
        result = json.loads(body)
        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Cache-Control"), "no-store")
        self.assertEqual(result["written"], list(mb.STOP_COMMANDS))
        self.assertTrue(wait_until(lambda: all(c in stack.fake.received for c in mb.STOP_COMMANDS)))
        self.assertTrue(any("SAFE_OFF_REQUESTED" in reply for reply in result["replies"]), result)
        before = list(stack.fake.received)
        for method, path in (("POST", "/api/motion/command"), ("PUT", "/api/bridge/stop"),
                             ("DELETE", "/api/bridge/stop"), ("POST", "/index.html")):
            response, _ = stack.request(method, path)
            self.assertIn(response.status, (404, 405), (method, path))
        time.sleep(0.2)
        extra = [line for line in stack.fake.received[len(before):] if not line.startswith("@MOTION TELEMETRY")]
        self.assertEqual(extra, [])

    def test_console_is_loopback_single_writer_and_sees_rx(self):
        stack = self.stack("tel")
        self.assertTrue(wait_until(stack.fresh))
        port = stack.console.server_address[1]
        first = socket.create_connection(("127.0.0.1", port), timeout=3)
        first_file = first.makefile("rb")
        self.assertIn(b"CONSOLE_ATTACHED", first_file.readline())
        second = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.assertIn(b"CONSOLE_BUSY", second.makefile("rb").readline())
        second.close()
        first.sendall(b"@MOTION VALIDATION JOINT 7\n")
        self.assertTrue(wait_until(lambda: "@MOTION VALIDATION JOINT 7" in stack.fake.received))
        lines = [first_file.readline() for _ in range(40)]
        self.assertTrue(any(line.startswith(b"@TEL ") for line in lines))
        self.assertTrue(stack.bridge.engine.status_payload(time.monotonic())["bridge"]["console_active"])
        first_file.close()
        first.close()
        self.assertTrue(wait_until(lambda: stack.bridge.console is None))
        self.assertFalse(stack.console.verify_request(None, ("192.168.1.50", 40000)))
        self.assertTrue(stack.console.verify_request(None, ("127.0.0.1", 40000)))
        self.assertEqual(stack.console.rejected_peers, 1)
        with self.assertRaises(ValueError):
            mb.ConsoleServer(("0.0.0.0", 0), stack.bridge)
        self.assertNotEqual(mb.main(["--console", "0.0.0.0:8766", "--fake-device", "--log-dir",
                                     os.path.join(stack.tmp, "refused")]), 0)

    def test_static_viewer_serving_gzip_and_traversal(self):
        stack = self.stack("tel")
        with open(os.path.join(stack.dist, "index.html"), "w") as handle:
            handle.write("<!doctype html><title>twin</title>")
        os.makedirs(os.path.join(stack.dist, "canonical"))
        with open(os.path.join(stack.dist, "canonical", "m.stl"), "wb") as handle:
            handle.write(b"\0" * 4096)
        response, body = stack.request("GET", "/")
        self.assertEqual((response.status, body), (200, b"<!doctype html><title>twin</title>"))
        response, body = stack.request("GET", "/canonical/m.stl", {"Accept-Encoding": "gzip"})
        self.assertEqual(response.getheader("Content-Encoding"), "gzip")
        self.assertLess(len(body), 4096)
        for path in ("/../../etc/passwd", "/%2e%2e/%2e%2e/etc/passwd", "/canonical/../../x"):
            self.assertEqual(stack.request("GET", path)[0].status, 404, path)


if __name__ == "__main__":
    unittest.main()

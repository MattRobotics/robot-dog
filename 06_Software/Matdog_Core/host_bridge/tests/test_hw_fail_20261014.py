#!/usr/bin/env python3
"""14 Oct regressions for the host bridge and diagnostics (offline, loopback only)."""
from __future__ import annotations

import http.server
import os
import socket
import sys
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import matdog_bridge as mb  # noqa: E402
import matdog_http_probe as probe  # noqa: E402
from test_matdog_bridge import tel_line  # noqa: E402


def serve(handler_cls):
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler_cls)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


class Trickle(http.server.BaseHTTPRequestHandler):
    """SSE heartbeats every 0.3 s and never a data event: defeats a per-op timeout."""
    def log_message(self, *a): pass
    def do_GET(self):
        self.send_response(200); self.send_header("Content-Type", "text/event-stream"); self.end_headers()
        try:
            for _ in range(200):
                self.wfile.write(b": heartbeat\n\n"); self.wfile.flush(); time.sleep(0.3)
        except OSError:
            pass


class Silent(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        time.sleep(30)


class Plain(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path == "/sse":
            self.send_response(200); self.send_header("Content-Type", "text/event-stream"); self.end_headers()
            self.wfile.write(b": hb\n\nid: 1\ndata: {\"now_ms\": 5, \"stale\": false}\n\n"); self.wfile.flush(); time.sleep(5); return
        body = b"<html>403 Forbidden ADMIN_AP_REQUIRED</html>" if self.path == "/forbidden" else b'{"now_ms": 7, "stale": true, "stale_scope": "MOTION_JOINT_FEEDBACK"}'
        self.send_response(403 if self.path == "/forbidden" else 200)
        self.send_header("Content-Type", "text/html" if self.path == "/forbidden" else "application/json")
        self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)


class ProbeTests(unittest.TestCase):
    def test_trickling_stream_is_bounded_by_one_total_deadline(self):
        server = serve(Trickle)
        try:
            r = probe.bounded_get(f"http://127.0.0.1:{server.server_address[1]}/api/motion/events", total_s=1.5)
            self.assertFalse(r["ok"]); self.assertTrue(r["error"].startswith("DEADLINE"))
            self.assertLess(r["elapsed_s"], 2.6)
        finally:
            server.shutdown()

    def test_silent_server_is_bounded(self):
        server = serve(Silent)
        try:
            t0 = time.monotonic()
            r = probe.bounded_get(f"http://127.0.0.1:{server.server_address[1]}/", total_s=1.0)
            self.assertFalse(r["ok"]); self.assertLess(time.monotonic() - t0, 2.2)
        finally:
            server.shutdown()

    def test_json_sse_and_non_json_are_reported_truthfully(self):
        server = serve(Plain)
        base = f"http://127.0.0.1:{server.server_address[1]}"
        try:
            ok = probe.bounded_get(base + "/status", 2.0)
            self.assertTrue(ok["ok"]); self.assertEqual(ok["stale_scope"], "MOTION_JOINT_FEEDBACK")
            sse = probe.bounded_get(base + "/sse", 2.0)
            self.assertTrue(sse["ok"]); self.assertEqual(sse["now_ms"], 5); self.assertLess(sse["elapsed_s"], 1.0)
            bad = probe.bounded_get(base + "/forbidden", 2.0)
            self.assertEqual(bad["status"], 403); self.assertFalse(bad["ok"]); self.assertFalse(bad["json"])
            self.assertEqual(probe.bounded_get("https://example.invalid/", 1.0)["error"], "HTTP_URL_REQUIRED")
        finally:
            server.shutdown()


class BridgeTransportTests(unittest.TestCase):
    def test_transport_staleness_is_independent_of_controller_joint_stale(self):
        engine = mb.TelemetryEngine()
        engine.link_opened(99.0); engine.note_tx(99.5, "bridge")  # pre-TX ring lines are never parsed
        # Controller (83c3410) declares stale only because joints are unmapped.
        engine.line(tel_line(1000, stale=True), 100.0)
        payload = engine.status_payload(100.2)
        self.assertTrue(payload["stale"])               # legacy aggregate unchanged
        self.assertFalse(payload["transport_stale"])    # the link itself is fresh
        self.assertTrue(engine.status_payload(102.0)["transport_stale"])  # > 1.5 s without samples
        self.assertTrue(mb.TelemetryEngine().status_payload(0.0)["transport_stale"])

    def test_handler_has_finite_keepalive_timeout_and_resets_are_counted_not_traced(self):
        self.assertTrue(0 < mb.BridgeHandler.timeout <= 30)
        server = mb.BridgeHTTPServer.__new__(mb.BridgeHTTPServer)
        server.client_resets = 0
        import io, contextlib
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            try:
                raise ConnectionResetError(104, "Connection reset by peer")
            except ConnectionResetError:
                server.handle_error(None, ("192.168.1.98", 50000))
        self.assertEqual(server.client_resets, 1)
        self.assertIn("ConnectionResetError", err.getvalue()); self.assertNotIn("Traceback", err.getvalue())


if __name__ == "__main__":
    unittest.main()

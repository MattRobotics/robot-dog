#!/usr/bin/env python3
"""Serve the shipped Dashboard on localhost with read-only offline fixtures.

This program imports no device drivers and has no hardware connection. Physical
semantic commands are always rejected. An offline MotionService trace can feed
status telemetry using --trace PATH (JSONL, array or {"frames": [...]}). STOP is only a
fixture acknowledgement, never a physical safety action.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from time import monotonic

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"

def portal_html() -> str:
    source = (SRC / "network/PortalPage.h").read_text()
    fragments = re.findall(r'R"MATDOG\(([\s\S]*?)\)MATDOG"', source)
    motion = re.search(r'R"MOTION\(([\s\S]*?)\)MOTION"', (SRC / "network/MotionDashboard.h").read_text())
    assert len(fragments) == 1 and motion
    return fragments[0] + motion[1] + "</html>"

def dashboard_csp() -> str:
    policy = (SRC / "network/TwinHostPolicy.h").read_text()
    return "".join(re.findall(r'^\s*"([^"\n]*)"', policy, re.MULTILINE))

def capabilities() -> dict:
    source = (SRC / "motion/MotionContract.cpp").read_text()
    matches = re.findall(r'A\("([^"]+)","([^"]+)","([^"]+)","([^"]+)","([^"]+)",(true|false),', source)
    assert len(matches) == 48
    actions = []
    for ident, category, label, parameters, capability, implemented in matches:
        actions.append(dict(id=ident, category=category, label=label, parameters=parameters, capability=capability,
            prerequisites="Controller evidence + explicit operational GO", implemented=implemented == "true",
            implementation="software_tested" if implemented == "true" else "planned",
            available=ident in ("motion.stop", "motion.status"), reason="OFFLINE_DISPLAY_ONLY"))
    return dict(version=1, hardware_approved=False, teleoperation=False,
        limits=dict(vx_m_s=.03, vy_m_s=.02, yaw_rate_rad_s=.15), gaits=["WALK"], actions=actions,
        reason="OFFLINE_DISPLAY_ONLY")

def load_trace(path: Path) -> list[dict]:
    """Adapt only offline exports; source values and evidence remain intact."""
    if path.stat().st_size > 10 * 1024 * 1024:
        raise ValueError("trace exceeds 10 MiB")
    text = path.read_text()
    try:
        loaded = json.loads(text)
    except json.JSONDecodeError:
        loaded = [json.loads(line) for line in text.splitlines() if line.strip()]
    frames = loaded if isinstance(loaded, list) else loaded.get("frames", [])
    if not frames or len(frames) > 10000:
        raise ValueError("expected 1–10000 status frames")
    previous_now = previous_sample = -1
    normalized = []
    for frame in frames:
        if not isinstance(frame, dict) or frame.get("version") != 1 or not isinstance(frame.get("telemetry"), dict):
            raise ValueError("trace must contain MotionService version 1 status frames")
        telemetry = dict(frame["telemetry"]);telemetry["source"]="OFFLINE_SYNTHETIC"
        telemetry.setdefault("sampled_at_ms", telemetry.get("stamp_ms"))
        now, sample = frame.get("now_ms"), telemetry.get("sampled_at_ms")
        if not isinstance(now, (int, float)) or not isinstance(sample, (int, float)) or now < previous_now or sample < previous_sample or sample > now:
            raise ValueError("missing or reordered source timestamps")
        previous_now, previous_sample = now, sample
        if isinstance(telemetry.get("feet"), dict):
            feet = telemetry["feet"]
            telemetry["feet"] = [dict(feet.get(leg, {}), leg=leg,
                contact={"STANCE": "support", "SWING": "swing"}.get(feet.get(leg, {}).get("state"), "unknown"))
                for leg in ("LF", "RF", "RH", "LH")]
        normalized.append(dict(frame, telemetry=telemetry, owner=frame.get("owner", "offline_trace"),
            fault=frame.get("fault", telemetry.get("fault") or False),
            reason="OFFLINE TRACE: " + str(frame.get("evidence", "source qualification unspecified"))))
    return normalized

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5185)
    parser.add_argument("--trace", type=Path)
    args = parser.parse_args()
    frames = []
    if args.trace:
        try:
            frames = load_trace(args.trace)
        except (ValueError, TypeError, OSError) as error:
            parser.error(str(error))
    start = monotonic()
    html = portal_html().encode()
    caps = capabilities()
    class Handler(BaseHTTPRequestHandler):
        def send(self, body: bytes, content_type="application/json", status=200):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            if content_type.startswith("text/html"):
                self.send_header("Content-Security-Policy", dashboard_csp())
                self.send_header("X-Frame-Options", "DENY")
                self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)
        def json(self, body, status=200):
            self.send(json.dumps(body).encode(), status=status)
        def do_GET(self):
            if self.path == "/":
                return self.send(html, "text/html; charset=utf-8")
            if self.path == "/status":
                return self.send(b"build_id=OFFLINE_DISPLAY_FIXTURE\nmode=MAINTENANCE\nauthority=NONE\ncalibration_hardware_motion_authorized=NO\nactuator_readiness_torque_enable=BLOCKED\nactuator_readiness_position_command=BLOCKED\nimu_rv_count=0\n", "text/plain")
            if self.path == "/wifi/status":
                profile = dict(ssid="", enabled=False, dhcp=True, ip="0.0.0.0", mask="0.0.0.0", gateway="0.0.0.0", dns="0.0.0.0")
                return self.json(dict(network_phase="OFFLINE_FIXTURE", profiles=[profile, profile], scan=[], active_profile=0,
                    sta_test=dict(seq=0), policy=dict(role="OFFLINE_FIXTURE"), build_id="OFFLINE_DISPLAY_FIXTURE"))
            if self.path == "/api/motion/capabilities":
                return self.json(caps)
            if self.path == "/api/motion/status":
                now = int((monotonic() - start) * 1000)
                if frames:
                    origin = frames[0]["now_ms"]
                    t = origin + now
                    selected = next((f for f in reversed(frames) if f["now_ms"] <= t), frames[0])
                    replay = json.loads(json.dumps(selected))
                    replay["now_ms"] = t
                    replay["stale"] = t - replay.get("telemetry", {}).get("sampled_at_ms", 0) > 500
                    return self.json(replay)
                return self.json(dict(version=1, now_ms=now, last_sequence=0, executor_state="OFFLINE_DISPLAY_ONLY",
                    stand_state="OBSERVE", owner="none", fault=False, reason="OFFLINE_DISPLAY_ONLY", stale=False,
                    telemetry=dict(sampled_at_ms=now, actual_rad=None, target_rad=None, quaternion=None, base_height_m=None, feet=None)))
            return self.json(dict(reason="NOT_FOUND"), 404)
        def do_POST(self):
            size = int(self.headers.get("Content-Length", "0"))
            if self.path != "/api/motion/command" or size > 1024:
                return self.json(dict(reason="OFFLINE_DISPLAY_ONLY"), 403)
            try:
                command = json.loads(self.rfile.read(size))
            except (ValueError, TypeError):
                return self.json(dict(reason="INVALID_JSON_SCHEMA"), 400)
            if command.get("type") not in ("MOTION.STOP", "MOTION.STATUS"):
                return self.json(dict(version=1, ack=True, outcome="REJECTED", reason="OFFLINE_DISPLAY_ONLY"), 409)
            return self.json(dict(version=1, id=command.get("id"), sequence=command.get("sequence"), ack=True,
                outcome="COMPLETED", reason="OFFLINE_FIXTURE_ACK_ONLY", owner="dashboard", executor_state="OFFLINE_DISPLAY_ONLY"))
        def log_message(self, *_):
            pass
    if frames:
        print(f"Offline trace: {len(frames)} samples from {args.trace}; no hardware qualification", flush=True)
    print(f"Offline Dashboard: http://127.0.0.1:{args.port} (all physical commands disabled)", flush=True)
    ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()
if __name__ == "__main__":
    main()

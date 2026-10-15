#!/usr/bin/env python3
"""Host emulation of the robot AP serving the phone twin. LOOPBACK ONLY.

Serves dist-mobile/ under /twin/ exactly as the ESP32 would (pre-gzipped
files, Content-Encoding: gzip, immutable cache for hashed assets) and a FAKE
/api/motion/status built from the sanitized 2026-10-14 @TEL counterexample.
?scenario= unmeasured | partial | measured | stale (sticky per server start
via --scenario). No serial port, no robot, no network beyond 127.0.0.1.
"""
import argparse, json, math, mimetypes, sys, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

RAW = [2018, 3002, 3146, 1999, 1196, 944, 2063, 1126, 994, 2049, 3023, 3131]
PRONE = [-2.29, 80.16, -91.76, 2.37, 79.54, -90.62, -2.55, 82.53, -91.41, 2.29, 82.35, -91.41]  # 13 Oct, deg
T0 = time.monotonic()

def status(scenario: str) -> dict:
    now = int(892599 + (time.monotonic() - T0) * 1000)
    yaw = 0.25 * math.sin((time.monotonic() - T0) / 4)
    q = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
    by_joint = [None] * 12
    if scenario in ('partial', 'measured'):
        by_joint = [round(d * math.pi / 180, 8) for d in PRONE]
        if scenario == 'partial': by_joint[4] = None
    mapped = sum(v is not None for v in by_joint)
    sampled = now - (2000 if scenario == 'stale' else 350)
    return {"version": 1, "now_ms": now, "last_sequence": 0, "owner": "none", "executor_state": "OFF", "stand_state": "IDLE",
            "fault": False, "reason": "RESTORE_REQUIRED" if mapped < 12 else "OPERATIONAL_LIMITS_REQUIRED",
            "stale": mapped < 12 or scenario == 'stale', "stale_scope": "MOTION_JOINT_FEEDBACK",
            "telemetry": {"sampled_at_ms": sampled, "actual_rad": by_joint if mapped == 12 else None, "actual_rad_by_joint": by_joint,
                          "raw_ticks": RAW, "target_rad": None, "quaternion": q if scenario != 'stale' else None,
                          "base_height_m": None, "source": "FAKE_AP_HOST_EMULATION", "feet": None},
            "channels": {"imu": {"valid": scenario != 'stale', "age_ms": 8 if scenario != 'stale' else None},
                         "raw": {"count": 12, "max_age_ms": 352}, "mapped": {"count": mapped, "mask": 0}},
            "restore": {"decision": "PENDING_OPERATOR_REVIEW" if mapped < 12 else "ALLOW_OPERATOR",
                        "reason": "INSTALLATION_ATTESTATION_ABSENT" if mapped < 12 else "OK", "trigger": "BOOT", "generation": 1,
                        "fingerprint": "0000000000000000"}}

def handler(root: Path, scenario: str):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/motion/status':
                body = json.dumps(status(scenario)).encode()
                self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body); return
            if path in ('/twin', '/twin/'): path = '/twin/mobile.html'
            if not path.startswith('/twin/'): self.send_error(404); return
            rel = Path(path[len('/twin/'):])
            if '..' in rel.parts: self.send_error(403); return
            gz = root / (str(rel) + '.gz')
            if not gz.is_file(): self.send_error(404); return
            data = gz.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', (mimetypes.guess_type(str(rel))[0] or 'application/octet-stream'))
            self.send_header('Content-Encoding', 'gzip')
            self.send_header('Cache-Control', 'public, max-age=31536000, immutable' if rel.parts[0] == 'assets' else 'no-cache')
            self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)
    return H

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=Path, default=Path('dist-mobile'))
    ap.add_argument('--port', type=int, default=8790)
    ap.add_argument('--scenario', default='unmeasured', choices=['unmeasured', 'partial', 'measured', 'stale'])
    a = ap.parse_args()
    ThreadingHTTPServer(('127.0.0.1', a.port), handler(a.dist.resolve(), a.scenario)).serve_forever()
    return 0

if __name__ == '__main__':
    sys.exit(main())

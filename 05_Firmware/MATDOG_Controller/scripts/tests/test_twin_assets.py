#!/usr/bin/env python3
"""Phone Digital Twin assets embedded in the firmware (directive 2026-10-14).

Checks src/network/TwinAssets.h: every gzip blob decompresses to the declared
size, ETags are the SHA-256 prefix of the raw asset, the page loads only
same-origin resources (no CDN, GitHub or cloud), the JS polls the robot's own
/api/motion/status (or, framed by the dashboard with ?feed=parent, renders the
status the dashboard posts: same origin, parent only, no poll of its own) and
contains no command/serial/servo path, every body is time-bounded, the routes
are registered on every listener, and the budget fits the app slot.
"""
import gzip, hashlib, re, sys
from pathlib import Path
FW = Path(__file__).resolve().parents[2]
SRC = FW / "src"
h = (SRC / "network/TwinAssets.h").read_text()
transport = (SRC / "network/HttpTransport.cpp").read_text()
checks = 0
def check(c, msg):
    global checks; checks += 1
    if not c: print("FAIL", msg); sys.exit(1)
arrays = {m.group(1): bytes(int(x) for x in m.group(3).split(",")) for m in re.finditer(r"const uint8_t (kAsset\d+)\[(\d+)\] = \{([0-9,]+)\};", h)}
table = re.findall(r'\{"(/twin/[^"]*)", "([^"]+)", "\\"([0-9a-f]{32})\\"", (kAsset\d+), sizeof\(kAsset\d+\), (\d+)\}', h)
check(len(table) == 4 and len(arrays) == 4, "four assets")
raws = {}
total = 0
for path, ctype, etag, name, raw_len in table:
    raw = gzip.decompress(arrays[name]); raws[path] = raw; total += len(arrays[name])
    check(len(raw) == int(raw_len), f"{path} raw size")
    check(hashlib.sha256(raw).hexdigest()[:32] == etag, f"{path} etag")
check(f"kTotalGzipBytes = {total};" in h, "total")
check(total < 300_000, f"budget {total}")
html = raws["/twin/"].decode()
check('src="./assets/twin.js"' in html or "src=\"./assets/twin.js\"" in html, "relative JS")
check(not re.search(r'(src|href)="(https?:)?//', html), "no external resource in HTML")
js = raws["/twin/assets/twin.js"].decode("utf-8", "replace")
check('"/api/motion/status"' in js or "'/api/motion/status'" in js, "polls robot status")
check(not re.search(r'fetch\(\s*["\']https?://', js), "no absolute fetch")
for token in ("/api/motion/command", "/api/calibration/restore", "navigator.serial", "requestPort", "/wifi/"):
    check(token not in js, f"no {token} in twin JS")
check(raws["/twin/matdog_collision.mtw"][:4] == b"MTW1", "MTW1 magic")
check(raws["/twin/matt_robodog_rev00.urdf"].lstrip().startswith(b"<?xml") or b"<robot" in raws["/twin/matt_robodog_rev00.urdf"][:400], "URDF")
check(hashlib.sha256(raws["/twin/matt_robodog_rev00.urdf"]).hexdigest() == "3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59", "canonical URDF unchanged")
for path in ("/twin", "/twin/", "/twin/assets/twin.js", "/twin/matdog_collision.mtw", "/twin/matt_robodog_rev00.urdf"):
    check(transport.count(f'"{path}"') == 2 or transport.count(f'"{path}",') >= 2, f"{path} on admin+TLS and read-only listeners")
handler = transport[transport.index("esp_err_t HttpTransport::handleTwin("):transport.index("esp_err_t HttpTransport::handleWifi(")]
check("dispatch(" not in handler and "service_" not in handler, "twin handler never reaches the Controller")
sender = transport[transport.index("esp_err_t sendBoundedResponse("):transport.index("} // namespace", transport.index("esp_err_t sendTwinAsset("))]
check('"If-None-Match"' in handler and "304 Not Modified" in handler, "ETag revalidation")
check(all(h in sender for h in ("Content-Length: %u", "Content-Encoding: gzip", "ETag: %s", "Cache-Control: no-cache", "Accept-Ranges: none")), "gzip + ETag + length, no resumable partial entry")
# 2026-10-24: every asset body goes through the time-bounded writer (test_twin_bounded_send.py).
check("return sendTwinAsset(req,*asset," in handler and "asset->gz" not in handler.replace("asset->gz_len", ""), "body only via sendTwinAsset")
check("kTwinSendBudgetMs" in sender and "select(" in sender and "kTwinWriteBytes" in sender, "budget + writability + one segment per write")
# 2026-10-15: the single httpd task never streams a large Twin asset while a
# Motion torque sequence/motion is in progress (STOP/heartbeat are not queued
# behind it); revalidation (304) stays available and comes first.
check("twin_deferred_.load()" in handler and "503 Service Unavailable" in handler and "TWIN_DEFERRED_MOTION_ACTIVE" in handler, "large assets deferred during Motion")
check(handler.index("304 Not Modified") < handler.index("twin_deferred_.load()"), "304 revalidation precedes the deferral")
check("twin_deferred_.store(service_->motionEngaged());" in transport, "deferral flag published by the Controller thread")
# The read-only Twin is never the operator heartbeat: no credentials, no CSRF.
check(re.search(r'credentials:\s*["\']omit["\']', js) is not None, "twin fetches omit credentials")
# Embedded in the dashboard: one poll per page; only the parent, same origin.
parent_mode = re.search(r'(\w+)=new URLSearchParams\(location.search\).get\("feed"\)==="parent"&&window\.parent!==window', js)
check(parent_mode is not None, "embedded mode bound to feed=parent and framed window")
feed = re.search(re.escape(parent_mode[1])+r'\?window\.addEventListener\("message",(\w+)=>\{\1\.origin!==location\.origin\|\|\1\.source!==window\.parent\|\|', js)
check(feed is not None, "feed=parent: listens to the framing dashboard only, same origin")
check(js.count('addEventListener("message"') == 1, "no other message channel")
check("X-Csrf-Token" not in js, "twin never presents a CSRF token")
status = transport[transport.index("case HttpRequestKind::MOTION_STATUS:"):transport.index("case HttpRequestKind::MOTION_CAPABILITIES:")]
check("portal_.authorize(" in status and "req.csrf" in status and "portal_.current(" not in status, "heartbeat requires session + CSRF (Dashboard only)")
print(f"TWIN_ASSETS: {checks} checks PASS (total gzip {total} B; static, same-origin, read-only)")

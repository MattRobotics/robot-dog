#!/usr/bin/env python3
"""Phone Digital Twin assets embedded in the firmware (directive 2026-10-14).

Checks src/network/TwinAssets.h: every gzip blob decompresses to the declared
size, ETags are the SHA-256 prefix of the raw asset, the page loads only
same-origin resources (no CDN, GitHub or cloud), the JS polls the robot's own
/api/motion/status and contains no command/serial/servo path, the four routes
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
check('"Content-Encoding","gzip"' in handler and '"If-None-Match"' in handler and "304 Not Modified" in handler, "gzip + ETag revalidation")
print(f"TWIN_ASSETS: {checks} checks PASS (total gzip {total} B; static, same-origin, read-only)")

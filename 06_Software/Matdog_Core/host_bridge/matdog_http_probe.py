#!/usr/bin/env python3
"""Deadline-bounded, read-only HTTP probe for MATDOG diagnostics.

urllib's `timeout` is per socket operation: a stream that trickles a byte
every few seconds (SSE heartbeats, a slow AP) never trips it, so a
"timeout=5" script can hang for minutes (14 Oct). This probe enforces ONE
total deadline across connect, headers and body, caps the body size, and for
text/event-stream returns at the first complete `data:` event. GET only: it
cannot send a command. Prints a summary, never the full body.
"""
from __future__ import annotations

import argparse
import http.client
import json
import socket
import sys
import time
import urllib.parse


def bounded_get(url: str, total_s: float = 3.0, max_bytes: int = 256 * 1024) -> dict:
    t0 = time.monotonic()
    deadline = t0 + total_s
    parts = urllib.parse.urlsplit(url)
    if parts.scheme != "http" or not parts.hostname:
        return {"ok": False, "error": "HTTP_URL_REQUIRED", "elapsed_s": 0.0}
    remaining = lambda: max(0.001, deadline - time.monotonic())  # noqa: E731
    conn = http.client.HTTPConnection(parts.hostname, parts.port or 80, timeout=min(1.0, remaining()))
    result: dict = {"ok": False, "url": url}
    try:
        conn.connect()
        sock = conn.sock  # http.client may detach conn.sock once the response owns it
        sock.settimeout(min(1.0, remaining()))
        conn.request("GET", (parts.path or "/") + (("?" + parts.query) if parts.query else ""),
                     headers={"Accept": "application/json, text/event-stream", "Connection": "close"})
        while True:
            try:
                response = conn.getresponse(); break
            except socket.timeout:
                if time.monotonic() >= deadline: raise TimeoutError("headers")
        result.update(status=response.status, content_type=response.getheader("Content-Type", ""))
        sse = result["content_type"].startswith("text/event-stream")
        body = b""
        while len(body) < max_bytes:
            if time.monotonic() >= deadline:
                raise TimeoutError("body")
            if response.isclosed():
                break
            try:
                sock.settimeout(min(1.0, remaining()))
            except OSError:
                pass  # http.client already closed the socket after a complete body
            try:
                chunk = response.read1(4096) if hasattr(response, "read1") else response.read(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            body += chunk
            if sse and b"\ndata: " in b"\n" + body and b"\n\n" in body.split(b"data: ", 1)[1]:
                break
        result["bytes"] = len(body)
        result["truncated"] = len(body) >= max_bytes
        text = body.decode("utf-8", "replace")
        if sse:
            line = next((l for l in text.splitlines() if l.startswith("data: ")), None)
            text = line[6:] if line else ""
        try:
            data = json.loads(text) if text else None
        except ValueError:
            data = None
        result["json"] = isinstance(data, dict)
        if isinstance(data, dict):
            result["keys"] = sorted(data)[:24]
            for key in ("now_ms", "stale", "transport_stale", "reason", "error", "stale_scope"):
                if key in data: result[key] = data[key]
        result["ok"] = 200 <= response.status < 300 and result["json"]
    except (TimeoutError, socket.timeout) as error:
        result["error"] = f"DEADLINE_{error}" if str(error) else "DEADLINE"
    except OSError as error:
        result["error"] = type(error).__name__
    finally:
        conn.close()
        result["elapsed_s"] = round(time.monotonic() - t0, 3)
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("url")
    ap.add_argument("--total-s", type=float, default=3.0)
    ap.add_argument("--count", type=int, default=1)
    ap.add_argument("--interval-s", type=float, default=0.5)
    a = ap.parse_args()
    if not 0 < a.total_s <= 30 or not 1 <= a.count <= 1000:
        print("total-s in (0,30], count in [1,1000]", file=sys.stderr); return 2
    worst = 0
    for i in range(a.count):
        r = bounded_get(a.url, a.total_s)
        print(json.dumps(r, separators=(",", ":")), flush=True)
        worst = max(worst, 0 if r["ok"] else 1)
        if i + 1 < a.count: time.sleep(a.interval_s)
    return worst


if __name__ == "__main__":
    sys.exit(main())

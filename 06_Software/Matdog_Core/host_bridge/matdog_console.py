#!/usr/bin/env python3
"""Operator console for matdog_bridge.py (replaces a second serial monitor).

Connects to the bridge's loopback console port, prints every Controller RX line
and writes each typed line to the serial port through the bridge (which
serializes it with its own writes and logs it). Exactly one console client can
be attached at a time; the bridge pauses legacy polling for 5 s after each
command so replies are not interleaved with poll traffic.

    python3 matdog_console.py                 # 127.0.0.1:8766
    python3 matdog_console.py --filter TEL    # hide @TEL lines (shown as a count)
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--console", default="127.0.0.1:8766", help="bridge console HOST:PORT")
    parser.add_argument("--filter", action="append", default=[],
                        help="hide RX lines starting with this prefix (repeatable), e.g. --filter @TEL")
    args = parser.parse_args()
    host, _, port = args.console.rpartition(":")
    try:
        sock = socket.create_connection((host or "127.0.0.1", int(port)), timeout=5)
    except OSError as exc:
        print(f"cannot reach bridge console {args.console}: {exc}", file=sys.stderr)
        return 1
    sock.settimeout(None)
    hidden = {"count": 0}
    prefixes = tuple(args.filter)

    def reader() -> None:
        with sock.makefile("r", encoding="utf-8", errors="replace") as stream:
            for line in stream:
                if prefixes and line.startswith(prefixes):
                    hidden["count"] += 1
                    continue
                sys.stdout.write(line)
                sys.stdout.flush()
        print("\n[console] bridge closed the connection", file=sys.stderr)

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    try:
        for line in sys.stdin:
            sock.sendall(line.rstrip("\r\n").encode("utf-8") + b"\n")
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        sock.close()
    if hidden["count"]:
        print(f"[console] {hidden['count']} filtered line(s) hidden", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

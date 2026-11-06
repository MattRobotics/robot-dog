# matdog-esp32-bridge — Jetson ↔ ESP32-S3 over USB CDC (dev.15)

**Status: IMPLEMENTED, VERIFIED OFFLINE (pseudo-terminal), NOT HARDWARE-VALIDATED, NOT INSTALLED.**
No Jetson has been connected. Nothing here is enabled or started automatically.

## Contract

The bridge is the only process that opens the Controller tty. It speaks three
dev.15 USB lines and the existing ones, all answered by the Controller's
existing `CommandRouter` through the same `MotionService` the Dashboard uses:

| Line | Reply (one line) |
|---|---|
| `@HOSTLINK HELLO` | `{"hostlink":1,"contract":1,"firmware":…,"version":…,"git_sha":…,"build_id":…,"motion_stack":"MATDOG_SIMPLE_V1","usb_owner":"local","line_max":319,"uptime_ms":…}` |
| `@MOTION CAPABILITIES` | the JSON of `GET /api/motion/capabilities` |
| `@MOTION STATUS` | the JSON of `GET /api/motion/status` (existing) |
| `@MOTION JSON <envelope>` | Motion contract v1 envelope, `owner:"local"` → JSON ack (or `MOTION=REJECTED REASON=…`) |
| `@MOTION STOP` | JSON ack (existing; always accepted) |

No new owner, authority, heartbeat or state machine: the Controller applies the
contract TTLs (`BASE_VELOCITY` ≤ 1000 ms, `EXPIRED`, `FUTURE_TIMESTAMP`), the
1 s joystick deadline (WALK STOP) and STOP exactly as for the Dashboard. With no
host, or a host that stops reading (zero-timeout TX ring, G3.1), Motion is
unchanged.

## Bridge behaviour

- Port: the unique `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<MAC>-if00`
  (`--serial <MAC>` to pin one Controller, `--port` to override). Two matches: refused.
- Open: `matdog_bridge.safe_open` — `os.open`, `TIOCEXCL`, raw termios, line
  speed preserved, never DTR/RTS (they map to EN/IO0 on the ESP32-S3); owner
  `flock`; refuses a tty another process holds.
- Each open: new `link_epoch`, lone newline, HELLO + CAPABILITIES check
  (protocol, contract, motion stack, `usb_owner`, required commands, optional
  `--expect-version/--expect-git`). Mismatch → `INCOMPATIBLE`: STOP only.
- One request in flight, matched by reply id; a timeout is reported, never
  retried. STOP is written immediately, ahead of a pending request.
- `issued_at_ms` = Controller uptime of the last reply + host elapsed time
  (never ahead of the Controller).
- Disconnect / re-enumeration: pending requests fail, reopen every second,
  new epoch. Motion requests must carry the current epoch, so a stream from
  before the drop is refused (`STALE_LINK_EPOCH`); nothing is replayed.
- `SIGTERM`: tty closed and lock released — the port is free for flashing.
- Status poll `--status-period` (default 1 s; 0 = on demand).

Client interface: Unix socket (`/run/matdog/esp32-bridge.sock`, mode 0660),
one JSON object per line each way:

```
{"op":"link"}                                   -> link state, epoch, firmware identity
{"op":"status","fresh":true}                    -> @MOTION STATUS
{"op":"capabilities"}
{"op":"stand","epoch":E,"height_m":0.15}
{"op":"lie_down","epoch":E}
{"op":"velocity","epoch":E,"vx":0.003,"vy":0,"yaw_rate":0}   (send ≥ 2 Hz while moving; silence ends the walk in 1 s)
{"op":"stop"}                                   -> any epoch, any state
```

## Offline tests

```
python3 -m unittest tests/test_matdog_esp32_bridge.py   # pseudo-terminal Controller simulator, 14 tests
```

Firmware side: `05_Firmware/MATDOG_Controller/scripts/tests/run_host_tests.sh`
(`hostlink_motion_contract` in `test_command_router_persistence.cpp`, the real
router + MotionService + SimpleMotion with an inert servo port).

## Future installation and first verification (NOT executed)

Preconditions: the dev.15 HostLink firmware is flashed by an explicit decision;
operator present; robot on its stand or in SAFE_OFF; physical STOP available.

1. Copy `matdog_esp32_bridge.py` and `matdog_bridge.py` to `/opt/matdog/host_bridge/`;
   create user `matdog` in group `dialout`; copy `deploy/matdog-esp32-bridge.service`
   to `/etc/systemd/system/` (do **not** `enable`).
2. Check the stable identity: `ls -l /dev/serial/by-id/` shows exactly one
   `usb-Espressif_USB_JTAG_serial_debug_unit_<MAC>-if00`; no other process holds it.
3. Start by hand: `sudo systemctl start matdog-esp32-bridge`;
   `journalctl -u matdog-esp32-bridge` shows `link READY: 1.0.1-dev.15 <git>`.
4. Read-only: `{"op":"link"}`, `{"op":"status","fresh":true}` through the socket;
   Controller `@MOTION STATUS` unchanged with and without the bridge.
5. With the robot in SAFE_OFF: `{"op":"stop"}` → ACCEPTED; unplug/replug USB →
   new epoch, no command sent after reconnect (bridge log, Controller status).
6. `sudo systemctl stop matdog-esp32-bridge` → `ls -l /proc/*/fd | grep ttyACM` empty;
   the port is free for flashing.
7. Only afterwards, under the supervised Motion sequence: STAND, joystick at
   25 %, release (1 s), STOP, measuring `timing` before/after the bridge.
8. Enable at boot (`systemctl enable`) only after a separate decision.

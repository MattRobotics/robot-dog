# MATDOG 1.0.1-dev.15 — HostLink USB CDC and Linux bridge (Jetson-ready)

> **Historical report, recovered on 2026-11-19.** Taken from local commit `bc1679f` on branch `matdog/dev15-joystick-motion`, which was never published. The text below is unchanged; only this note was added.
>
> **Current state, Stable `v1.0.2` (`1314bc4`):**
> - The HostLink firmware commands described here are integrated: `@HOSTLINK HELLO`, `@MOTION CAPABILITIES`, and `@MOTION JSON` with owner `local`. The candidate source `7997ad4` is an ancestor of `main`.
> - The Linux bridge is in [`06_Software/Matdog_Core/host_bridge/`](../../../06_Software/Matdog_Core/host_bridge/README_ESP32_BRIDGE.md).
>
> **The operational Jetson link is NOT qualified:** no Jetson, no real USB host, and the bridge service is not installed. The item stays open in [OPEN_ITEMS](../../OPEN_ITEMS.md).
>
> The APP `b81d3a4f…`, the build paths and the budgets below belong to the dev.15 candidate of 06/11.

**Firmware: IMPLEMENTED, VERIFIED OFFLINE. Bridge: VERIFIED OFFLINE on a pseudo-terminal simulator. Jetson compatibility: NOT VALIDATED (no Jetson, no real USB opened). Nothing installed or enabled.** No flash, OTA, reset, NVS/EEPROM write, push or merge.

The requested handoff `MATDOG_DEV15_HOSTLINK_JETSON_HANDOFF_2026-11-06.md` was not found on this machine; the work follows the mandate and the repository (I6 HostLink audit, `matdog_bridge.py`).

## Candidate

| Item | Value |
|---|---|
| Source | `7997ad4d4497cd2675421d832362b3a0c144a46d`, CLEAN (Motion candidate `928b8b7` + HostLink) |
| APP SHA256 | `b81d3a4ffaa27a7690b196e77e7174c0cc7851563fcfd64cd3bddb40e62f9e13` (1,691,328 B; `928b8b7` 1,690,160 B) |
| Merged | `a6cfd932337f1baecc55dc05c9231335a23f83cb20bae8d6164db03cedc06044` |
| Partition table | `8f756ecb…` unchanged; `ROBOT_POWERED`, OTA ingest 0 |
| Artifacts | `~/MATDOG/build-output/dev15-joystick-motion-20261105/hostlink-build-7997ad4/artifacts/ROBOT_POWERED-ota0/` |
| Motion checkpoint | `928b8b7` / APP `66066030…` kept (build output unchanged) |

## Inventory

Already present (reused unchanged): USB CDC `HWCDC` with G3.1 (TX timeout 0; ring now 12288 B, enlarged earlier for `@TEL`, not 3072 B), bounded 320 B line reader (64 B per visit, overflow/NUL recovery), no DTR/RTS dependency; `@STOP`/`@MOTION STOP`, `@MOTION STATUS` (JSON), `@MOTION STAND <mm>`, `@MOTION WALK STANDARD|STOP`, `@MOTION PRONE`, `@MOTION DIAG`, `@MOTION TIMING`, `@SYSTEM SOURCE_SIGNATURE`, `@HOSTLINK READINESS` (calibration-era readiness vocabulary); Motion contract v1 (`parseMotionJson`, `validCommand` TTL/timestamp rules, per-owner replay, owners dashboard/jetson/local); `MotionService` capabilities JSON; host tool `matdog_bridge.py` (reset-safe open, owner lock).

Added: `@HOSTLINK HELLO` (protocol 1, contract, firmware/build identity, uptime), `@MOTION CAPABILITIES`, `@MOTION JSON <envelope>` (owner `local`, as the USB text commands; other owners `LOCAL_OWNER_REQUIRED`). Fixed: capabilities still described the removed torque-ON PRONE terminal. Bridge `matdog_esp32_bridge.py`, systemd unit (not installed), `README_ESP32_BRIDGE.md`.

Not added (by design): owner/authority/session/heartbeat/state machine; ROS 2, HTTP, message broker, database.

## Budget versus `928b8b7`

| | `928b8b7` | `7997ad4` |
|---|---|---|
| APP | 1,690,160 B | 1,691,328 B (+1,168 B flash: text +536, rodata +624) |
| RAM `.bss` / `.data` / IRAM | 110,904 / 23,708 / 81,967 B | identical (capabilities share the `@MOTION STATUS` buffer) |
| Stack `SimpleMotion::update` / `generateWalk` | 8,208 / 6,368 B | identical |
| Stack `CommandRouter::update` chain / `handleLine` frame | 10,768 / 2,544 B | identical |
| Motion release cost (libm calls, all scenarios) | — | identical counts |

Nothing runs inside `SimpleMotion::update`: the lines are handled by the existing router visit (already timed as `ROUTER` in MotionTiming). The router reads at most 64 B per visit as before; one HostLink line costs one bounded parse (≤ 319 B) and one `snprintf`. ESP32-S3 timing of these lines: not measured.

## Tests

On `7997ad4` (18/18 PASS): build, `run_host_tests.sh` (real router: `hostlink_motion_contract` + 15,056 router checks; framing mutation controls), `run_simple_motion_tests.sh`, `run_stop_support_tests.sh`, `run_simple_dashboard_tests.sh`, `run_motion_cost_probe.sh`, HTTP fence/transport/slack/static network audit, STOP adapter, Twin assets, dev identity, release gate, motion convergence, `static_audit.py` (PASS), bridge tests (14 new + 20 existing `matdog_bridge`).

Bridge tests (pseudo-terminal Controller simulator): unique by-id discovery; handshake; incompatible protocol / old firmware / unexpected version → STOP only; malformed client requests, NaN, refusal lines; timeout reported, never retried, late reply never answers the next request; one request in flight under 4 concurrent clients; STOP overtakes a pending request; disconnect → new epoch, stale-epoch refusal, nothing replayed; Controller clock reset → FUTURE_TIMESTAMP, no retry; late velocity → EXPIRED at the Controller; only TIOCEXCL ioctl (never DTR/RTS); port and owner lock released on stop; Unix socket JSON lines.

Not testable offline: a host that stops reading the real HWCDC (inherited G3.1 property), Linux re-enumeration of the real device, Jetson USB stack and power.

## Jetson compatibility (not validated)

Expected: standard `cdc_acm`/USB-Serial-JTAG enumeration as `/dev/ttyACM*` with the by-id link, Python 3.8+ stdlib only. To verify on the Jetson: by-id name, `dialout`, no ModemManager/brltty probing the port (DTR/RTS toggling resets the ESP32-S3), USB autosuspend, cable and power path, latency under load. Procedure: `06_Software/Matdog_Core/host_bridge/README_ESP32_BRIDGE.md`.

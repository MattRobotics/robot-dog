# Hardware validation — MATDOG NEXTGEN INTEGRATED CANDIDATE V3, first flash

**Date:** 2026-09-26 · **Branch:** `feat/controller-nextgen-integration-v1` @ `c45858c532e97a5a104fb8330ad9cd909b75663e`

Powered, no-motion hardware session, operator physically present throughout. **First-ever hardware
flash and validation of the I7/I8 network transport (`HttpTransport`/`OtaSession`/`HttpMailbox`) and
of the `OTA_INGEST_ENABLED=1` build-time override.** No motion, no torque, no EEPROM/DALY write, no
merge to `main`.

## Candidate flashed

```text
SOURCE_HEAD          c45858c532e97a5a104fb8330ad9cd909b75663e
BUILD_ID             c45858c532e9
HARDWARE_PROFILE     ROBOT_POWERED
OTA_INGEST_ENABLED   1  (explicit build override; source default remains 0)
APPLICATION_SIZE     1027472
APPLICATION_SHA256   13a1e9504e950d3d4bbc14ed78032019c8bf8bacb75804ca57555069787d5b71
APPLICATION_OFFSET   0x010000 (partition app0, size 0x300000)
```

**Note on provenance:** the operator's authorization referenced source `ad08db6` /
`APPLICATION_SHA256=a067cbb5...`. Between that point and the flash, one documentation-only commit
(`c45858c`, adding a single `.md` file under `09_Logs/`, zero bytes changed under
`05_Firmware/`) advanced the integration branch HEAD. Because `build.sh` embeds the current HEAD's
short hash as a literal `BUILD_ID` string baked into the binary, rebuilding from the now-current,
independently-verified HEAD necessarily produced a different SHA256 despite byte-identical firmware
logic. This was flagged explicitly in the pre-write Section-6 report before the operator's `YES`, and
the binary that was actually flashed is the one built from `c45858c` (verified above), not the one
named in the original authorization request.

## Flash result

`scripts/flash_app_only.sh`, invoked with `MATDOG_FLASH_PROFILE=ROBOT_POWERED
MATDOG_FLASH_OTA_INGEST=1` and the fresh recovery backup as `MATDOG_FLASH_BACKUP`/
`MATDOG_FLASH_BACKUP_MANIFEST`. One retry was needed: the first invocation refused
(`REFUSED=NO_EXPECTED_HASH`) because the manifest's default filename convention
(`<backup>.bin.manifest.txt`) didn't match the manifest file's actual name
(`<backup>.manifest.txt`) — a naming mismatch from the manual recovery session, not a defect in the
gate logic; corrected by passing `MATDOG_FLASH_BACKUP_MANIFEST` explicitly. **Nothing was written to
flash before this correction** — the refusal happened entirely within the pre-write gate chain.

```text
APPLICATION_ONLY_FLASH   = PASS
Write: 1,027,472 bytes (670,654 compressed) at 0x00010000 in 6.0 s — hash verified immediately
       after write by esptool's own write-time digest check.
Independent post-write verify-flash: "Verification successful (digest matched)."
Regions written:     application partition only (app0 @ 0x010000)
Regions NOT written: bootloader (0x0), partition table (0x8000), boot_app0/NVS/otadata (0xe000),
                      servo EEPROM, DALY configuration
```

## Post-flash boot/identity validation

Read-only USB CDC probe (raw termios, no DTR/RTS touch, per the established safe-probe method —
see memory `esp32s3-readonly-probe-without-reset`). `@SYSTEM SOURCE_SIGNATURE`:

```text
build_id=c45858c532e9  firmware=MATDOG Controller  version=0.1.0  profile=ROBOT_POWERED
board=YD-ESP32-S3 N16R8
ota_running_build_id=c45858c532e9  ota_running_image_state=UNDEFINED  reset_reason=OTHER
partition=app0 address=0x010000 size=0x300000
```

Build ID matches the flashed candidate exactly. `reset_reason=OTHER` is noted, not explained — the
esptool RTS-pin hard reset following the flash likely does not map to one of this firmware's named
reset-reason categories; `fatal=NO` was reported, so the firmware's own self-check does not treat it
as a fault.

## Fail-closed re-verification (live, on hardware)

Every property held throughout the entire session, including after starting the Web server and
running the full servo preflight:

```text
@ACTUATOR STATUS   limits_admitted=0  transforms_admitted=0  geometry_bound=NO
                   runtime adapter has no production backend (nullptr)
                   hardware_motion_authorized=NO
@AUTHORITY STATUS  owner=NONE  generation=0  grants=0 (ever)
                   operating_mode=MAINTENANCE  motion_allowed=NO
@CALIBRATION STATUS  hardware_motion=BLOCKED  state=STALE_PENDING_FULL_RECALIBRATION
@OTA STATUS        OTA_INGEST=ENABLED (compiled in, as intended) but
                   OTA_UPDATE state=IDLE  gate=REFUSED_NO_GATE_INSTALLED
@WEB SERVER STATUS started=NO at boot (confirmed before any command touched it)
```

No actuator authority was ever granted. No torque was ever enabled (every servo preflight record
shows `torque_enable=0`). No OTA update was started or attempted.

## Powered no-motion hardware validation

**A. IMU** — BNO085 `ONLINE`/`PASS`, `stream=ON`, `runtime_resets=0`, orientation (`RV`) and
magnetometer telemetry streaming continuously and plausibly (a stationary robot's `GYR` reads ~0 rad/s
throughout). Read-only.

**B. BMS** — DALY `ONLINE`/`PASS`, `comm=OK`, pack 11.8 V, 87.9% SOC, 3 cells reporting
(3956/3949 mV, 7 mV delta — well-balanced), `charge_mos=ON discharge_mos=ON`, `state=STATIONARY`, no
alarms. Read-only; no DALY configuration write attempted.

**C. LED** — `init=OK`, 12 pixels, `data_pin_driven=YES`, `presentation=BOOTING` (later
`READY`-consistent once SYSTEM health transitioned). `@LED TEST` was deliberately NOT exercised this
session — `@LED STATUS` alone already confirms the presentation path works, and lighting the physical
ring wasn't necessary to prove it.

**D. Wi-Fi** — **first-ever real hardware association for this project.** `WIFI_STATE=CONNECTED`,
SSID `WiFi_MANICARDI`, DHCP-assigned `192.168.1.136`, connected on the first attempt
(`radio_starts=1 attempts=1 connects=1 timeouts=0 link_losses=0`). **RSSI measured at -92/-93 dBm** —
right at the edge of receiver sensitivity; noted as a real, un-glossed finding, not a design defect.
**`WIFI_TICK max_us=45612`** (45.6 ms) — the first real measurement of the Wi-Fi runtime's worst-case
per-tick duration under actual hardware conditions; notably higher than a "bounded, non-blocking"
tick budget would suggest at a glance, though this is a one-time cost observed during the initial
connection sequence, not a demonstrated recurring per-tick cost. Both figures are recorded here
exactly as measured, for a future session to characterize further; neither was clamped, hidden, or
explained away.

**E. HostLink/Web** — `@WEB SERVER START`/`STOP`/`START`/`STATUS` cycle completed cleanly on real
hardware (the Item-3 lifecycle fix's first hardware exercise): stop-then-restart succeeded with no
leaked-resource failure, and `@STATUS` continued answering normally over USB CDC while the server
was running, confirming the Controller thread stayed responsive. **`GET /status` from the operator's
machine (192.168.1.149, confirmed to have a direct route to 192.168.1.0/24) could not be completed** —
both `ping` and `curl` to `192.168.1.136` failed with no response (`curl: (7) ... No route to host`
/ 100% ping loss), despite the firmware continuing to self-report `WIFI_STATE=CONNECTED`. Given the
measured RSSI (-92/-93 dBm), the most likely explanation is a link too weak for sustained IP-layer
traffic despite having completed the (lighter-weight) association and DHCP exchange — this is not
confirmed, and no further network troubleshooting (AP configuration, physical repositioning) was
attempted, as that is outside this session's software-validation scope. The Web server itself was
left in the **stopped** state at the end of the session (confirmed via `@WEB SERVER STATUS`).

**F. OTA authentication transport, non-writing phase** — **not exercised.** Reaching `/ota/challenge`
requires the same network path that failed in E; no OTA session, challenge, or firmware transfer of
any kind was attempted or is pending. `OTA_INGEST_ENABLED=1` remained compiled in but structurally
unreachable all session (MAINTENANCE-gated start plus HMAC authentication both still required; the
Web server itself was stopped by the end of the session).

**G. Servo — read only** — `@SERVO PREFLIGHT`: **12/12 joints PASS**, zero mismatches, zero
no-response, zero incomplete. Every joint's expected bus ID matched its observed bus ID exactly;
every joint's persistent EEPROM profile matched the expected `MATDOG_C018_V1` profile exactly
(`persistent_profile=MATCH` on all 12); **`torque_enable=0` on every single joint**, confirming no
torque was ever engaged by this read-only diagnostic. `present_position` values recorded are raw
liveness ticks, not calibrated `q0` — no calibration or motion inference is drawn from them.

## Known-open / TO_TEST items surfaced this session

- Wi-Fi link RSSI (-92/-93 dBm) and its effect on sustained data-plane traffic — `KNOWN_OPEN`,
  not investigated further.
- `WIFI_TICK max_us=45612` — a real measurement, not previously available; whether this recurs on
  every connection attempt or was a one-time cost of this specific session is `TO_TEST`.
- `GET /status`/OTA-challenge network reachability — `TO_TEST`, blocked on resolving the Wi-Fi link
  quality question above (or testing from a client closer to the robot).
- `reset_reason=OTHER` after an esptool RTS-pin hard reset — `KNOWN_OPEN`, not investigated (not
  flagged as fatal by the firmware's own self-check).
- Everything validated this session (boot, IMU, BMS, LED presentation, Wi-Fi association, Web server
  start/stop lifecycle, servo preflight) is now `HARDWARE_VALIDATED` for the specific checks run,
  superseding the `HARDWARE_TO_TEST` classification those items carried in every prior freeze
  document this session produced.

## Outcome

Flash `PASS`. Boot, fail-closed, IMU, BMS, LED, Wi-Fi association, and 12/12 servo preflight all
validated on real, powered hardware for the first time against this candidate. Web server lifecycle
validated over USB CDC; its network-reachable surface (`GET /status`, OTA challenge/authentication)
remains untested pending the Wi-Fi link-quality question above. No motion, no torque, no EEPROM/DALY
write occurred at any point. No merge to `main`, no branch/worktree deletion.

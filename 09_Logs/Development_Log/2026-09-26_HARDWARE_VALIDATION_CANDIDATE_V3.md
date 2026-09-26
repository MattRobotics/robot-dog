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

- `reset_reason=OTHER` after an esptool RTS-pin hard reset — `KNOWN_OPEN`, not investigated (not
  flagged as fatal by the firmware's own self-check).
- Everything validated this session (boot, IMU, BMS, LED presentation, Wi-Fi association, Web server
  start/stop lifecycle, servo preflight) is now `HARDWARE_VALIDATED` for the specific checks run,
  superseding the `HARDWARE_TO_TEST` classification those items carried in every prior freeze
  document this session produced.

## Follow-up session (same date) — network-path diagnosis, no reflash

Continuation on the SAME flashed candidate (`build_id=c45858c532e9`, re-confirmed unchanged; no
rebuild, no reflash). Goal: diagnose the `GET /status`/OTA network-reachability gap left open above,
systematically rather than assuming RSSI alone. Physical RF inspection (antenna, coax, internal
wiring) was explicitly out of scope this round.

**Diagnosis, on the ASUS (192.168.1.149, `enp5s0`):**

- `ip route get 192.168.1.136` resolves via `enp5s0`, as expected (lowest-metric route to that
  subnet).
- `ip neigh show 192.168.1.136` reports **`FAILED`** — no MAC address was ever resolved via ARP.
  For comparison, the router (`192.168.1.254`) shows `REACHABLE`, and three other LAN devices show
  `STALE` (previously resolved, just aged out of cache) — `FAILED` specifically means broadcast ARP
  requests for `192.168.1.136` went unanswered.
- This **rules out a local host firewall** as the cause: ARP resolution happens below any IP/TCP
  packet-filtering layer, so a firewall rule on the ASUS could not produce an ARP-level failure.
  (`iptables`/`nft`/`ufw` were not directly inspectable — none installed/accessible without sudo in
  this environment — but the ARP evidence makes that check moot: a firewall is not a candidate
  explanation for a failure this far down the stack.)
- 6 independent `WIFI STATUS` reads across the session: RSSI ranged **-89 to -93 dBm** — consistently
  at or below the -90 dBm threshold this session's instructions treat as too weak for a meaningful
  transport test.

**Classification: `RF_LINK_TOO_WEAK_FOR_MEANINGFUL_TRANSPORT_TEST`.** Per instruction, network
validation (`GET /status`, `/ota/challenge`, HMAC auth rejection/acceptance) stopped here without
touching firmware, without attempting the tests again against a link already shown non-functional at
Layer 2, and without asking the operator for any physical RF intervention (antenna, coax, cover,
internal wiring — all explicitly out of scope).

**`WIFI_TICK` re-characterized, with a materially better picture than the raw `max_us` figure alone
suggested:** 5 consecutive steady-state reads while already `CONNECTED` showed `last_us` of
**3-19 µs** — tiny, matching the "bounded, non-blocking" design intent exactly.
`max_us=45612` (45.6 ms) stayed byte-identical across every single reading this session and the
prior one, consistent with a **one-time cost from the initial connection/association sequence**,
not a recurring per-tick cost. This meaningfully de-risks the open question the first flash session
left about Wi-Fi tick timing under load — the number to watch going forward is the small, stable
steady-state figure, not the historical spike.

```text
WEB_STATUS_NETWORK           FAIL   (blocked upstream by the RF classification above)
OTA_CHALLENGE_NETWORK        FAIL   (same)
OTA_AUTH_REJECTION           FAIL   (same — never reached)
OTA_VALID_AUTH_NONWRITING    NOT_APPLICABLE
WIFI_RSSI                    -89 to -93 dBm (6 reads)
WIFI_TICK_STEADY_LAST_US     3-19 us (5 reads)
WIFI_TICK_HISTORICAL_MAX_US  45612 (unchanged; one-time, not recurring)
CONTROLLER_RESPONSIVE        YES
FAIL_CLOSED_GATE             PASS
```

## Outcome

Flash `PASS`. Boot, fail-closed, IMU, BMS, LED, Wi-Fi association, and 12/12 servo preflight all
validated on real, powered hardware for the first time against this candidate. Web server lifecycle
validated over USB CDC; its network-reachable surface (`GET /status`, OTA challenge/authentication)
remains blocked on Wi-Fi link quality, now precisely classified as `RF_LINK_TOO_WEAK_FOR_MEANINGFUL_TRANSPORT_TEST`
(ARP-level failure, not merely reduced throughput) rather than left as an unexplained gap. No motion,
no torque, no EEPROM/DALY write occurred at any point across either session. No merge to `main`, no
branch/worktree deletion.

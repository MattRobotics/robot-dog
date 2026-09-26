# MATDOG LED Status Manager V2 final — 2026-09-26

## Status and scope

**IMPLEMENTED / OFFLINE-VALIDATED, with focused hardware PASS for BOOTING breathing,
READY SOC mapping and `@LED SOC TEST`/automatic resume.** Live charging-specific
animations remain **TO_TEST**; reserved FULL/warning/critical facts have no producer.
The exact hardware-validated source is `88062e1a1217f288ebcc161c6213dbb543ea6f8a`; see
[the focused hardware record](2026-09-26_LED_STATUS_MANAGER_V2_HW_VALIDATION.md).

Base remote `origin/main` was fetched and verified exactly as
`f14aa40faf3c8c3fb9bace03dd8b71132f09edd5` before edits. Implementation branch:
`feat/led-status-manager-v2-final`.

This change consumes cached facts and changes LED presentation only. The implementation
passes performed no hardware/serial access or flash. A later operator-authorized
session installed and validated source `88062e1...` as recorded separately. The third
commit aligns documentation only; it is not the flashed firmware source. Motion,
calibration, servo safety, KEY/MOS control and OTA/Wi-Fi behavior remain unchanged.
Main remains unmerged during this closeout.

## Physical map and quantization

| Physical pixel | Clock position |
|---|---|
| 0 | 11 |
| 1 | 12 |
| 2 | 1 |
| 3 | 2 |
| 4 | 3 |
| 5 | 4 |
| 6 | 5 |
| 7 | 6 |
| 8 | 7 |
| 9 | 8 |
| 10 | 9 |
| 11 | 10 |

Native progression is clockwise. Explicit SOC constants freeze the start at pixel 1
(noon), clockwise direction and order `{1,2,3,4,5,6,7,8,9,10,11,0}`.

Completed segments are `floor(clamp(BMS_REPORTED_SOC,0,100)*12/100)`. The stored
binary32 value is promoted to binary64 before multiplication, then truncated after
clamping. There is no epsilon or upward rounding. Tests straddle every exact rational
boundary with adjacent representable floats; 25%, 50%, 75%, 100% are exactly representable.
For example, 8.33% is below one complete twelfth and displays zero completed segments;
8.4% displays one. DALY currently reports in tenths of a percent. Finite negative and
above 100% inputs clamp to zero/twelve; NaN and infinities are indeterminate.

This is a view of BMS-reported SOC, with no claim of capacity accuracy or true FULL.

## Rendering and priority

Priority, highest first:

`FAULT > FIRMWARE_UPDATE_IN_PROGRESS > CALIBRATION_IN_PROGRESS > CHARGING_FAULT > BATTERY_CRITICAL > DEGRADED > BATTERY_WARNING > WIFI_CONNECTING > BOOTING > CHARGE_COMPLETE_VERIFIED > CHARGING > READY`

| State | Presentation at the current brightness ceiling 60 |
|---|---|
| FAULT | Solid red, 60 |
| OTA | Blue `(0,80,255)`, breathing 6..60 over 2 s |
| Calibration | Violet `(160,0,220)`, breathing 6..60 over 2 s |
| Charging fault | Red, breathing 6..60 over 2 s |
| Battery critical (reserved) | Red `(255,0,0)`, breathing 6..30 over 3 s |
| Degraded | Solid amber `(255,140,0)`, 30 |
| Battery warning (reserved) | Amber `(255,140,0)`, breathing 6..20 over 3 s |
| Wi-Fi connecting | Cyan `(0,200,200)`, breathing 6..60 over 2 s |
| BOOTING | White, breathing 6..20 over 3 s |
| Verified charge complete (reserved) | All green, breathing 6..20 over 3 s |
| Charging | Completed green segments 20; next logical segment breathing 6..20 over 3 s |
| READY | Completed green segments 20, remaining pixels off; zero segments means all off |
| Indeterminate battery | Amber, breathing 6..20 over 3 s within READY/CHARGING when numeric SOC is unavailable |

At charging 100%, logical segment 11 (physical 0, eleven o'clock) breathes while the
other eleven stay fixed green. Reported 100% never selects verified charge complete.
The reserved fact defaults false and has no production producer. Its renderer is
prepared for a future reviewed power policy; no voltage/current/stability heuristic
was added. The presentation also gates that future fact on fresh telemetry and no alarm.

## Cached telemetry contract

`Controller` supplies cached sample validity, latest communication result, sample age,
SOC, `state_name == CHARGING` and whether any of four alarm words is nonzero.
There is no additional DALY transaction or scheduler invocation.

`kDalyTelemetryFreshnessMs=5000` sits next to `DalySample`; the pre-existing
`kDalyKeyWriteMaxTelemetryAgeMs` aliases it without changing the 5000 ms write gate.
The scheduler polls every 2000 ms, allows 750 ms for telemetry and serializes operator
transactions with quiet gaps. This reuses the existing bound rather than introducing
an LED timeout. A sample is usable only when valid, latest comm is OK and sample age
is **at most 5000 ms**. An unsuccessful poll invalidates presentation immediately even
if an older good sample remains cached. Charging and alarms share the same gate.

Controller obtains the LED clock after DALY update so a newly stamped sample cannot
appear ancient through unsigned subtraction from the tick-start time.

## Ownership and diagnostics

`Controller -> LedStatusPolicy -> LedStatusManager -> LedRing` remains the only
periodic presentation path. The policy remains pure and host-linked; `LedRing` alone
uses NeoPixel. Per-pixel scaling reproduces the existing NeoPixel `(brightness+1)/256`
channel quantization, preserving legacy effect output while permitting one pulsing pixel.

`@LED TEST` remains the native 0..11 chase, 120 ms per step and completion at 1560 ms with
regular updates. `@LED SOC TEST` is LED-only and automatically renders 0..12 levels,
a 1200 ms pause and 12..0, with 600 ms per level and completion at 16800 ms. It uses absolute
elapsed time, handles skipped ticks and clock wrap, and ignores an older tick timestamp
on its start tick. Neither diagnostic blocks. The manager yields frames during either
diagnostic, keeps its snapshot current and resumes automatically when it ends.

`@LED STATUS` reads that snapshot: presentation, SOC validity/value/segments, charging,
charging fault, verified completion, battery warning/critical and diagnostic
NONE/CHASE/SOC_TEST. Invalid SOC is printed UNKNOWN. There is no remote arbitrary pixel API.

USB_ONLY refuses both starts, keeps GPIO47 INPUT and never calls NeoPixel begin/show.
Host tests compile the actual ring and manager under each profile against inert transport
stubs; these tests exercise production behavior without device access.

## Offline evidence

- Full C++ host suite: PASS, including real LED policy and both driver/manager profiles.
- Full `scripts/static_audit.py`: PASS, including existing DALY and actuator mutation
  suites, Python provenance/partition suites and new LED boundary/production mutations.
- Both `USB_ONLY` and `ROBOT_POWERED` firmware builds: PASS using `scripts/build.sh`.
- `git diff --check`: PASS. Changed files reviewed for scope and credential leakage.
- Local Wi-Fi/OTA credential files remained ignored and untracked; contents were not
  printed or committed. No dependency was added.

The implementation passes generated clean builds after their source commits. Their ignored
manifests bind application size/SHA256, profile, OTA ingest fact and exact source commit.
The final handoff report records those candidate values. The USB_ONLY build uses ingest 0;
the ROBOT_POWERED validation candidate uses the existing explicit ingest 1 build override,
matching the previously installed candidate's capability. Source defaults remain
USB_ONLY and ingest 0. No OTA ingest operation or flash was performed during those
offline implementation passes.

## Focused physical validation and remaining checks

The [focused hardware session](2026-09-26_LED_STATUS_MANAGER_V2_HW_VALIDATION.md)
passed application-only installation/digest verification, BOOTING breathing, servo
preflight 12/12 with `torque_enable=0`, READY transition, real 75.x% SOC to nine LEDs,
noon/clockwise mapping, and SOC diagnostic fill/drain/automatic resume. Authority
remained NONE and no calibration motion occurred. BOOTING was perceived as white /
slightly cyan-ish; exact hue was not calibrated. The firmware source remains
`88062e1a1217f288ebcc161c6213dbb543ea6f8a`, regardless of later docs or merge commits.

Live CHARGING/next-segment and 100% tail breathing, a real charging-fault alarm, and
stale/invalid amber were not exercised by this focused session. `@LED SOC TEST` uses
fixed bar frames and does not simulate charging, so absence of a charging pulse in
that diagnostic is expected. FULL/warning/critical active presentations remain
unvalidated on hardware and have no production producers.

True charge-completion policy, autonomous dock and unattended charging qualification
remain FUTURE. RF/data-plane and OTA network authentication remain open from the prior
hardware session. Full calibration is a separate workstream after LED review and focused
hardware validation; this change does not authorize or implement calibration motion.

## Contract completion — reserved battery-policy facts

The second normal commit on `feat/led-status-manager-v2-final` extends the previous
candidate `021cab7f70a4eb4c1c345ea58349ba3852416503` with `battery_warning` and
`battery_critical` in the presentation input and snapshot. Both default false and
have **no production producer**. Controller remains unchanged and leaves both false.
There is no SOC, voltage or current threshold, new DALY transaction or control action.
A future separately reviewed battery-policy owner must decide these facts and their
validity; the renderer copies them without deriving battery policy from telemetry.

The priority and effect table above describe the completed contract. Warning is
amber `(255,140,0)`, breathing 6..20 over 3 s. Critical is red `(255,0,0)`, breathing
6..30 over 3 s. FAULT remains solid red at 60; charging fault remains red breathing
6..60 over 2 s; DEGRADED remains fixed amber at 30. `@LED STATUS` exposes both facts
from the manager snapshot, including when another presentation has higher priority.

Physical mapping, conservative SOC quantization, READY/charging rendering, reported
100% charging behavior, reserved true FULL, freshness, both diagnostics and USB_ONLY
protection retain the previous candidate's contract. LED presentation still has one
owner; motion/calibration/servo and BMS-control semantics are unchanged.

At this implementation pass, the completion was **IMPLEMENTED / OFFLINE-VALIDATED**.
The later focused hardware results and remaining boundaries are recorded above.
The full host suite and static/mutation audits passed, including exhaustive new
priorities and reserved-fact producer tripwires. Clean USB_ONLY and ROBOT_POWERED
builds were generated from the second commit; the latter retains the
existing validation OTA-ingest override. Their manifests and exact application
identity are recorded in the final completion report. No hardware access, flash,
merge, amendment or force-push was part of that implementation pass.

# G3 — ROBOT_POWERED live validation plan

**Status:** `TO_TEST` — designed during G2, **NOT EXECUTED**.
**Authorization:** requires explicit operator authorization for a powered session. G2
authorized software preparation only.

This document is the procedure design produced at the end of G2. Nothing in it has been
performed. No external rail was energized during G2; the robot's servo power, LED 5 V
rail and DALY-protected domain were all OFF for the entire G2 session.

---

## 0. What G2 delivered that G3 consumes

| Capability | State after G2 |
|---|---|
| `ROBOT_POWERED` profile | `IMPLEMENTED` — one-symbol switch, offline-tested, **not validated** |
| Canonical 17 / expected-now 13 / absent-by-design 4 | `IMPLEMENTED` |
| `@SERVO CENSUS` structured classification | `IMPLEMENTED` — offline-tested only |
| `SAFE_OFF` readback semantics | `VALIDATED` at V0.1 (USB_ONLY, unpowered bus) — real powered readback is a G3 first |
| DALY read-only decoder | `IMPLEMENTED` — never exercised against a live BMS |
| LED ring | `IMPLEMENTED` — never driven; rail has always been unpowered |

Note the asymmetry G3 must respect: under `USB_ONLY` a `SAFE_OFF` could only ever return
`UNVERIFIED_NO_RESPONSE`. P5 below is the **first time** `VERIFIED_OFF` can be observed,
so it is the first real test of that classification path, not a re-confirmation.

## 1. Entry preconditions (all must hold before any rail is energized)

Software:
- G2 offline acceptance PASS on the exact commit to be flashed.
- Working tree clean; compiled build id matches `HEAD`.
- 16 MiB full-flash backup verified (size **and** SHA256) immediately before flashing.
- Firmware built with the profile override:

  ```bash
  MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh
  ```

  **This is the moment the static audit's `USB_ONLY`-default gate is deliberately
  overridden for one build** — via the build-time `-D`, not by editing the source
  default, so the repository default remains `USB_ONLY` and the audit keeps protecting
  `main`. `build.sh` echoes `profile : ROBOT_POWERED (OVERRIDE — requires G3
  authorization)` and the boot banner reports the same, so such an image cannot be
  produced or flashed silently. Confirm both before energizing anything.
- Flashed with `scripts/flash_app_only.sh` only. No full upload, no erase, no partition
  rewrite.

Physical:
- Robot mechanically supported/suspended; legs cannot bear load or reach a surface.
- Accessible hardware disconnect and DALY `KEY` reachable without leaning over the robot.
- Fuse/protection installed; polarity verified; no exposed short risk.
- Battery state known and within a sane range before connection.
- USB-C recovery path still physically connected and proven working beforehand.

Abort immediately and preserve evidence on any of the global stop conditions
(unexpected motion, SAFE_OFF non-response, duplicate/unexpected bus ID, backfeed,
DALY protection event, unstable 5 V, repeated resets, overheating, wiring uncertainty,
provenance mismatch).

## 2. Sequence

Each phase is a **stop point**: record the result, confirm it matches the expectation,
then proceed. One step at a time — do not batch hardware commands.

### P1 — protected-domain boot (no servo power yet if separable)
Energize battery → DALY → P- → step-down → ESP32.

Expect: clean single boot; banner shows `profile : ROBOT_POWERED (servo_power=YES
battery=YES led_rail=YES)`; `reset_reason : POWERON`; `startup_motion/startup_torque/
startup_servo_scan : DISABLED`; no servo twitch of any kind at power-on.

Fail → cut power. Do not proceed.

### P2 — DALY live, read-only
`@BMS STATUS`, then `@BMS STREAM ON` for a few minutes.

Expect: `DALY init=OK detected=ONLINE expected=REQUIRED result=PASS`; plausible pack
voltage/current/SOC; cell count and per-cell min/max/delta consistent with a 3S pack;
temperatures; MOS states; alarm words; sample age staying fresh.

This is also the first live test that `expected=REQUIRED` is correct — under
`ROBOT_POWERED` a silent BMS is now a `FAULT`, by design.

No DALY writes. `@SYSTEM SHUTDOWN` must not be issued during G3.

### P3 — LED ring live
`@LED STATUS` (confirm `data_pin_driven`), then a single conservative `@LED TEST`.

Expect: boot state OFF; test runs at the capped brightness; ring returns to a defined
state; **no** disturbance to BNO085/DALY/ST3215 timing while it runs.

Verify the LED classification is `OPTIONAL`, not `REQUIRED` — a ring fault must degrade,
never fault.

### P4 — ST3215 powered read-only census
`@MODE STATUS` (confirm `MAINTENANCE`), then `@SERVO CENSUS`.

Expected result — the acceptance criterion for this gate:

```text
SERVO_CENSUS=PASS lo=11 hi=55
  canonical_allocated=17 expected_now=13
  present_expected=13 missing_expected=0 absent_by_design=4
  absent_by_design_present=0 unexpected_id=0 not_probed=0 truncated=NO
```

Explicitly: 52/53/54/55 silent is the **correct** outcome, not a failure. Any
`MISSING_EXPECTED`, `UNEXPECTED_ID` or `ABSENT_BY_DESIGN_PRESENT` → stop and resolve
bus/power/wiring before anything else.

Then `@SERVO READ <id>` for each of the 13, recording position/speed/load/voltage/
temperature/**torque state**. Torque must read 0 on every servo. Repeat the census 3–5
times to confirm stability across scans.

No motion. No Torque ON. No GoalPosition. No EEPROM access.

### P5 — SAFE_OFF real readback
`@SERVO SAFE_OFF <id>` for each of the 13 expected servos, one at a time.

Expect `VERIFIED_OFF` on every one. `UNVERIFIED_NO_RESPONSE` on a powered bus means the
servo did not answer the readback — investigate, do not retry blindly.
`VERIFY_FAILED` (responded but torque non-zero) is a stop condition.

### P6 — concurrent soak
BNO085 streaming + DALY streaming + repeated 13-servo reads + LED active + USB CDC, run
simultaneously for a sustained period.

Monitor: heap free / min free, reset reason unchanged, sample freshness for all three
buses, `max_ping_us` stability, no serial collisions, no watchdog events, no module
starvation, no unexpected torque, and the aggregated system health staying `READY`.

### P7 — power-down
Return to a known-safe state, SAFE_OFF confirmed, then hardware power off via `KEY`.
Observe and record actual `KEY` behaviour — do not assert `KEY` equals discharge-MOS
control until measured.

## 3. PASS criteria

All of: clean boot, no resets, DALY live and plausible, LED validated without timing
interference, census exactly `PASS` (13 present / 4 absent by design / 0 unexpected /
0 missing) and stable across repeats, `VERIFIED_OFF` on all 13, soak stable, and **no
motion of any kind observed at any point**.

## 4. Explicitly out of scope for G3

Torque ON, GoalPosition, any motion, servo EEPROM/ID writes, `CalibrationOfs`, factory
reset, broadcast writes, DALY writes / MOS control, BNO085 DCD writes, provisioning, QC
motion, calibration (H2+), partition rewrite, full erase.

A G3 pass proves **powered bus visibility and safe-state semantics**. It does not
supersede formal Full Leg Calibration H1 (last formal result: 6/12), and it does not
make any joint motion-eligible — calibration remains
`CALIBRATION_RESET_PENDING_FULL_RECALIBRATION`.

## 5. After G3

If PASS, record the evidence in `VALIDATION.md` with the exact flashed source commit and
application SHA256, and only then consider promoting `ROBOT_POWERED` from
`IMPLEMENTED` to `VALIDATED`. The repository's default profile stays `USB_ONLY` until a
reviewed decision says otherwise.

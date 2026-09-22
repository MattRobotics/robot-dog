# H0 — CURRENT 12/12 LEG SERVO READ-ONLY PREFLIGHT

**Runbook for the operator · prepared 2026-09-22 · NOT YET EXECUTED**

Strictly read-only on the servo bus. The only write permitted anywhere in this gate is
`SAFE_OFF` / torque OFF, which the architecture already authorises as a safety de-escalation.

```text
NO Torque ON · NO GoalPosition · NO EEPROM write · NO provisioning
NO motion · NO flashing without a new explicit authorization
```

---

## 1. FIRMWARE CAPABILITY STATUS

### 1.1 What is actually on the board

Determined from the repository's own flash records — **not** assumed from the local branches.
None of `feat/controller-wifi-ota-v1`, `…-calibration-manager-v1`, `…-safe-actuator-layer-v1` or
`…-calibration-bootstrap-v1` has ever been flashed.

| Identity | Value |
|---|---|
| Last recorded flash | 2026-09-19, DALY KEY single-write session |
| Source commit | **`6322563`** — `fix(controller): recheck mode before DALY KEY write` |
| Branch | `feat/daly-key-readonly-probe` (frozen parent), contains `main` @ `dd5746c` |
| Application SHA256 | `e9283ced5801d87d5fe44f95411ead2de6d6f6dad2101c88b210c31cd0e645b6` |
| Hardware profile | `ROBOT_POWERED` build override |
| Flash path | `MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh`, `app0 @ 0x010000` |

> **Documentation discrepancy, resolved.** `VALIDATION.md` § *Present-day baseline* still says
> the powered build on the robot is `e2fc605` / `e2b474b5…`. That table was written on
> 2026-09-18 (commit `055d93a`) and was never updated after the 2026-09-19 flash. The later
> record at `VALIDATION.md` § *DALY KEY — SINGLE LIVE CONFIGURATION WRITE* is authoritative:
> `6322563` is what is installed. **Step 3 of the procedure confirms this from the board
> itself** rather than trusting either document.

### 1.2 Capability matrix — the H0 record, field by field

| # | H0 field | Installed `6322563` | How |
|---:|---|---|---|
| 1 | `physical_unit` | ⚠️ **NOT OBSERVABLE** | Firmware has no unit label. Expected value comes from `MATDOG_SERVO_ALLOCATION.yaml`; it cannot be confirmed against the servo. |
| 2 | `joint` | ✅ | canonical `bus_id → joint` table, printed on anomalies |
| 3 | `expected_bus_id` | ✅ | allocation YAML / canonical table |
| 4 | `observed_bus_id` | ✅ | `@SERVO CENSUS` |
| 5 | `model` | ❌ **BLOCKER** | `ServoBus::readModel()` exists but **no command reaches it** |
| 6 | `PositionOffset` | ❌ **BLOCKER** | no register read exists; `SMS_STS_OFS_L/H` are *forbidden literals* in `static_audit.py` |
| 7 | persistent profile result | ❌ **BLOCKER** | no `MATDOG_C018_V1` verification in the Controller at all |
| 8 | `TorqueEnable` | ✅ | `@SERVO READ <id>` → `torque=` |
| 9 | `PresentPosition` / raw | ✅ | `@SERVO READ <id>` → `position=` |
| 10 | `result` | ✅ | derived from the above |

### 1.3 Verdict

```text
H0 AS FULLY SPECIFIED      CANNOT BE EXECUTED on the installed firmware
                           3 of 10 record fields are unreachable

H0-A  population / identity / liveness / raw    EXECUTABLE NOW, no flash
H0-B  model / PositionOffset / profile          BLOCKED, needs firmware capability
```

**I have not flashed anything and will not without a new explicit authorization.** §7 states
exactly what H0-B would require, so the decision is yours.

H0-A is still the gate worth running: it is the half that converts the historical **6/12** into
a current result, and it needs no firmware change.

---

## 2. SAFETY ENVELOPE

| | |
|---|---|
| Bus owner | the ESP32-S3 Controller, exclusively. Do **not** attach any bench tool to the servo bus. |
| Mode required | `MAINTENANCE` (the boot default). `@SERVO CENSUS` and `@SERVO READ` are refused in `RUN`. |
| Bus traffic | `Ping` and register **reads** only |
| Only write allowed | `@SERVO SAFE_OFF <id>` — torque OFF, cannot add torque, never touches EEPROM |
| Primary abort | **hardware power cut.** Have the fused disconnect within reach before starting. |
| Secondary abort | `@SERVO SAFE_OFF <id>` per joint — secondary only, never a substitute |
| Physical | robot on a flat surface, all four feet down, nothing under the legs, no hand in the leg workspace |

**The KEY switch is not a safety barrier.** The `B-`/`P-` bypass makes it inconclusive; the fused
disconnect is the trusted isolation.

### The USB connection must not reset the board

The ESP32-S3 enumerates as its **native USB-Serial-JTAG** peripheral (`303a:1001`,
`/dev/ttyACM0`), which maps DTR/RTS to EN/IO0. A careless port open resets the board mid-session.

Use the no-reset method already proven on 2026-09-17 and 2026-09-19:

- `os.open(O_RDWR|O_NOCTTY|O_NONBLOCK)` + `termios` raw mode;
- **never** touch DTR/RTS (no `TIOCMBIS`/`TIOCMBIC`) — avoid `pyserial`, which sets them;
- **never** change the line speed — preserve `ispeed`/`ospeed` verbatim (a 1200-baud touch is
  bootloader entry);
- **the first write after opening is dropped.** Collect for ~1 s, clear `ECHO`, send a lone
  `\n` to flush the Controller's line buffer, *then* send the first real command;
- do **not** use `esptool` for anything in this gate — every `esptool` path resets the board.

Confirm no reset happened: `runtime_resets=0` and a monotonically increasing `uptime_ms` across
the whole session.

---

## 3. PROCEDURE

Run the whole session in **one** persistent connection. Log everything.

### Step 1 — pre-session, host side, no hardware

```bash
ls -l /dev/serial/by-id/          # MAC is in the symlink; do NOT use `esptool read-mac`
```

Expected: `usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00`.

### Step 2 — power and connect

1. Robot on a flat surface, four feet down, workspace clear.
2. Fused disconnect reachable.
3. Power the robot normally (servo rail up — H0 needs a powered bus).
4. Open the port with the no-reset method. Collect ~1 s. Clear `ECHO`. Send a bare `\n`.

### Step 3 — confirm identity and mode *before any bus traffic*

```text
@STATUS
@MODE STATUS
```

**Do not proceed** unless `MODE=MAINTENANCE`. If it reports `RUN`:

```text
@MODE MAINTENANCE
```

Record from `@STATUS`: health, power state, mode, `uptime_ms`, `runtime_resets`, the `SERVO_POP`
line, and the DALY block (pack voltage — a sagging rail invalidates a census).

> `@STATUS` does not print the build id — that appears only in the boot banner, and obtaining it
> costs a reboot. **Do not reboot to get it.** Identity is established by the flash record in
> §1.1 plus behavioural confirmation: the installed firmware must accept `@BMS KEY STATUS` and
> must **not** know `@AUTHORITY STATUS` or `@CALIBRATION STATUS`. Send both as a probe:
>
> ```text
> @BMS KEY STATUS          -> expect a cached KEY snapshot   (present in 6322563)
> @AUTHORITY STATUS        -> expect ERROR / unknown command (absent in 6322563)
> ```
>
> If `@AUTHORITY STATUS` answers, the board is **not** running `6322563`. **Stop and report.**

### Step 4 — census (the population and bus-ID half)

```text
@SERVO CENSUS
```

Asynchronous: it prints `SERVO_CENSUS=STARTED lo=11 hi=55` immediately and the result some time
later. **Keep collecting** until the `SERVO_CENSUS=<verdict>` block arrives.

Run it **twice**, with ≥5 s between runs. Two identical censuses is the standard set by G3.

### Step 5 — per-joint read, 12 joints

One command per leg servo, in this order:

```text
@SERVO READ 11
@SERVO READ 12
@SERVO READ 13
@SERVO READ 21
@SERVO READ 22
@SERVO READ 23
@SERVO READ 31
@SERVO READ 32
@SERVO READ 33
@SERVO READ 41
@SERVO READ 42
@SERVO READ 43
```

ID 51 (`NECK_ROTATION`) is installed and will appear in the census, but it is **not** part of the
12-joint leg record.

### Step 6 — SAFE_OFF confirmation, 12 joints

```text
@SERVO SAFE_OFF 11
@SERVO SAFE_OFF 12
… through …
@SERVO SAFE_OFF 43
```

This is a torque-OFF write. It cannot add torque and never touches EEPROM. Allowed in any mode.

### Step 7 — re-read to confirm torque is off

Repeat Step 5. Every joint must now report `torque=0`.

### Step 8 — close out

```text
@STATUS
```

Confirm `runtime_resets=0` and that `uptime_ms` increased monotonically across Steps 3 and 8.
Then close the port without touching DTR/RTS.

---

## 4. EXPECTED OUTPUT

### `@SERVO CENSUS` — expected PASS shape

```text
SERVO_CENSUS=STARTED lo=11 hi=55
SERVO_CENSUS=PASS lo=11 hi=55
  canonical_allocated=17 expected_now=13
  present_expected=13 missing_expected=0 absent_by_design=4
  absent_by_design_present=0 unexpected_id=0 not_probed=0 truncated=NO
SERVO init=OK detected=ONLINE expected=REQUIRED result=PASS
```

13 present = 12 legs + ID 51. 4 absent by design = IDs 52/53/54/55 — **their silence is
healthy, not a failure.**

Any of these lines means **not PASS**:

```text
  MISSING_EXPECTED id=<n> joint=<J>          a leg servo did not answer
  ABSENT_BY_DESIGN_PRESENT id=<n> joint=<J>  an unmounted servo answered
  UNEXPECTED_ID id=<n>                       something outside the allocation answered
```

### `@SERVO READ <id>` — expected shape

```text
SERVO_READ id=11 position=<0..4095> speed=<n> load=<n> voltage=<n> temp=<n> torque=<0|1>
```

`position` is the **raw encoder tick**. Nothing else is available per joint in this firmware.

> **`position` is NOT q0.** This capture is preflight liveness evidence only. q0 is a separate,
> later step that requires the robot manually aligned to the nominal URDF q=0 pose with torque
> confirmed off, and it has its own gate. Do not record these values as calibration.

### `@SERVO SAFE_OFF <id>` — expected shape

```text
SERVO_SAFE_OFF id=11 result=VERIFIED_OFF
```

| Result | Meaning |
|---|---|
| `VERIFIED_OFF` | readback responded and confirms `TorqueEnable == 0` — **the only PASS** |
| `UNVERIFIED_NO_RESPONSE` | no readback at all; proves nothing. On a powered bus this is a **failure**. |
| `VERIFY_FAILED` | readback responded and torque is still on — **failure, stop** |

### The record — one row per joint, 12 rows

| physical_unit | joint | expected_bus_id | observed_bus_id | model | PositionOffset | profile result | TorqueEnable | PresentPosition | result |
|---|---|---:|---:|---|---|---|---|---|---|
| M22 | LF_HIP | 13 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| ELR01 | LF_UPPER | 12 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| M33 | LF_LOWER | 11 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| NEW01 | RF_HIP | 23 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| ELR03 | RF_UPPER | 22 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| NEW03 | RF_LOWER | 21 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| NEW06 | RH_HIP | 33 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| ELR02 | RH_UPPER | 32 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| NEW05 | RH_LOWER | 31 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| M43 | LH_HIP | 43 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| M42 | LH_UPPER | 42 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |
| M41 | LH_LOWER | 41 | | `NOT_AVAILABLE` | `NOT_AVAILABLE` | `NOT_AVAILABLE` | | | |

`physical_unit` is the **expected** allocation, carried for the record. The firmware cannot
confirm it — see blocker BL-4.

---

## 5. PASS / FAIL CRITERIA

### H0-A PASS requires **all** of:

1. `MODE=MAINTENANCE` throughout;
2. `SERVO_CENSUS=PASS` — **twice**, identically: `present_expected=13`, `missing_expected=0`,
   `absent_by_design=4`, `absent_by_design_present=0`, `unexpected_id=0`, `not_probed=0`,
   `truncated=NO`;
3. all **12/12** leg IDs answered `@SERVO READ` — no `NO_RESPONSE`;
4. `position` within `0..4095` and plausible for a servo at rest on all 12;
5. `voltage` and `temp` sane and consistent across the 12; the DALY pack voltage did not sag;
6. `SAFE_OFF` returned **`VERIFIED_OFF` on all 12**;
7. the Step-7 re-read shows `torque=0` on **all 12**;
8. `runtime_resets=0` and `uptime_ms` monotonic — no reset occurred during the session.

**Anything less is FAIL. 11/12 is FAIL.** Partial success is not a gate result.

### H0-A FAIL — stop and report, do not improvise

| Symptom | Likely cause | Action |
|---|---|---|
| `MISSING_EXPECTED` | cable, connector, dead unit, wrong ID | **stop**; do not re-ID anything |
| `UNEXPECTED_ID` / `ABSENT_BY_DESIGN_PRESENT` | allocation does not match reality | **stop**; the allocation YAML is the authority and must be reconciled by review |
| `SAFE_OFF = UNVERIFIED_NO_RESPONSE` on a powered bus | the servo is not reachable | **stop** |
| `SAFE_OFF = VERIFY_FAILED` | torque is on and stayed on | **cut power immediately** |
| `RANGE_INCOMPLETE` | scan truncated | re-run once; if it persists, **stop** |
| `runtime_resets` increased | the port open reset the board | **stop**, fix the connection method, restart the session |

### What H0-A PASS does **not** establish

```text
model identity                 NOT VERIFIED  (blocker BL-1)
PositionOffset = 0             NOT VERIFIED  (blocker BL-2)
MATDOG_C018_V1 profile         NOT VERIFIED  (blocker BL-3)
physical unit ↔ joint binding  NOT VERIFIED  (blocker BL-4)
q0, direction, calibration     out of scope entirely
```

A PASS here authorises **nothing beyond itself**. Hardware motion stays BLOCKED.

---

## 6. EVIDENCE PATH

```text
09_Logs/Validation_Reports/H0_Leg_Preflight_<YYYY-MM-DD>/
├── README.md              gate result, operator, conditions, the 12-row record
├── session_transcript.txt the complete raw session, every byte, unedited
├── census_run_1.txt
├── census_run_2.txt
├── servo_read_pre.txt     Step 5
├── safe_off.txt           Step 6
├── servo_read_post.txt    Step 7
├── status_open.txt        Step 3
├── status_close.txt       Step 8
└── SHA256SUMS
```

Matching the convention of `ST3215_Provisioning_2026-08-27/` and the DALY KEY session: raw
transcripts committed verbatim, hashed, never edited into prose. Record the operator name, the
date, the firmware identity from §1.1, and the pack voltage at open and close.

---

## 7. BLOCKERS

### BL-1 — model identity is not reachable · **BLOCKER**

`ServoBus::readModel(id, &model)` exists and reads register `0x03` (expected `777` for the
ST-3215-C018). **No command in `CommandRouter` calls it**, and neither `@SERVO CENSUS` nor
`@SERVO SCAN` prints a model — the census emits only `FOUND id=`.

*To unblock:* expose the existing primitive. No new bus capability, no new register.

### BL-2 — `PositionOffset` cannot be read · **BLOCKER**

There is no read of register `0x1F` (int16 LE, expected `0`). The Controller's servo surface is
`Ping` + the six `readRuntimeState` registers + the model word, and nothing else.

*To unblock, and this one needs a decision:* `SMS_STS_OFS_L` and `SMS_STS_OFS_H` are in
`check_forbidden_literals` in `static_audit.py`. That ban is **total — it blocks reads as well as
writes.** Adding a read means deliberately narrowing the prohibition from "this register does not
appear" to "this register is never written", and re-proving the write ban by mutation. Reading it
with a magic number instead would evade exactly the audit that makes the ban meaningful. Do it
properly or not at all.

### BL-3 — `MATDOG_C018_V1` profile verification does not exist in the Controller · **BLOCKER**

The profile is fully specified and machine-readable
(`06_Software/Matdog_Core/config/MATDOG_ST3215_C018_V1.yaml`): **20 EEPROM registers** hashing to
the canonical fingerprint

```text
ae3ae5ce6ddda1c003fcde9f8ce12fed7f1272639b183e1ba23e27eae0cc5b4d
```

verified identical on all 17 units after a true cold power cycle, plus `BaudRate` (`0x06`,
expected `0`, verification-only). Every one of those is a **read**.

The Controller implements none of it. The only tool that does is the frozen
`ST3215_Bench_Tools/Provisioner_V6`, which is a **write-capable bench instrument** that owns the
bus itself.

> **Do not attach the bench provisioner to the assembled robot.** It would be a second bus
> owner, it is write-capable, and the architecture permits exactly one owner. Using it here would
> trade the thing H0 is meant to establish for the thing H0 is meant to avoid.

*To unblock:* a read-only profile verifier in the Controller — 20 register reads, compare, hash,
report `MATDOG_C018_V1 = MATCH | MISMATCH` per unit.

### BL-4 — the firmware cannot confirm physical-unit identity · **BLOCKER (structural)**

`kCanonicalServos` in the installed firmware is `{bus_id, joint, current_config}` — **no physical
unit field.** The firmware knows bus 11 is `LF_LOWER`; it has no way to know the unit answering
there is `M33`.

This is the same root cause as B2 in
[`CALIBRATION_MAINTENANCE_REQUIREMENT.md`](CALIBRATION_MAINTENANCE_REQUIREMENT.md): unit identity
is compile-time configuration, not a runtime concept.

**It may not be solvable at all by reading.** An ST3215 has no unit serial number exposed on the
bus; `M33` is a MATDOG label from the provisioning campaign, not something the servo knows. The
honest options are:

1. accept `physical_unit` as **declared, not observed**, and rely on the allocation YAML plus the
   provisioning session records — which is what the record template in §4 does today; or
2. bind it indirectly through BL-3: if a unit's profile fingerprint and `PositionOffset` match its
   provisioning session record, that is corroboration of identity, not proof; or
3. treat unit identity as a physical/process control (labelling and assembly discipline), which is
   how it was established on 2026-08-27 in the first place.

**Option 3 is what actually holds today.** It should be stated as such rather than implied by a
column in a table.

---

## 8. SUMMARY

```text
FIRMWARE INSTALLED            6322563, ROBOT_POWERED, app e9283ced…  (confirm at Step 3)
H0-A  EXECUTABLE NOW          census + 12 reads + 12 SAFE_OFF + re-read
H0-B  BLOCKED                 BL-1 model, BL-2 PositionOffset, BL-3 profile, BL-4 unit identity
FLASHING                      NOT performed, NOT scheduled, needs new explicit authorization
HARDWARE MOTION               BLOCKED
DEFAULT WRITE REACHABILITY    torque OFF only
```

Nothing in this runbook has been executed.

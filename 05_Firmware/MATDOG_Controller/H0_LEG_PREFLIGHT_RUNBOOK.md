# H0 — CURRENT 12/12 LEG SERVO READ-ONLY PREFLIGHT

**Runbook for the operator · prepared 2026-09-22 · NOT YET EXECUTED**
**Amended 2026-09-22:** H0-B is now implemented in this branch. The board is **unchanged** —
see §1.1 and §7.

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

### 1.2b After the H0-B work on this branch

Every gap above is now **implemented in `feat/h0-current-leg-preflight-v1`** and none of it is on
the board:

| # | H0 field | In this branch | On the installed firmware |
|---:|---|---|---|
| 5 | `model` | ✅ `@SERVO PREFLIGHT`, compared against 777 | ❌ |
| 6 | `PositionOffset` | ✅ `ServoBus::readPositionOffset()`, int16 two's complement | ❌ |
| 7 | persistent profile | ✅ 20 registers compared against the generated `MATDOG_C018_V1` table | ❌ |
| 1 | `physical_unit` | ✅ reported as `expected_physical_unit`, from configuration | ❌ |

**Reaching any of it on hardware requires a flash, and a flash requires a new explicit
authorization.** None has been performed.

### 1.3 Verdict

```text
INSTALLED FIRMWARE  6322563  -  UNCHANGED, and 3 of 10 record fields are unreachable on it

H0-A  population / identity / liveness / raw    EXECUTABLE NOW, no flash
H0-B  model / PositionOffset / profile          IMPLEMENTED in this branch,
                                                REQUIRES A FLASH to reach hardware
```

**I have not flashed anything and will not without a new explicit authorization.**

H0-A remains worth running on its own: it converts the historical **6/12** into a current result
and needs no firmware change. H0-B upgrades the same gate to the full ten-field record once the
operator authorises a flash.

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

### Step 5b — H0-B full record · **only after a flash, only with new authorization**

Not available on the installed firmware. Once `feat/h0-current-leg-preflight-v1` is flashed:

```text
@SERVO PREFLIGHT
```

Asynchronous, like the census: it prints `SERVO_PREFLIGHT=STARTED joints=12 profile=…`
immediately and one joint is evaluated per Controller tick. Keep collecting until the
`SERVO_PREFLIGHT=PASS|FAIL` block arrives.

It replaces Step 5 rather than supplementing it: the same twelve joints, with model,
`PositionOffset` and the twenty-register `MATDOG_C018_V1` comparison added. Strictly read-only —
`Ping` plus register reads, and each joint is pinged first so an absent unit costs one bounded
timeout instead of twenty-three.

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

### `@SERVO PREFLIGHT` — expected shape (H0-B only)

```text
SERVO_PREFLIGHT=PASS profile=MATDOG_C018_V1 source_sha256=70aee512…
  evaluated=12 pass=12 no_response=0 mismatch=0 incomplete=0
  JOINT expected_physical_unit=M33 joint=LF_LOWER expected_bus_id=11 observed_bus_id=11
        model=777 position_offset=0 persistent_profile=MATDOG torque_enable=0
        present_position=2051 result=PASS
  … eleven more …
  NOTE present_position is a raw liveness tick, NOT q0
```

| Field | Meaning |
|---|---|
| `expected_physical_unit` | **configuration**, from `MATDOG_SERVO_ALLOCATION.yaml`. An ST3215 exposes no unit serial, so this is never an observation. |
| `observed_bus_id` | the **only** identity a servo supplies: it answered at this address. `0` means it did not. |
| `model` | register `0x03`; `777` for the ST-3215-C018 |
| `position_offset` | register `0x1F`, int16 two's complement; `UNREAD:` prefix if the read failed — a failed read is never reported as zero |
| `persistent_profile` | `MATCH` / `MISMATCH` / `INCOMPLETE` over the twenty registers. `INCOMPLETE` is never a pass: an unread register is never assumed correct. |
| `result` | `PASS` only when the unit answered, model matched, offset read and zero, and the profile matched |

A profile disagreement adds a detail line naming the first offending register:

```text
    PROFILE_MISMATCH count=1 first_addr=0x15 expected=32 observed=16
```

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

## 7. THE BLOCKERS — now implemented, still unflashed

All four are closed in `feat/h0-current-leg-preflight-v1`. **None of it is on the board**: reaching any of it requires a flash and a new explicit authorization.

### BL-1 — model identity · **IMPLEMENTED**

`ServoBus::readModel(id, &model)` exists and reads register `0x03` (expected `777` for the
ST-3215-C018). **No command in `CommandRouter` calls it**, and neither `@SERVO CENSUS` nor
`@SERVO SCAN` prints a model — the census emits only `FOUND id=`.

**Done:** `@SERVO PREFLIGHT` reads register `0x03` per joint and compares it against `777` from the generated invariants. The existing `ServoBus::readModel()` primitive is reused; no new bus capability and no new register.

### BL-2 — `PositionOffset` · **IMPLEMENTED, with the audit gate narrowed deliberately**

There is no read of register `0x1F` (int16 LE, expected `0`). The Controller's servo surface is
`Ping` + the six `readRuntimeState` registers + the model word, and nothing else.

**Done, properly.** The total ban was replaced, not evaded:

```text
read through exactly ServoBus::readPositionOffset()  = ALLOWED
any PositionOffset write                             = FORBIDDEN
CalibrationOfs                                       = FORBIDDEN
```

`check_position_offset_boundary` enforces that `SMS_STS_OFS_L` appears in exactly one file and
one function; that the accessor uses `readWord` and contains no write primitive; that no
`writeByte`/`writeWord`/`genWrite`/`RegWrite` anywhere names the offset register under **any**
spelling, including the bare address `0x1F`; and that the decoder is two's complement rather than
sign-magnitude. Five mutations prove it bites — including one that inserts a literal
`writeWord(id, SMS_STS_OFS_L, 0)` into the accessor itself.

### BL-3 — `MATDOG_C018_V1` profile verification · **IMPLEMENTED**

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

**Done.** `matdog_servo_profile_export.py` reduces the reviewed YAML to a `constexpr` table —
the twenty values are never retyped — and `ServoProfile` compares them register by register.
**No runtime SHA-256**: individual values are compared, which is both cheaper and more
diagnosable than a digest, and a mismatch names the first offending register.

The three layers stay apart, and the exporter fails if they blur: the twenty **persistent**
registers; the **invariants** (model, offset, baud, raw centre) which are checked but are not
part of the delta set; and **runtime RAM state** (`TorqueLimit`, `GoalSpeed`, `Acc`) which is
absent entirely.

### BL-4 — physical-unit identity · **RESOLVED AS A SEMANTIC**

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

**Option 3 is what holds, and it is now stated rather than implied.** `CanonicalServo` carries
the expected unit from the allocation YAML; the report column is named
`expected_physical_unit`; and `check_h0_preflight_boundaries` fails the build if it is ever
renamed to suggest an observation, or if the report drops its `NOT q0` note. A mutation proves
each.

---

## 8. SUMMARY

```text
FIRMWARE INSTALLED            6322563, ROBOT_POWERED, app e9283ced…  UNCHANGED
H0-A  EXECUTABLE NOW          census + 12 reads + 12 SAFE_OFF + re-read, no flash
H0-B  IMPLEMENTED, UNFLASHED  @SERVO PREFLIGHT - needs a flash + new authorization
FLASHING                      NOT performed, NOT scheduled
HARDWARE MOTION               BLOCKED
DEFAULT WRITE REACHABILITY    torque OFF only (SAFE_OFF)
```

### The path after H0

```text
H0 12/12 PASS
  -> manual URDF q=0 pose
  -> read-only current q0 capture
  -> q0 sanity analysis
  -> motorDirection read from the current URDF   (no measurement, no campaign)
  -> current raw<->rad transforms
  -> conservative runtime motion settings
  -> SafeActuator runtime adapter
  -> C4-C CURRENT-INSTALLATION STAND REVALIDATION
```

The stand is a **revalidation**: the C4-C 51-frame trajectory has already run on real hardware to
roughly 150 mm body height with stable four-foot Torque-ON hold. Reuse it unless current
q0/provenance analysis shows a concrete incompatibility. Contact calibration is a permanent
MAINTENANCE capability and is **not** a prerequisite for this stand.

Nothing in this runbook has been executed.

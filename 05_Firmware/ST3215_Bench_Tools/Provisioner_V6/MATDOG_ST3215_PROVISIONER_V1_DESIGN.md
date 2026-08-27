# MATDOG ST3215 Provisioner V1 — design decisions

**Tool:** `matdog_st3215_provisioner_v1`
**Profile:** `MATDOG_C018_V1`
**Date:** 2026-08-27
**Status:** offline implementation complete; **HARDWARE FREEZE BLOCKED** (see §11).

This document records the decisions that govern the provisioner. It supersedes
specific earlier statements where explicitly noted. It does **not** modify,
replace or invalidate any earlier document: the 2026-08-26 handoff package, the
2026-08-26 review addendum and the frozen read-only survey remain as written and
remain the evidence base this design is derived from.

---

## 1. Superseding decision — the source-profile gate is replaced

### 1.1 What the earlier requirement said

`MATDOG_ST3215_PROVISIONING_REVIEW_ADDENDUM_2026-08-26.md` §9 made a
**source-profile gate** blocking: no write-capable provisioner could be frozen
until every allocated physical unit had a persisted as-found EEPROM signature,
so that the provisioner could classify each source into an accepted
profile family before writing.

### 1.2 What replaces it

For Provisioner V1 the source-profile classification gate is **SUPERSEDED** by a
**PRE-PROVISION SAFETY GATE** (§4).

The reasoning is a correction of what the BEFORE snapshot is actually for:

> **The target profile is the source of truth.**
> The BEFORE snapshot does not decide whether P/D/protections are already
> canonical. Those fields are exactly what V1 is provisioning.

Classifying a servo as "known legacy profile A" versus "known legacy profile B"
changes nothing about the writes performed: in both cases the provisioner writes
the delta between the observed value and `MATDOG_C018_V1`. A classification step
would add a failure mode (an unrecognised-but-harmless source value blocks a
unit) without removing any real risk.

The BEFORE snapshot is therefore retained in full, unchanged in scope, and used
for six purposes:

| Purpose | Use |
|---|---|
| **audit** | the exact as-found 71 bytes, persisted before any write |
| **identity** | model 0x03, ID 0x05, baud 0x06 — is this the servo we think it is |
| **communication health** | ping status, model read status, ResponseStatus 0x08, full-block readability |
| **offset math** | the old signed `PositionOffset`, required to compute the centering goal |
| **safety** | voltage / temperature / status / lock / torque state before any motion |
| **recovery** | the only record of the pre-provisioning state if anything fails |

### 1.3 What is *not* relaxed

The safety gate is strictly fail-closed and is **wider** than the old profile
gate on identity and health. Nothing about the following changes:

- exactly one responder on a full 0..253 scan;
- model 777 read from **0x03**;
- clean ping status and clean model-read status;
- `ResponseStatus == 1` before any write;
- full 71-byte block readable;
- zero EEPROM writes and zero motion if any mandatory gate fails.

### 1.4 Consequence for the survey

The read-only survey (`matdog_st3215_source_survey_v1`, frozen) remains valid
and useful as independent evidence, and its NEW01 record is used in this
project's offline tests as a real-hardware vector. It is no longer a *blocking
precondition* for provisioning a unit: the provisioner captures its own BEFORE
evidence, before any write, in the same session.

---

## 2. Superseding decision — post-recode identity proof

### 2.1 What the earlier requirement said

Addendum §8 required a **second full 0..253 bus scan** after the ID write, with
exactly one responder at the target ID.

### 2.2 What replaces it

V1 uses **direct target/old-ID verification** instead of a second full scan.

This is an intentional superseding decision for the V1 lean workflow:

- the initial full scan has *already* established that exactly one servo is on
  the bus and that the target ID was not independently occupied — the physical
  procedure guarantees exactly one connected servo for the whole session, and no
  servo can be added mid-session without the operator doing so deliberately;
- a full scan costs ~25 s (254 pings at the ~100 ms `IOTimeOut`) and adds no
  information that the direct proof does not already give;
- the direct proof is *stronger* per-ID than a scan line, because it reads the
  full 71-byte block at the target ID and compares the persistent profile.

The authoritative post-recode proof is therefore:

1. target ID responds to `Ping`, with clean status;
2. model at target ID (0x03) == 777;
3. **old source ID does not respond** (only when source != target);
4. full 71-byte snapshot at target ID, with ID register 0x05 == target ID;
5. persistent canonical profile still exact;
6. `PositionOffset == 0`;
7. `Lock == 1`;
8. `TorqueEnable == 0`.

Any ambiguity → `FAIL_AMBIGUOUS_RECODE`, no retry, operator must remove servo
power. The ID write ACK is advisory only and is never used as proof.

---

## 3. Architecture

Three files, deliberately lean, no framework:

```
matdog_st3215_provisioner_v1.ino   ESP32-S3 firmware, owns the state machine
matdog_st3215_provisioner_v1.py    host runner, owns evidence persistence
MATDOG_ST3215_PROVISIONER_V1_DESIGN.md
```

The firmware owns the allocation map, the canonical profile, every write, and
the motion watchdog. The host owns nothing safety-critical: it cannot compose a
write, cannot name a register, cannot choose an ID, and cannot command motion.

### 3.1 Command surface — exactly three commands

```
@BEGIN <PHYSICAL_LABEL>
@EXECUTE <SESSION_TOKEN>
@HELP
```

There is no generic EEPROM write command, no arbitrary ID command, no arbitrary
GoalPosition command, no `CalibrationOfs`, no host-exposed Torque ON, no factory
reset, and no broadcast write. Any other input is rejected by a single parser
choke point before a byte reaches the servo bus.

`@BEGIN` performs **zero servo writes**. It scans, snapshots, gates, and binds a
session token to (physical label, source ID, target ID, BEFORE state digest),
then waits. This exists so the host can make the BEFORE evidence durable on disk
*before* the first write is possible.

### 3.2 Host is not trusted with the target ID

Host usage is:

```
python3 matdog_st3215_provisioner_v1.py --unit NEW01
```

No source ID argument, no confirmation string. The host sends the physical
label; the **firmware** maps label → target ID from its own table. The host
computes the same mapping only to cross-check the firmware's answer and to fail
closed on disagreement.

---

## 4. PRE-PROVISION SAFETY GATE

Evaluated inside `@BEGIN`, before the token exists. Every item is mandatory.

| Gate | Requirement |
|---|---|
| responders | exactly one on a full 0..253 scan |
| model | 777, read at **0x03** (2 bytes, little endian) |
| ping status | `0x00` |
| model read status | `0x00` |
| ID register | 0x05 == discovered source ID |
| baud register | 0x06 == 0 (1 Mbps) — **verified, never written** |
| ResponseStatus | 0x08 == 1 |
| snapshot | full 71 bytes 0x00..0x46 read in one transaction, clean status |
| PositionOffset | decodes as int16 two's complement inside an explicitly bounded domain |
| PresentPosition | 0..4095 |
| Lock | readable, 0 or 1 |
| voltage | 40..140 raw (4.0..14.0 V) |
| temperature | <= 70 °C |
| status byte | 0x40 == 0 |
| torque | `TorqueEnable == 0` at entry |
| target ID | valid 0..253, from the firmware allocation table |

Failure of any item:

```
EEPROM_WRITES = NONE
MOTION        = NONE
BEGIN_RESULT FAIL
```

The gate deliberately does **not** require P / D / I / protections / limits to
already equal the target. Those are the delta V1 writes.

### 4.1 Bounded PositionOffset domain

Per addendum §6, suspicious values fail closed rather than being normalised:

```
-2048 <= signed_offset <= 2047
```

Values outside this (e.g. a corrupt 0x8000) abort with zero writes. The offset
decoder is a dedicated two's-complement int16; it is never shared with the
sign-magnitude decoders used for speed / load / current.

---

## 5. Register rules

- **Model word is read at 0x03.** NormaCore
  `software/drivers/st3215/src/protocol/memory.rs:46` currently declares
  `ModelNumber => (0x00, 2, ...)`. That declaration is a source-map defect for
  the C018 and is **not** used here. The installed SCServo header agrees with
  this design: `SMS_STS_MODEL_L = 3`.
- **PositionOffset 0x1F..0x20** is signed int16, little endian, two's complement.
  Independently corroborated by
  `robot-dog/06_Software/Matdog_Core/calibration/matdog_digital_zero_calibration.py`,
  which uses `EEPROM_OFFSET = 0x1F` with `struct.unpack_from("<h", ...)`.
- **Relation** (same file uses `raw_unoffset = (present + offset) % 4096`):

```
physical_raw = (displayed_present_position + signed_position_offset) mod 4096
displayed    = (physical_raw - signed_position_offset) mod 4096
```

- **Centering goal under an old offset O:**

```
center_goal_displayed = floor_mod(2048 - O, 4096)
```

- `floor_mod` is an explicit helper returning 0..4095 in both languages. Raw
  C/C++ `%` is never applied to a possibly-negative numerator.
- **GoalPosition is strictly unsigned 0..4095.** `SMS_STS::WritePosEx` takes an
  `s16` and encodes a negative value as sign-magnitude with bit 15 set. The
  provisioner validates the domain *before* the call so that path is
  unreachable.

### 5.1 Worked example — NEW01 (real hardware)

From the frozen survey record `NEW01__20260826_193057Z.json`:

```
source ID            1
PositionOffset       +85
PresentPosition      254 (displayed)
physical_raw         (254 + 85) mod 4096 = 339
center_goal_displayed floor_mod(2048 - 85, 4096) = 1963
physical_raw at goal  (1963 + 85) mod 4096 = 2048
```

This vector is asserted in the offline test suite.

---

## 6. Write semantics

Installed SCServo `Ack()` returns **1 = success, 0 = failure**. `wr < 0` is
meaningless for these primitives and appears nowhere in this project.

Every ordinary write requires **all three**:

```
write_result == 1
AND st.Error == 0
AND exact register readback == expected
```

The ID recode is the single exception: its write ACK is **advisory only**,
because the servo may already be answering under the new ID. Identity proof
(§2.2) is authoritative.

---

## 7. EEPROM transaction policy

`@EXECUTE` stage order, each stage ending with EEPROM locked and torque off:

1. **Torque OFF**, exact verify. First action, always.
2. **Canonical profile.** Compute the delta allowlist from the BEFORE snapshot;
   already-correct fields are skipped and reported as `SKIP`. If the delta is
   empty the whole unlock/lock transaction is skipped. Otherwise: unlock →
   write only the delta → verify every write → lock → verify lock → full
   snapshot → exact profile verification. **No PositionOffset. No ID.**
3. **Centering** (§8). RAM only.
4. **PositionOffset = 0** (§9). Own unlock/lock transaction.
5. **ID recode** (§10). Own unlock/lock transaction, last identity change.
6. **Warm final** 71-byte state, read-only verification.
7. **Cold power cycle(s)** (§11), read-only.

**Idempotent and resumable.** A partially provisioned servo can be rerun: the
delta list simply shrinks. A fully canonical servo writes nothing in stage 2 —
this is not a theoretical case, NEW01's surveyed state is already exactly
`MATDOG_C018_V1`, so its stage-2 delta is empty.

On any failure: attempt EEPROM lock, attempt Torque OFF, **no rollback, no
blind retry, no factory reset**, fail closed. No implicit movement is ever
issued as cleanup.

---

## 8. Centering

Canonical center is **physical RAW 2048**, reached while the old offset is still
active. The servo is assumed unloaded and mechanically free.

Frozen by `04_MATDOG_C018_CANONICAL_PROFILE_V1.md` §6:

```
CENTER_TORQUE_LIMIT = 300
CENTER_SPEED        = 365
CENTER_ACC          = 50
```

Acceptance after settling: **±1 tick** on physical raw. A larger residual is
logged with its measured value and fails closed; the criterion is not widened.

### 8.1 Prime

`WritePosEx` writes the 7-byte block at 0x29 (Acc, GoalPosition, GoalTime,
GoalSpeed). Pre-writing Acc/GoalSpeed separately therefore protects nothing —
the following `WritePosEx` overwrites them. The prime sequence is:

1. Torque OFF verified.
2. Write `TorqueLimit` (0x30) = 300, exact readback.
3. Read current `PresentPosition`.
4. `WritePosEx(current_position, 365, 50)`.
5. **Assume torque is now ON** — observed `PRIME TORQUE : 1` in **26/26** QC
   V6.1 campaign runs. This is expected behaviour, not a rare possibility.
6. Immediately force Torque OFF and verify.
7. Measure the position delta across the prime and require it inside the prime
   window (§11 — **UNRESOLVED**).

### 8.2 Motion transaction

Explicit `EnableTorque(1)` + verified readback inside the authorized
transaction, then `WritePosEx(center_goal_displayed, 365, 50)`, then a
**firmware-local** monitoring loop. The loop never reads the host serial port,
so a host crash or disconnect cannot extend a move: the ESP32 owns the watchdog
and always converges to Torque OFF or an explicit FAULT requiring power removal.

Monitored every sample: PresentPosition, PresentSpeed, PresentLoad,
PresentCurrent, PresentVoltage, PresentTemperature, Status, Moving.

---

## 9. PositionOffset zero

Only after physical raw center is verified:

```
Torque OFF verified
unlock  → verify lock == 0
write PositionOffset (0x1F, word) = 0
verify readback == 0
lock    → verify lock == 1
verify PresentPosition ~ 2048 and physical_raw ~ 2048
```

No movement may be caused by the offset write; position is re-read and compared.
`CalibrationOfs()` is never called — note that it is implemented as
`writeByte(ID, TORQUE_ENABLE, 128)`, so writing 128 to 0x28 is equally
forbidden and is rejected by the write allowlist.

---

## 10. ID recode

Occurs only after canonical profile verified, physical center verified,
`PositionOffset == 0` verified, torque OFF.

If source ID == target ID the write is **skipped** (idempotent rerun).

Otherwise: unlock via source ID → write ID once (**no retry**) → prove identity
per §2.2 → lock via the target ID → full verification snapshot.

If identity is ambiguous (target silent, or old ID still answering, or both
answering): `FAIL_AMBIGUOUS_RECODE`, best-effort lock/torque-off on whichever ID
answers, and `CUT_SERVO_POWER_NOW`. The operator must remove servo power.

---

## 11. Constants — provenance, and what is UNRESOLVED

Every safety/timing constant is either reused from validated MATDOG evidence or
declared unresolved. Nothing was chosen because it looked reasonable.

### 11.1 Resolved by reuse

Source: `matdog_servo_commissioning.ino` (QC V6.1, frozen), exercised across the
26-run campaign in `runtime/esp32/qc_campaign/` on this exact 17-servo
population, same bus, same 1 Mbps, same SCServo build.

| Constant | Value | Provenance |
|---|---|---|
| polling interval | 2000 µs (500 Hz) | `QC_FAST_PERIOD_US` |
| motion timeout | `qcComputeMoveTimeoutMs()` formula, verbatim | QC V6.1; reproduces logged `window_ms` exactly (478 ticks @ speed 600 → 3794 ms) |
| no-progress | residual > 32 and no progress for 1.5 s | QC characterization loop |
| composite stall | load ≥ 800 **and** current ≥ 139 **and** \|speed\| ≤ 50 **and** no progress 250 ms, for 50 samples | `QC_STALL_*`; load alone can never stop a move |
| overcurrent | ≥ 308 raw (~2.0 A) for 50 samples (100 ms) | `QC_OVERCURRENT_2A_RAW` |
| stall current | ≥ 416 raw (~2.7 A) for 10 samples (20 ms) | `QC_STALL_CURRENT_RAW` |
| current scale | 6.5 mA/raw | `QC_CURRENT_MA_PER_RAW` |
| thermal | > 70 °C, 3 confirmations 5 ms apart | `QC_THERMAL_*` |
| voltage | outside 40..140 for 50 samples | QC voltage safety gate |
| status byte | nonzero for 25 samples | `QC_STATUS_PERSIST_SAMPLES` |
| telemetry loss | 25 consecutive failed reads (50 ms) | QC characterization |
| wrong direction | 16 ticks against the commanded direction | QC characterization `directionalTravel < -16` |
| settle | \|speed\| ≤ 10 and Moving == 0 for 40 samples (80 ms) | QC characterization |
| reached residual | ≤ 3 ticks (settle detection only) | QC characterization |
| post-write settle delay | 20 ms before readback | QC `forceTorqueOffVerified` |
| centering torque/speed/acc | 300 / 365 / 50 | canonical profile doc §6 |
| acceptance | ±1 tick | canonical profile doc §5 |

**Deliberate omission:** the micro32 `OVERSHOOT_LIMIT = 12` is **not** reused.
It was validated only for a 64-tick move at speed 100 / acc 10 and does not
transfer to a move of up to 2048 ticks at speed 365 / acc 50. The QC
characterization loop, which did run at speeds up to unlimited, uses no
overshoot abort — only the direction guard and the current/stall guards. This
design follows the characterization loop. Choosing a new overshoot number would
have been an invention.

### 11.2 UNRESOLVED_CONSTANT — blocks hardware freeze

**`PRIME_MAX_DELTA_TICKS`** — provisional 0, not justified.

The available evidence is strong but does not cover the mandated parameters. In
26/26 campaign runs the position measured before the prime equals the position
measured after prime + verified Torque OFF + verified Torque ON, exactly, on
every unit (`START POSITION` == first `MEASURE_BEGIN start=`, delta 0 in all 26
logs). However:

- the campaign primed with `WritePosEx(pos, 300, 20)`; this design mandates
  `(365, 50)`;
- the observed delta spans prime + OFF + ON, not the prime alone;
- no campaign ever asserted a tolerance band, so no validated threshold exists —
  only an observation that the value was zero.

A 0-tick threshold is an extrapolation to different motion parameters; any
nonzero threshold is an invention. **Resolve by measuring the prime-only delta
at (365, 50) on the pilot before freezing.**

**`COLD_ABSENCE_DEBOUNCE_MS` and `COLD_RETURN_STABLE_SAMPLES`** — provisional
1500 ms / 10 samples, not justified.

No MATDOG or NormaCore code has ever detected a servo power cycle. There is no
measurement anywhere in the evidence base of ST-3215-C018 rail-collapse timing
or post-power-on boot-to-first-ACK latency. Both numbers would be invented.
**Resolve by measuring one real power cycle on the pilot before freezing.**

### 11.3 Non-safety bound

`COLD_OPERATOR_WINDOW_MS = 180000` is how long the firmware waits for the
operator to flip the servo rail. It is not a safety constant: torque is off, the
EEPROM is locked, and the cold phase is strictly read-only. Expiry aborts the
session read-only. It is documented here so it is not mistaken for a validated
threshold.

### 11.4 Effect of the block

While any UNRESOLVED constant remains:

- the firmware sets `PROVISIONER_HARDWARE_FREEZE_BLOCKED = 1`;
- `@BEGIN` reports each unresolved constant and `HARDWARE_FREEZE=BLOCKED`;
- `@EXECUTE` aborts **before the first write** with
  `EXECUTE_ABORT: HARDWARE_FREEZE_BLOCKED`;
- the host runner refuses to transmit `@EXECUTE` at all.

Clearing the block is a deliberate, auditable, single-constant edit that must be
accompanied by the measurements above.

---

## 12. True cold power cycle

An ESP32 reset is **not** a servo power cycle, and the design makes that
structurally true rather than a matter of trust: the cold state machine lives in
ESP32 RAM inside a single `@EXECUTE` invocation, and its precondition is
`observed_present_before_absence == true`. An ESP32 reset destroys that state,
ends the session, and no cold PASS can ever be emitted from a fresh boot — the
firmware comes up in `IDLE` with no token and no session.

Detection is read-only (`Ping` + model read only, no writes):

```
target present
→ absent continuously for COLD_ABSENCE_DEBOUNCE_MS      [UNRESOLVED]
→ responds again
→ COLD_RETURN_STABLE_SAMPLES consecutive clean responses [UNRESOLVED]
→ full 71-byte cold snapshot, read-only verification
```

Cold requirements: target ID, model 777, baud 0, offset 0, canonical profile
exact, Lock 1, TorqueEnable 0, PresentPosition ≈ 2048.

`TorqueLimit` after cold boot is **observed and recorded, never asserted** and
never treated as persistent operating policy. The runtime controller must
preload its mode-specific TorqueLimit before any future torque ON.

**Pilot policy:** NEW01 requires **two** independent cold cycles before the
batch build is frozen. Every other unit requires one. The firmware owns this
per-label in its allocation table.

---

## 13. Evidence

One exclusively-created directory per session. The host creates it with
`os.makedirs(..., exist_ok=False)`; a pre-existing directory fails closed.

```
01_before_state71.bin
02_profile_state71.bin
03_centered_old_offset_state71.bin
04_offset_zero_state71.bin
05_after_recode_warm_state71.bin
06_after_cold_state71.bin
07_after_cold_cycle2_state71.bin   (pilot second cycle only)
session_console.log
provisioning_report.json
SHA256SUMS
```

Every `.bin` is written with `open(path, "xb")` → `write` → `flush` →
`os.fsync` → on-disk length verified == 71 → SHA256 recorded. Evidence is never
overwritten.

`01_before_state71.bin` and the session skeleton are durable **before**
`@EXECUTE` is transmitted. That ordering is the reason `@BEGIN` and `@EXECUTE`
are separate commands.

---

## 14. Failure semantics

No hidden continuation. Any exception, fault or unexpected condition produces:

```
verdict = FAIL
reason  = typed reason (exception type + message, or firmware abort code)
evidence persisted when it can still be written
EEPROM lock restoration attempted
Torque OFF attempted
serial port closed
```

No rollback. No factory reset. No blind ID retry. No implicit movement as
cleanup. If communication is lost after a potentially ambiguous ID write, the
tool requests physical removal of servo power and stops.

# MATDOG ST3215 Characterize V1 — design decisions

**Tool:** `matdog_st3215_characterize_v1`
**Unit under characterization:** NEW01 (unprovisioned, current ID 1)
**Date:** 2026-08-27
**Status:** offline implementation complete; compiled; not uploaded; no hardware contacted.

This document records the decisions that govern the characterization harness.
It does **not** modify, replace or invalidate any earlier document. The frozen
read-only survey, the 2026-08-26 handoff package, the 2026-08-26 review
addendum and `MATDOG_ST3215_PROVISIONER_V1_DESIGN.md` remain exactly as
written, and none of the provisioner's three files is touched by this project.

---

## 1. Why this exists

`MATDOG_ST3215_PROVISIONER_V1_DESIGN.md` §11.2 blocks the provisioner's
hardware freeze on three constants that have no direct evidence:

| Constant | Provisional | Why it is unresolved |
|---|---|---|
| `PRIME_MAX_DELTA_TICKS` | 0 | the 26-run campaign primed at speed 300 / acc 20, not the mandated 365 / 50; it measured prime + OFF + ON, not the prime alone; and it never asserted a tolerance band |
| `COLD_ABSENCE_DEBOUNCE_MS` | 1500 | no MATDOG or NormaCore code has ever detected a servo power cycle |
| `COLD_RETURN_STABLE_SAMPLES` | 10 | no measurement of C018 boot-to-first-ACK latency exists anywhere |

This harness measures the missing quantities on the pilot. It is the smallest
tool that can produce that evidence safely.

### 1.1 What it deliberately does not do

**It selects no constant.** Every report it writes states `DEFERRED_TO_REVIEW`
for all three names, the firmware carries
`CHARACTERIZER_SELECTS_CONSTANTS = false` as a greppable declaration, and both
the firmware and the runner print
`PROVISIONER_HARDWARE_FREEZE=UNCHANGED_BLOCKED`. Running this tool does not
clear the block. Clearing it remains a deliberate, auditable edit to the
provisioner made by a human after reading the measurements.

**It is not a provisioner.** It has no allocation table, no target IDs, no
canonical profile table, no EEPROM unlock primitive and no centering move. A
self-test asserts that the strings `PhysicalUnit` and `ProfileField` do not
appear in its firmware at all.

---

## 2. Complete write capability

The firmware's entire write capability, statically enumerated:

```
0x28 TorqueEnable = 0     RAM, value 0 ONLY, never 1, never 128
0x30 TorqueLimit  = 300   RAM, that one value ONLY
WritePosEx(<position the primitive just read itself>, 365, 50)
```

There is nothing else. No EEPROM write, no unlock, no ID write, no
`PositionOffset` write, no centering move, no arbitrary position, no
mid-position calibration, no factory reset, no broadcast write, no baud write,
and no host-reachable Torque ON.

### 2.1 How that is enforced, and how it is proven

Three functions own every write:

| Function | Primitive | Gate |
|---|---|---|
| `charWriteWordVerified` | `st.writeWord` | `charWriteAllowed`, then exact readback |
| `charTorqueOffCommand` | `st.writeByte` | `charWriteAllowed`, value 0 only |
| `charPrimeWritePosEx` | `st.WritePosEx` | stage gate + self-read position + domain check |

`charWriteAllowed` references exactly two registers and its default is
`return false`. The runner's `--static-audit` proves this from the firmware
source: it enumerates every `st.<write primitive>(` call site with its
enclosing function, refuses any site not on a three-entry approved list,
asserts eleven other write primitives have **zero** call sites, and records a
digest of the comment-stripped allowlist body so any future edit is visible.

A separate host mirror of `charWriteAllowed` is exercised exhaustively in the
self-test over every address `0x00..0x46`, both widths, thirteen values and all
three stages: exactly three `(addr, width, value, stage)` combinations are
writable, and they are the three above.

### 2.2 An arbitrary goal position is unrepresentable, not merely refused

`charPrimeWritePosEx` takes **no position argument**:

```cpp
static bool charPrimeWritePosEx(uint8_t id, PrimeTiming &t)
```

It reads `PresentPosition` itself, validates `0..4095` itself, and commands
that exact value with speed and acceleration pinned to `CHAR_SPEED` /
`CHAR_ACC`. A caller cannot express "go somewhere else" — there is no parameter
in which to say it. The static audit asserts there is exactly one
`st.WritePosEx(` call site in the whole firmware and that its argument list is
literally `id, (s16)present, CHAR_SPEED, CHAR_ACC`.

Because the domain is validated before the call, `WritePosEx`'s negative
sign-magnitude branch is unreachable, exactly as in the provisioner.

---

## 3. Command surface — exactly four commands

```
@ARM
@PRIME <SESSION_TOKEN>
@TRACE <SESSION_TOKEN>
@HELP
```

No generic write command. No arbitrary address/value command. No arbitrary
`GoalPosition`. No arbitrary ID. No EEPROM unlock. No Torque ON. Any other
input is rejected by a single parser choke point on the firmware side and by
`assert_command_allowed` on the host side, before a byte reaches the bus.

`@ARM` performs **zero servo writes**. It scans `0..253`, gates, emits the
as-found 71 bytes, and binds a session token to the discovered ID and the
BEFORE digest. It exists so the host can make the BEFORE evidence durable on
disk *before the first write is possible*.

`@TRACE` is refused until `@PRIME` has completed, and is accepted at most
twice per session.

The host does not supply an ID, a register name, a value, a position or a
confirmation string. It can send four literal strings and nothing else.

---

## 4. Discovery gate

Evaluated inside `@ARM`, before the token exists, and independently recomputed
by the host from the same 71 bytes. Every item is mandatory:

| Gate | Requirement |
|---|---|
| responders | exactly one on a full `0..253` scan |
| model | 777, read at **0x03** (2 bytes, little endian) |
| ping status | `0x00` |
| model read status | `0x00` |
| ID register | `0x05` == discovered ID |
| baud register | `0x06` == 0 (1 Mbps) — verified, never written |
| `ResponseStatus` | `0x08` == 1 |
| snapshot | full 71 bytes `0x00..0x46` in one transaction, clean status |
| `PositionOffset` | decodes as int16 two's complement inside `-2048..2047` |
| `PresentPosition` | `0..4095` |
| `Lock` | readable, 0 or 1 |
| voltage | 40..140 raw |
| temperature | ≤ 70 °C |
| status byte | `0x40` == 0 |
| torque | `TorqueEnable == 0` at entry |

Failure of any item ⇒ `EEPROM_WRITES=NONE`, `MOTION=NONE`, `ARM_RESULT FAIL`.

### 4.1 What the gate deliberately does not require

NEW01 has **not** been provisioned. The gate therefore does **not** require
`PositionOffset == 0`, does not require `Lock == 0`, does not require any
canonical profile field and does not require any particular `TorqueLimit`.
NEW01's surveyed `PositionOffset = +85`, `Lock = 1` and `TorqueLimit = 1000`
all pass, and the self-test asserts that explicitly so the gate cannot be
tightened into a provisioning gate by accident.

The `PositionOffset` bound exists only to reject a corrupt word (e.g. `0x8000`)
before it is used in any modular arithmetic.

---

## 5. Characterization A — prime only

### 5.1 Parameters

Measured at the **exact final parameters**, not the campaign's:

```
CENTER_TORQUE_LIMIT = 300
CENTER_SPEED        = 365
CENTER_ACC          = 50
```

Frozen by `04_MATDOG_C018_CANONICAL_PROFILE_V1.md` §6 and mandated by the
provisioner design §8. Measuring at anything else would reproduce the evidence
gap that blocked the freeze in the first place. `--static-audit` asserts the
three firmware constants are literally 300 / 365 / 50 and that `TorqueLimit` is
the only word write in the firmware.

### 5.2 Sequence, per repetition

1. Torque OFF, exact readback (`ACK == 1`, status `0x00`, readback `0`).
2. `TorqueLimit = 300`, exact readback.
3. Read `PresentPosition` → `P0`, inside `charPrimeWritePosEx`.
4. Validate `0 ≤ P0 ≤ 4095`.
5. `WritePosEx(P0, 365, 50)`.
6. Torque is **assumed** ON — observed `PRIME TORQUE : 1` in 26/26 QC V6.1 runs.
7. Torque OFF commanded as the **very next statement**. Nothing is printed and
   nothing is delayed between step 5 returning and step 7 issuing; timestamps
   are captured into a struct and printed afterwards.
8. Prove the OFF landed: `write_result == 1`, status `0x00`, readback `0`.
9. Observe at the proven MATDOG/QC polling interval — `QC_FAST_PERIOD_US`,
   2000 µs, 500 Hz — until the QC characterization settle criterion
   (`|speed| ≤ 10` and `Moving == 0` for 40 consecutive samples) or the
   observation window expires.
10. Positional delta computed with the wrap-safe modular helper.
11. Record everything.

**Five repetitions.** No existing hardware evidence argues for another count,
so the instructed preference stands.

The shaft is never touched. No centering move is issued. Between repetitions
the firmware dwells 500 ms (pacing, not safety).

### 5.3 What is recorded, per repetition

```
P0                 PresentPosition before the prime
P_FIRST            first sampled position after Torque OFF was proven
P_SETTLED          position at the settle criterion
DELTA_IMMEDIATE    circularDelta(P_FIRST, P0)
DELTA_TICKS        circularDelta(P_SETTLED, P0)
PEAK_ABS_DELTA     largest |delta| seen at 500 Hz across the window
MIN_DELTA/MAX_DELTA signed excursion bounds
READ_US            duration of the PresentPosition read
WRITEPOSEX_US      duration of the WritePosEx call
EXPOSURE_US        WritePosEx return -> Torque-OFF write completion
TOTAL_US           WritePosEx start -> Torque-OFF write completion
current, voltage, temperature, peak speed/load/current, status, samples
```

`EXPOSURE_US` is the quantity the instruction asked for: the interval between
`WritePosEx` returning and the Torque-OFF write completing. `TOTAL_US` bounds
the whole window in which torque could have been armed.

The host recomputes `DELTA_TICKS` and `DELTA_IMMEDIATE` from `P0`, `P_FIRST`
and `P_SETTLED` and fails closed if the firmware's numbers disagree.

### 5.4 The safety abort is not the threshold

The harness needs *some* bound at which unintended motion stops being a
measurement and becomes a safety event. Inventing one would be exactly the
failure this project exists to avoid, so it reuses the only validated MATDOG
precedent:

> QC V6.1 aborts a monitored move on 16 ticks of travel **against** the
> commanded direction (`directionalTravel < -16`,
> `matdog_servo_commissioning.ino`).

`PRIME_SAFETY_ABORT_TICKS = 16` is that number, reused unchanged, and the
firmware prints it with `ROLE=GROSS_MOTION_ABORT NOT_A_THRESHOLD_CANDIDATE=1`.

**Do not promote 16 into `PRIME_MAX_DELTA_TICKS`.** They answer different
questions. The threshold must be selected from the measured deltas, and on the
campaign evidence it is expected to be far smaller.

The remaining guards are reused verbatim from QC V6.1: overcurrent 308 raw for
50 samples, stall current 416 raw for 10 samples, voltage outside 40..140 for
50 samples, status nonzero for 25 samples, telemetry loss 25 consecutive failed
reads, thermal limit 70 °C, post-write settle 20 ms.

### 5.5 Failure semantics

Any failure — unverified write, gross motion, status fault, communication
fault, electrical or thermal fault — converges identically:

```
FAULT REASON=<typed reason>
Torque OFF, verified, retried once if the first attempt fails
RECOVERY TORQUE_OFF=OK|FAILED
EEPROM_WRITES=NONE
REMOVE_SERVO_POWER_NOW
PRIME_RESULT FAIL
```

No rollback, no blind retry, no factory reset, and **no implicit movement as
cleanup**. `enterFault` contains no motion primitive at all; the static audit
asserts the string `WritePosEx` does not appear in its body.

---

## 6. Characterization B — true power-cycle trace

Strictly read-only. `Ping` and a `Model` read are the only bus traffic, and the
host aborts the session if a single `WRITE` line appears in a trace transcript.

### 6.1 Why it records episodes instead of detecting a power cycle

The obvious design — "wait for absence longer than *X*, then wait for *Y*
consecutive good replies" — cannot be built here, because *X* and *Y* are
precisely the two unknowns. Any value chosen for them inside the tool would be
the invention this project is trying to prevent, and would also silently decide
which observed dropout "was" the power cycle.

So the trace classifies nothing. It records **every** absence episode it sees,
with measured timestamps, and prints the table:

```
TRACE_EPISODE TRACE=1 INDEX=1 LAST_PRESENT_T_MS=… FIRST_ABSENT_T_MS=…
    FIRST_RETURN_T_MS=… ABSENT_POLLS=… DURATION_MS=… GAP_MS=…
    ENDED=1 STABLE=1 RETURN_TO_STABLE_MS=… RETURN_TO_STABLE_POLLS=…
    UNCLEAN_AFTER_RETURN=… STREAK_BREAKS=…
```

A 120 ms single-poll dropout and a 1.7 s rail collapse both appear, both timed.
Which one is the power cycle is obvious to a human reading the table and is
never decided by the firmware. The offline test suite asserts this explicitly
with a synthetic transcript containing a glitch *and* a real absence.

### 6.2 Poll definition and phases

Each poll is `Ping`, and — only if the ping succeeded — a `Model` word read.
Two independent qualities are tracked per poll:

* **present** — `Ping` returned the ID with status `0x00`;
* **clean** — present *and* `Model == 777` with read status `0x00`.

The distinction matters: the question `COLD_RETURN_STABLE_SAMPLES` answers is
"how long after the servo starts answering are its register reads
trustworthy?", and only a register read can measure that.

| Phase | Behaviour |
|---|---|
| ARM | poll and wait for the operator; heartbeat once per second with poll/clean/fail counts; enter OBSERVE on the first failed ping |
| OBSERVE | emit a full per-poll record for every poll, plus transition events |

`polls − clean − fail` from a heartbeat line gives the number of
present-but-unclean polls during ARM.

### 6.3 Windows, and why none of them is a threshold

| Parameter | Value | Role |
|---|---|---|
| `TRACE_POLL_PERIOD_MS` | 20 | inter-poll dwell |
| `TRACE_ARM_WINDOW_MS` | 300000 | operator patience; generous by instruction |
| `TRACE_OBSERVE_MIN_MS` | 20000 | minimum observation after the first dropout |
| `TRACE_OBSERVE_MAX_MS` | 60000 | hard cap |
| `TRACE_STABLE_TARGET` | 20 | **data-collection endpoint**, explicitly not the threshold |
| `TRACE_MAX_TRACES` | 2 | two power cycles per session, matching the pilot policy |

The trace ends when the observation window has run at least
`TRACE_OBSERVE_MIN_MS`, at least one absence episode has ended, and 20
consecutive clean responses have been seen — or at the hard cap. None of these
segments an absence or classifies one; they only decide how long to look.

An **unusable** trace — the operator never cycled the rail, or the return never
stabilised — does not consume a trace slot and emits no cold snapshot. The
operator simply re-issues `@TRACE`.

### 6.4 Measurement resolution — a hard limit worth knowing

The installed `SCSerial::IOTimeOut` is **100 ms**. A failing `Ping` therefore
blocks for ~100 ms, and the poll cadence during an absence is ~120 ms rather
than ~21 ms. Consequences:

* absence timing cannot be resolved finer than ~100 ms by **any** ping-based
  detector, this one or the provisioner's;
* `FIRST_RETURN_T_MS` carries the same ~±100 ms uncertainty;
* any `COLD_ABSENCE_DEBOUNCE_MS` eventually chosen must exceed that
  granularity by a comfortable margin to mean anything.

The firmware prints this as `TRACE_RESOLUTION … BASIS=SCSERIAL_IOTIMEOUT` in
every trace so the number is never read without its error bar.

### 6.5 What the trace cannot see

The instant the operator's hand moves the switch is not observable. What is
measured is **communication-level absence**, which begins at the first failed
ping after the rail actually collapses. `DURATION_MS` is therefore an
upper-bounded estimate of comms absence, not a rail-collapse time, and
`COLD_ABSENCE_DEBOUNCE_MS` derived from it inherits that meaning — which is the
correct meaning, since the provisioner also detects power cycles by ping.

### 6.6 Independent re-derivation

The firmware emits both the per-poll records and its own episode table. The
host rebuilds the episode table from the per-poll records alone and fails
closed if the two disagree. A tampered `DURATION_MS` in the transcript is
rejected by the offline suite.

One value cannot be derived from the per-poll log: the last present timestamp
*before* the log starts, since the first logged poll is already the first
absent poll. It is seeded from the firmware's episode 1 and the report marks it
`episode_1_gap_ms_seeded_from_firmware`.

---

## 7. Constant provenance

### 7.1 Reused from frozen QC V6.1 evidence

| Constant | Value | Provenance |
|---|---|---|
| polling interval | 2000 µs | `QC_FAST_PERIOD_US` |
| settle criterion | \|speed\| ≤ 10, `Moving == 0`, 40 samples | QC characterization loop |
| overcurrent | ≥ 308 raw, 50 samples | `QC_OVERCURRENT_2A_RAW` |
| stall current | ≥ 416 raw, 10 samples | `QC_STALL_CURRENT_RAW` |
| voltage | outside 40..140, 50 samples | QC voltage safety gate |
| status byte | nonzero, 25 samples | `QC_STATUS_PERSIST_SAMPLES` |
| telemetry loss | 25 consecutive failed reads | QC characterization |
| thermal | > 70 °C | `QC_THERMAL_LIMIT_C` |
| post-write settle | 20 ms | QC `forceTorqueOffVerified` |
| gross-motion abort | 16 ticks | QC `directionalTravel < -16` |
| prime parameters | 300 / 365 / 50 | canonical profile doc §6 |

### 7.2 Data-collection parameters (not thresholds, not safety)

`PRIME_REPETITIONS = 5`, `PRIME_SETTLE_MIN_MS = 250`,
`PRIME_SETTLE_MAX_MS = 2000`, `PRIME_INTER_REP_DWELL_MS = 500`,
`TRACE_POLL_PERIOD_MS = 20`, `TRACE_ARM_WINDOW_MS = 300000`,
`TRACE_OBSERVE_MIN_MS = 20000`, `TRACE_OBSERVE_MAX_MS = 60000`,
`TRACE_STABLE_TARGET = 20`, `TRACE_MAX_TRACES = 2`,
`TRACE_MAX_EPISODES = 8`.

Every one of these decides only how much is observed and for how long. Torque
is off, the EEPROM is never unlocked, and none of them can shorten a safety
response.

### 7.3 Deliberately not selected

`PRIME_MAX_DELTA_TICKS`, `COLD_ABSENCE_DEBOUNCE_MS` and
`COLD_RETURN_STABLE_SAMPLES` appear in this project **only** as the string
`DEFERRED_TO_REVIEW`.

---

## 8. Evidence

One exclusively-created directory per session:

```
characterization/sessions/<LABEL>__<UTC>/
    01_before_state71.bin
    02_after_prime_state71.bin
    03_after_cold_cycle1_state71.bin
    04_after_cold_cycle2_state71.bin      (second trace only)
    session_console.log
    characterization_report.json
    SHA256SUMS
```

Every `.bin` is written with `open(path, "xb")` → `write` → `flush` →
`os.fsync` → on-disk length verified `== 71` → SHA256 recorded. Evidence is
never overwritten; a pre-existing directory or file fails closed.

### 8.1 Evidence-before-write is enforced, not asserted

`EvidenceGate` sits inside the transport. Before any command leaves the host it
is checked, and `@PRIME` — the only write-capable command — is **refused**
unless `01_before_state71.bin` exists on disk, is exactly 71 bytes, was created
exclusively, was fsync'd, and re-hashes to the recorded SHA256. A tampered file
re-raises the refusal. The self-test exercises all of that, including the case
where `@ARM` fails and proves no write-capable command was ever transmitted.

The session report also carries an ordering ledger with the index of the BEFORE
persist and the index of the first write-capable command; a run whose ordering
cannot be proven is downgraded to `FAIL` even if every phase passed.

### 8.2 Cold state is observed, never asserted

The post-trace 71-byte snapshot is recorded and decoded, and nothing about it
is required to equal anything. NEW01 is unprovisioned: its `PositionOffset` is
+85, its `Lock` is 1, and its RAM `TorqueLimit` is expected to revert from 300
to 1000 across the power cycle. That reversion is itself useful evidence for
the provisioner's §12 note that `TorqueLimit` after cold boot must never be
treated as persistent operating policy.

---

## 9. Offline verification

* `arduino-cli compile` at the frozen provisioner FQBN — PASS, no warnings
  from this sketch.
* `python3 -m py_compile` on the runner — PASS.
* `--self-test` — 234 checks, 0 failures, no serial port opened.
* `--static-audit` — 37 firmware checks, all true.

The suite covers every item the task required: 71-byte evidence-before-write
ordering; the exactly-one-responder, model 777 and `ResponseStatus` gates; the
`0..4095` position domain; modular distance across the wrap; only Torque OFF;
only `TorqueLimit = 300`; `WritePosEx` always using the current position; speed
exactly 365; acc exactly 50; no EEPROM write primitives; no ID write; no
`PositionOffset` write; no mid-position calibration; no factory reset; no
broadcast write; failure paths converging to Torque OFF; and the power-cycle
trace timestamp calculations.

### 9.1 Sketch layout note

`arduino-cli` requires the sketch directory name to match the main `.ino` name.
The task asked for a flat `characterization/` layout, so compilation is done
from a throwaway copy:

```bash
mkdir -p /tmp/chz/matdog_st3215_characterize_v1 && cp characterization/matdog_st3215_characterize_v1.ino /tmp/chz/matdog_st3215_characterize_v1/ && arduino-cli compile --fqbn "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi" /tmp/chz/matdog_st3215_characterize_v1
```

No build directory is created inside the project tree.

---

## 10. Honest limitations

1. **The exposure window is timed but not sampled.** Position cannot be read
   between `WritePosEx` returning and the Torque-OFF write without lengthening
   the interval in which torque may be armed, which would be the wrong trade.
   `PEAK_ABS_DELTA` therefore begins after the OFF write completes. A transient
   confined entirely to `EXPOSURE_US` (order 10² µs) would be visible only
   through its residual.
2. **The golden-path trace comparison is not fully independent.** In the
   offline suite the synthetic firmware episode table is produced by the same
   host derivation that verifies it. Independence comes from the hand-computed
   timing assertions (1680 ms = 14 × 120 ms, 399 ms = 19 × 21 ms, gap 1800 ms)
   and from the tamper tests, not from that comparison. On real hardware the
   comparison *is* independent, because the two tables then come from different
   machines.
3. **Absence resolution is ~100 ms**, set by `SCSerial::IOTimeOut` (§6.4).
4. **The operator's switch instant is unobservable** (§6.5).
5. **`TorqueLimit` is left at 300 in RAM.** Restoring 1000 would be a write
   outside the permitted list, so it is not performed. The value is
   non-persistent and reverts at the next servo power-on.
6. **Single-unit evidence.** These are NEW01 measurements. Whether they
   generalise across the 17-servo population is a separate question this tool
   does not answer.

---

## 11. Operator procedure

```bash
python3 characterization/matdog_st3215_characterize_v1.py --self-test
```

```bash
python3 characterization/matdog_st3215_characterize_v1.py --static-audit
```

Then, only when hardware work is authorised: upload the firmware, and run

```bash
python3 characterization/matdog_st3215_characterize_v1.py --characterize --unit NEW01 --traces 2
```

The run scans, gates, persists the BEFORE evidence, performs five prime
repetitions, and then prints

```
POWER CYCLE NEW01 SERVO RAIL NOW
SERVO RAIL OFF, THEN ON
LEAVE ESP32 USB POWER CONNECTED
```

Switch **only** the servo rail. The ESP32 stays on USB — an ESP32 reset would
destroy the trace and end the session. No typed confirmation is required or
accepted.

---

## 12. After the measurements

Selecting the three constants is a human decision made from the reported
evidence, outside this tool. The evidence to read:

* **`PRIME_MAX_DELTA_TICKS`** — `PRIME_DELTA … DELTA_TICKS` and
  `PEAK_ABS_DELTA` across the five repetitions, and the spread between them.
  The threshold must cover the observed transient excursion, not only the
  settled residual.
* **`COLD_ABSENCE_DEBOUNCE_MS`** — the episode table. The separation between
  the shortest spurious dropout and the real rail-collapse absence, read
  against the ~100 ms resolution floor.
* **`COLD_RETURN_STABLE_SAMPLES`** — `RETURN_TO_STABLE_POLLS`,
  `UNCLEAN_AFTER_RETURN` and `STREAK_BREAKS`. If a returning servo answers
  `Ping` before its register reads are trustworthy, those three numbers say by
  how much.

Whatever is chosen, the change belongs in
`matdog_st3215_provisioner_v1.ino` together with
`PROVISIONER_HARDWARE_FREEZE_BLOCKED`, cited to the session evidence directory
produced by this harness. Nothing in this project performs that edit.

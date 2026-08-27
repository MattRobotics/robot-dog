# MATDOG RF calibrator — canonical handoff to Claude

**Date:** 2026-08-05  
**Purpose:** transfer the complete state of the MATDOG RF mechanical calibrator after an unsuccessful OpenAI development session, while preserving the only hardware-validated baseline: LF V25.

This document is the authoritative starting point for any further RF work. It is documentation only. It does not certify any RF program for hardware execution.

---

## 1. Repository policy after cleanup

The repository must contain exactly two branches:

```text
main
release/matdog-lf-calibrator-v25
```

`release/matdog-lf-calibrator-v25` is immutable and must remain exactly at:

```text
f87dd1fbc7e8100d275c74f9af448642f3429680
```

The release branch is the exact reviewed and hardware-validated LF V25 source. Do not rewrite it, force-push it, merge into it, rebase it, delete it or use it as a scratch branch.

`main` is the sole development and documentation branch after cleanup. Future RF work must begin from the current `main`, with only one clearly named development branch created by the next engineer/model. Temporary dispatcher branches, version-numbered branches, repair branches and one-shot workflow branches are prohibited.

All RF packages produced before this handoff are revoked. No package named or associated with the following experimental checkpoints may be run again:

```text
763c767d085bbb7c5025886c89f34191a11e9722
5d7ac696739b823a4532f433ca5a3bc47a491221
408d4b511ab6ae61d17e8f518efd03a58e2a5c3a
80b6078ea7d56367567ab23682c825547960c5ed
9ed73efcc69862c2ca4865c583ce6c58ff3797b0
b5b1256e3309c4d4d299194f48b9272ecb4ec9ce
dca9d104c358841099a86910cb380fcd9eecea23
9482086baba6fac0c266c3dc509352b6547d0365
```

The final experimental RF head before cleanup was:

```text
branch: matdog/rf-native-calibrator
head:   9482086baba6fac0c266c3dc509352b6547d0365
PR:     #18
```

That head is an experimental archive reference only. It is not a valid implementation base and must not be merged into `main` wholesale.

---

## 2. Canonical LF V25 baseline

The only mechanically hardware-validated MATDOG calibration implementation is LF V25.

Canonical source:

```text
release/matdog-lf-calibrator-v25
f87dd1fbc7e8100d275c74f9af448642f3429680
implementation PR: #11
```

Validated LF result:

```text
measurement sequence: 58/58 DONE
six LF contacts: PASS
affine/URDF gate: PASS
hardware witness: PASS
RAM stage: LF_STAGED
EEPROM transaction: LF_FROZEN
persistent profile: LF_FROZEN
global torque OFF: verified
```

Final LF contact evidence:

| Joint | Motor | MIN contact | MAX contact | affine q0 before EEPROM |
|---|---:|---:|---:|---:|
| LF HIP | M13 | 2535 | 1600 | 2067 |
| LF UPPER | M12 | 1439 | 3443 | 2040 |
| LF LOWER | M11 | 3093 | 1658 | 2074 |

Measured mechanical spans:

| Joint | URDF span | LF measured span | Difference |
|---|---:|---:|---:|
| HIP | 90.00° | 82.18° | -7.82° |
| UPPER | 174.99° | 176.13° | +1.14° |
| LOWER | 129.55° | 126.12° | -3.43° |

Frozen LF EEPROM result:

| Motor | Previous Position Offset | Frozen Position Offset | displayed q0 |
|---|---:|---:|---:|
| M11 LOWER | 101 | 127 | 2046 |
| M12 UPPER | 859 | 851 | 2051 |
| M13 HIP | -505 | -486 | 2048 |

The exact LF release contains historical internal filenames from development. Those filenames do not define a later release. The validated version remains LF V25.

### Principal LF files

```text
software/drivers/st3215/src/auto_calibrate/matdog.rs
software/drivers/st3215/src/auto_calibrate/matdog_test.rs
software/drivers/st3215/src/bin/matdog_lf_freeze.rs
tools/matdog/matdog_headless_auto_calibrate.py
tools/matdog/matdog_lf_profile.py
tools/matdog/matdog_native_observer_contract.py
```

The next implementation must compare RF against LF V25 line by line and preserve LF behavior. Do not copy the failed RF branch and then try to patch it incrementally without first auditing the exact release.

---

## 3. Permanent architecture and safety contracts

These rules are binding and may not be relaxed to make a test pass.

### Serial ownership

- NormaCore Station is the sole owner of the ST3215 serial adapter during motion.
- No `pyserial`, second Station instance, direct servo process or competing serial client.
- The Python runner observes and orchestrates through Station; it does not own the bus.

### Motor topology

Exactly these twelve IDs must be present, once each:

```text
11 12 13 21 22 23 31 32 33 41 42 43
```

Any missing, duplicated or unexpected ID is a hard block.

### Goal position and memory policy

- `GoalPosition` is always unsigned standard `0..4095`.
- Signed-wrap is permanently forbidden.
- Mechanical measurement is RAM-only.
- No EEPROM write, Position Offset write, LOCK write, RegWrite, Action, ResetCalibration or freeze during RF measurement.
- Persistent RF freeze is prohibited until a complete supervised six-contact RF measurement passes all gates and is reviewed separately.

### Motion and cleanup

- One active probing joint at a time.
- All prerequisite joints are actively held and continuously checked.
- All nonparticipants remain torque-off and are checked for drift/state integrity.
- Every success and failure path ends in verified global torque OFF.
- Station must stop gracefully and release the serial adapter.
- Hardware failures must remain fail-closed; no acceptance by changing a label, hiding an error or widening a limit after observing a failure.

### Detector and guard policy

The LF V25 detector combines:

```text
command direction
encoder progress
persistent loss of progress
velocity / derivative evidence
adaptive current relative to baseline
fresh telemetry
valid servo state
model-derived contact corridor
travel guard
timeout
maximum displacement
```

After suspected contact:

```text
stop
controlled backoff
recovery verification
fine approach #1
backoff
fine approach #2
fine-to-fine repeatability
```

The coarse contact is a scout and is not used as the final endpoint measurement.

Do not widen or move:

- external mechanical guards;
- hard-current abort;
- timeouts without direct proof;
- repeatability gate;
- affine gate;
- witness gate merely to accept a failed trace.

LF V25 contains a reviewed bounded HOME-facing adaptation for chamfer/friction behavior. It must be reused generically, not replaced by a per-motor exception.

### Temperature policy

The reviewed V25 thermal behavior is preserved:

- configured temperature limit must be exactly 70°C;
- Station performs direct confirmation reads;
- a transient bulk-read spike is evidence but not automatically real heating;
- real over-temperature requires confirmed readings according to the existing V25 logic;
- do not add thermal filtering to the observer or launcher.

---

## 4. Canonical mapping and directions

Motor mapping:

| Leg | HIP | UPPER | LOWER |
|---|---:|---:|---:|
| LF | M13 | M12 | M11 |
| RF | M23 | M22 | M21 |
| RH | M33 | M32 | M31 |
| LH | M43 | M42 | M41 |

Verified encoder-to-URDF directions:

```text
LF
M13 HIP    -1
M12 UPPER  +1
M11 LOWER  -1

RF
M23 HIP    -1
M22 UPPER  -1
M21 LOWER  +1

RH
M33 HIP    +1
M32 UPPER  -1
M31 LOWER  +1

LH
M43 HIP    +1
M42 UPPER  +1
M41 LOWER  -1
```

Canonical conversion concept:

```text
q_rad = direction × signed_tick_delta(present_tick, zero_tick) × 2π / 4096
```

The signed delta is only a local mathematical difference on the encoder circle. It does not authorize signed `GoalPosition` commands.

URDF limits used by MATDOG:

| Joint type | URDF MIN | URDF MAX |
|---|---:|---:|
| HIP | -45° | +45° |
| UPPER | approximately -52.5° | approximately +122.5° |
| LOWER | approximately -97.5° | approximately +37.5° |

RF is mechanically the mirrored counterpart of LF. The mechanisms have the same intended angular geometry and excursion, but the servo installation can translate both RF encoder endpoints. Therefore:

```text
same mechanical span / geometry
!=
same absolute encoder ticks around 2048
```

This distinction caused one of the major failed attempts.

---

## 5. Required RF physical sequence

The user requires the RF procedure to reproduce LF V25 physically, not merely reproduce labels or nominal tick formulas.

### Overall sequence

```text
preflight once
initial recovery to digital HOME where required
RH UPPER M32 parking once
RF UPPER: first endpoint -> second endpoint
RF UPPER to V25 horizontal prerequisite
RF LOWER: first endpoint -> second endpoint
RF LOWER to V25 folded/parallel prerequisite
RF HIP: physical downward endpoint first -> physical upward endpoint second
compute affine models and q0 from measured pairs
return HIP, LOWER, UPPER and parking in the same reviewed V25 order
global torque OFF
```

No HOME reset is inserted between the two endpoints of the same joint unless LF V25 does it.

### Parking

For RF front-leg calibration the reviewed parking joint is RH UPPER M32. The nominal parking target used in the experiments was:

```text
M32 = 1707
```

Startup admission must reuse the LF V25 home-side settling rule symmetrically. A valid near-HOME position may be actively normalized before parking. Do not invent a direct jump from an arbitrary position to parking.

### UPPER and LOWER prerequisite pose for HIP

The V25 raw prerequisite deltas around digital HOME are:

```text
RF UPPER M22 = 1024
RF LOWER M21 = 1058
```

These values came from applying the verified RF directions to the same V25 geometric deltas used on LF. A previous attempt replaced them with measured-affine targets around `1044/1032`; that did not reproduce the requested physical pose and was rejected.

The user’s non-negotiable visual/mechanical requirement is:

- during HIP probing, LOWER must be in the same LF V25 parallel/folded relationship to UPPER;
- it must not remain visibly more open;
- after return to q=0, RF LOWER must not remain visibly raised relative to LF.

Do not modify these targets by visual guess. Audit the LF V25 prerequisite construction and the RF direction mapping, then prove the RF joint angles from URDF math and the measured/frozen zero convention.

### HIP execution semantics

The user requires the first physical HIP movement to go downward to the LF-equivalent first stop, followed by an upward movement to the opposite stop.

Because RF M23 has verified direction `-1`, label-only reasoning is unsafe. The experimental branch eventually used:

```text
RF first physical/downward search: decreasing M23 ticks
RF second physical/upward search: increasing M23 ticks
```

The experimental code represented this as `URDF MAX` first and `URDF MIN` second, then reordered the evidence container. Claude must verify this against the actual URDF joint axis and the frozen direction mapping before retaining or renaming it. The physical sequence is authoritative; printed MIN/MAX labels must match the geometry and must not mislead the operator.

The RF HIP excursion must be mechanically equal to LF’s excursion within the validated witness policy. A plateau substantially before the expected second endpoint must not be accepted merely because speed reached zero.

---

## 6. LF V25 mechanical witness versus RF encoder offsets

The following two ideas must not be confused.

### Correct invariant

RF must reproduce the hardware-validated LF mechanical span for the same joint type, within the reviewed LF witness tolerance.

Reference LF spans from final V25 contacts:

```text
HIP span   = |2535 - 1600| = 935 ticks
UPPER span = |3443 - 1439| = 2004 ticks
LOWER span = |3093 - 1658| = 1435 ticks
```

Depending on the exact historical helper and contact snapshot used by the frozen source, experimental tests also referenced close development-era values such as 1999 ticks for UPPER. Claude must extract the authoritative span directly from the immutable release code/data, not hardcode a value copied from this narrative without verification.

### Incorrect invariant

Do not require RF endpoints to equal fixed absolute mirror coordinates around 2048, such as:

```text
UPPER 2653 / 654
LOWER 1003 / 2430
HIP   2479 / 1561
```

That assumption was implemented in `dca9d104` and failed at RF UPPER after both contacts were measured. The affine gate passed, but the absolute endpoint witness failed. The failure proved that both RF endpoints may be translated together by the mechanical-to-encoder installation offset.

### Required relative-span method

The correct conceptual method is:

1. measure the first RF contact normally with the existing detector, corridor and guard;
2. use that measured contact as the RF encoder anchor;
3. obtain the authoritative LF V25 measured span for the same joint type;
4. predict the region of the second RF endpoint by advancing from the measured first RF contact in the second probe direction by the LF span;
5. do not allow a HOME-facing friction plateau to be accepted before the dynamically predicted second-contact entry;
6. measure the second contact with coarse scout, backoff, fine #1, backoff, fine #2;
7. compare the measured RF span to LF V25 using the reviewed LF hardware-witness tolerance;
8. derive RF q0 and scale from the RF measured pair through the existing affine solver;
9. never force RF q0 to 2048 and never force the RF endpoints to be centered on 2048.

At experimental head `9482086...`, the branch already contained helpers conceptually named:

```text
lf_v25_reference_span_ticks
rf_relative_second_contact_entry_tick
configure_rf_relative_second_contact_entry
profile_home_facing_before_relative_search_entry
rf_span_witness_deviation
```

It also constrained adaptive acceptance using a dynamically calculated second-contact entry. These ideas are useful experimental evidence, but the branch also contained temporary workflows and accumulated unrelated changes. Reimplement or selectively port only after comparing them against the LF release.

The branch still used:

```text
RF_MIRROR_SPAN_TOLERANCE_TICKS = REPEATABILITY_TOLERANCE_TICKS
```

which equals 16 ticks. A pending one-shot publisher intended to change this to the existing LF contact-witness tolerance of 24 ticks because a real RF UPPER trace produced a 17-tick span deviation. That publication never reached a clean canonical commit.

Claude must resolve this by reading the immutable LF V25 witness contract. The intended rule is not “pick 24 because the trace needs 17”; it is “reuse the exact LF hardware-witness tolerance already reviewed for LF.” If the frozen release defines 24 ticks for that witness, use 24. Do not invent a new RF number.

---

## 7. Failure ledger and lessons

### A. Initial RF package and M32 parking admission

An early RF run failed before calibration because M32 current-position priming near HOME was rejected. The generalized parking admission had not mirrored the LF V25 home-side tolerance.

A correction admitted the direction-symmetric near-HOME case and kept the parking corridor unchanged. Lesson: startup normalization and parking must copy LF V25 semantics, not use strict equality to 2048.

### B. M22 UPPER MIN guard discretization

Observed RF trace:

```text
target 2676
present 2667
next fixed coarse target 2740
guard 2709
```

The 64-tick fixed step skipped the remaining safe distance to the existing guard. A general helper was introduced to make the final command land on the existing guard rather than jump beyond it. The guard itself was not widened.

Lesson: fixed discretization may terminate on the existing guard; never extend the guard.

### C. M21 LOWER chamfer / soft contact

A run reached approximately:

```text
target 2439
present 2399
current 48
velocity 0
```

The position was just HOME-side of the base corridor. LF V25 already contained a bounded 32-tick HOME-facing scout adaptation for chamfer/friction. The correction generalized that existing behavior rather than adding an RF exception.

Lesson: reuse the existing V25 coarse/fine geometry; no per-motor special case.

### D. HIP order and prerequisite drift

Multiple attempts confused logical URDF labels with physical direction. One version moved the first HIP search upward when the user required downward. Later versions corrected the physical direction but still did not faithfully reproduce the LF prerequisite geometry.

Lesson: verify physical direction using the frozen mapping and URDF. Do not trust labels alone.

### E. Measured-affine prerequisite attempt

A version computed RF HIP prerequisite targets from the newly measured UPPER/LOWER affine maps, producing approximately:

```text
M22 1044
M21 1032
```

The user observed LOWER was not in the same LF V25 parallel pose. The program also ended with an M23 q0 error of 11 ticks against a 10-tick gate.

Lesson: this was not a justified copy of LF V25. Do not revive it. Do not hide the 11-tick failure by widening the q0 tolerance.

### F. RF witness bypass

One experimental implementation contained an unconditional RF witness acceptance equivalent to:

```rust
Leg::Rf => true
```

This allowed a false PASS even when the second HIP search stopped early around 2467 ticks without reaching the true mechanical stop.

Lesson: RF must have a real mechanical witness. Unconditional acceptance is prohibited.

### G. Absolute mirrored endpoint witness

The next attempt replaced the bypass with absolute mirrored LF endpoint coordinates around 2048. It failed at UPPER with:

```text
affine_accepted=true
witness_accepted=false
```

LOWER and HIP did not start; cleanup and torque OFF were correct.

Lesson: equal mechanical span does not imply equal absolute RF encoder coordinates.

### H. Relative span work left unfinished

The final experimental head implemented a first-contact anchor and dynamically predicted second-contact entry, but accumulated temporary workflows and never reached a clean, fully certified package. The head was 101 commits ahead of main and changed eleven files, including three temporary workflows.

Lesson: do not merge the branch wholesale. Use it only as an archive of experiments and tests.

---

## 8. Experimental branch snapshot before deletion

Before repository cleanup:

```text
main: f6f18405ea3515e076e1d612e1bc6e908f7a9793
RF experimental branch: 9482086baba6fac0c266c3dc509352b6547d0365
release LF V25: f87dd1fbc7e8100d275c74f9af448642f3429680
```

RF experimental branch versus main:

```text
101 commits ahead
0 commits behind
```

Files changed:

```text
.github/workflows/matdog-native-calibrator-check.yml
.github/workflows/matdog-rf-relative-span-v25-tolerance-repair.yml
.github/workflows/matdog-rf-repair-target.yml
.github/workflows/matdog-rf-v25-span-final-publish.yml
software/drivers/st3215/src/auto_calibrate/matdog.rs
software/drivers/st3215/src/auto_calibrate/matdog_test.rs
software/drivers/st3215/src/port.rs
tools/matdog/matdog_headless_auto_calibrate.py
tools/matdog/matdog_v42_pinned_launcher.py
tools/matdog/test_matdog_headless_auto_calibrate.py
tools/matdog/test_matdog_v42_pinned_launcher.py
```

The branch included useful experiments but also workflow scaffolding and cumulative repairs. PR #18 and its closed technical PRs preserve the review history. They are not valid development branches.

Relevant PRs:

```text
#18 RF experimental development and failure record
#19 guard discretization dispatcher
#20 V25 HOME-side coarse adaptation dispatcher
#21 HIP profile parity dispatcher
#22/#23 measured prerequisite experiments
#24/#25/#26 relative span tolerance publisher attempts
```

All open PRs are to be closed without merging experimental code into main. The documentation in this file replaces the need to keep their branches alive.

---

## 9. What Claude must do

Claude must not continue by editing the deleted experimental branch. The required workflow is:

### Phase 1 — read-only audit

1. Verify remote `main` and immutable release SHA directly from GitHub.
2. Read this handoff and `tools/matdog/README.md`.
3. Read the exact LF V25 source from `release/matdog-lf-calibrator-v25`.
4. Inspect closed PR #18 and experimental head `9482086...` only as an error/evidence archive.
5. Compare LF release and experimental RF line by line.
6. Produce a written implementation plan before changing code.

The plan must explicitly map every RF behavior to one of:

```text
unchanged LF V25 engine behavior
RF data-only substitution
necessary generic correction proven by both LF and RF tests
rejected historical experiment
```

### Phase 2 — one clean development branch

Create exactly one branch from current `main`, for example:

```text
matdog/rf-calibrator-from-lf-v25
```

No other branch is allowed. No temporary dispatcher branch is allowed.

### Phase 3 — minimal implementation

The preferred result is one shared engine with data-driven LF/RF geometry, but minimality is more important than an abstract refactor. Do not rewrite LF V25 to make RF easier.

Required properties:

- LF V25 regression remains byte-equivalent in behavior;
- exact RF mapping and directions;
- exact physical sequence and prerequisite pose;
- first RF contact used as encoder anchor;
- second endpoint search constrained by LF V25 mechanical span;
- RF witness compares span, not absolute endpoint ticks;
- RF q0/scale derived from measured RF pair;
- no unconditional RF witness;
- no per-motor exception;
- no change to guard/current/temperature/torque/speed without direct LF contract evidence;
- no EEPROM path in RF measurement package.

### Phase 4 — tests using real failures

Tests must reproduce at least:

1. M32 near-HOME parking admission and a value outside the admitted band.
2. M22 final coarse step landing on the existing guard without extending it.
3. M21 chamfer contact inside the existing V25 HOME-facing adaptation and a point one tick outside the allowed band.
4. RF first HIP physical movement in the required downward direction.
5. RF second HIP physical movement upward.
6. Identical held prerequisite pose for both HIP endpoints.
7. Rejection of the early HIP plateau around 2467 when it is before the dynamically predicted second endpoint.
8. Acceptance of translated RF endpoint pairs whose span matches LF V25.
9. Rejection of pairs with correct-looking affine q0 but wrong mechanical span.
10. Rejection at one tick beyond the LF witness tolerance.
11. LF V25 six-contact regression unchanged.
12. V25 temperature confirmation behavior unchanged.
13. Global torque-OFF cleanup on every injected failure.
14. Exact twelve-ID topology.
15. No EEPROM/register-write path from the RF runner.

Tests must validate behavior, not merely search source strings.

### Phase 5 — CI discipline

Use only the durable workflows already present on `main`:

```text
.github/workflows/matdog-native-calibrator-check.yml
.github/workflows/matdog-native-observer-check.yml
```

Do not create one-shot workflows, self-deleting workflows or temporary CI branches. Extend the durable tests only when necessary and keep the workflow history clean.

Required gates:

```text
rustfmt PASS
RUSTFLAGS=-D warnings cargo test --package st3215 --all-targets PASS
LF runner self-test PASS
RF runner self-test PASS
Python observer/launcher tests PASS
Station release build PASS
package self-test PASS
git diff --check PASS
```

### Phase 6 — human review before hardware

Before producing a hardware package, show the user:

- exact diff against main;
- exact diff of shared engine behavior against LF V25;
- table of RF data substitutions;
- proof of physical HIP direction;
- proof of LOWER/UPPER prerequisite geometry;
- proof that the relative span witness cannot accept the previous false plateau;
- list of unchanged safety constants.

No claim of hardware readiness is allowed before this review.

### Phase 7 — package rules

A hardware package must:

- include a precompiled warning-free Station binary;
- pin one exact commit SHA;
- have unique filenames containing the short SHA and purpose;
- include manifest and SHA256 certificate;
- refuse any repository HEAD other than the certified commit;
- verify runner, observer, launcher and Station hashes;
- use Station as sole serial owner;
- remain RAM-only;
- save complete evidence logs;
- verify global torque OFF and Station shutdown;
- never claim hardware PASS in advance.

### Phase 8 — hardware validation

The robot is suspended/supported, RF leg free, cables clear and operator present.

Operator-visible checkpoints:

- M32 parks correctly;
- RF UPPER reaches both true endpoints;
- RF LOWER reaches both true endpoints and assumes the same LF V25 prerequisite relationship;
- before HIP, LOWER appears parallel/folded as intended;
- first HIP movement is physically downward;
- second HIP movement is physically upward and continues to the true stop rather than accepting an early plateau;
- final RF q0 pose visually matches LF counterpart;
- no EEPROM is written;
- global torque OFF verified.

Only after complete RF measurement PASS may a separate, reviewed transactional RF freeze be designed.

---

## 10. Explicit prohibitions for Claude

Claude must not:

- merge or revive PR #18 wholesale;
- use any revoked package;
- modify the immutable LF V25 branch;
- create more than one development branch;
- create temporary workflow branches;
- create one-shot workflows to edit the repository;
- force-push or rewrite history;
- introduce signed GoalPosition;
- open the serial outside Station;
- loosen the 10-tick q0 gate merely because a run ended at 11 ticks;
- accept RF witness unconditionally;
- force RF endpoints around 2048;
- hardcode RF absolute contacts from mirrored LF ticks;
- tune a tolerance from one failing trace when LF already defines the contract;
- change LOWER prerequisite targets by visual guess;
- rename MIN/MAX without checking physical geometry;
- claim PASS from offline tests alone;
- write EEPROM during measurement;
- leave failed PRs, temporary branches or noncanonical workflows behind.

---

## 11. Communication contract with Matteo

The next model must:

- treat GitHub remote as the source of truth;
- verify before reporting;
- state exactly what changed and what remains unproven;
- never present a package as certified when only offline tests passed;
- stop immediately when the user says stop;
- avoid long autonomous workflow loops;
- give progress updates during long operations;
- preserve repository order and cleanliness continuously, not only at the end;
- propose architectural changes before implementing them;
- follow the agreed contract literally unless new hardware evidence proves a conflict.

The objective is not to invent a better calibrator. The objective is to reproduce LF V25 on RF using RF geometry, directions and measured encoder translation, while keeping the existing validated safety behavior.

---

## 12. Canonical restart statement

```text
Start from current main.
Treat release/matdog-lf-calibrator-v25 at f87dd1f... as immutable truth.
Treat PR #18 / head 9482086... only as a failure and experiment archive.
Implement RF on one clean branch.
Copy LF V25 behavior; substitute only RF mapping, direction, parking and geometry.
Use the first RF contact as encoder anchor and LF V25 measured span as the mechanical witness for the second contact.
Do not require absolute mirrored endpoint ticks around 2048.
Do not alter safety constants or EEPROM policy.
Do not produce a package until the user has reviewed the exact diff and all durable tests pass.
```

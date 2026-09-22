# MATDOG calibration — source precedence, oracle recovery and V25 assessment

**Audit date:** 2026-09-21 · **Branch:** `feat/controller-calibration-manager-v1`
**Amended:** 2026-09-22 — read-only follow-up audit resolved D4; see §5.

This document records the C0 evidence audit that preceded the native calibration
foundation. It exists so the next person does not have to re-derive which of several
contradictory-looking sources is authoritative, and so that the places where they
genuinely disagree stay visible instead of being quietly merged.

---

## 1. Source precedence

| Source | Classification |
|---|---|
| `06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml` → `calibration_reset:` | **AUTHORITATIVE CURRENT STATE** |
| `06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml` | **AUTHORITATIVE CURRENT STATE** |
| `MATDOG_LF_V25_HARDWARE_EVIDENCE_2026-08-04.json` | **HARDWARE ORACLE** (`IMMUTABLE_EXTERNAL_EVIDENCE`) |
| `MATDOG_LF_CALIBRATION_V25_FINAL.md` | **HARDWARE ORACLE** |
| `09_Logs/Historical/.../LF_V25_Hardware_Oracle/source/` | **HARDWARE ORACLE** (algorithms) · **HISTORICAL ONLY** (implementation) |
| `.../Generic_V25_Superseded_WIP/` | **SUPERSEDED WIP** — never hardware validated |
| `.../RF_Calibrator_Local_Only/` | **SUPERSEDED WIP** — never committed |
| `.../NormaCore_Main_MATDOG_Content/` | **HISTORICAL ONLY** (`matdog.rs` byte-identical to the LF oracle copy) |
| `matdog_geometry_*` V5 compiler | **REUSABLE DESIGN REFERENCE** |
| tag `archive/2026-08-29/full-leg-calibrator-v1-h0` (ex-branch `matdog/full-leg-calibrator-v1`) | **ARCHIVED HISTORICAL HARDWARE ORACLE** — see §5 D4 |

The ordering rule, which the repository states itself: current state defines what is
valid; hardware-validated LF V25 defines historical behaviour; generic work supplies
abstractions but never hardware truth; historical values never override a current
reset declaration.

---

## 2. Current calibration truth

```text
state:                          CALIBRATION_RESET_PENDING_FULL_RECALIBRATION
effective:                      2026-08-27
all_joint_data_below_is_stale:  true
hardware_motion_authorized:     false
```

**A trap the file documents about itself.** `robot.calibration_status` still reads
`DIGITAL_ZERO_CALIBRATED_AND_VERIFIED`. It was deliberately left stale because six
tools hard-assert that exact string; the file says to read `calibration_reset:`
instead and states those tools are unsafe to run against hardware. A naive reader of
the enum gets the opposite of the truth.

**The three meanings of 2048.** These are separate quantities that happen to share a
value, and conflating them is the single most dangerous error available here:

| Quantity | Value | What it means |
|---|---|---|
| `physical_raw_center` | 2048 | ST3215 raw centre. "Says nothing about joint zero, mounting or direction." |
| displayed after freeze | 2048 ± 10 | what the servo reads back once `PositionOffset` is written |
| measured q0 | **2067 / 2040 / 2074** | LF V25's actual affine q0 — **none of them is 2048** |

The repository's rule: *"The final value MUST BE MEASURED, not assumed or asserted.
Do not impose q0_correction = 0."*

**The identity trap.** `MATDOG_SERVO_ALLOCATION.yaml` is keyed by **physical unit**,
with `bus_id` current and `source_id` historical:

```text
unit M33   -> LF_LOWER,   bus_id 11,  source_id 33
unit ELR01 -> LF_UPPER,   bus_id 12,  source_id 6
unit M22   -> LF_HIP,     bus_id 13,  source_id 22
unit M11   -> NECK_PITCH                          <- LF V25's "M11" is now the neck
```

Bus ids 11/12/13 still denote the LF joints, but they are **different physical
servos**. Historical per-unit evidence must never be applied by bus id.

---

## 3. What LF V25 actually proved — confirmed from the archive

| Fact | Value | Source |
|---|---|---|
| Sequence steps | **58** | `matdog_headless_auto_calibrate.py::EXPECTED_FULL_TOTAL_STEPS` |
| Contacts executed | **6**, LF only | `MATDOG_LF_V25_HARDWARE_EVIDENCE…json` (6 endpoints) |
| Profile table | **24** = 4×3×2 | `matdog.rs::all_profiles()` + `matdog_test.rs::profile_table_covers_exactly_24_unique_contacts` |
| Session states | **18** | `matdog.rs::enum LfSessionState` |
| Contact witness band | 24 ticks | `LF_CONTACT_WITNESS_TOLERANCE_TICKS` |
| Artefact lifecycle | `LF_STAGED → LF_FROZEN` | `matdog_lf_profile.py` |

LF V25 is the **only** MATDOG full-leg calibration ever hardware-validated, and it
covered one leg. Claiming 24 hardware-validated contacts would be false.

---

## 4. The three evidence vocabularies — deliberately not merged

| Vocabulary | Where | Status |
|---|---|---|
| `LF_STAGED` → `LF_FROZEN`, per-record `accepted`, global `PARTIAL` → `ACTIVE` | LF V25 artefacts | historical |
| `HARDWARE_VALIDATED` / `HARDWARE_CONTRADICTED` / `GEOMETRIC_ENDPOINT_CANDIDATE` | geometry compiler V5 | current, geometry domain |
| `MEASURED` → `CANDIDATE` → `ACCEPTED` → `PROMOTED` | `DEVELOPMENT_GATES.md` PASS CRITERIA | **current requirement** |

The implemented lifecycle is the third. **`PROMOTED` is a current requirement, not a
recovered LF V25 semantic** — the archive has no such stage. The mapping is recorded
here rather than collapsed in code.

---

## 5. Four discrepancies found, preserved rather than resolved by fiat

**D1 — `H1` names two unrelated things.** It appears **nowhere** in the LF V25
archive. In the current repository it means (a) the Full-Leg population gate over the
**12 leg servos** (ID 51 excluded), last formal result **6/12**; and (b) `H1 — Boot`,
the Controller's own first hardware test in `VALIDATION.md`. New code therefore uses
**leg population gate**; historical documents keep their wording.

**D2 — there is no "direction witness".** `direction` is a compile-time `JointSpec`
constant used arithmetically (`tick = HOME_TICK + direction × q_delta`), never
measured. The witness that exists is the **contact witness**. The model records
direction provenance so a spec constant can never be mistaken for evidence;
`MEASURED_CANDIDATE` and `ACCEPTED` are **TO_IMPLEMENT** — no historical mechanism
exists to recover.

**D3 — three evidence vocabularies.** See §4.

**D4 — `matdog/full-leg-calibrator-v1` was archived, not lost.** *(Resolved 2026-09-22;
the 2026-09-21 audit recorded this as UNRESOLVED because it looked for a branch.)*

The branch no longer exists as a branch — not local, not on `origin`. Its content is
preserved, intact, under a different ref type:

| Item | State |
|---|---|
| Commit `15f3fb8f378e6cadf6bc479bfcaca2947741c9fd` | **EXISTS** — `calibrator: authorize H1 read-only census`, 2026-08-29 |
| Archive tag | **`archive/2026-08-29/full-leg-calibrator-v1-h0`** — annotated (`e3fa747c…`) → `15f3fb8f…`, local **and** on `origin` |
| Prior provenance | branch `matdog/full-leg-calibrator-v1` @ `15f3fb8f…` |
| PR #22 | CLOSED, not merged (2026-09-15); `refs/pull/22/head` on `origin` still resolves to `15f3fb8f…` |
| Worktree artifacts | `~/MATDOG/archive/full-leg-calibrator-v1/` — 17 files, `sha256sum -c SHA256SUMS` 17/17 OK |

**The preservation mechanism is the tag and the out-of-Git archive, not the branch.**
The tag's own message is the archival record: it states the branch it replaced, PR #22's
outcome, the H0 PASS / H1 FAIL (`CENSUS_PRESENT=6/12`) hardware status, and that it was
archived 2026-09-18 after the G3/G3.1 Controller closeout. Deleting the branch retired a
pointer; it removed no oracle material.

Classification: **ARCHIVED HISTORICAL HARDWARE ORACLE**. Reference implementation only —
its calibration numbers are not valid for the current robot without revalidation, per the
`calibration_reset:` rule in §2.

Git does not record which command removed the branch and worktree (no reflog entry
survives: the branch ref's log went with the ref, and the per-worktree `HEAD` log went
with `robot-dog-full-leg-calibrator`). **Exact deletion command/actor: UNKNOWN** — which
does not weaken the archival evidence above. Note also that the NormaCore archive's own
provenance records only `release/matdog-lf-calibrator-v25`: it is a *separate*, older
preservation, not this one.

To read the oracle: `git show archive/2026-08-29/full-leg-calibrator-v1-h0` — do not
recreate the branch.

---

## 6. Generic V25 — component assessment

Branch `matdog/generic-v25-full-leg-engine` @ `f4a87a44…`, 16 unique commits.
**Hardware validated: NO.** G0–G3C gates completed offline; no end-to-end run, offline
or on hardware; RF/RH/LH profiles never finished.

| Component | Historical purpose | Offline tested | HW validated | Decision |
|---|---|---|---|---|
| `all_profiles()` 4×3×2 | four-leg contact table | yes | no (LF subset only) | **REUSE AS-IS** — byte-identical to LF; already recovered |
| `LegSessionStateMachine` | replace the LF-only machine | partly | no | **REDESIGN** — still holds `LfSessionState` and `contacts[3]`, and its `mode` field is `#[allow(dead_code)]` "until the legacy modes migrate". The migration never happened |
| `RawLegCalibrationSpec` | inert, leg-agnostic spec data | yes | no | **PORT SELECTIVELY** — the data/behaviour split is the good idea |
| `ArmableLfSessionSpec` + `validate_lf_v25` | sealed, single-producer spec | yes | no | **PORT SELECTIVELY (concept only)** — the sealing discipline is excellent; the rules are not portable, see below |
| `LFID-1..LFID-9` identity rules | reject a malformed spec | yes | no | **SUPERSEDED** — LFID-3 hard-codes motor IDs 13/12/11 and LFID-4 the directions. Both are exactly what the 2026-08-27 reassembly invalidated |
| `GoalNode` / `GoalWriteRoute` / `EmittedGoal` | one owner for `GoalPosition` writes | yes | no | **PORT SELECTIVELY** — maps onto the future Safe Actuator Layer, not onto this phase |
| `ContactVerdict` / `FreshObservation` | make single-owner telemetry explicit | yes | no | **REUSE AS-IS (concept)** — thin `motor_id` + payload wrappers |
| `ContactState` detector | contact detection state machine | yes | **yes, via LF V25** | **REUSE AS-IS** — already recovered into the domain |
| Phase sequencing (18 states) | LF execution order | yes | **yes, via LF V25** | **REUSE AS-IS** — recovered; belongs to a future execution engine |
| Timeout policy | command/telemetry/motion bounds | yes | **yes, via LF V25** | **PORT SELECTIVELY** — recovered as a failure taxonomy; the numbers are Station-era |
| Restore abstractions | ordered return + torque off | yes | **yes, via LF V25** | **REUSE AS-IS (semantics)** — recovered as intent; the motion is not ported |
| `NormalizationStage` / `LegacyEntryStage` / `GrammarNode` | progress/grammar modelling | partly | no | **INSUFFICIENT EVIDENCE** — no completed design to judge |
| Station ownership, UART, scheduler | the whole runtime premise | n/a | n/a | **SUPERSEDED** — the ESP32-S3 owns the bus |

**Net:** the generic engine's *structure* is instructive and its *implementation* is
not portable. Nothing from it was imported; the concepts marked PORT SELECTIVELY are
recorded for the future execution engine and the Safe Actuator Layer.

---

## 7. EEPROM boundary

LF V25 performed a transactional EEPROM freeze (backup → unlock → staged `RegWrite` →
`Action` → readback → relock → rollback on failure), via a separate binary
(`matdog_lf_freeze.rs`) that ran only after Station exited and released the serial
adapter.

| Behaviour | Classification |
|---|---|
| LF V25 EEPROM freeze | **HISTORICAL ARTIFACT** — Station-era, ran outside the calibrator |
| `PositionOffset` write | **PROVISIONING RESPONSIBILITY** — all 17 units already hold `PositionOffset = 0`; the YAML forbids rewriting it to compensate mounting error |
| `CalibrationOfs`, one-key-middle | **FORBIDDEN** — named in the YAML's `forbidden:` list |
| Persisting an accepted calibration | **TO_DESIGN** — evidence does not settle whether this belongs to `PROVISIONING` or to a separate post-acceptance transaction |

**None of it was migrated.** The current Controller responsibility is **NOT
IMPLEMENTED**, deliberately. `ActuatorAuthority::PROVISIONING` already exists as a
distinct owner, which is what keeps the question answerable later.

---

## 8. Related

- [`DEVELOPMENT_GATES.md`](DEVELOPMENT_GATES.md) — the calibration gate
- [`../../09_Logs/Historical/NormaCore_MATDOG_Archive/README.md`](../../09_Logs/Historical/NormaCore_MATDOG_Archive/README.md)
- [`../../06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml`](../../06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml)

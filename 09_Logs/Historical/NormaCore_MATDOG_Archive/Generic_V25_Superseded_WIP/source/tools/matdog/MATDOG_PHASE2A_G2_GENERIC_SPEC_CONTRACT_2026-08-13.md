# MATDOG — Phase 2A G2 Generic Spec Contract
## Design only — Revision 2.2d, 2026-08-13

## 0. Status, scope, revision record

```text
GATE       P2A-G2
REVISION   2.2d  (revision 2.2 plus architecture-gate, call-graph, CI-closure and M-27s sync)
MODE       DESIGN ONLY
CODE       none written, none modified
BASE       origin/main 4a8ed6337261553b79c928975808d294c9ca723b
BRANCH     matdog/generic-v25-full-leg-engine
```

Revision 2 is a coherent rewrite after independent Codex review #2. It is not an
addendum. Where revisions 0 or 1 disagree with the normative text below, they are
superseded; §0.3 lists what changed and why. Any mention of a superseded concept
outside §0.3 is marked **NON-NORMATIVE / SUPERSEDED**.

Governing inputs:

```text
MATDOG_PHASE2A_G0_G1_EXECUTOR_REPORT_2026-08-13.md      the audit
MATDOG_PHASE2A_G1_GATE_DECISION_2026-08-13.md           rulings D1..D9
MATDOG_PHASE2A_CODEX_G2_REVIEW_2026-08-13.md            independent review #1
MATDOG_PHASE2A_G2_REVIEW_DECISION_2026-08-13.md         rulings R1..R14
MATDOG_PHASE2A_CODEX_G2_REVIEW_2_2026-08-13.md          independent review #2
MATDOG_PHASE2A_G2_REVIEW_2_DECISION_2026-08-13.md       rulings R2-1..R2-18
MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md  entry contract
```

### 0.1 Anti-drift declaration

```text
NO new framework          NO new dependency
NO new serialization      NO new state-machine library
NO new geometry convention NO new motor convention
NO new detector algorithm NO new threshold
NO new q0 convention      NO new EEPROM workflow
NO new test dependency solely to prove Rust privacy      (R2-14.8)
```

Type sketches are specification text, not source files, and use only what the `st3215`
crate already has.

### 0.2 Method

Every LF claim in this revision was re-derived from the immutable source
(`software/drivers/st3215/src/auto_calibrate/matdog.rs`, blob `65c16f3c…`, byte-identical
on `release/matdog-lf-calibrator-v25` `f87dd1fb…`) with line references. Every geometry
claim was re-derived from the canonical **D/W4** artifacts on robot-dog `origin/main`
`bd5aa8ed…`. Revision 1's assumptions were not reused.

### 0.3 What revision 2 changed — HISTORICAL RECORD, NON-NORMATIVE

The table below records what revision 2 did relative to revision 1. It is retained for
audit continuity only. Where it names a construct that revision 2.1 later withdrew — in
particular `MotionGrant` — the construct is **not** part of the normative design; see
§0.4 and §8.

| Finding | Revision 1 status | Revision 2 resolution |
|---|---|---|
| B1 | CLOSED | §2 retained unchanged |
| B2 | PARTIALLY_CLOSED — forgeable, replayable, incomplete vocabulary | §7 full source inventory **first**, then §8 sealed non-`Clone`/non-`Copy` one-shot `MotionGrant` bound to session/epoch/phase/motor/intent |
| B3 | STILL_OPEN — 24 runtime values, shared nominal type | §4 two nominal types; §5 runtime resolver reduced to reviewed LF sentinels only |
| M1 | PARTIALLY_CLOSED — spoofable participants, standalone path outside grammar | §4.3 canonical identity validation; §6 both LF session modes inside one grammar |
| M2 | **REGRESSED** — normalization reduced to 3 motors, tolerance changed | §10 exact all-12 normalization, effective predicate `>10 \|\| speed>4`; `InitialRecovery` corrected to a no-motion verification state |
| M3 | PARTIALLY_CLOSED — one fixed kind, wrong Hip-Max pose, wrong staged/cleanup timing | §11 context-scoped prerequisites; full-session Hip MAX = 1024; §9 exact staged/cleanup timing |
| M4 | PARTIALLY_CLOSED — undefined manifest, wrong artifact for `target_domain`, C/W1 | §3 `TrustedGeometryManifest`, per-field artifact map, D/W4 hashes |
| M5 | PARTIALLY_CLOSED — LF label not validated | §13 oracle reachable only after exact canonical LF identity validation |
| M6 | PARTIALLY_CLOSED — impossible/incomplete rows | §16 rewritten; T-3 removed, M-16/M-18 removed or rewritten |
| M7 | CLOSED | §17 retained; ID 115 C+D split corrected (D-3) |
| N1 | PARTIALLY_CLOSED — 16 operations not derived | §14 all 16 enumerated from source and mapped to states |
| N2 | PARTIALLY_CLOSED — not constructible specs, C/W1 | §15 complete constructible offline raw specs on D/W4 |
| N3 | CLOSED | retained; D-1..D-4 corrected |
| NOTE-A | open | R2-6: predicate documented, capability **not implemented** in G3 |
| NOTE-B | open | R2-18: single-session only; no concurrency claim |

### 0.4 What revision 2.1 changes (targeted, after review #3)

Revision 2.1 is a **targeted** correction. Every revision-2 section listed as
independently validated by review #3 is preserved unchanged: five-axis B1 semantics,
D/W4 deployment identity, PASS deny-only, hardware evidence non-authorizing, full-session
all-12 normalization, the `>10 || speed>4` rule, full-session M12=1024, staged
remove-before-move / re-add-after-arrival, cleanup held-target timing, endpoint-scoped V5
parking, LF final-contact semantics, `DirectGeometryTarget` exclusion, and global C
ownership.

| Review-3 finding | Revision 2 defect | Revision 2.1 resolution |
|---|---|---|
| BLOCKER — four LF modes removed | ARM-2 / M-16r deleted `LF_UPPER_M12_MIN/MAX` and `LF_LOWER_M11_MIN/MAX` | §5 restores all four as runtime modes of the one engine (R3-1) |
| BLOCKER — MotionGrant multiply issuable / stale | first-class capability with undefined lifecycle | §8 **withdraws** it; immediate engine-owned authorization + emission (R3-3) |
| BLOCKER — two-mode grammar cannot express HipPair-20 / Single-16 | only Full and HipPair modelled, HipPair incompletely | §6.2 three modes; §14.3–14.5 exact ordered oracles (R3-2, R3-9) |
| MAJOR — CI not mechanically supported | compile-fail and module-graph claims | §16 rewritten to existing-tool checks only (R3-7, R3-8) |
| MAJOR — armable brand unsealed | `ArmableLfSessionSpec(ArmableInner)` constructible in-module | §4.4 nested sealed module (R3-6) |
| MAJOR — trust root incomplete | `ArtifactRef` undefined, manifest file SHA and reconciliation content SHA unpinned | §3.3 `ExpectedGeometryV5DW4` + `ArtifactRef` + both hashes (R3-12) |
| MAJOR — Full startup applied to all modes | all-12 normalization not scoped | §10 scoped to Full; HipPair/Single keep restart-safe entry (R3-11) |
| MINOR — TABLE A defects | A2 non-write; HOME writes conflated; baseline/scout mode attribution; stop-pressure evidence | §7 corrected, writes only (R3-4, R3-10) |
| MINOR — TABLE B Preflight overstated | "no session exists" throughout Preflight | §9 corrected (E-2) |
| MINOR — progress strings lack label prefix | only inner phase arguments listed | §14 gives full `label: phase` strings (R3-9.3) |
| MINOR — G3 surface too wide | offline importer inside G3 | §20 tightened; deferred to the later gate (R3-13) |
| NM2 — participants vs endpoint deps | §6.4 unioned any parking reference | §6.4 split by scope (R3-14) |

### 0.5 What revision 2.2 changes (surgical, after the final review)

Revision 2.2 is a **surgical** correction. The final review closed A (LF single-contact
runtime surface), B (one engine), E (sealed brand, nominal), I (D/W4 trust root) and
L (contact and global safety); those sections are untouched.

| Final-review finding | Revision 2.1 defect | Revision 2.2 resolution |
|---|---|---|
| BLOCKER — write inventory incomplete | legacy home-only recovery conflated a torque-OFF prime with a torque-ON reassertion in one row | §7 splits them into W2 and W3; inventory is now **23/23** (F1) |
| BLOCKER — authority under-bound | motion methods accepted caller-selected motor/step/scout/pose/evidence | §8.2 methods take identifiers and evidence only; §8.4 derives every authority-relevant input (F3) |
| MAJOR — pre-session writes unauthorized | no context existed before the session object | §8.0 `EngineContext::Entry` with explicit `EntryStep`s (F2) |
| MAJOR — sink scan evadable | only two named sinks were enumerated | §8.3 seals the `RamRegister::GoalPosition` construction boundary; M-27s is a three-stage scan (F4) |
| MAJOR — motion-bearing fields unvalidated | order, prerequisite poses, restore refs were caller-supplied | §4.5 derives all of them internally from the sealed mode (F5) |
| MAJOR — progress semantics conflated | 58/20/16 described as "all externally emitted updates" | §14.0 separates increments 58/20/16 from external streams 60/22/18 with an explicit filter (F6) |
| MAJOR — CI roots undefined | "relevant tree"; natural scopes hit deliberate test literals | §16.0b defines exact Rust and Python roots with an enumerated exclusion (F7) |
| MAJOR — rejection tests too narrow | only admitted/neighbour values | M-28s becomes the full F8 rejection matrix |
| MAJOR — M-19s contradicted G3 scope | required a 24-endpoint Geometry dataset test | M-19s removed from G3, deferred with the importer (F9) |
| MINOR — counts and stale references | "ten"/"eleventh"; `M-27r`; A-row references | corrected throughout (G-1..G-6) |

### 0.6 What revision 2.2a changes (consistency only, after the architecture gate)

The substantive revision-2.2 architecture was **accepted for final review**. Revision 2.2a
changes **no** architecture: W1..W23, the twelve engine operations, the three mode traces,
58/20/16 versus 60/22/18, the LF runtime token surface, the sealed brand, contact
semantics, global safety, the D/W4 hashes and the LF-only G3 scope are all untouched.

| Ruling | Defect | Correction |
|---|---|---|
| AG-1 | 13 normative `D/W5` labels; no such materialization exists | all corrected to `D/W4`; §3.1 states only C/W1 and D/W4 exist. **No hash changed** — the labels drifted when revision 2.2's TABLE A renumbering regex `\bW4\b` matched inside the literal `D/W4` |
| AG-2 | IMP-3b implied `expected_role` is compared to artifact content | IMP-3b now checks `expected_schema` against the artifact and treats `expected_role` purely as the trust-root slot selector |
| AG-3 | §11.5 said `prerequisite_or_parking_move` accepts a V5 record-bound pose, contradicting BRAND-3 and §8.4 | §11.5 now separates G3 runtime (LF-historical only, derived, no pose argument) from future offline records (inert, no engine consumer) |
| AG-4 | the Rust safety root covered all of `auto_calibrate/`, which holds generic calibrators that legitimately use EEPROM (`calibrator.rs:424` writes `EepromRegister::Offset`) | §16.0b defines SCOPE A (the `matdog` namespace, with a namespace-escape guard) and SCOPE B (bridge/port); `calibrator.rs`, `elrobot.rs`, `so101.rs` are explicitly out of the MATDOG token scan |
| AG-5 | "staleness is impossible by construction" over-claimed | now: no authorize-now/emit-later window exists, **and** observation/evidence freshness remains an explicit runtime check per DER-1 and §8.6 |
| clarity | §6.3 could read as G3 engine input | §6.3 marked as future generic/offline design; G3 derives LF choreography internally (§4.5) |

### 0.7 What revision 2.2b changes (call-graph consistency only)

AG-1..AG-5 are accepted and unchanged. Revision 2.2b resolves the last normative
contradiction and changes **no** architecture: W1..W23, the twelve engine operations, the
LF traces, constants, runtime modes, contact semantics, provenance and G3 scope are all
untouched, and no motion behavior changes.

| Ruling | Defect | Correction |
|---|---|---|
| CG-1..CG-8 | §8.3 said the two policy writers call the single raw constructor, while M-27s said the twelve operations call it. Both could not hold | §8.3 freezes the three-layer graph — twelve operations → two policy writers → one raw constructor → `RamRegister::GoalPosition` — with explicit terminology, and records in CG-7 the verified source reason the two writers must stay distinct (they gate on different motor sets: `MATDOG_MOTOR_IDS` at :3837 versus `profile.allowed_motor_ids` at :4443) |
| M-27s | asserted a two-layer caller set | rewritten as a seven-stage scan (a)–(g) matching the frozen graph; it must not claim the twelve operations directly call the raw constructor |
| §16.0b | the namespace-escape inventory listed only top-level `.rs` | inventory is now explicitly **recursive** under `auto_calibrate/`, so a nested non-matdog path cannot escape |
| §22 | header said 2.2a while the executor position still said "REVISION 2.2 COMPLETE" | version wording aligned to 2.2b |

### 0.8 What revision 2.2c changes (CI closure only)

The final independent closure check returned PASS on every axis except CI feasibility:
0 blockers, 2 CI majors, 1 documentation minor. Revision 2.2c resolves exactly those and
reopens nothing. Unchanged: 23 write paths, 12 engine operations, 2 policy writers,
1 raw constructor, the LF modes, traces, constants, contact semantics, provenance,
global safety and G3 scope.

| Ruling | Defect | Correction |
|---|---|---|
| CI-C1 | M-27s did not define its Scope-B production locations or its `#[cfg(test)]` exclusion, so "production count == one" was unevaluable. Source shows all four `port.rs` GoalPosition occurrences sit inside its `#[cfg(test)]` block at :1838, and 3 of 5 `matdog.rs` production occurrences are allowlist/match references rather than constructions | CG-1 now counts **write-construction sites** only and excludes `#[cfg(test)]`/`matdog_test.rs`; §16.0b names the exact SCOPE-B anchors and states they are not part of the count; M-27s becomes stages 1–6 over SCOPE A production plus a **separate** stage 7 for the bridge/port anchors |
| CI-C2 | the broad `tools/matdog` dynamic-import scan was conflated with K-11's observer policy; K-11's forbidden tokens legitimately appear in the runner and tests | §16.0b defines PYTHON SCOPE 1 (all eight files, dynamic-import mechanisms only) and PYTHON SCOPE 2 (K-11 forbidden tokens, `matdog_native_observer_contract.py` **only**, matching the real workflow); M-26s and K-11 now name their scopes |
| CI-C3 | live lifecycle prose still used revision-2.1 A-row numbers, and §10.1 described legacy recovery with only W2 | A-rows mapped from §7/source to W-rows and engine-operation names (A3→W4, A4→W5, A5→W6, A12→W15, A13→W16, A14→W17, A15→W18, A16→W19, A17→W20); §10.1 now shows W2 → torque enable → W3 |

### 0.9 What revision 2.2d changes (live M-27s row synchronization only)

CI-C2 and CI-C3 are accepted and unchanged. CI-C1's ruling was already correct in §0.8,
§8.3 CG-1, §16.0b and the CI-closure record — but the **live normative M-27s row in
§16.3** had not actually been synchronized and still carried the superseded formulation
("scan SCOPE A plus SCOPE B together", "enumerate every `RamRegister::GoalPosition`
occurrence", "production count == one"). That is precisely the ambiguity that was
blocked.

Cause: the revision-2.2c edit matched the **first** `| M-27s |` row in the file, which is
a cell in the §0.7 revision-record table, so the corrected text landed there and the live
§16.3 row was left untouched.

| Item | Correction |
|---|---|
| live §16.3 M-27s row | replaced with wording mechanically identical in meaning to CI-C1 and §16.0b: stages 1-6 over SCOPE A production only, write-construction sites only, `#[cfg(test)]` structurally excluded, one raw constructor, two policy writers, twelve engine callers, W1..W3 / W4..W23 coverage, and stage 7 for SCOPE B kept separate |
| §0.7 record row | restored to its original short description |
| §22 summary | "3-stage scan" corrected to "7-stage scan" |

No architecture, no other technical content, no other row changed.

**NON-NORMATIVE / SUPERSEDED (do not implement):** revision 0's `ExecutableTick`;
revision 0's `HardwareValidation`-based admission; revision 1's `AuthorizedGoal`
(`Clone + Copy`, six purposes); revision 1's single `ValidatedLegSpec` shared by the
offline API and the engine; revision 1's `InitialRecovery`-performs-recovery model;
revision 1's full-session Hip-MAX `UPPER_85_DELTA`; revision 1's C/W1 provenance pins;
revision 1's CI rows T-3, M-16, M-18; **revision 2's first-class `MotionGrant`,
`Issuer`, `Sink::emit` and their AUTH-1..AUTH-8 lifecycle** (withdrawn by R3-3 — no
motion-authority token exists in this design); **revision 2's ARM-2 removal of the four
LF single-contact runtime modes**; revision 2's CI rows M-17r, M-25r, M-26r, M-27 as
worded; **revision 2.1's 22-row write inventory and its conflated legacy HOME row**;
**revision 2.1's caller-argument motion API** (`probe_advance_step(step, scout)`,
`return_home(motor)`, `staged_affine_q0(ev)` and siblings taking free choices);
**revision 2.1's CI row M-19s** (removed from G3 by F9).

---

## 1. The corrected chain

```text
trusted canonical D/W4 bundle + policy + reconciliation                §3
        ▼   compared against the COMPILED EXPECTED TRUST ROOT (§3.3)
        ▼   per-field verification against the artifact that actually holds the field
RawLegCalibrationSpec (inert, leg-agnostic)                            §4
        ▼   validate_offline()                    ▼  validate_lf_v25()
OfflineLegCalibrationSpec                         ArmableLfSessionSpec  §4
  (RF/RH/LH and LF; NEVER accepted by the           (LF only, SEALED;
   engine; NOT implemented in G3 — R3-13)            exact identity — §4.3/§4.4)
                                                  ▼
                              one engine, fixed grammar, THREE LF modes    §6
                                                  ▼
                              intent-specific engine motion operations     §8
                              (authorize + emit in the same call)
                                                  ▼
                              private raw GoalPosition sink                §8.3
                                                  ▼
                              unsigned ST3215 GoalPosition 0..=4095        §17
```

Five structural negatives:

```text
N-1  An OfflineLegCalibrationSpec has no engine consumer and no motion operation.
N-2  There is no motion-authority token at all: no value exists that represents
     "permission to move later", so none can be duplicated, retained or replayed.
N-3  An UnsignedTick carries no authority.
N-4  A diagnostic endpoint has no direct-goal path — and none is implemented at all.
N-5  An external PASS creates no eligibility.
```

Terminal safety is deliberately outside this chain: verified global torque-off, hard
abort and operator stop never depend on normal per-goal authorization (§8.4, R3-5.4).

---

## 2. Five orthogonal authority axes (CLOSED — retained)

```rust
enum V5TargetDomain { ExecutableUrdfDomain, DiagnosticGeometryOutsideUrdfLimits }
enum ExternalPolicyVerdict { Pass, Fail, Unresolved }
enum HardwareContactEvidence { SupervisedPassRecorded, None }   // evidence only (§13)
```

```text
eligible_for_direct_geometry_target(endpoint)
    = domain == ExecutableUrdfDomain  AND  policy_verdict == Pass

Hardware evidence appears nowhere in it.  PASS only retains; FAIL/UNRESOLVED deny.
```

Per **R2-6** this predicate is **documented design only**. No `DirectGeometryTarget`
intent, minting function or command path exists in the Phase 2A vocabulary (§8), and
G3 must not implement one. Denial tests remain (§16).

Why hardware evidence must never upgrade a domain — canonical LF reconciliation:

```text
lf_hip_joint:min        geometry -46.012   hardware -42.803   -3.209   DISAGREES
lf_hip_joint:max        geometry +45.223   hardware +39.375   +5.848   DISAGREES
lf_upper_leg_joint:min  geometry -52.133   hardware -53.525   +1.393   AGREES
lf_upper_leg_joint:max  geometry +121.875  hardware +122.607  -0.732   AGREES
lf_lower_leg_joint:min  geometry -92.074   hardware -91.846   -0.229   AGREES
lf_lower_leg_joint:max  geometry +38.180   hardware +34.277   +3.902   DISAGREES
```

The three `DISAGREES` rows are diagnostic endpoints. Hardware evidence is the reason
they stay diagnostic.

### 2.1 Canonical domain/policy matrix (24 endpoints)

```text
EXECUTABLE_URDF_DOMAIN   8   — the four *_upper_leg_joint min+max, one pair per leg
DIAGNOSTIC…OUTSIDE…     16   — every hip and every lower endpoint, all four legs
PASS 16   UNRESOLVED 8   FAIL 0   motion authorizations 0
direct-eligible 8, of which implemented in Phase 2A: 0
```

| Leg | hip:min | hip:max | upper:min | upper:max | lower:min | lower:max |
|---|---|---|---|---|---|---|
| LF | Diag / UNRES | Diag / PASS | **Exec** / PASS | **Exec** / PASS | Diag / UNRES | Diag / PASS |
| RF | Diag / PASS | Diag / UNRES | **Exec** / PASS | **Exec** / PASS | Diag / UNRES | Diag / PASS |
| RH | Diag / PASS | Diag / UNRES | **Exec** / PASS | **Exec** / PASS | Diag / UNRES | Diag / PASS |
| LH | Diag / UNRES | Diag / PASS | **Exec** / PASS | **Exec** / PASS | Diag / UNRES | Diag / PASS |

---

## 3. Canonical provenance — D/W4, with per-field artifact binding

Per **R2-10** and **R2-11**.

### 3.1 Canonical materialization

```text
CANONICAL DEPLOYMENT SOURCE   2026-08-11_131818_..._REMEDIATION_BENCHMARK_D_W4_*
DETERMINISM ORACLE ONLY       2026-08-11_131818_..._REMEDIATION_BENCHMARK_C_W1_*

These are the ONLY two materializations that exist. There is no W2, W3, W5 or any
other W-index bundle on robot-dog origin/main. Any document or reader inferring a
"D/W5" bundle is mistaken: that label appeared transiently in revision 2.2 as a
textual defect and never corresponded to an artifact.                       (AG-1)
```

The final external safety policy consumes the **D/W4** parking artifact
(`input_parking_artifact.file_sha256 = e561e7fb…`), which fixes the deployment chain.

```text
Semantic hashes are IDENTICAL across C/W1 and D/W4:
    endpoint  de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e
    parking   67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139
    combined  0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51

THEREFORE semantic equality MUST NOT be treated as provenance identity.
File identity is what pins the materialization.
```

Canonical D/W4 file identities:

```text
run manifest FILE    sha256   0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17
run manifest CONTENT sha256   4e7172d473b5d19c79112252e50212565ef73505551bff993b6a0c927f2aaee7
endpoint_profile              dd8cb42c3b916d067f97a321c5ffcdfb013dde1f2f2a6ba71f73becff360dc0f
parking_json                  e561e7fb98880843e590f4676e8559374c51d0e641102f89a803b3619722c4d7
combined_profile              448ebcb3ed56d7f5225e6f9906e2efeb622010d8ecc457fe9ad00bda6407b00c
endpoint_report               3d4db1daf7c04cd28dfb80bbcc6c783637390139efc1b712d7b184c97358bc6c
parking_report                142650d32ebcf55bd39fdb9a045a8265e49a26af7252dc0709bd515a20f2dadf
oracle_json                   b8cc71e386e8b8a32aad8f5ec292aa1652372fb28561355de4a624a5c14785f6
oracle_report                 3d8bc62f44bc407f6928c7e7b734959189b9850097d188149cfe8ce6d6240401

final external safety policy JSON   82f00a9414df06676babed1101a73e1a9d7bfa126592cdd7a425f66d5f01f1cc
final LF hardware reconciliation JSON (FILE)
                                   d7fa04e2b8cde6b1049d4c34c5ba15fa1febde8b2fbf9fd8f14853034b98cc9a
final LF hardware reconciliation CONTENT (field `reconciliation_content_sha256`)
                                   0af31e9dbcae0aae22978faeaf12a668c8d402abf921d2d63cea49f11195e70c

URDF   03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf
       sha256 3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59
compiler source_combined_sha256
       e4eea175e90131b1d8e55ae381fe8e88d6f966c0f87d4eee5c064d0a6967737f
collision meshes  17 links, each with its own sha256
policy source sha256   8b124ef45934cb318af6ce4bcb873ce078b505dfa74ad017e6394b4f51b674f6
policy semantic sha256 e5cb2a4c33082c59c6f5d381f90fcc19680cc899e326d13c9c9129932bde5d08
policy id / schema     MATDOG_REFERENCE_MINIMUM_CLEARANCE / matdog.geometry_safety_policy.v1
threshold_m            0.003
reconciler source sha256  111da4045c983c4f3a5acd0061dbbe3c7b77fbda7e120693a927f5f0a40f5dfe
LF physical hardware evidence sha256
                          6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4
frozen G4 legacy replay content sha256 (NON-CANONICAL, reject on sight)
                          4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61
```

Per **R2-11.9** two distinct identities are kept apart:

```text
PHYSICAL LF hardware evidence artifact    6eae3201…   (the measurement record)
GENERATED LF reconciliation artifact      d7fa04e2…   (geometry-vs-hardware comparison)
```

### 3.2 Which artifact actually holds which field (R2-11.6, R2-11.7)

This is the table revision 1 got wrong.

| Field | Artifact that actually contains it | JSON location |
|---|---|---|
| `presentation_id`, `joint_name`, `limit_side` | endpoint_profile `dd8cb42c…` | `endpoint_searches[].identity` |
| `declared_limit_rad` | endpoint_profile | `endpoint_searches[].declared_limit_rad` |
| geometric contact angle, bracket, link pair, search params | endpoint_profile | `endpoint_searches[].geometric_contact` |
| `minus_declared_limit_rad` | endpoint_profile | `endpoint_searches[].geometric_contact` |
| contact `status` | endpoint_profile | `endpoint_searches[].geometric_contact.status` |
| path obstruction status / relation | endpoint_profile | `endpoint_searches[].path_obstruction` |
| `structural_depth`, `articulated_branch_id`, `active_revolute_pair` | endpoint_profile | `endpoint_searches[]` |
| URDF sha, collision-mesh shas, compiler shas | endpoint_profile | `provenance` |
| endpoint semantic sha | endpoint_profile | `semantic_content_sha256` |
| **`target_domain`** | **parking_json `e561e7fb…`** | **`plans[].target_domain`** |
| `target_within_urdf_limits`, `target_source`, `target_angle_rad` | parking_json | `plans[]` |
| `canonical_endpoint_index`, `endpoint_id` | parking_json | `plans[]` |
| `outcome`, `parking_degrees_of_freedom`, `parking_configuration_rad` | parking_json | `plans[]` |
| `evaluated_1d_candidates`, `evaluated_2d_candidates` | parking_json | `plans[]` |
| summary counts (8 executable / 16 diagnostic / 18 NOT_NEEDED / 6 plans / 94 / 0) | parking_json | `summary` |
| parking semantic sha | parking_json | `semantic_content_sha256` |
| policy verdict, clearance, clearance kind, motion authorization | policy JSON `82f00a94…` | `endpoint_policy_results[]` |
| policy id, schema, threshold, source sha, semantic sha | policy JSON | `policy`, `provenance`, `semantic_content_sha256` |
| geometry-vs-hardware deg values, delta, result | reconciliation JSON `d7fa04e2…` | per joint/side record (6 records) |
| agreement threshold, reconciler source sha, physical evidence sha | reconciliation JSON | header |

### 3.3 `ExpectedGeometryV5DW4` — the compiled expected trust root (R3-12)

A bundle must not be able to authenticate itself. Internal self-consistency proves only
that a bundle is *internally* coherent; a fabricated bundle is internally coherent too.
Authority therefore comes from an **immutable expected trust root compiled into the
verifier**, against which the read bytes are compared.

```rust
/// One expected artifact identity. Every field is an EXPECTED value, compiled in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ArtifactRef {
    /// Repository-relative artifact identity/path, exactly as materialized.
    relative_path: &'static str,
    /// Expected SHA-256 of the artifact FILE BYTES.
    expected_file_sha256: &'static str,
    /// Expected schema-level semantic content hash, where the artifact declares one.
    expected_semantic_sha256: Option<&'static str>,
    /// Expected schema identity, where the artifact DECLARES one
    /// (e.g. "matdog.calibration_geometry_profile.v5"). None when the artifact has no
    /// schema field; the check is then skipped rather than defaulted.
    expected_schema: Option<&'static str>,
    /// The reviewer-assigned SLOT this artifact occupies in the expected trust root
    /// (e.g. "endpoint_profile", "parking_json", "policy_json"). It is NOT read from
    /// the artifact and is NOT compared against artifact content; it names which
    /// expected entry the read bytes must match. Artifacts carrying no top-level role
    /// field are therefore fully supported.                                    (G-6)
    expected_role: &'static str,
}

/// The immutable expected trust root for the canonical D/W4 deployment bundle.
/// Compiled into the verifier. NOT read from any artifact.
struct ExpectedGeometryV5DW4 {
    benchmark_id: &'static str,                    // "D"
    run_manifest: ArtifactRef,                     // FILE 0db86e63...
    run_manifest_expected_content_sha256: &'static str, // CONTENT 4e7172d4...
    run_manifest_schema: &'static str,             // matdog.geometry_compiler_v5.integrated_run.v2

    endpoint_profile: ArtifactRef,                 // FILE dd8cb42c..., SEMANTIC de205209...
    parking_json:     ArtifactRef,                 // FILE e561e7fb..., SEMANTIC 67c58430...
    combined_profile: ArtifactRef,                 // FILE 448ebcb3..., SEMANTIC 0a772234...
    endpoint_report:  ArtifactRef,                 // FILE 3d4db1da...
    parking_report:   ArtifactRef,                 // FILE 142650d3...
    oracle_json:      ArtifactRef,                 // FILE b8cc71e3...
    oracle_report:    ArtifactRef,                 // FILE 3d8bc62f...

    policy_json: ArtifactRef,                      // FILE 82f00a94..., SEMANTIC e5cb2a4c...
    policy_expected_source_sha256: &'static str,   // 8b124ef4...
    policy_expected_id: &'static str,              // MATDOG_REFERENCE_MINIMUM_CLEARANCE
    policy_expected_threshold_m: f64,              // 0.003

    /// GENERATED comparison artifact.
    reconciliation_json: ArtifactRef,              // FILE d7fa04e2...
    reconciliation_expected_content_sha256: &'static str, // CONTENT 0af31e9d...
    reconciler_expected_source_sha256: &'static str,      // 111da404...
    /// PHYSICAL measurement artifact — a DIFFERENT identity (R2-11.9).
    physical_evidence_expected_sha256: &'static str,      // 6eae3201...

    urdf: ArtifactRef,                             // FILE 3890a3f0...
    compiler_expected_source_combined_sha256: &'static str, // e4eea175...
    /// Exactly 17 entries, one per collision-mesh link, each with its own expected sha.
    collision_meshes: &'static [ArtifactRef],

    /// Identities that must be REFUSED wherever a deployment identity is expected.
    refused_identities: &'static [&'static str],   // C/W1 file shas; G4 replay 4a2a2324...
}
```

**Where verification happens (R3-12.6, R3-13):**

```text
G3           NO Geometry V5 parser, importer or trust-root verifier is implemented.
             G3 is LF generic-engine extraction and LF behavioral preservation only.
             The LF spec is compiled-in and validated by §4.3/§4.4 canonical-identity
             checks, which read no geometry artifact.
             ExpectedGeometryV5DW4, ArtifactRef, RawLegCalibrationSpec's offline
             validation and OfflineLegCalibrationSpec are DESIGN ONLY at this gate.

LATER        A separate offline importer/validator READS the canonical D/W4 artifacts and
non-LF       compares them against the COMPILED ExpectedGeometryV5DW4 before any field is
offline      trusted, then compares every field to the artifact that actually holds it
gate         (§3.2), and only then constructs an OfflineLegCalibrationSpec. That importer
             never links against the engine.
```

### 3.4 Import invariants (fail closed)

```text
IMP-0   EVERY comparison below is against the COMPILED ExpectedGeometryV5DW4 (§3.3).
        No value read from the bundle is trusted to validate another value from the
        same bundle. Self-consistency alone NEVER authenticates (R3-12.4).
IMP-1   run manifest FILE sha == expected 0db86e63…; run manifest CONTENT sha ==
        expected 4e7172d4…; benchmark_id == "D"; manifest schema matches
IMP-2   every ArtifactRef.expected_file_sha256 matches the bytes actually read
IMP-3   every expected_semantic_sha256 matches the artifact's declared semantic hash
IMP-3b  every expected_schema, WHEN Some, matches the artifact-declared schema.
        expected_role SELECTS/BINDS the expected ArtifactRef slot in
        ExpectedGeometryV5DW4; it is a reviewer-assigned slot name and is NEVER
        asserted to equal artifact content. An artifact with no top-level role
        field is fully supported; an artifact with no schema field skips the
        schema check rather than defaulting it.                          (AG-2)
IMP-4   any identity in `refused_identities` appearing in a deployment position is
        REJECTED — this covers C/W1 file identities (R2-10.3) and the G4 replay content
IMP-5   identity fields are mutually consistent: presentation_id <-> leg/joint/side,
        and endpoint_profile.identity <-> parking_json.endpoint_id <-> policy endpoint id
IMP-6   canonical_endpoint_index is 0..=23, unique, and consistent across parking_json
        and policy JSON
IMP-7   declared_limit_rad compared to the endpoint_profile record
IMP-8   geometric contact angle compared to the endpoint_profile record
IMP-9   minus_declared_limit_rad equals contact - declared within schema tolerance
IMP-10  contact status and path-obstruction status compared to the endpoint_profile
IMP-11  structural_depth compared to the endpoint_profile
IMP-12  target_domain compared to parking_json.plans[].target_domain    (NOT endpoint)
IMP-13  target_within_urdf_limits compared to parking_json, and consistent with
        target_domain (true iff ExecutableUrdfDomain)
IMP-14  target_source and target_angle_rad compared to parking_json
IMP-15  outcome, parking_degrees_of_freedom and the full parking_configuration_rad map
        compared to parking_json, element by element
IMP-16  evaluated_1d_candidates and evaluated_2d_candidates compared to parking_json
IMP-17  policy verdict, clearance_m, clearance kind and motion-authorization string
        compared to the policy JSON record
IMP-18  policy id, schema, threshold_m, source sha and semantic sha compared to the
        policy JSON header
IMP-19  reconciliation geometry_deg, hardware_deg, delta_deg and result compared to the
        reconciliation JSON, for each of its six joint/side records
IMP-20  physical LF evidence sha (6eae3201…) and the GENERATED reconciliation identity
        (FILE d7fa04e2… / CONTENT 0af31e9d…) and the reconciler source sha (111da404…)
        are three SEPARATE expected identities, each bound independently (R2-11.9)
IMP-20b compiler source_combined sha, URDF file sha and all 17 collision-mesh shas are
        each cross-bound against the expected trust root, not merely against the
        bundle's own provenance block (R3-12.5)
IMP-20c policy id, schema, threshold_m and source sha are bound against the expected
        trust root, not only against the policy artifact's own header
IMP-21  aggregate cross-checks: 8 executable / 16 diagnostic / 16 PASS / 8 UNRESOLVED /
        0 FAIL / 0 motion authorizations / 24 endpoints / 18 NOT_NEEDED /
        6 FEASIBLE_1DOF_PLAN_FOUND / 94 1DOF candidates / 0 2DOF candidates
IMP-22  the frozen G4 replay content sha (4a2a2324…) appearing in any deployment
        position is REJECTED
IMP-23  any mismatch is Err. No warning path, no default, no fallback.
```

```text
IMP-7..IMP-19 are what make M4 closable: a correct hash beside an invented value fails,
because each value is compared to the record in the artifact that actually holds it.
```

---

## 4. Two nominal spec types

Per **R2-1**.

```rust
/// Inert, leg-agnostic, unvalidated. Constructible anywhere. No engine consumer,
/// no motion operation, no oracle accessor.
/// Complete topology: leg identity; three JointSpec entries (joint name, motor id,
/// direction); six endpoint entries (joint, side, limit_delta, record reference);
/// calibration order (joint order + per-joint side order); six prerequisite reference
/// sets; an optional parking reference; restore references; provenance references.
struct RawLegCalibrationSpec { /* fields as enumerated above */ }

/// OFFLINE ONLY. Produced for RF/RH/LH and for LF analysis. Nominally distinct.
/// The engine has NO function that accepts this type.
/// DESIGN ONLY at this gate — NOT implemented in G3 (R3-13).
struct OfflineLegCalibrationSpec(OfflineInner);
```

```text
R2-1 / R3-6 enforcement
  4.1  fn validate_offline(raw, &ExpectedGeometryV5DW4)
           -> Result<OfflineLegCalibrationSpec, _>
       available for all four legs. DESIGN ONLY; deferred out of G3 (R3-13).
  4.2  The sealed module of §4.4 is the ONLY producer of the armable type.
  4.3  There is NO public or general conversion
           OfflineLegCalibrationSpec -> ArmableLfSessionSpec
       and no From/Into/TryFrom between them.
  4.4  The engine signature is  fn run(spec: &ArmableLfSessionSpec, ..)
```

### 4.3 Exact LF canonical identity validation (R2-1.5, R2-12.2)

`validate_lf_v25` rejects unless **every** field matches the immutable canonical LF
identity. An `LF` label is never sufficient.

```text
LFID-1  leg == Leg::Lf
LFID-2  joint names exactly  lf_hip_joint / lf_upper_leg_joint / lf_lower_leg_joint
LFID-3  motor IDs exactly    Hip 13, Upper 12, Lower 11
LFID-4  directions exactly   Hip -1, Upper +1, Lower -1
LFID-5  limits exactly       Hip -512/+512, Upper -597/+1394, Lower -1047/+427
LFID-6  historical oracle identity matches the LF V25 oracle (§13), including the
        physical evidence sha 6eae3201…
LFID-7  the requested session mode is one of the reviewed LF modes (§6.2)
LFID-8  the parking dependency, when present, is exactly M42 = LH upper, and the motor
        named by the parking reference is the canonical motor of that named joint
LFID-9  participants derived under §6.4 equal exactly {11,12,13} ∪ {42}
```

A hybrid raw object labelled LF but carrying RF/RH/LH motors, directions, names or
limits fails at LFID-2..LFID-5 — **before** any oracle or probe authority exists.

### 4.4 The sealed LF armable brand (R3-6)

Revision 2 declared `ArmableLfSessionSpec(ArmableInner)` in the same module as the
engine, so same-module code could construct it directly and bypass `validate_lf_v25`.
Prose is not a Rust property. The brand is therefore sealed the same way the raw
GoalPosition sink is sealed (§8.3).

```rust
mod lf_brand {                     // private nested module: the sealed boundary
    /// LF ONLY. The single type the engine accepts.
    /// Inner representation is private TO THIS MODULE, so no code outside it —
    /// including the rest of the calibrator module — can construct one.
    pub(super) struct ArmableLfSessionSpec {
        inner: ArmableInner,       // private field, private type
    }

    /// The ONLY safe producer. Performs LFID-1..LFID-9 of §4.3.
    pub(super) fn validate_lf_v25(
        raw: RawLegCalibrationSpec,
        mode: LfSessionMode,
    ) -> Result<ArmableLfSessionSpec, LfIdentityError>;

    impl ArmableLfSessionSpec {
        /// Read-only accessors only. No setter, no `inner()` escape hatch,
        /// no Clone, no Default, no Deserialize.
        pub(super) fn mode(&self) -> LfSessionMode;
        pub(super) fn joints(&self) -> &[JointSpec; 3];
        pub(super) fn participants(&self) -> &[u8];
        pub(super) fn oracle(&self) -> &'static LfV25Oracle;   // §13
    }
}
```

```text
SEAL-1  Safe code outside `mod lf_brand` cannot name ArmableInner or construct
        ArmableLfSessionSpec.
SEAL-2  validate_lf_v25 is the ONLY safe producer (R3-6.3).
SEAL-3  The brand is REQUIRED to create a session and to reach any intent-specific
        motion operation (§8) or the LF oracle (§13).
SEAL-4  An LF string/enum label is never authority (R3-6.4).
SEAL-5  No conversion exists from any offline/non-LF object to this brand (R3-6.5).
```

### 4.5 Every motion-bearing LF field is derived, not accepted (F5)

LFID-1..LFID-9 cover identity. They do **not** cover the remaining fields that influence
motion: mode-specific joint order, side order, prerequisite identities and their exact
historical poses, parking identity and lifetime, restore identities and order, the
historical pose source, and the oracle binding. If those stayed caller-supplied, a
modified `RawLegCalibrationSpec` could alter LF runtime choreography *after* receiving
the brand.

**Preferred design, adopted for G3 (F5.3):** the immutable LF choreography is **derived
internally from the sealed LF mode** rather than accepted as raw fields.

```text
DERIVED FROM LfSessionMode INSIDE mod lf_brand — never read from the raw spec
    joint order and per-joint side order              (§6.2 table)
    prerequisite identities and contexts              (§11.1, §11.2)
    prerequisite exact historical poses               341 / 967 / 1024 / -990 (§11.5)
    parking identity and lifetime                     M42, whole-session for Full only
    restore identities and order                      engine-owned LIFO (§11.4)
    historical pose source                            PoseReference::LfHistoricalV25
    LF oracle binding                                 lf_v25_oracle() (§13)

VALIDATED FROM THE RAW SPEC — LFID-1..LFID-9
    leg, joint names, motor IDs, directions, limits, parking identity, oracle identity,
    requested session mode
```

```text
BRAND-1  A raw field that would influence motion is either compared against the canonical
         LF value or NOT READ AT ALL. There is no third option (F5.2).
BRAND-2  Mutating calibration order, side order, a prerequisite pose, a restore
         reference or a parking lifetime in the raw spec CANNOT change runtime motion,
         because the engine reads none of them (F5.4).
BRAND-3  Runtime engine operations reject a V5 record-bound pose: LF runtime poses must
         carry PoseReference::LfHistoricalV25. V5 record-bound poses are offline-only.
BRAND-4  If a future gate needs a genuinely per-leg spec-driven choreography, it must
         re-open this decision explicitly; it is not implied by Phase 2A.
```

---

## 5. Runtime arm resolution vs offline enumeration

Per **R2-2** as corrected by **R3-1**. Revision 2's ARM-2 removed four supported
immutable LF hardware modes; that removal is withdrawn. **No behavior deprecation is
authorized.**

### 5.1 What immutable V25 actually supports

```text
build_profile :441            arm_value = "{LEG}_{JOINT}_M{motor}_{SIDE}"
profile_for_arm_value :511    recognizes all 24 single-contact tokens
hardware_profile_allowed :536 blocks ONLY isolated HIP

token                recognized   hardware admitted   immutable dispatch
LF_UPPER_M12_MIN     yes          YES                 run_profile
LF_UPPER_M12_MAX     yes          YES                 run_profile
LF_LOWER_M11_MIN     yes          YES                 run_profile
LF_LOWER_M11_MAX     yes          YES                 run_profile
LF_HIP_M13_MIN       yes          NO (blocked)        none
LF_HIP_M13_MAX       yes          NO (blocked)        none
RF/RH/LH x18         yes          Upper/Lower would be admitted today

witnesses  matdog_test.rs :279-286  isolated Hip blocked, LF_LOWER_M11_MIN allowed
           matdog_test.rs :2493-2498 LF_UPPER_M12_MIN recognized
```

### 5.2 Phase 2A runtime arm resolution (exhaustive)

```text
"LF_LEG_STATE_MACHINE"  -> LfFullLegSession
"LF_HIP_M13_MIN_MAX"    -> LfHipPairLegacy
"LF_UPPER_M12_MIN"      -> LfSingleContactLegacy { joint: Upper, side: Min }
"LF_UPPER_M12_MAX"      -> LfSingleContactLegacy { joint: Upper, side: Max }
"LF_LOWER_M11_MIN"      -> LfSingleContactLegacy { joint: Lower, side: Min }
"LF_LOWER_M11_MAX"      -> LfSingleContactLegacy { joint: Lower, side: Max }
NOTHING ELSE.
```

```text
ARM-1   No RF/RH/LH arm value is resolvable at runtime in Phase 2A (R3-1.5).
        The eighteen non-LF single-contact tokens are removed from RUNTIME resolution
        while remaining recognizable OFFLINE profile data.
ARM-2r  The four LF Upper/Lower single-contact tokens ARE runtime-supported, through
        the ONE engine as LfSingleContactLegacy (R3-1.3). This restores revision 2's
        unauthorized removal.
ARM-3r  LF_HIP_M13_MIN and LF_HIP_M13_MAX remain RECOGNIZED historical profile data and
        remain hardware BLOCKED exactly as immutable V25 requires (R3-1.4). Isolated HIP
        hardware is reachable only through LfHipPairLegacy or LfFullLegSession.
ARM-4   run_profile does NOT survive as a second physical executor. Its behavior becomes
        the LfSingleContactLegacy expansion of the one engine (R3-2.2, §6.2).
ARM-5   OFFLINE enumeration of all 24 profiles remains available for geometry/spec
        validation and tests, with no motion capability (R2-2.1).
```

Net runtime surface change versus today: the eighteen RF/RH/LH tokens are removed; all
six LF tokens keep exactly their current recognition and admission behavior.

---

## 6. One engine, explicit reviewed LF session modes

Per **R2-3**.

```text
FORBIDDEN permanently:
    LfSessionStateMachine / RfSessionStateMachine / RhSessionStateMachine / LhSessionStateMachine
    per-leg states, per-leg validators, per-leg corridors
    a generic legacy run_profile motion path beside the engine
```

### 6.1 Fixed lifecycle grammar (engine-owned, not spec data)

```text
Session := Entry(mode)
           [ InitialRecovery ]               Full mode only
           [ Parking ]                       Full mode only (whole-session M42 hold)
           [ Prerequisites ]                 HipPair / Single legacy restart-safe entry
           JointBlock+                       one per joint, in the mode's joint order
           [ Diagnostics ]                   Full mode only
           [ ReturnStaged+ ]                 Full mode only
           [ RestoreParking ]                iff Parking was applied
           [ RestorePrerequisites ]          iff Prerequisites were established
           Cleanup
           TorqueOff

Entry(Full)              exact-set verify -> verified global torque OFF
                         -> all-12 q0 normalization -> session creation      (§10)
Entry(HipPair | Single)  exact-set verify -> verified global torque OFF
                         -> restart-safe profile-entry inspection
                         -> home-only joint recovery                          (§10.1)

JointBlock := ContactSearch(first side)
              [ ContactSearch(second side) ]  absent for Single (one selected side)
              [ StaticHold ]                  iff a LATER joint block's prerequisites
                                              reference this joint at a non-home pose
```

```text
GRM-1   Entry is first and is mode-specific; it is never shared across modes (R3-11).
GRM-2   Diagnostics, where present, follows every JointBlock and precedes ReturnStaged.
GRM-3   ReturnStaged only after Diagnostics.
GRM-4   Cleanup -> TorqueOff is the terminal suffix; nothing follows TorqueOff.
GRM-5   Cleanup is reachable from EVERY phase (universal fail-closed escape).
GRM-6   Exactly one motor may be actively commanded at any instant.
GRM-7   Held sets are DERIVED from completed operations, never declared.
GRM-8   Roles are DERIVED (§11), never assigned by data.
GRM-9   RestoreParking exists iff Parking existed; RestorePrerequisites exists iff
        Prerequisites were established.
GRM-10  StaticHold is emitted only by the derivation rule above.
GRM-11  ALL THREE LF modes (§6.2) are expansions of THIS grammar. There is no second
        executor and no run_profile path (R3-2.2).
GRM-12  A mode's observable and physical choreography is preserved exactly. Shared
        implementation primitives must not normalize the three traces (R3-2.4, R3-11.4).
```

### 6.2 The three reviewed LF session modes (R3-1.2, R3-2)

```rust
enum UpperOrLower { Upper, Lower }

enum LfSessionMode {
    /// Immutable 58-operation full-leg calibration. Arm "LF_LEG_STATE_MACHINE".
    LfFullLegSession,
    /// Immutable HIP MIN+MAX shared-geometry sequence. Arm "LF_HIP_M13_MIN_MAX".
    LfHipPairLegacy,
    /// Immutable single-contact sessions. Arms LF_UPPER_M12_{MIN,MAX} and
    /// LF_LOWER_M11_{MIN,MAX}. Only Upper/Lower x Min/Max are valid; isolated Hip
    /// remains hardware-blocked (ARM-3r).
    LfSingleContactLegacy { joint: UpperOrLower, side: ContactSide },
}
```

All three expand through §6.1. Their differences are grammar parameters, not a second
engine. Every row below is source-derived:

| Aspect | `LfFullLegSession` | `LfHipPairLegacy` | `LfSingleContactLegacy` |
|---|---|---|---|
| immutable oracle | `run_lf_state_machine` :2850 | `run_lf_hip_min_max` :2725 | `run()` :2633 |
| entry | all-12 q0 normalization :2857 | restart-safe profile entry :2744 | restart-safe profile entry :2640 |
| `InitialRecovery` state | present, **no motion** (§9) | absent | absent |
| prerequisites | full-session accumulated (§11.2) | `StandaloneHeld` (§11.1) | `StandaloneHeld` (§11.1) |
| parking hold | M42 @ `UPPER_30_DELTA`, whole session | via prerequisites | via prerequisites |
| joint blocks | Upper, Lower, Hip | Hip only | one selected Upper **or** Lower |
| sides per block | Min then Max | Min then Max | **one selected side** |
| moving baseline | **relative** `_forward` :3294 | **absolute** :2760 :2793 | **absolute** :2656 |
| coarse pass | scout `None` | scout `None` :2763 :2796 | scout `None` :2659 |
| fine passes | **two**, scout `Some` :3259 :3267 | **one**, scout **`None`** :2770 :2803 | **two**, scout `Some` :2666 :2674 |
| between-side return | none (`stop_pressure` only) | **M13 HOME return** :2775-2779 | n/a |
| after last contact | staged affine q0 returns | HOME return + torque off :2808-2814 | HOME return + torque off :2680-2687 |
| Diagnostics / ReturnStaged | present | absent | absent |
| restore | `RestoreParking` (M42 → HOME) | LIFO `restore_prerequisites` :2818 | LIFO `restore_prerequisites` :2690 |
| post-restore settle | probe re-home in `ReturnStaged` | re-prime + settle :2821-2834 | re-prime + settle :2696-2712 |
| derived operations | **58** (§14.2) | **20** (§14.3) | **16** (§14.4) |

```text
The HipPair fine pass uses approach(FINE_STEP_TICKS) = approach_with_scout(.., None).
Implementing it with scout = Some would ADD a friction/chamfer continuation that
immutable HipPair execution does not have. Revision 2.1's TABLE A row A9 (historical
A-numbering, superseded by W1..W23) asserted the
wrong behavior here; it is corrected in §7.
```

### 6.3 What per-leg data may supply — FUTURE GENERIC/OFFLINE DESIGN

```text
SCOPE NOTE (AG-4 clarity fix). This subsection describes the FUTURE generic/offline
design surface. The G3 LF runtime does NOT consume any of these raw motion-bearing
fields: the sealed LF choreography is derived internally from LfSessionMode under §4.5
(BRAND-1, BRAND-2). Read §6.3 as "what a per-leg spec would carry once non-LF legs are
gated in", not as engine input for G3.
```

```rust
struct CalibrationOrder {
    joint_order: &'static [JointKind],          // LF full: [Upper, Lower, Hip]
    side_order: &'static [[ContactSide; 2]],    // LF full: [Min,Max] for each
}
```

Plus prerequisite references (§11), a parking requirement reference, restore references,
and provenance references. **No phase list, no active IDs, no hold sets, no predecessor
constraints, no terminal phases, no participant list, no step count.**

`RestoreData` is defined here to close the revision-1 gap: it is a set of *references*
(which joints receive a staged q0, and in what order), never an order of operations.
Restore *ordering discipline* (LIFO release) stays engine-owned (§11.4).

The LF values shown above are the *documented* LF choreography, reproduced here so the
generic shape is legible. In G3 they are produced by `mod lf_brand` from the sealed mode,
not read from a raw spec (§4.5).

### 6.4 Participant derivation (R2-4 of review #1, tightened by R2-1.5)

```text
RUNTIME session participants are derived from the SELECTED SESSION MODE (R3-14.1):

participants(mode) = { canonical motor of (Lf, Hip),
                       canonical motor of (Lf, Upper),
                       canonical motor of (Lf, Lower) }
                   ∪ { the mode's own session-scoped hold motors }

  LfFullLegSession        ∪ { M42 }   whole-session parking hold, LF historical (§11.2)
  LfHipPairLegacy         ∪ { M42 }   established as a StandaloneHeld prerequisite (§11.1)
  LfSingleContactLegacy   ∪ { the prerequisite motors of THAT profile } (§11.1)

Each element is resolved from the CANONICAL (Leg, JointKind) -> motor mapping and
cross-checked against the spec's own JointSpec (LFID-3). A spec cannot introduce an
unrelated motor, and any hold motor must be the canonical motor of the joint named in
the reference that requires it (LFID-8).
```

**Session participants are NOT V5 endpoint path dependencies (R3-14).**

```text
PART-1  A Geometry V5 endpoint-local parking/path dependency belongs to that ENDPOINT
        RECORD only. It never enters a session-wide participant set (R3-14.3).
PART-2  Offline specs therefore derive participants from the leg's own three joints
        alone; an endpoint's parking plan is recorded against that endpoint (§15).
        RF's participants are {21,22,23} even though endpoint #9 carries an M32 parking
        plan — that plan is a property of rf_upper_leg_joint:max, not of an RF session.
PART-3  LF's whole-session M42 participation is LF V25 HISTORICAL behavior (§12 V5-3),
        not a consequence of any V5 endpoint plan.
```

---

## 7. TABLE A — LF V25 GoalPosition **write** inventory

Per **R2-5** as corrected by **R3-4**, **R3-10** and **F1**. This table contains **only
actual `GoalPosition` writes**. Helpers that emit no write are excluded and listed
separately in §7.1. Line numbers refer to `matdog.rs` blob `65c16f3c…`.

```text
SOURCE WRITE PATH COUNT    = 23
CONTRACT WRITE ROW COUNT   = 23
MATCH                      = YES
```

There are exactly **two** raw write sinks in the immutable source:

```text
set_motor_goal_verified(motor, target)         :4371   armed-profile writer
set_startup_home_goal_verified(motor, target)  :3792   startup-home writer
```

**F1 correction.** Legacy home-only recovery performs **two** physical writes around the
torque enable, with different torque state and different preconditions:

```text
:3452 prepare_startup_home_recovery_motor
          :3787  GoalPosition = HOME     <-- W2, torque OFF
          :3789  torque ON
:3453 move_profile_entry_motor_to_target(startup_writer = true)
          :3532  GoalPosition = HOME     <-- W3, torque ON
```

Revision 2.1 conflated these into one torque-OFF row, leaving the second write with no
valid operation. The Full path by contrast writes only once: `move_startup_home_motor_to_q0`
:3651 dwells on the goal written at :3787 and issues no write of its own.

Legend — Mode: `F` = `LfFullLegSession`, `H` = `LfHipPairLegacy`, `S` =
`LfSingleContactLegacy`. All three are runtime modes of the one engine (§5.2, §6.2).

| # | Write | Source write path | Mode | Phase | Motor role | Target-value origin | Required observation / evidence | Guard / corridor | Engine operation (§8.2) | Failure / cleanup | Witness |
|---:|---|---|---|---|---|---|---|---|---|---|---|
| W1 | Full startup q0 prime, torque OFF | `normalize_all_matdog_joints_to_q0` :3698 → `prepare_startup_home_recovery_motor` :3720 → `set_startup_home_goal_verified(m, HOME_TICK)` :3787 | **F only** | Entry, all-12 normalization, **pre-session** (§8.0 Entry context) | none (torque OFF) | literal `HOME_TICK` | fresh observation; torque verified OFF :3745; unsigned position valid :3751 | exact-HOME allowance :736 | `home_normalization_prime` | error → abort before session; `run_lf_full_calibration` still runs verified global torque OFF | `full_lf_q0_normalization_has_no_distance_admission_window` |
| W2 | Legacy home-only q0 prime, **torque OFF** | `recover_home_only_joints` :3430 → `prepare_startup_home_recovery_motor` :3452 → `set_startup_home_goal_verified(m, HOME_TICK)` :3787 | **H S only** | Entry, restart-safe home-only recovery | none (torque OFF) | literal `HOME_TICK` | fresh observation; torque verified OFF :3745; entry-hold snapshot :3436 | exact-HOME allowance :736 | `home_normalization_prime` | error → caller runs verified global torque OFF | `startup_v10_pose_classifies_m42_as_valid_prerequisite_residue` |
| W3 | Legacy home-only q0 **reassertion, torque ON** | `recover_home_only_joints` :3430 → `move_profile_entry_motor_to_target(.., startup_writer = **true**)` :3453 → `set_startup_home_goal_verified` :3532 — issued **after** :3789 enabled torque | **H S only** | Entry, restart-safe home-only recovery, immediately after W2 | actively commanded (**torque ON**) | literal `HOME_TICK` | fresh observation; torque verified **ON**; entry-hold snapshot :3559 | exact-HOME allowance :736 | `home_reassert_torque_on` | error → caller runs verified global torque OFF | `startup_envelopes_match_exhaustive_oracle_for_all_profiles_and_ticks` |
| W4 | prime at present observation before torque enable | `prepare_motor` → `set_motor_goal_verified(m, initial.position)` :3884 | F H S | any phase first energizing a motor | becomes actively commanded | **observed present position** of that motor | fresh observation :3879 checked safe :3880 | participant corridor | `prime_at_present` | error → caller aborts to verified global torque OFF | `startup_probe_and_prerequisite_corridors_are_restart_safe` |
| W5 | whole-session parking move | `move_lf_session_motor_to(42, target, 10)` :2870 → `set_motor_goal_verified` :3137 | F | Parking | actively commanded → held | `static_target(Lh, Upper, UPPER_30_DELTA)` = LF historical | LF session role snapshot each frame :3150 | `lf_parking_corridor` | `prerequisite_or_parking_move` | timeout/role error → `Cleanup` | `front_profiles_park_only_the_ipsilateral_rear_upper` |
| W6 | restart-safe prerequisite establish | `establish_prerequisites_restart_safe` → `prepare_motor` :3491 then `move_profile_entry_motor_to_target(.., startup_writer = **false**)` :3492 → `set_motor_goal_verified` :3535 | H S | Entry, prerequisite establishment | actively commanded → held | prerequisite target tick of that profile | entry-hold snapshot :3479/:3513 | `startup_prerequisite_bounds` :1537 | `prerequisite_or_parking_move` | error → caller aborts to verified global torque OFF | `startup_envelopes_match_exhaustive_oracle_for_all_profiles_and_ticks` |
| W7 | moving baseline — **relative** | `acquire_moving_current_baseline_forward` → `set_motor_goal_verified` :3309 | **F only** | ContactSearch, pre-scout | contact probe | `advance_tick(present, probe_sign, BASELINE_TRAVEL_TICKS=64)` :3298 | fresh observation :3296 | refused if `passed_guard` :3303 | `moving_baseline_step` | error → `Cleanup` | `robust_current_baseline_uses_median_and_mad` |
| W8 | moving baseline — **absolute** | `acquire_moving_current_baseline` → `set_motor_goal_verified(m, profile.baseline_target_tick)` :3909 | **H S only** | ContactSearch, pre-scout | contact probe | `profile.baseline_target_tick` = `HOME ± 64` :425 | fresh observation | profile corridor | `moving_baseline_step` | error → caller aborts to verified global torque OFF | `robust_current_baseline_uses_median_and_mad` |
| W9 | probe advance — coarse | `approach_with_scout(COARSE_STEP_TICKS=64, scout=None)` → `set_motor_goal_verified(m, next_target)` :3997 | F H S | ContactSearch | contact probe | `advance_tick(previous_target, probe_sign, 64)` :3982 | fresh observation each settle sample | **mechanical guard only** :3983 | `probe_advance_step` | guard reached → Err before write; stall → `stop_pressure` then Err | `direction_generic_detector_confirms_stall_in_both_tick_directions` |
| W10 | probe advance — fine, **scouted** | `approach_with_scout(FINE_STEP_TICKS=8, scout=Some(coarse))` :3997 | **F S only** | ContactSearch | contact probe | `advance_tick(previous_target, probe_sign, 8)` | as W9 | mechanical guard | `probe_advance_step` | as W9 | `v38_repeatability_compares_two_identical_fine_approaches_not_the_coarse_scout` |
| W11 | probe advance — fine, **unscouted** | `approach(FINE_STEP_TICKS=8)` = `approach_with_scout(.., **None**)` :2770 :2803 → :3997 | **H only** | ContactSearch | contact probe | `advance_tick(previous_target, probe_sign, 8)` | as W9 | mechanical guard | `probe_advance_step` | as W9 | `lf_hip_sequence_orders_min_then_max_before_single_home_recovery` |
| W12 | stop pressure at **fresh observed position** | `stop_pressure(m, observation.position)` :4054 (final accepted contact), :4058 (early stall), :4150 (adaptive contact), :4154 (tracking failure) → :4250 | F H S | ContactSearch | contact probe | **the observation that produced the verdict** | that same fresh observation | none — this is the stop primitive | `stop_pressure_at_observation` | must remain emittable in the failure path; if it cannot be issued, fall through to verified global torque OFF (§8.4) | `detector_confirms_only_persistent_stall_inside_profile_corridor` |
| W13 | stop pressure at **saved accepted-contact tick** | `stop_pressure(m, minimum.second_tick)` :3220; `stop_pressure(m, previous)` :3245; `stop_pressure(m, minimum_second)` :2776; `stop_pressure(m, maximum_second)` :2809; `stop_pressure(m, second_tick)` :2681 → :4250 | F H S | between contact sides / before return | contact probe | **a previously recorded accepted-contact tick**, not a fresh observation | the recorded contact result | none | `stop_pressure_at_recorded_contact` | as W12 | `coarse_scout_is_persisted_but_cannot_change_fine_metrology_or_q0` |
| W14 | backoff after contact | `backoff_and_verify` → `move_motor_to(m, advance_tick(contact, -probe_sign, 96))` :4216 → :4264 | F H S | ContactSearch | actively commanded | derived from the just-recorded contact tick | that contact tick; post-move current must recover :4227 | refused if `crossed_home` :4217 | `backoff_step` | error → `Cleanup` / caller torque OFF | (covered by the fine/coarse pass witnesses) |
| W15 | static-hold transition, Upper → horizontal | `move_motor_to(12, UPPER_90_DELTA=1024, 10)` :2897 → :4264 | F | StaticHold(Upper) | actively commanded → held | LF historical pose | LF session role snapshot each frame | `lf_participant_corridor` | `static_hold_transition` | error → `Cleanup` | `lf_lower_profiles_use_horizontal_upper_and_exact_unsigned_numbers` |
| W16 | static-hold transition, Lower → folded | `move_motor_to(11, LOWER_FOLDED_DELTA=-990, 10)` :2927 → :4264 | F | StaticHold(Lower) | actively commanded → held | LF historical pose | as W15 | `lf_participant_corridor` | `static_hold_transition` | error → `Cleanup` | `lf_hip_sequence_uses_one_horizontal_parallel_pose_for_both_contacts` |
| W17 | staged affine q0 — Hip | `move_motor_to(13, outcome.joints[0].affine.estimated_zero_tick, 10)` :3023 → :4264 | F | ReturnStaged(Hip) | actively commanded → held | **affine** `estimated_zero_tick` only | accepted evidence (affine ∧ witness) :2106 | `lf_participant_corridor`; shift ≤ 96 | `staged_affine_q0` | error → `Cleanup` | `accepted_endpoint_q0_is_used_only_for_transactional_staging` |
| W18 | staged affine q0 — Lower | `move_motor_to(11, …affine…, 10)` :3034 → :4264 | F | ReturnStaged(Lower) | as W17 | as W17 | as W17 | as W17 | `staged_affine_q0` | error → `Cleanup` | `full_lf_final_order_stages_m13_m11_m12_then_restores_m42` |
| W19 | staged affine q0 — Upper | `move_motor_to(12, …affine…, 10)` :3045 → :4264 | F | ReturnStaged(Upper) | as W17 | as W17 | as W17 | as W17 | `staged_affine_q0` | error → `Cleanup` | as W18 |
| W20 | parking restore to home | `move_motor_to(42, HOME_TICK, 10)` :3055 → :4264 | F | RestoreParking | actively commanded | literal `HOME_TICK` | LF session role snapshot | `lf_parking_corridor` | `return_home` | error → `Cleanup` | `full_lf_final_order_stages_m13_m11_m12_then_restores_m42` |
| W21 | probe return home | `move_motor_to(m, HOME_TICK, PROBE_HOME_TOLERANCE_TICKS=16)` :2652 :2683 :2756 :2778 :2811 :3949 → :4264 | H S | post-baseline / between sides / post-contact | actively commanded | literal `HOME_TICK` | fresh observation each frame | participant corridor | `return_home` | error → caller torque OFF | `probe_home_tolerance_is_scoped_to_startup_home_endpoint_and_active_probe_returns` |
| W22 | post-restore probe settle | `move_motor_to(m, HOME_TICK, STATIC_TOLERANCE_TICKS=10)` :2697 :2822 → :4264 | H S | after `restore_prerequisites` | actively commanded → torque OFF | literal `HOME_TICK` | fresh observation; final settle checked :2703 :2828 | participant corridor | `return_home` | error → caller torque OFF | `probe_reverse_recovery_accepts_observed_2031_then_requires_final_rehome` |
| W23 | prerequisite restore to home | `restore_prerequisites` → `move_motor_to(m, HOME_TICK, 10)` :3606 → :4264, LIFO pop | H S | RestorePrerequisites | actively commanded → torque OFF | literal `HOME_TICK` | fresh observation each frame | participant corridor | `return_home` | error → caller torque OFF | `front_lower_restore_order_keeps_rear_parking_until_active_leg_is_home` |

```text
Twenty-three write paths reduce to TWELVE engine operations (§8.2).
Distinct rows are NOT merged where the value origin, torque state or required evidence
differs (R3-10.5, F1.3):
    W1 / W2   same writer and value, different MODE and different entry context
    W2 / W3   same mode, SAME value, but opposite TORQUE STATE and different
              preconditions — this is the F1 split
    W7 / W8   relative versus absolute baseline
    W9 / W10 / W11   step size and scout policy
    W12 / W13 fresh verdict observation versus recorded accepted-contact tick
    W21 / W22 different settle tolerance (16 versus 10) at different phases
```

### 7.1 Operations deliberately EXCLUDED — they emit no GoalPosition

Revision 2 wrongly listed the first of these as a write (R3-4.3).

```text
startup q0 settle       set_startup_home_torque_verified(m, true) :3789 then
                        move_startup_home_motor_to_q0 :3651
                        -> enables torque and DWELLS on the goal written by W1/W2.
                        It issues NO GoalPosition write of its own.
torque enable/disable   set_motor_torque_verified :4388, set_startup_home_torque_verified :3808
RAM envelope writes     TorqueLimit / Acc / GoalSpeed :3773-3785, :3886-3899
global torque OFF       global_torque_off_writes / sync_write_ram_verified :2278 :4417
                        -> TorqueEnable only; never GoalPosition
```

### 7.2 Two corrections this inventory forces

```text
C-1  probe_advance_step is bounded by the MECHANICAL GUARD, not by the contact-acceptance
     corridor. The acceptance corridor governs whether a detected stall COUNTS as contact
     (:800, :814); it never gated goal admission.

C-2  prime_at_present, stop_pressure_at_observation and stop_pressure_at_recorded_contact
     take their value from an observation or a recorded contact result, never from a spec
     or geometry record. A design requiring every goal to be record-bound cannot express
     them.
```

---

## 8. Immediate engine-owned authorization and emission

Per **R3-3**, tightened by **F2**, **F3** and **F4**. Revision 2's first-class
`MotionGrant` capability is **withdrawn** and is not replaced by nonce/session/epoch
token machinery.

```text
Phase 2A does not require a reusable motion-authority token, so the design does not
create one. There is no value that means "permission to move later", and therefore
nothing to clone, copy, retain, replay, expire or revoke.
```

### 8.0 Engine contexts, including the pre-session Entry context (F2)

Full-mode all-12 normalization happens **before** the LF session-state object exists
(`normalize_all_matdog_joints_to_q0` :3698 runs before `inspect_lf_native_session_entry`
:3062). The startup writes W1/W2/W3 therefore need an authorizing context that predates
the session.

```rust
/// Engine-owned state. NOT a value handed to callers, NOT transferable,
/// NOT storable as permission. It exists for the duration of the entry phase and is
/// replaced by SessionContext once the session object is created.
enum EngineContext {
    /// Created directly from the sealed ArmableLfSessionSpec (§4.4 SEAL-3).
    /// Knows: the validated LF mode, the canonical participant set, and which entry
    /// step is currently executing.
    Entry { mode: LfSessionMode, step: EntryStep },
    /// After session creation. Knows the current grammar node and active motor.
    Session { mode: LfSessionMode, node: GrammarNode, active: Option<u8> },
}

enum EntryStep {
    ExactSetVerify,
    GlobalTorqueOff,
    FullNormalization { motor: u8, stage: NormalizationStage }, // Full only
    LegacyProfileEntry { motor: u8, stage: LegacyEntryStage },  // HipPair / Single only
}

enum NormalizationStage { Prime, Settle }              // Prime writes (W1); Settle does not
enum LegacyEntryStage   { Prime, Reassert, Prerequisite }  // W2, W3, W6
```

```text
CTX-1  EngineContext is engine state, never a transferable motion token (F2.3).
CTX-2  It is constructible only from the sealed brand; there is no public constructor.
CTX-3  Full and legacy modes retain distinct entry behavior: EntryStep::FullNormalization
       is reachable only in LfFullLegSession, EntryStep::LegacyProfileEntry only in
       LfHipPairLegacy and LfSingleContactLegacy (F2.4, §10).
CTX-4  Every operation in §8.2 checks the current EngineContext variant and step/node
       before writing.
```

### 8.1 The rule

```text
AUTH-1  UnsignedTick proves 0..=4095 and NOTHING else.
AUTH-2  The lowest GoalPosition construction/emission boundary is PRIVATE (§8.3).
AUTH-3  Only the intent-specific engine operations of §8.2 may reach that boundary.
AUTH-4  Each operation, in ONE call and in this order:
            derives its own value and evidence from the current EngineContext,
            checks mode / context step or grammar node / active motor / observation
              freshness / guard,
            emits the GoalPosition write.
AUTH-5  There is NO "authorize now, store capability, emit later" path.
AUTH-6  Operations are engine methods reachable only from a context created from the
        sealed ArmableLfSessionSpec (§4.4 SEAL-3).
AUTH-7  Operations DERIVE their authority-relevant inputs; they do not accept them as
        free caller choices (§8.4, F3).
```

Because authorization and emission are the same call, there is **no authorize-now /
emit-later staleness window**: no decision can be made and then become out of date
before it is used.

```text
AG-5 precision. This does NOT claim that observations or evidence cannot become
stale. They can. Observation and evidence freshness remain an EXPLICIT runtime
invariant and MUST be checked immediately before the write, per DER-1 and §8.6.
What is eliminated is the deferred-permission window, not the need to check
freshness.
```

### 8.2 The twelve intent-specific engine operations

Derived from TABLE A **after** the F1 correction (**R2-5.5**, **F1.4**). Each is a
private method; each maps to one or more W-rows; together they cover W1..W23 exactly.

```rust
// All private to the engine module; all take &mut self; all emit within the call.
// Arguments are IDENTIFIERS and EVIDENCE, never free target selections (§8.4).
fn home_normalization_prime(&mut self)                    -> Result<()>;  // W1  W2
fn home_reassert_torque_on(&mut self)                     -> Result<()>;  // W3   (F1)
fn prime_at_present(&mut self, obs: &FreshObservation)    -> Result<()>;  // W4
fn prerequisite_or_parking_move(&mut self)                -> Result<()>;  // W5  W6
fn moving_baseline_step(&mut self)                        -> Result<()>;  // W7  W8
fn probe_advance_step(&mut self)                          -> Result<()>;  // W9  W10 W11
fn stop_pressure_at_observation(&mut self, v: &Verdict)   -> Result<()>;  // W12
fn stop_pressure_at_recorded_contact(&mut self)           -> Result<()>;  // W13
fn backoff_step(&mut self)                                -> Result<()>;  // W14
fn static_hold_transition(&mut self)                      -> Result<()>;  // W15 W16
fn staged_affine_q0(&mut self)                            -> Result<()>;  // W17 W18 W19
fn return_home(&mut self)                                 -> Result<()>;  // W20 W21 W22 W23
```

There is **no** direct-geometry operation. `DirectGeometryTarget` is not implemented in
G3 (**R2-6**, **R3-4.4**, **F9**); only its eligibility predicate survives, as
documentation (§2).

### 8.3 The frozen GoalPosition call graph, sealed (F4, CG-1..CG-8)

**Terminology.** Three layers, three different names. Do not call all of them "sinks".

```text
raw constructor / emitter   the UNIQUE lowest construction boundary
policy writers              the TWO reviewed semantic/gating wrappers
engine operations           the TWELVE authority-owning callers of §8.2
```

**Immutable V25 source today** has two lowest historical GoalPosition sinks, each
building its own write envelope:

```text
set_motor_goal_verified        :4371   ->  write_motor_ram_verified        :4436
set_startup_home_goal_verified :3792   ->  write_startup_home_ram_verified :3830
```

**G3 target** inserts a single construction boundary beneath their successors:

```text
TWELVE engine motion operations (§8.2)
        |
        v   each calls exactly ONE of two private POLICY WRITERS
   +----+---------------------------------+
   |                                      |
A. startup-home policy writer         B. normal armed-goal policy writer
   successor of set_startup_home_        successor of set_motor_goal_verified
   goal_verified                         used for W4 .. W23
   used ONLY for W1, W2, W3
   |                                      |
   +----+---------------------------------+
        |
        v
   ONE private raw GoalPosition constructor / emitter
        |
        v
   RamRegister::GoalPosition
```

```text
CG-1  Exactly ONE production location in MATDOG SCOPE A constructs the
      RamRegister::GoalPosition write envelope.
      "Construction site" means a call passing RamRegister::GoalPosition TOGETHER WITH A
      VALUE PAYLOAD to a RAM-write helper. Bare enum references inside register
      allowlists and match arms are NOT construction sites and legitimately remain —
      today matdog.rs has three such references (:576, :606, :2256) alongside its two
      historical construction sites (:3802, :4382), and a correct G3 implementation
      keeps the three while collapsing the two into one.
      #[cfg(test)] material and matdog_test.rs are structurally EXCLUDED from this
      production count (CI-C1.2).
CG-2  The DIRECT caller set of that raw constructor is exactly TWO private policy
      writers. The twelve operations are INDIRECT callers, two levels up.
CG-3  The startup-home policy writer is reachable only from the reviewed startup
      operations covering W1/W2/W3.
CG-4  The normal armed-goal policy writer is reachable only from the remaining reviewed
      engine operations covering W4..W23.
CG-5  Across both policy writers the reviewed engine caller set is exactly the TWELVE
      §8.2 operations.
CG-6  No other production helper, direct constructor, third writer or caller exists.
CG-7  The two policy writers RETAIN their different historical gating semantics and are
      NOT collapsed. Verified in source, they gate on different motor sets:
          startup-home writer :3837  admits any of MATDOG_MOTOR_IDS (all twelve),
                              :3840  plus a local ram_write_allowed_for_profile check
          normal writer       :4443  admits ONLY profile.allowed_motor_ids
      W1 normalizes all twelve motors and W2/W3 recover home-only joints that are not
      participants of the armed profile, so routing them through the normal writer would
      reject those motors and break startup. Routing W4..W23 through the startup writer
      would drop the armed-profile allowlist. Collapsing is a behavior change in both
      directions and is forbidden.
CG-8  Terminal verified global torque OFF, hard abort and operator stop remain OUTSIDE
      this graph and continue to write TorqueEnable only (§8.5).
```

CI enforces this graph mechanically in §16.3 M-27s.

### 8.4 Inputs are derived, not chosen (F3)

This is what makes "correctly-named method, different argument" unable to reach an
off-trace target.

| Quantity | Derived from | Never a free caller choice |
|---|---|---|
| current motor | `EngineContext::{Entry.step, Session.active}` | ✔ |
| target / reference pose | the current grammar node's LF-historical reference (§11.5) | ✔ |
| probe step size | the current sub-operation: coarse ⇒ `COARSE_STEP_TICKS`, fine ⇒ `FINE_STEP_TICKS` | ✔ |
| scout policy | the validated mode: Full/Single fine ⇒ `Some(coarse)`, HipPair fine ⇒ `None` | ✔ |
| baseline rule | the validated mode: Full ⇒ relative, HipPair/Single ⇒ absolute | ✔ |
| recorded contact value | the contact result recorded earlier in **this** session | ✔ |
| staged evidence | the accepted evidence for the **current staged joint** of **this** session | ✔ |
| restore / probe motor | the currently selected restore or probe motor of the current node | ✔ |

```text
DER-1  A fresh observation may enter only as a `FreshObservation` that the engine
       REBINDS before use: it must carry the exact expected motor, the latest
       observation identity/stamp, pass the freshness gate, and correspond to the
       current operation or verdict (F3.3).
DER-2  A `Verdict` passed to stop_pressure_at_observation is the detector verdict
       produced in the current approach iteration; the value written is that verdict's
       own observation position.
DER-3  No operation takes a raw tick, a raw motor id, a step size, a scout flag, a pose
       value or an evidence record as a caller-selected argument.
DER-4  Consequently a caller cannot select an off-trace target by calling a
       correctly-named method with different arguments (F3.5).
```

### 8.5 Terminal safety is independent (R3-5.4, R3-5.5, R3-15.3, F4.5)

```text
SAFE-1  Verified global torque-off, hard abort, operator stop and terminal cleanup do
        NOT depend on normal per-goal authorization. They write TorqueEnable, never
        GoalPosition, and remain reachable from every phase and every context.
SAFE-2  If stop_pressure_at_observation or stop_pressure_at_recorded_contact cannot
        safely perform its immediate pressure-release write, the engine falls through to
        the existing verified global torque-off path.
SAFE-3  A failure inside any §8.2 operation propagates to the mode's existing failure
        route, which always terminates in verified global torque OFF.
```

### 8.6 Per-operation preconditions

| Operation | Preconditions checked immediately before the write |
|---|---|
| `home_normalization_prime` | context is `Entry`; step is `FullNormalization{Prime}` (Full) or `LegacyProfileEntry{Prime}` (HipPair/Single); derived motor ∈ canonical 12; **torque currently OFF**; unsigned position valid; value is exactly `HOME_TICK` |
| `home_reassert_torque_on` | context is `Entry`; step is `LegacyProfileEntry{Reassert}`; mode ∈ {HipPair, Single}; **torque currently ON** (enabled by the preceding prime); derived motor equals the motor primed by W2; value is exactly `HOME_TICK` |
| `prime_at_present` | derived motor matches the rebound `FreshObservation`; that observation is fresh and safe; torque currently OFF; value equals that observation's position |
| `prerequisite_or_parking_move` | derived motor ∈ derived participants (§6.4); value equals the current node's referenced pose (LF-historical for LF runtime, §11.5); inside that motor's corridor; entry-hold snapshot valid |
| `moving_baseline_step` | node is ContactSearch; derived motor is the active probe; value follows the **mode-derived** baseline rule; not past guard |
| `probe_advance_step` | node is ContactSearch; derived motor is the active probe; step and scout are **derived** from the current sub-operation and mode; value = `advance_tick(previous_target, probe_sign, step)`; **not past guard** |
| `stop_pressure_at_observation` | derived motor is the active probe; value equals the verdict's own observation position; **must remain issuable on the failure path** |
| `stop_pressure_at_recorded_contact` | derived motor is the active probe; value equals a contact tick recorded earlier in **this** session |
| `backoff_step` | value derived from the contact tick just recorded in this node; does not cross home |
| `static_hold_transition` | node is StaticHold(joint); value equals that node's LF-historical pose |
| `staged_affine_q0` | node is ReturnStaged(joint); evidence is **this session's** accepted evidence for **that joint** (affine ∧ LF witness, §13); value = `affine.estimated_zero_tick`; shift ≤ 96; **fixed-scale can never satisfy this** |
| `return_home` | derived motor is the current node's selected restore or probe motor; value is exactly `HOME_TICK`; tolerance is the node's own (16 for probe return W21, 10 for post-restore settle W22) |

Adding a thirteenth operation requires a new method, a new precondition row and a new CI
row (§16.3 M-18s) — visible and reviewable.

---

## 9. TABLE B — LF V25 exact lifecycle and role oracle

Per **R2-5** and **R2-8**, for `LfFullLegSession`, from source reality.

| State | Eligible motors | Active-motor semantics | Held on entry | Held during operation | Held on exit | Passive torque-OFF requirement | Progress emissions | Cleanup / error transition |
|---|---|---|---|---|---|---|---|---|
| `Preflight` | none during the first three operations | no active motor; the first three preflight operations act on all 12 motors before any session object exists | — | — | — | all 12 torque OFF after the verified global torque-OFF; each normalized motor returns to torque OFF | **4** (ID-set verify, global torque OFF, all-12 q0 normalization, session creation) | **E-2 precision:** errors during the first three operations occur before the session exists; after `inspect_lf_native_session_entry()` :3062 the session object DOES exist, so an error during the `InitialRecovery` transition or role verification occurs with a live session and takes the normal `→ Cleanup` route. In both cases `run_lf_full_calibration` still performs verified global torque OFF |
| `InitialRecovery` | `11 \| 12 \| 13` per `active_motor_allowed` :1168 | **no motion occurs in the full session.** All recovery already happened in Preflight normalization. This is a verification-only pass-through | `{}` | `{}` | `{}` | M11/M12/M13 torque OFF, within their passive corridors | **0** | `→ Cleanup` |
| `Parking` | `42` :1169 | M42 primed (W4 `prime_at_present`) then moved (W5 `prerequisite_or_parking_move`), then promoted to held | `{}` | active M42 | `{42}` | 11/12/13 passive at home; other 8 non-participating | **1** | `→ Cleanup` |
| `UpperMin` | `12` :1172 | M12 primed once (W4 `prime_at_present`) then contact probe | `{42}` | `{42}`; M12 is the probe | `{42}` | **M11 and M13 passive torque-OFF at home** | **1** (prepare) **+ 7** (contact side) | `→ Cleanup` |
| `UpperMax` | `12` | probe continues on the opposite side; requires previous active M12 :1146 | `{42}` | `{42}` | `{42}` | M11, M13 passive torque-OFF | **7** | `→ Cleanup` |
| `UpperHorizontal` | `12` | M12 moved to `UPPER_90_DELTA` (W15 `static_hold_transition`) then promoted to held | `{42}` | active M12 | `{12,42}` | M11, M13 passive torque-OFF | **1** | `→ Cleanup` |
| `LowerMin` | `11` :1173 | M11 primed (W4 `prime_at_present`) then contact probe | `{12,42}` | `{12,42}` | `{12,42}` | **M13 passive torque-OFF at home** | **1 + 7** | `→ Cleanup` |
| `LowerMax` | `11` | opposite side; requires previous active M11 :1147 | `{12,42}` | `{12,42}` | `{12,42}` | M13 passive torque-OFF | **7** | `→ Cleanup` |
| `LowerFolded` | `11` | M11 moved to `LOWER_FOLDED_DELTA` (W16 `static_hold_transition`) then held | `{12,42}` | active M11 | `{11,12,42}` | M13 passive torque-OFF | **1** | `→ Cleanup` |
| `HipMin` | `13` :1176 | M13 primed (W4 `prime_at_present`) then contact probe, using `lf_hip_sequence_profile(Min)` | `{11,12,42}` | `{11,12,42}` | `{11,12,42}` | none remaining | **1 + 7** | `→ Cleanup` |
| `HipMax` | `13` | opposite side, `lf_hip_sequence_profile(Max)`; **M12 remains at `UPPER_90_DELTA`/1024, not 967** | `{11,12,42}` | `{11,12,42}` | `{11,12,42}` | none | **7** | `→ Cleanup` |
| `Diagnostics` | `13` | **`self.active` is deliberately NOT cleared** :1101 so `Diagnostics → ReturnHip` satisfies `required_previous_active` M13 :1150 | `{11,12,42}` | `{11,12,42}` | `{11,12,42}` | none | **1** | rejection here fails **before any motion**; `→ Cleanup` |
| `ReturnHip` | `13` | M13 (never held) moved to staged affine q0 (W17 `staged_affine_q0`), then promoted to held | `{11,12,42}` | active M13 | `{11,12,13,42}` | none | **1** | `→ Cleanup` |
| `ReturnLowerHeld` | `11` :1180 | **`remove_held_target(11)` BEFORE the move** :3032, move (W18 `staged_affine_q0`), **`upsert_held_target(11)` after stable arrival** :3036 | `{11,12,13,42}` | `{12,13,42}` + active M11 | `{11,12,13,42}` | none | **1** | `→ Cleanup` |
| `ReturnUpper` | `12` :1181 | `remove_held_target(12)` :3043, move (W19 `staged_affine_q0`), re-add :3047 | `{11,12,13,42}` | `{11,13,42}` + active M12 | `{11,12,13,42}` | none | **1** | `→ Cleanup` |
| `RestoreParking` | `42` :1169 | `remove_held_target(42)` :3054, move to HOME (W20 `return_home`); **not re-added** | `{11,12,13,42}` | `{11,12,13}` + active M42 | `{11,12,13}` | none | **1** (+ the final "Final verified global torque OFF" emission) | `→ Cleanup` |
| `Cleanup` | none :1182 | no active motor; entry **bypasses** `validate_transition_entry` :1096 | **holds are still present** | `sync_write(TorqueEnable=0)` to all 12, then verify all 12 report OFF :4418-4426 | `{}` — cleared **only after** verification :4427 | all 12 must read torque OFF | **0** | terminal path |
| `TorqueOff` | none | terminal | `{}` | — | `{}` | all 12 torque OFF | **0** | — |

```text
Total progress emissions, by state in order:
  Preflight 4 | InitialRecovery 0 | Parking 1
  UpperMin (1+7) | UpperMax 7 | UpperHorizontal 1
  LowerMin (1+7) | LowerMax 7 | LowerFolded 1
  HipMin   (1+7) | HipMax   7 | Diagnostics 1
  ReturnHip 1 | ReturnLowerHeld 1 | ReturnUpper 1 | RestoreParking (1+1)
  Cleanup 0 | TorqueOff 0
  = 4+0+1 +8+7+1 +8+7+1 +8+7+1 +1+1+1+2 +0+0 = 58     (enumerated in §14)
```

### 9.1 Corrections this table forces

```text
B-1  InitialRecovery performs NO motion in the full session. `lf_initial_recovery_needed`
     is called ONLY from the normalization loop :3709 — it is a Preflight predicate,
     not an InitialRecovery behavior. Revisions 0 and 1 both described it wrongly.
B-2  Full-session Hip MAX holds M12 at UPPER_90_DELTA (1024). `lf_hip_sequence_profile`
     :465 overwrites the prerequisite list identically for BOTH sides, and
     run_lf_state_machine :2934 uses it for Min and Max. UPPER_85_DELTA (967) reaches
     hardware only through the standalone build_profile path.
B-3  Staged returns REMOVE the held target before moving and re-add after stable arrival.
B-4  Cleanup RETAINS holds through the verified global torque-OFF and clears them only
     afterwards. "Cleanup held = {}" on entry is wrong.
B-5  During Upper contact, M11 and M13 are PASSIVE TORQUE-OFF at home — not held.
```

---

## 10. Exact LF startup — per mode

Per **R2-7** as scoped by **R3-11**. All-12 q0 normalization belongs to
`LfFullLegSession` **only**. Applying it to `LfHipPairLegacy` or
`LfSingleContactLegacy` would change their legacy entry choreography and is forbidden.

### 10.0 `LfFullLegSession` entry

```text
Order (immutable, run_lf_state_machine :2851-2865):
  1. wait_for_exact_motor_set()                       — verified exact MATDOG 12-ID set
  2. global_torque_off_verified()                     — verified global torque OFF
  3. normalize_all_matdog_joints_to_q0()              — ALL TWELVE motors
  4. inspect_lf_native_session_entry() + transition(InitialRecovery) + role verification
```

```text
STARTUP-1  Normalization iterates `for motor_id in MATDOG_MOTOR_IDS` :3705 — all twelve.
           It MUST NOT be reduced to LF participants.

STARTUP-2  The exact effective recovery predicate :3708-3709 is

               recovery_needed = circular_distance(present, HOME_TICK) > STATIC_TOLERANCE_TICKS
                                 || lf_initial_recovery_needed(present)

               where lf_initial_recovery_needed(o) =
                       circular_distance(o.position, HOME_TICK) > PROBE_HOME_TOLERANCE_TICKS
                       || speed_magnitude(o.velocity) > LF_HELD_MAX_SPEED_RAW

               STATIC_TOLERANCE_TICKS = 10, PROBE_HOME_TOLERANCE_TICKS = 16,
               LF_HELD_MAX_SPEED_RAW = 4.

           The `> 16` clause is SUBSUMED by `> 10`. The effective predicate is therefore

               distance > 10  ||  speed > 4

           A 16-tick approximation is FORBIDDEN: M11 at HOME+11 moves in immutable LF
           and would be skipped by a `>16` rule. A displaced M21 is normalized by
           immutable LF and would be skipped by an LF-participants-only rule.

STARTUP-3  Per motor requiring recovery, the exact sequence is
               verify uniform snapshot :3719
               prepare_startup_home_recovery_motor  (TorqueLimit, Acc, GoalSpeed, then
                                                     GoalPosition=HOME while torque OFF,
                                                     then torque ON)          :3720
               move_startup_home_motor_to_q0  with StableTargetGate at
                                              STATIC_TOLERANCE_TICKS (10)     :3721
               set_startup_home_torque_verified(false)                        :3723
               insert into home_ready_motors; re-verify uniform snapshot      :3725-3726

STARTUP-4  Completion requires home_ready_motors.len() == 12 :3729.
STARTUP-5  STARTUP-1..STARTUP-4 apply to LfFullLegSession ONLY (R3-11.1, R3-11.2).
```

### 10.1 `LfHipPairLegacy` and `LfSingleContactLegacy` entry

These modes keep their own restart-safe profile-entry behavior exactly. They never run
all-12 normalization.

```text
Order (immutable, run_lf_hip_min_max :2738-2752 / run() :2634-2648):
  1. wait_for_exact_motor_set()                — verified exact MATDOG 12-ID set
  2. global_torque_off_verified()              — verified global torque OFF
  3. inspect_profile_entry()                   — restart-safe classification :3343
                                                 StartupRole::{Probe, Prerequisite, HomeOnly}
                                                 startup envelopes :1589 / bounds :1537 :1558
  4. recover_home_only_joints()                — ONLY the classified home-only motors :3430
                                                 TWO writes per recovered motor:
                                                 W2  torque-OFF HOME prime
                                                     prepare_startup_home_recovery_motor
                                                     -> :3787          (home_normalization_prime)
                                                 then torque enable :3789
                                                 W3  torque-ON HOME reassertion
                                                     move_profile_entry_motor_to_target(
                                                       startup_writer = true) -> :3532
                                                                       (home_reassert_torque_on)
                                                 recovery is complete only after BOTH
  5. establish_prerequisites_restart_safe()    — profile prerequisites, armed writer (W6)
                                                 each becomes StandaloneHeld :3511
```

```text
LEGACY-1  Entry recovers ONLY the motors classified HomeOnly by inspect_profile_entry.
          It does NOT iterate all 12 motors and does NOT apply the §10 STARTUP-2 rule.
LEGACY-2  Admission uses the startup envelopes of :1589, including
          STARTUP_PREREQUISITE_HOME_SETTLE_TICKS (16) and PROBE_HOME_TOLERANCE_TICKS (16)
          exactly where immutable source uses them.
LEGACY-3  A wrong-profile residue is rejected (`startup_wrong_profile_residue_is_rejected`).
LEGACY-4  The engine may share implementation primitives with §10, but the observable and
          physical choreography of each mode is preserved exactly (GRM-12, R3-11.4).
```

---

## 11. Prerequisite contexts and derived roles

Per **R2-8**.

```rust
enum PrerequisiteContext {
    /// Standalone-contact session: every prerequisite is torque-ON and actively held
    /// for the duration of that single contact profile.
    StandaloneHeld,
    /// Full-session: promoted to a held target after its own completed move and
    /// retained across subsequent joint blocks.
    SessionAccumulatedHold,
    /// Torque OFF, required to REMAIN at home inside its passive corridor.
    PassiveAtHome,
}
```

```text
PRQ-1  Context is a property of (session mode, joint, side, referenced motor) — never a
       single fixed attribute of an endpoint (R2-8.1).
PRQ-2  The same referenced motor may carry different contexts in different modes.
PRQ-3  Roles are DERIVED from context plus completed operations, never declared.
```

### 11.1 `LfHipPairLegacy` / standalone context

Every prerequisite is `StandaloneHeld`: `establish_prerequisites_restart_safe` :3473
primes each prerequisite motor (W4 `prime_at_present`) and moves it (W6
`prerequisite_or_parking_move`) with torque ON, and
`restore_prerequisites` :3604 returns them LIFO to home with verified torque OFF.

### 11.2 `LfFullLegSession` context map (exact)

| Joint block | Referenced motor | Pose | Context |
|---|---|---|---|
| Upper Min/Max | M42 | `UPPER_30_DELTA` (341) | `SessionAccumulatedHold` (applied once in Parking) |
| Upper Min/Max | M13 | home | **`PassiveAtHome`** |
| Upper Min/Max | M11 | home | **`PassiveAtHome`** |
| Lower Min/Max | M42 | `UPPER_30_DELTA` | `SessionAccumulatedHold` |
| Lower Min/Max | M12 | `UPPER_90_DELTA` (1024) | `SessionAccumulatedHold` (applied in StaticHold(Upper)) |
| Lower Min/Max | M13 | home | **`PassiveAtHome`** |
| Hip **Min** | M42 / M12 / M11 | 341 / **1024** / −990 | `SessionAccumulatedHold` |
| Hip **Max** | M42 / M12 / M11 | 341 / **1024** / −990 | `SessionAccumulatedHold` |

```text
PRQ-4  Full-session Hip MAX uses UPPER_90_DELTA = 1024 for M12 (R2-8.4).
       UPPER_85_DELTA = 967 is a STANDALONE-path value only and must never be
       substituted into the full session.
```

### 11.3 Staged-return timing (R2-8.5)

```text
for each staged joint, in the mode's restore order:
    remove the joint from held state
    issue StagedAffineQ0 and move
    await stable arrival
    re-add to held state
```

### 11.4 Cleanup timing (R2-8.6)

```text
transition(Cleanup)                       holds still present; entry bypasses hold checks
sync-write TorqueEnable=0 to all 12
verify every one of the 12 reports torque OFF
THEN clear held targets
complete_verified_cleanup() -> TorqueOff
```

Restore *ordering discipline* remains engine-owned LIFO so contralateral parking is
released last (OPEN-5).

### 11.5 LF historical poses are not V5 records

```text
UPPER_30_DELTA      341 ticks   29.9707 deg
UPPER_85_DELTA      967 ticks   84.9902 deg   (standalone path only)
UPPER_90_DELTA     1024 ticks   90.0000 deg
LOWER_FOLDED_DELTA -990 ticks  -87.0117 deg
```

Canonical V5 parking values are different numbers entirely (35.0000°, 64.1667°,
93.3333°, §12). LF historical poses are therefore bound to a distinct reference kind,
`PoseReference::LfHistoricalV25`, constructible only inside the LF oracle module and
rejected on any non-LF spec.

**G3 runtime versus future offline data (AG-3).** These are two disjoint worlds and
there is no path from the second to the first:

```text
G3 RUNTIME
    prerequisite_or_parking_move derives ONLY the exact LF historical pose required
    by the sealed LF mode (§8.4, §8.6). It takes no pose argument. A V5 record-bound
    pose is REJECTED by BRAND-3 and is unreachable by construction.

FUTURE OFFLINE GEOMETRY DESIGN
    V5 record-bound poses may exist as INERT offline records (§12, §15), with NO
    engine consumer and NO motion authority. They are not implemented in G3.
```

This is what closes revision 1's gap: LF runtime poses have a legal binding path
(`LfHistoricalV25`, derived internally), and V5 poses have none.

---

## 12. Geometry V5 path/parking evidence is endpoint-scoped

Per **R2-9**.

```text
V5-1  A parking plan belongs to the endpoint record that produced it. It is stored as
      part of that endpoint's record binding and nowhere else.
V5-2  An endpoint-scoped plan MUST NOT be promoted into a whole-session parking hold.
V5-3  LF's whole-session M42 parking is separate LF V25 HISTORICAL behavior; it is not
      derived from, and must not be justified by, any V5 endpoint parking plan.
```

Canonical scope, exactly as recorded:

```text
lf_upper_leg_joint:max   parks lh_upper_leg_joint = 0.610865238198 rad (35.0000 deg)
rf_upper_leg_joint:max   parks rh_upper_leg_joint = 0.610865238198 rad (35.0000 deg)
lf_lower_leg_joint:min   parks lf_upper_leg_joint = 1.119919603363 rad (64.1667 deg)
rf_lower_leg_joint:min   parks rf_upper_leg_joint = 1.119919603363 rad (64.1667 deg)
rh_lower_leg_joint:min   parks rh_upper_leg_joint = 1.628973968528 rad (93.3333 deg)
lh_lower_leg_joint:min   parks lh_upper_leg_joint = 1.628973968528 rad (93.3333 deg)
the other 18 endpoints   NOT_NEEDED
```

RF's M32 parking is validated for `rf_upper_leg_joint:max` **only** — not for every RF
contact and not for a combined RF session.

---

## 13. LF hardware oracle — opaque, JointKind-keyed, non-spoofable

Per **R2-12**.

```text
module lf_v25_oracle
    opaque LfV25Oracle
        reference_contact_ticks(JointKind) -> (min, max)   keyed, never positional
            Hip (2535, 1617)   Upper (1443, 3442)   Lower (3093, 1666)
        tolerance_ticks() -> 24
        reconciliation(JointKind, ContactSide) -> HardwareEvidenceBinding
            SIX records, one per joint/side (revision 1 sketched only one)
        physical_evidence_sha256() -> 6eae3201…
        reconciler_source_sha256() -> 111da404…
    fn lf_v25_oracle() -> &'static LfV25Oracle          // sealed; module-private ctor
```

```text
ORA-1  The oracle is reachable ONLY from an ArmableLfSessionSpec, which exists only
       after §4.3 LFID-1..LFID-9 pass. An LF-labelled hybrid fails before the oracle
       or any probe authority exists (R2-12.2).
ORA-2  OfflineLegCalibrationSpec has NO oracle accessor for any leg.
ORA-3  HardwareContactEvidence is inert data. Its SupervisedPassRecorded variant grants
       nothing; authority comes only from ORA-1 (R2-12.4).
ORA-4  Keyed by JointKind, so the oracle's historical [Hip, Upper, Lower] layout can
       never be transposed against the calibration order [Upper, Lower, Hip].
ORA-5  No non-LF construction path can name the oracle type or its tick data.
```

The freeze gate keeps its immutable shape — `affine.accepted && contact_witness_accepted`
— with the witness term supplied only through ORA-1. A spec with no oracle cannot reach
`accepted == true`, so `StagedAffineQ0` is unreachable for it.

---

## 14. Progress derived from the actual operation trace — three exact oracles

Per **R2-13** as completed by **R3-9**.

```text
PRG-1  The engine expands the grammar into an operation trace. Progress-emitting
       operations are exactly those that call the progress primitive.
PRG-2  expected_total = count of progress-emitting operations in the expanded trace.
       It is NOT stored, NOT spec data, NOT a constant.
PRG-3  DONE requires executed_progress == expected_total.
PRG-4  LF regression proves the ACTUAL increments AND the historical strings in order —
       not that a function returns a number (R2-13.6, R3-9.6).
```

### 14.0 Two distinct quantities (F6)

Revision 2.1 called the 58/20/16 list "the actual emitted strings". That is inaccurate
for an unfiltered external observer: the wrappers additionally emit a step-zero preflight
update and a terminal completed update.

```text
OPERATIONAL PROGRESS INCREMENTS  — the next_phase() operations counted by the engine trace
    LfFullLegSession       58
    LfHipPairLegacy        20
    LfSingleContactLegacy  16

COMPLETE SUCCESSFUL EXTERNAL PROGRESS STREAM — everything an external observer sees
    LfFullLegSession       60   =  1 preflight + 58 increments + 1 completed
    LfHipPairLegacy        22   =  1 preflight + 20 increments + 1 completed
    LfSingleContactLegacy  18   =  1 preflight + 16 increments + 1 completed
```

The two extra events, from source:

```text
step-zero preflight  publish_progress(0, "<mode> preflight", InProgress)
    :2532  run_lf_full_calibration  "single-session LF native calibration preflight"
    :2455  run_lf_hip_min_max       "LF HIP MIN+MAX shared-geometry preflight"
    :2386  run_profile              "MATDOG native profile preflight"

terminal            mark_done() :4761
    publish_progress(self.total_steps, "completed", CalibrationStatus::Done)
```

Their emitted strings, with the label prefix of `publish_progress` :4744:

```text
Full     LF_LEG_STATE_MACHINE: single-session LF native calibration preflight   (current 0)
         ... 58 increments ...
         LF_LEG_STATE_MACHINE: completed                                        (current 58, Done)

HipPair  LF_HIP_M13_MIN_MAX: LF HIP MIN+MAX shared-geometry preflight           (current 0)
         ... 20 increments ...
         LF_HIP_M13_MIN_MAX: completed                                          (current 20, Done)

Single   <LABEL>: MATDOG native profile preflight                               (current 0)
         ... 16 increments ...
         <LABEL>: completed                                                     (current 16, Done)
```

**The filtering predicate used by regression tests (F6.2):**

```text
operational_increment(event) :=
        event.status == CalibrationStatus::InProgress
    AND event.current >= 1
    AND event.current <= expected_total

This excludes the step-zero preflight (current == 0) and the terminal completed
(status == Done). A failed run emits status == Failed and is likewise excluded.
```

```text
PRG-8  §14.2/§14.3/§14.4 are the OPERATIONAL INCREMENT oracles (58/20/16).
PRG-9  A separate external-stream oracle asserts the full 60/22/18 sequence including
       the two wrapper events, their exact strings and their current/status values.
PRG-10 Neither oracle may be described as the other (F6.3).
```

### 14.1 The externally emitted string form (R3-9.3)

`publish_progress` :4744-4758 emits

```rust
format!("{}: {phase}", self.profile.label)
```

so the **profile label is part of the observable string**, and it changes during a
session. The oracles below give the complete `label: phase` strings. Labels come from
`ContactProfile.label`, which `build_profile` :441 sets to
`"{LEG}_{JOINT}_M{motor}_{SIDE}"`, and which `lf_full_sequence_profile` :484 and
`lf_hip_sequence_profile` :465 override with their sentinel values.

### 14.2 `LfFullLegSession` — 58 operational increments, in order

Label transitions in this mode (`self.profile = …` at :2895, :3020, :3211, :3223):

```text
start                                  LF_LEG_STATE_MACHINE
:3211 measure pair -> minimum_profile  LF_UPPER_M12_MIN
:3223 measure pair -> maximum_profile  LF_UPPER_M12_MAX
:2895 after UpperHorizontal emission   LF_LOWER_M11_MIN
:3211 / :3223 for the Lower pair       LF_LOWER_M11_MIN then LF_LOWER_M11_MAX
:3211 / :3223 for the Hip pair         LF_HIP_M13_MIN_MAX (both sides)
:3020 after the ReturnHip emission     LF_LEG_STATE_MACHINE
```

| # | Emitted string |
|---:|---|
| 1 | `LF_LEG_STATE_MACHINE: Verify exact MATDOG ID set once` |
| 2 | `LF_LEG_STATE_MACHINE: Verified global torque OFF once at session entry` |
| 3 | `LF_LEG_STATE_MACHINE: Normalize every displaced MATDOG joint to q=0 with one uniform rule` |
| 4 | `LF_LEG_STATE_MACHINE: Create LF state machine from verified q=0 session entry` |
| 5 | `LF_LEG_STATE_MACHINE: Park LH upper M42 once for the complete LF session` |
| 6 | `LF_LEG_STATE_MACHINE: Prepare LF UPPER M12 once` |
| 7–13 | `LF_UPPER_M12_MIN: ` + the seven contact-side phases of §14.5 |
| 14–20 | `LF_UPPER_M12_MAX: ` + the seven contact-side phases of §14.5 |
| 21 | `LF_UPPER_M12_MAX: Transition M12 directly from MAX contact to horizontal hold` |
| 22 | `LF_LOWER_M11_MIN: Prepare LF LOWER M11 once` |
| 23–29 | `LF_LOWER_M11_MIN: ` + the seven contact-side phases |
| 30–36 | `LF_LOWER_M11_MAX: ` + the seven contact-side phases |
| 37 | `LF_LOWER_M11_MAX: Transition M11 directly from MAX contact to HIP parallel hold` |
| 38 | `LF_LOWER_M11_MAX: Prepare LF HIP M13 once` |
| 39–45 | `LF_HIP_M13_MIN_MAX: ` + the seven contact-side phases |
| 46–52 | `LF_HIP_M13_MIN_MAX: ` + the seven contact-side phases |
| 53 | `LF_HIP_M13_MIN_MAX: Derive endpoint and affine q0 diagnostics from all fine contacts` |
| 54 | `LF_HIP_M13_MIN_MAX: Move LF HIP M13 from MAX contact to URDF-derived staged q=0` |
| 55 | `LF_LEG_STATE_MACHINE: Move LF LOWER M11 to URDF-derived staged q=0 and hold` |
| 56 | `LF_LEG_STATE_MACHINE: Move LF UPPER M12 to URDF-derived staged q=0 while M11 holds` |
| 57 | `LF_LEG_STATE_MACHINE: Restore LH upper M42 once at end of LF calibration` |
| 58 | `LF_LEG_STATE_MACHINE: Final verified global torque OFF` |

```text
16 direct next_phase calls in run_lf_state_machine
+ 7 per contact side x 6 sides
= 58        matches total_steps = 58 :2531
```

Two label subtleties, both source-exact and both easy to get wrong:

```text
emission 21  fires BEFORE :2895 reassigns the profile, so it carries LF_UPPER_M12_MAX
emission 38  carries LF_LOWER_M11_MAX, because no profile assignment occurs between
             emission 37 and the Hip pair's :3211
emission 54  fires BEFORE :3020 restores the sentinel, so it carries LF_HIP_M13_MIN_MAX
```

### 14.3 `LfHipPairLegacy` — 20 operational increments, in order

The label is `LF_HIP_M13_MIN_MAX` for the whole session: both the MIN and the MAX
profiles come from `lf_hip_sequence_profile`, which sets the same sentinel label, so the
`self.profile = maximum_profile` assignment at :2789 does not change the prefix.

| # | Emitted string |
|---:|---|
| 1 | `LF_HIP_M13_MIN_MAX: Verify exact MATDOG ID set` |
| 2 | `LF_HIP_M13_MIN_MAX: Verified global torque OFF` |
| 3 | `LF_HIP_M13_MIN_MAX: Inspect restart-safe LF HIP sequence entry` |
| 4 | `LF_HIP_M13_MIN_MAX: Recover home-only joints to digital home` |
| 5 | `LF_HIP_M13_MIN_MAX: Set M12 horizontal and M11 parallel` |
| 6 | `LF_HIP_M13_MIN_MAX: Prime LF HIP M13 at digital home` |
| 7 | `LF_HIP_M13_MIN_MAX: LF HIP MIN moving-current baseline` |
| 8 | `LF_HIP_M13_MIN_MAX: LF HIP MIN coarse approach` |
| 9 | `LF_HIP_M13_MIN_MAX: LF HIP MIN backoff and recovery` |
| 10 | `LF_HIP_M13_MIN_MAX: LF HIP MIN fine repeat approach` |
| 11 | `LF_HIP_M13_MIN_MAX: LF HIP MIN repeatability` |
| 12 | `LF_HIP_M13_MIN_MAX: Return M13 home between MIN and MAX` |
| 13 | `LF_HIP_M13_MIN_MAX: LF HIP MAX moving-current baseline` |
| 14 | `LF_HIP_M13_MIN_MAX: LF HIP MAX coarse approach` |
| 15 | `LF_HIP_M13_MIN_MAX: LF HIP MAX backoff and recovery` |
| 16 | `LF_HIP_M13_MIN_MAX: LF HIP MAX fine repeat approach` |
| 17 | `LF_HIP_M13_MIN_MAX: LF HIP MAX repeatability` |
| 18 | `LF_HIP_M13_MIN_MAX: Return LF HIP M13 home` |
| 19 | `LF_HIP_M13_MIN_MAX: Restore M11, M12 and M42 to home` |
| 20 | `LF_HIP_M13_MIN_MAX: Final verified global torque OFF` |

```text
20 direct next_phase calls in run_lf_hip_min_max   matches total_steps = 20 :2454

Distinctive behavior this oracle pins:
  ONE coarse (scout None) + ONE fine (scout None) per side — not two scouted fine passes
  an absolute moving baseline per side
  a between-side M13 HOME return at emission 12
  contact_result.coarse_scout_tick == first_tick for both sides :2782 :2841
```

### 14.4 `LfSingleContactLegacy` — 16 operational increments, in order

The label is the selected profile's arm value for the whole session — one of
`LF_UPPER_M12_MIN`, `LF_UPPER_M12_MAX`, `LF_LOWER_M11_MIN`, `LF_LOWER_M11_MAX`. It never
changes: `run()` performs no profile reassignment.

| # | Emitted string (`<LABEL>` = the selected arm value) |
|---:|---|
| 1 | `<LABEL>: Verify exact MATDOG ID set` |
| 2 | `<LABEL>: Verified global torque OFF` |
| 3 | `<LABEL>: Inspect restart-safe profile entry` |
| 4 | `<LABEL>: Recover home-only joints to digital home` |
| 5 | `<LABEL>: Establish geometry prerequisites from restart-safe state` |
| 6 | `<LABEL>: Prime and return probing joint home` |
| 7 | `<LABEL>: Acquire moving-current baseline` |
| 8 | `<LABEL>: Coarse scouting approach — measurement discarded` |
| 9 | `<LABEL>: Backoff after coarse scout` |
| 10 | `<LABEL>: First fine metrology approach` |
| 11 | `<LABEL>: Backoff between identical fine approaches` |
| 12 | `<LABEL>: Second fine metrology approach` |
| 13 | `<LABEL>: Verify fine-to-fine repeatability` |
| 14 | `<LABEL>: Return probing joint home` |
| 15 | `<LABEL>: Restore prerequisite joints one at a time` |
| 16 | `<LABEL>: Final verified global torque OFF` |

```text
16 direct next_phase calls in run()                matches total_steps = 16 :2385

Distinctive behavior this oracle pins:
  restart-safe entry (no all-12 normalization)
  an absolute moving baseline, then a HOME return :3949
  ONE coarse scout (scout None) + TWO fine passes with scout Some(coarse) :2666 :2674
  LIFO restore_prerequisites :2690, then a post-restore re-prime and settle :2696-2712
```

### 14.5 The seven contact-side phases

Emitted by `measure_lf_contact_side_efficient` :3240-3273, used only by
`LfFullLegSession`.

**These seven build the label into the phase argument themselves**, e.g. :3240-3243

```rust
self.next_phase(&format!("{} moving baseline from current pose", self.profile.label))?;
```

and `publish_progress` then prefixes `"{label}: "` again. The externally emitted string
is therefore **doubled**, and the oracle records it exactly as emitted:

```text
1  <L>: <L> moving baseline from current pose
2  <L>: <L> coarse scouting pass
3  <L>: <L> coarse backoff
4  <L>: <L> fine metrology pass 1
5  <L>: <L> fine metrology backoff
6  <L>: <L> fine metrology pass 2
7  <L>: <L> fine-to-fine repeatability

where <L> is the then-current profile label.
```

Concretely, emission 7 of §14.2 is

```text
LF_UPPER_M12_MIN: LF_UPPER_M12_MIN moving baseline from current pose
```

```text
This doubling is immutable observable behavior. It must be preserved, not "cleaned up".
G4 compares against captured output, never against a reconstruction.
The 16 direct next_phase calls of Full, and all 20 HipPair and 16 Single calls, pass a
plain phase string and are therefore prefixed only once.
```

### 14.6 What the derivation must satisfy

```text
PRG-5  expected_total is derived by expanding the selected mode's grammar, then counting
       operational increments. Full -> 58, HipPair -> 20, Single -> 16.
       The complete external streams are 60 / 22 / 18 (§14.0).
PRG-6  A standalone numeric constant is NOT sufficient evidence (R3-9.6).
PRG-7  Regression captures the ACTUAL ordered emitted strings and compares them to the
       oracle for that mode (§16.5).
```

---

## 15. Non-LF offline specs — complete and inert, DESIGN ONLY

Per **R2-15**. These are complete enough to construct a `RawLegCalibrationSpec` and run
`validate_offline` against the D/W4 trust root. None is runtime-armable (ARM-1) and none
is convertible to `ArmableLfSessionSpec` (4.3, SEAL-5).

```text
DEFERRED FROM G3 (R3-13). This section is DESIGN-complete so the later non-LF offline
gate has an exact specification, but G3 implements none of it: no OfflineLegCalibrationSpec,
no importer, no trust-root verifier, no RF/RH/LH runtime code.
```

### 15.1 Canonical endpoint records — all 24

`declared` / `contact` / `Δ` in radians. Domain from parking_json (§3.2); policy from the
policy JSON; obstruction and depth from endpoint_profile.

| # | endpoint | declared | contact | Δ | domain | policy | obstruction | depth |
|---:|---|---:|---:|---:|---|---|---|---:|
| 0 | lf_hip_joint:min | −0.785398163397 | −0.803055986689 | −0.017657823292 | Diagnostic | UNRESOLVED | none | 0 |
| 1 | lf_hip_joint:max | +0.785398163397 | +0.789284248060 | +0.003886084663 | Diagnostic | PASS | none | 0 |
| 2 | lf_upper_leg_joint:min | −0.916297857297 | −0.909889226450 | +0.006408630847 | **Executable** | PASS | none | 1 |
| 3 | lf_upper_leg_joint:max | +2.138028333693 | +2.127120025868 | −0.010908307825 | **Executable** | PASS | PATH_OBSTRUCTION | 1 |
| 4 | lf_lower_leg_joint:min | −1.605702911835 | −1.606998273389 | −0.001295361554 | Diagnostic | UNRESOLVED | PATH_OBSTRUCTION | 2 |
| 5 | lf_lower_leg_joint:max | +0.654498469498 | +0.666361254258 | +0.011862784760 | Diagnostic | PASS | none | 2 |
| 6 | rf_hip_joint:min | −0.785398163397 | −0.789284248060 | −0.003886084663 | Diagnostic | PASS | none | 0 |
| 7 | rf_hip_joint:max | +0.785398163397 | +0.803055986689 | +0.017657823292 | Diagnostic | UNRESOLVED | none | 0 |
| 8 | rf_upper_leg_joint:min | −0.916297857297 | −0.909889226450 | +0.006408630847 | **Executable** | PASS | none | 1 |
| 9 | rf_upper_leg_joint:max | +2.138028333693 | +2.127120025868 | −0.010908307825 | **Executable** | PASS | PATH_OBSTRUCTION | 1 |
| 10 | rf_lower_leg_joint:min | −1.605702911835 | −1.606998273389 | −0.001295361554 | Diagnostic | UNRESOLVED | PATH_OBSTRUCTION | 2 |
| 11 | rf_lower_leg_joint:max | +0.654498469498 | +0.666361254258 | +0.011862784760 | Diagnostic | PASS | none | 2 |
| 12 | rh_hip_joint:min | −0.785398163397 | −0.788125240354 | −0.002727076957 | Diagnostic | PASS | none | 0 |
| 13 | rh_hip_joint:max | +0.785398163397 | +0.803055986689 | +0.017657823292 | Diagnostic | UNRESOLVED | none | 0 |
| 14 | rh_upper_leg_joint:min | −0.916297857297 | −0.909889226450 | +0.006408630847 | **Executable** | PASS | none | 1 |
| 15 | rh_upper_leg_joint:max | +2.138028333693 | +2.127120025868 | −0.010908307825 | **Executable** | PASS | none | 1 |
| 16 | rh_lower_leg_joint:min | −1.605702911835 | −1.606998273389 | −0.001295361554 | Diagnostic | UNRESOLVED | PATH_OBSTRUCTION | 2 |
| 17 | rh_lower_leg_joint:max | +0.654498469498 | +0.666361254258 | +0.011862784760 | Diagnostic | PASS | none | 2 |
| 18 | lh_hip_joint:min | −0.785398163397 | −0.803055986689 | −0.017657823292 | Diagnostic | UNRESOLVED | none | 0 |
| 19 | lh_hip_joint:max | +0.785398163397 | +0.788125240354 | +0.002727076957 | Diagnostic | PASS | none | 0 |
| 20 | lh_upper_leg_joint:min | −0.916297857297 | −0.909889226450 | +0.006408630847 | **Executable** | PASS | none | 1 |
| 21 | lh_upper_leg_joint:max | +2.138028333693 | +2.127120025868 | −0.010908307825 | **Executable** | PASS | none | 1 |
| 22 | lh_lower_leg_joint:min | −1.605702911835 | −1.606998273389 | −0.001295361554 | Diagnostic | UNRESOLVED | PATH_OBSTRUCTION | 2 |
| 23 | lh_lower_leg_joint:max | +0.654498469498 | +0.666361254258 | +0.011862784760 | Diagnostic | PASS | none | 2 |

All 24 carry `contact_status = GEOMETRIC_CONTACT_FOUND`, `target_source = GEOMETRIC_CONTACT`,
and `motion_authorization = NOT_GRANTED_OFFLINE_EVIDENCE_ONLY`.

### 15.2 Endpoint-scoped parking records — all 24

| # | endpoint | outcome | DOF | parking configuration (rad) | 1DOF cand. |
|---:|---|---|---:|---|---:|
| 0,1 | lf_hip:min,max | NOT_NEEDED | 0 | — | 0 |
| 2 | lf_upper:min | NOT_NEEDED | 0 | — | 0 |
| 3 | lf_upper:max | FEASIBLE_1DOF_PLAN_FOUND | 1 | lh_upper_leg_joint = 0.610865238198 | 33 |
| 4 | lf_lower:min | FEASIBLE_1DOF_PLAN_FOUND | 1 | lf_upper_leg_joint = 1.119919603363 | 7 |
| 5 | lf_lower:max | NOT_NEEDED | 0 | — | 0 |
| 6,7 | rf_hip:min,max | NOT_NEEDED | 0 | — | 0 |
| 8 | rf_upper:min | NOT_NEEDED | 0 | — | 0 |
| 9 | rf_upper:max | FEASIBLE_1DOF_PLAN_FOUND | 1 | rh_upper_leg_joint = 0.610865238198 | 33 |
| 10 | rf_lower:min | FEASIBLE_1DOF_PLAN_FOUND | 1 | rf_upper_leg_joint = 1.119919603363 | 7 |
| 11 | rf_lower:max | NOT_NEEDED | 0 | — | 0 |
| 12,13 | rh_hip:min,max | NOT_NEEDED | 0 | — | 0 |
| 14,15 | rh_upper:min,max | NOT_NEEDED | 0 | — | 0 |
| 16 | rh_lower:min | FEASIBLE_1DOF_PLAN_FOUND | 1 | rh_upper_leg_joint = 1.628973968528 | 7 |
| 17 | rh_lower:max | NOT_NEEDED | 0 | — | 0 |
| 18,19 | lh_hip:min,max | NOT_NEEDED | 0 | — | 0 |
| 20,21 | lh_upper:min,max | NOT_NEEDED | 0 | — | 0 |
| 22 | lh_lower:min | FEASIBLE_1DOF_PLAN_FOUND | 1 | lh_upper_leg_joint = 1.628973968528 | 7 |
| 23 | lh_lower:max | NOT_NEEDED | 0 | — | 0 |

```text
18 NOT_NEEDED + 6 FEASIBLE_1DOF_PLAN_FOUND = 24;  94 1DOF candidates;  0 2DOF.
```

### 15.3 Constructible raw offline specs (RF, RH, LH)

Each field below is an independent literal, not a shared constant and not derived from
another leg (**R2-15.2**). Joint identity, motor ID and direction are per-leg literals;
`limit_delta` is stated per endpoint.

**RF**

```text
leg                Rf
joints             Hip   rf_hip_joint        M23  direction -1
                   Upper rf_upper_leg_joint  M22  direction -1
                   Lower rf_lower_leg_joint  M21  direction +1
endpoint limit_delta (ticks, per endpoint, independent literals)
                   hip:min  -512   hip:max  +512
                   upper:min -597  upper:max +1394
                   lower:min -1047 lower:max +427
endpoint records   #6..#11 (§15.1), each bound per §3.2
target domains     hip both Diagnostic; upper both Executable; lower both Diagnostic
policy verdicts    hip:min PASS, hip:max UNRESOLVED, upper both PASS,
                   lower:min UNRESOLVED, lower:max PASS
calibration order  joint_order [Upper, Lower, Hip]; side_order [Min,Max] each
                   (offline analysis order; carries no motion authority)
prerequisite refs  endpoint-scoped only (§12): rf_lower:min references
                   rf_upper_leg_joint = 1.119919603363 rad, from record #10
parking refs       endpoint-scoped only: rf_upper:max references
                   rh_upper_leg_joint = 0.610865238198 rad, from record #9
                   NO session-wide parking hold is inferred (V5-2)
restore refs       none (offline object performs no motion)
participants       derived {21,22,23}; a parking dependency exists ONLY for endpoint #9
hardware evidence  NONE; no oracle accessor (ORA-2)
provenance         D/W4 manifest 4e7172d4…, endpoint dd8cb42c…, parking e561e7fb…,
                   policy 82f00a94…
```

**RH**

```text
leg                Rh
joints             Hip   rh_hip_joint        M33  direction +1
                   Upper rh_upper_leg_joint  M32  direction -1
                   Lower rh_lower_leg_joint  M31  direction +1
endpoint limit_delta  hip -512/+512, upper -597/+1394, lower -1047/+427 (independent)
endpoint records   #12..#17
target domains     hip both Diagnostic; upper both Executable; lower both Diagnostic
policy verdicts    hip:min PASS, hip:max UNRESOLVED, upper both PASS,
                   lower:min UNRESOLVED, lower:max PASS
calibration order  [Upper, Lower, Hip]; [Min,Max] each
prerequisite refs  rh_lower:min references rh_upper_leg_joint = 1.628973968528 rad (#16)
parking refs       NONE — every RH endpoint is NOT_NEEDED
restore refs       none
participants       derived {31,32,33} — no parking motor
hardware evidence  NONE
provenance         as RF
```

**LH**

```text
leg                Lh
joints             Hip   lh_hip_joint        M43  direction +1
                   Upper lh_upper_leg_joint  M42  direction +1
                   Lower lh_lower_leg_joint  M41  direction -1
endpoint limit_delta  hip -512/+512, upper -597/+1394, lower -1047/+427 (independent)
endpoint records   #18..#23
target domains     hip both Diagnostic; upper both Executable; lower both Diagnostic
policy verdicts    hip:min UNRESOLVED, hip:max PASS, upper both PASS,
                   lower:min UNRESOLVED, lower:max PASS
calibration order  [Upper, Lower, Hip]; [Min,Max] each
prerequisite refs  lh_lower:min references lh_upper_leg_joint = 1.628973968528 rad (#22)
parking refs       NONE
restore refs       none
participants       derived {41,42,43} — no parking motor
hardware evidence  NONE
provenance         as RF

M42 is LH's own upper joint AND the motor LF parks. Participant derivation (§6.4)
resolves this per spec. Phase 2A is SINGLE-SESSION ONLY (§19); no concurrency claim.
```

### 15.4 Front/hind asymmetry is measured, not conventional

```text
hip:min contact  LF −0.803056  RF −0.789284  RH −0.788125  LH −0.803056
hip:max contact  LF +0.789284  RF +0.803056  RH +0.803056  LH +0.788125
      -> four distinct per-leg hip endpoint pairs                       (D-4 corrected)

lower:min parking  front 1.119920 rad (64.1667 deg) | hind 1.628974 rad (93.3333 deg)
upper:max parking  front REQUIRED (contralateral hind upper, 35.0000 deg) | hind NOT_NEEDED
```

Declared URDF limits are numerically equal across legs; measured contacts and parking
plans are not. Numerical equality is never derivation authority (**R2-15.2**).

---

## 16. CI migration — mechanically feasible with the existing harness

Per **R2-14** as corrected by **R3-7** and **R3-8**. This section is self-contained:
it does not import rows "as written in revision 1" (E-3).

### 16.0 What the existing harness actually provides

```text
AVAILABLE — all of these EXIST TODAY and are all RETAINED (F7.6)
            inline python3 source-shape gate                  matdog-native-calibrator-check.yml :47-
            rustfmt --check over an explicit file list        matdog-native-calibrator-check.yml :~205
            cargo test --package st3215                       matdog-native-calibrator-check.yml :220
            python3 -m unittest (headless runner, lf_profile) matdog-native-calibrator-check.yml :226
            Station release build                             matdog-native-calibrator-check.yml :245
            observer boundary source-shape checks             matdog-native-observer-check.yml :39
            EEPROM forbidden-token checks                     inside the calibrator gate (K-1)
            standard shell / python3 / cargo

NOT AVAILABLE (and NOT authorized to add — R3-7.2)
            trybuild, compiletest, compile-fail fixtures, negative-trait infrastructure
```

```text
G-5 correction: revision 2.1's inventory omitted the Python unittest step and the
Station release build. Both exist today and are retained; §20's prohibition on weakening
CI covers them.
```

### 16.0b Exact scan roots — two distinct Rust scopes (F7.1, F7.2, F7.4, AG-4)

`software/drivers/st3215/src/auto_calibrate/` also contains **unrelated generic robot
calibrators that legitimately use EEPROM functionality**. Applying MATDOG's
forbidden-token scan to the whole directory would false-fail on pre-existing non-MATDOG
code, verified today:

```text
calibrator.rs   :5    use crate::protocol::{RamRegister, EepromRegister};
                :389  RamRegister::Lock.address()
                :424  EepromRegister::Offset.address()      <- a real EEPROM Offset write
elrobot.rs      :4    use crate::protocol::EepromRegister;
so101.rs        :4    use crate::protocol::{RamRegister, EepromRegister};
```

The scan therefore uses two clearly different scopes.

```text
SCOPE A — MATDOG IMPLEMENTATION SAFETY SCOPE
    software/drivers/st3215/src/auto_calibrate/matdog.rs
    software/drivers/st3215/src/auto_calibrate/matdog_*.rs
    software/drivers/st3215/src/auto_calibrate/matdog/**/*.rs

    Every new G3 MATDOG Rust implementation module MUST live inside this `matdog`
    namespace. A new file there is picked up automatically — no module-graph parser.

    Namespace-escape guard: the gate lists all *.rs RECURSIVELY under
    software/drivers/st3215/src/auto_calibrate/ (not only its top level), subtracts
    SCOPE A and the known non-MATDOG set {calibrator.rs, elrobot.rs, so101.rs, mod.rs},
    and FAILS if the remainder is non-empty. A MATDOG implementation file placed outside
    the namespace — including in a future nested subdirectory — therefore fails the gate
    rather than silently escaping the scan.

    matdog_test.rs
        INCLUDED in rustfmt and cargo test;
        INCLUDED in structural and runtime tests;
        EXCLUDED from forbidden-token LITERAL scanning ONLY, because it deliberately
          contains those strings (e.g. the literals asserted by
          canonical_matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path).
        The literal-scan exclusion list is itself asserted: exactly this one file, by
          name; the gate fails if the list grows without review.

SCOPE B — GLOBAL BRIDGE / PORT SCOPE
    software/drivers/st3215/src/auto_calibrate/mod.rs
    software/drivers/st3215/src/port.rs

    PURPOSE: verify the existing MATDOG driver/port bridge and armed port authority
    boundary. SCOPE B is NOT part of the CG-1 GoalPosition construction count (CI-C1.4).

    Exact production anchors, confirmed present in current source:
        mod.rs    `mod matdog;`                              (1)
                  `matdog_calibrator_is_armed`               (1)
                  `matdog_armed_ram_write_allowed`           (1)
                  `matdog_ram_write_allowed_for_arm_value`   (1)
        port.rs   `matdog_command_allowed_with`              (4)
                  `matdog_armed_command_allowed`             (5)
                  `native MATDOG profile arming is active`   (1)
    These are exactly the existing K-6 / K-7 assertions, retained verbatim.

    #[cfg(test)] EXCLUSION. port.rs contains four RamRegister::GoalPosition occurrences
    (:2016, :2066, :2076, :2086) and ALL FOUR are inside its #[cfg(test)] block at :1838.
    They are test fixtures and MUST NOT affect CG-1. Production-location counts exclude
    #[cfg(test)] modules/blocks structurally.
    mod.rs contains no GoalPosition occurrence at all.

EXPLICITLY OUT OF THE MATDOG FORBIDDEN-TOKEN SCAN
    calibrator.rs, elrobot.rs, so101.rs
    They are unrelated generic calibrators. They are covered only if a future,
    separately reviewed invariant explicitly requires it (AG-4).

PYTHON SCOPE 1 — DYNAMIC-IMPORT SAFETY ROOT (broad, conservative, mechanical)
    tools/matdog/*.py     -- all eight files, production and test:
        matdog_headless_auto_calibrate.py     test_matdog_headless_auto_calibrate.py
        matdog_lf_profile.py                  test_matdog_lf_profile.py
        matdog_native_observer_contract.py    test_matdog_native_observer_contract.py
        matdog_v42_pinned_launcher.py         test_matdog_v42_pinned_launcher.py

    This broad scope checks ONLY the dynamic-code/import mechanisms the contract bans:
        __import__      importlib      exec(      eval(
    Any occurrence FAILS CLOSED and requires separate review (F7.5).
    No dynamic-import resolution is claimed.
    This scope carries NO forbidden-policy tokens.

PYTHON SCOPE 2 — K-11 OBSERVER POLICY ROOT (narrow, forbidden tokens)
    tools/matdog/matdog_native_observer_contract.py     -- THIS FILE ONLY

    This is literally what the existing observer workflow enforces:
        found = [token for token in forbidden_observer if token in observer]
    where `observer` is that single file. The forbidden list (`M42`, `HOME_TICK`,
    `EXPECTED_ACTIVE_TORQUE_LIMIT`, ...) legitimately appears in the headless runner and
    in tests, so applying K-11 tree-wide would false-fail correct files (CI-C2).

    The observer REQUIRED-token check additionally spans
        matdog_native_observer_contract.py + matdog_v42_pinned_launcher.py
    exactly as the existing workflow does (`combined = observer + launcher`).

    K-11 is NOT applied to runner files, test files, fixtures or unrelated MATDOG Python
    utilities. Those remain covered by SCOPE 1, by their own unit tests, and by their
    existing checks. The whole tools/matdog tree is NOT "the observer boundary".
```

```text
CI-RULE-1  A normal compiling unit test can NEVER prove that some other code fails to
           compile. No row below claims that (R3-7.1).
CI-RULE-2  Primary invariant = compiler-enforced API/type signatures.
           Secondary evidence = runtime tests for behavior that can actually execute,
           plus existing-tool source-shape scans.
CI-RULE-3  No row claims more than the check establishes (R3-7.6).
```

### 16.1 Retained verbatim

| # | Assertion | Invariant |
|---|---|---|
| K-1 | `forbidden_source` EEPROM/register list (`EepromRegister`, `RamRegister::Lock`, `ST3215Request::`, `reg_write: Some`, `reset: Some`, `reset_calibration: Some`, `freeze_calibration: Some`, `action: Some`, `Offset.address`) | EEPROM unreachable — **verbatim** |
| K-2 | `forbidden_source` `matdog_v2`, `LF_FIXED_SCALE_REJECT` | stale-symbol guard |
| K-3 | `forbidden_other` `write.motor_id != 12`, `matdog_pilot_is_armed`, `matdog_armed_motor_ids` | stale-symbol guard |
| K-4 | `HOME_TICK = 2048` | digital zero |
| K-5 | `GUARD_OVERSHOOT_TICKS = 64` | guard margin, non-spec-tunable |
| K-6 | `required_port` tokens | port-level motion authority |
| K-7 | `required_module` tokens | driver↔port bridge |
| K-8 | `ram_write_allowed_for_profile`, `global_torque_off_verified`, `MAX_TELEMETRY_AGE` | RAM gate, terminal safety, liveness |
| K-9 | `matdog_v2*` stale-file guard | anti-duplication |
| K-10 | affine stages; `fixed_scale.estimated_zero_tick` forbidden as a staging target | staging authority |
| K-11 | observer forbidden-token list (`M42`, `HOME_TICK`, `current_step`, `calibration.phase`, `CONTROLLED_GOAL_CORRIDORS`, `CONTROLLED_POSITION_CORRIDORS`, `NONPARTICIPATING_MOTOR_IDS`, `EXPECTED_ACTIVE_TORQUE_LIMIT`), applied to **§16.0b PYTHON SCOPE 2 only** — `tools/matdog/matdog_native_observer_contract.py`. The required-token check additionally spans that file plus `matdog_v42_pinned_launcher.py` | observer boundary — **verbatim**, exact scope per CI-C2 |
| K-12 | `PROBE_TRACKING_ERROR_FLOOR_TICKS`, `probe_tracking_error_limit` | tracking floor |
| K-13 | `q0_affine`, `estimated_q0_tick={record['q0_affine']}` | Python serializer stages affine q0 |

### 16.2 Rows withdrawn from revision 2

| Withdrawn | Reason | Replacement |
|---|---|---|
| T-3 | port sees only serialized bytes; typed provenance is erased | already withdrawn in revision 2; stays withdrawn |
| M-16r (as worded) | would pin a resolver set that excluded the four supported LF single-contact modes | **M-16s** below (R3-1) |
| M-17r | claimed an ordinary test proves ill-typed calls fail to compile, and that no `From`/`Into`/`TryFrom` exists | **M-17s** below: signature + source-shape scan + runtime rejection (R3-7.4) |
| M-18r (as worded) | froze an intent enum derived before TABLE A was corrected | **M-18s** below, keyed to §8.2 |
| M-25r | required a Rust module-graph parser | **M-25s** below: directory-tree scan (R3-8.2) |
| M-26r | required an import resolver | **M-26s** below: file-tree scan + fail-closed on dynamic import (R3-8.3, R3-8.4) |
| M-27 | claimed runtime proof of non-constructibility / absence of `Clone`/`Copy` / reuse-after-move | **M-27s** below: raw-sink call-site enumeration (R3-5.3) |
| M-28 | referenced A2, which is not a write | **M-28s** below, keyed to W1..W23 |
| M-20r offline-importer rows | G3 does not implement the importer | **deferred to the later non-LF offline gate** (E-4, R3-13) |
| **M-19s** | required a runtime eligibility test over a compiled 24-endpoint Geometry dataset, contradicting the documented-only predicate and G3's exclusion of Geometry data | **REMOVED from G3** entirely; deferred with the importer/offline gate (F9) |
| M-16s (as worded) | runtime enumeration alone cannot prove "every other string" is rejected | **M-16s** strengthened below with a structural match-arm check (F7) |
| M-27s (as worded) | enumerated callers of two named sinks; a third or direct `RamRegister::GoalPosition` construction could evade it | **M-27s** strengthened below to a three-stage construction scan (F4.3) |
| M-28s (as worded) | checked only admitted/neighbour values | **M-28s** expanded below into the F8 rejection matrix |
| M-25s / M-26s (as worded) | scan roots not concretely defined | roots now defined in §16.0b (F7.1–F7.5) |

### 16.3 Rows for G3

| # | Assertion | Mechanism | Layer | Invariant |
|---|---|---|---|---|
| M-16s | (a) runtime test: the resolver accepts exactly the six LF tokens of §5.2 and rejects all eighteen RF/RH/LH tokens plus malformed input; (b) **structural** source-shape check: the resolver's match arms are exactly those six literals plus a default-reject arm | runtime test + python scan | calibrator | ARM-1, ARM-2r |
| M-16t | `LF_HIP_M13_MIN` and `LF_HIP_M13_MAX` remain **recognized** as profile data and remain **hardware-blocked** | runtime test | calibrator | ARM-3r |
| M-17s | (a) the engine entry signature names `ArmableLfSessionSpec` and nothing else — compiler-enforced; (b) source-shape scan finds no `impl From<…> for ArmableLfSessionSpec`, no `impl TryFrom`, no `into_armable`; (c) runtime tests prove `validate_lf_v25` rejects each LFID-1..LFID-9 violation individually | signature + python scan + runtime tests | calibrator | §4.3, §4.4, R3-7.4 |
| M-18s | The engine motion-operation set contains **exactly** the twelve methods of §8.2 — no more, no fewer — and contains no direct-geometry operation | source-shape scan of the engine impl block | calibrator | §8.2, R2-6, F1 |
| M-25s | Forbidden-token scan input is **§16.0b SCOPE A** listed mechanically; a new `matdog*` file is included automatically; the literal-scan exclusion list contains exactly `matdog_test.rs` and the gate fails if it grows; the gate fails if the listing is empty; the **namespace-escape guard** fails if any unclassified `.rs` appears in `auto_calibrate/`. `calibrator.rs`, `elrobot.rs` and `so101.rs` are explicitly out of scope | `find`/glob in the existing inline scan | build/CI | R3-8.2, F7.2, AG-4 |
| M-26s | Scan scope is **§16.0b PYTHON SCOPE 1** — the broad dynamic-import safety root `tools/matdog/*.py` (all eight files). It checks ONLY `__import__`, `importlib`, `exec(` and `eval(`; any occurrence **fails closed** and requires review; no dynamic-import resolution is claimed. M-26s carries **no** forbidden-policy tokens — those belong to K-11 / PYTHON SCOPE 2 alone | python source-shape scan | build/CI | R3-8.3, R3-8.4, F7.4/F7.5, CI-C2 |
| M-27s | **Stages 1-6 run over §16.0b SCOPE A PRODUCTION ONLY** — `matdog_test.rs`, every `#[cfg(test)]` module/block and any demonstrably test-only code are structurally excluded. (1) Enumerate GoalPosition **write-construction sites** only: a call passing `RamRegister::GoalPosition` **together with a value payload** to a RAM-write helper. Bare enum references in allowlists, `matches!`, match arms and register-policy predicates are **NOT** construction sites. The required G3 production count is exactly **ONE** raw constructor/emitter. (2) Identify that one raw constructor/emitter. (3) Enumerate its DIRECT production callers — expected exactly **TWO** private policy writers, the startup-home writer and the normal armed-goal writer. (4) Enumerate callers of those two policy writers — their union must equal exactly the **TWELVE §8.2 engine operations**, with no additional production caller. (5) Startup-home policy writer covers only **W1..W3**. (6) Normal armed-goal policy writer covers only **W4..W23**. Any second raw construction site, direct bypass, third policy writer, third raw caller or unreviewed policy-writer caller **FAILS**. **STAGE 7 — SCOPE B, SEPARATE.** SCOPE B is **not** included in the GoalPosition construction occurrence/count logic. Separately retain the exact §16.0b production anchors: `mod.rs` — `mod matdog;`, `matdog_calibrator_is_armed`, `matdog_armed_ram_write_allowed`, `matdog_ram_write_allowed_for_arm_value`; `port.rs` — `matdog_command_allowed_with`, `matdog_armed_command_allowed`, `native MATDOG profile arming is active`; and retain the existing K-6/K-7 assertions. `port.rs`'s four GoalPosition occurrences inside its `#[cfg(test)]` block have **ZERO** effect on CG-1 / M-27s production counts. The gate must NOT assert that the twelve operations directly call the raw constructor | python source-shape scan | calibrator | CG-1..CG-8, CI-C1, AG-4 |
| M-28s | **F8 rejection matrix.** For every §8.2 operation and every W-row it covers, runtime tests exercise: admitted immutable value; wrong mode; wrong context step / grammar node; wrong motor; wrong historical/reference pose; wrong coarse/fine step; stale or wrong-motor observation where applicable; wrong-session or wrong-joint staged evidence; guard violation; unsigned-range violation. Only mechanically executable cases are claimed | runtime tests | calibrator | §7, §8.4, §8.6, F8 |
| M-29 | `probe_advance_step` is bounded by the **guard**, not the acceptance corridor: an early coarse step far from the endpoint is admitted; a step past the guard is refused | runtime test | calibrator | C-1 |
| M-30 | Full-mode startup: normalization iterates all 12 motors and the effective predicate is `distance > 10 \|\| speed > 4`, including the M11-at-HOME+11 case and a displaced M21 | runtime test | calibrator | §10 |
| M-30b | HipPair/Single startup does **not** run all-12 normalization and preserves restart-safe profile entry | runtime test | calibrator | §10.1, R3-11 |
| M-31 | Full-session Hip MAX holds M12 at 1024; 967 never appears in a full-session hold | runtime test | calibrator | PRQ-4 |
| M-32 | Staged-return timing: remove-before-move and re-add-after-arrival for M11 and M12 | runtime test | calibrator | §11.3 |
| M-33 | Cleanup timing: holds still present when the torque-OFF sync-write is issued; cleared only after all 12 verify OFF | runtime test | calibrator | §11.4 |
| M-34 | Fine-pass scout argument matches the mode: Full and Single use `Some(coarse)`, **HipPair uses `None`** | runtime test | calibrator | §6.2, W10/W11 |
| M-35 | **Operational increments**: filter the captured stream with the §14.0 predicate and compare to §14.2 / §14.3 / §14.4 — 58 / 20 / 16, including the doubled contact-side form of §14.5 | runtime test over simulated telemetry | calibrator | §14, R3-9, F6 |
| M-35b | **Complete external stream**: compare the UNFILTERED captured stream to the 60 / 22 / 18 oracle, including the step-zero preflight string with `current == 0` and the terminal `completed` event with `status == Done` | runtime test over simulated telemetry | calibrator | §14.0, F6 |
| M-36 | Terminal safety independence: verified global torque-off, hard abort and operator stop write only `TorqueEnable` and are reachable without any §8.2 operation; a stop-pressure failure falls through to verified global torque OFF | runtime test | calibrator | §8.5, R3-5.4/5.5 |
| M-37 | Entry context: `home_normalization_prime` is reachable only in `EntryStep::{FullNormalization,LegacyProfileEntry}{Prime}` with torque OFF; `home_reassert_torque_on` only in `LegacyProfileEntry{Reassert}` with torque ON; neither is reachable from a `Session` context | runtime test | calibrator | §8.0, F1, F2 |

### 16.4 The ten historical test names

Retained individually. **Nine keep their exact names; only T-1 is renamed** (E-1
corrects revision 2's "eight preserved, two renamed").

| # | Name | Disposition |
|---|---|---|
| T-1 | `profile_table_covers_exactly_24_unique_contacts` → `endpoint_records_cover_exactly_24_unique_contacts` | renamed; same enumeration and uniqueness assertion, additionally cross-checked against `canonical_endpoint_index` 0..=23 |
| T-2 | `front_lower_restore_order_keeps_rear_parking_until_active_leg_is_home` | name unchanged; re-pointed at the production LIFO restore (removes the test-only mirror) |
| T-3 | `armed_ram_gate_restricts_registers_values_motors_and_goal_windows` | name unchanged; unchanged assertion |
| T-4 | `canonical_matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path` | name unchanged; scan target becomes the M-25s **SCOPE A** listing, not the whole `auto_calibrate/` directory (AG-4) |
| T-5 | `accepted_endpoint_q0_is_used_only_for_transactional_staging` | name unchanged; additionally asserts staging occurs only via `staged_affine_q0` |
| T-6 | `full_lf_final_order_stages_m13_m11_m12_then_restores_m42` | name unchanged; order derived from the grammar, same asserted sequence |
| T-7 | `historical_contacts_use_affine_and_uniform_witness_freeze_gate` | name unchanged; invokes the production evidence function |
| T-8 | `v24_m13_fine_tracking_uses_detector_consistent_global_floor` | name unchanged; unchanged assertion |
| T-9 | `lf_contact_witness_gate_is_uniform_and_rejects_v24_m12_cable_obstruction` | name unchanged; witness obtained from `lf_v25_oracle()` keyed by `JointKind` |
| T-10 | `affine_gate_accepts_real_span_while_fixed_scale_stays_diagnostic` | name unchanged; plus the three-joint staging proof |

Additional retained rows, restated self-containedly (E-3): the affine × witness
production-path truth table; the `MATDOG_LF_PROFILE_V1` byte-equality golden record; the
three-joint staged-return proof; and the Python affine-serializer checks of K-13.

### 16.5 Three-mode regression strategy (R3-9, R2-13.5)

```text
For each mode in { LfFullLegSession, LfHipPairLegacy, LfSingleContactLegacy }:
  1. Drive the engine over simulated telemetry through a complete session of that mode,
     using the existing test harness only.
  2. Record the ACTUAL ordered list of emitted progress events
     (string, current, status) — UNFILTERED.
  3. COMPLETE EXTERNAL STREAM (M-35b):
       assert the unfiltered length equals 60 / 22 / 18 respectively;
       assert event[0] is the mode's step-zero preflight string with current == 0
         and status == InProgress;
       assert the last event is "<label>: completed" with current == total and
         status == Done.
  4. OPERATIONAL INCREMENTS (M-35):
       apply the §14.0 filter (status == InProgress && 1 <= current <= expected_total);
       assert the filtered length equals 58 / 20 / 16 respectively;
       assert the filtered list equals the §14.2 / §14.3 / §14.4 oracle, in order,
         including label prefixes and the doubled contact-side form.
  5. Assert expected_total derived from the expanded grammar equals the filtered length.
  6. Assert DONE is refused when any operation is skipped.
For LfSingleContactLegacy, repeat for all four supported arm values.
```

### 16.6 Deferred to the later non-LF offline gate (R3-13, E-4)

```text
provenance per-field mutation rejection (the IMP-0..IMP-23 suite)
ExpectedGeometryV5DW4 trust-root verification tests
OfflineLegCalibrationSpec construction/validation tests
RF/RH/LH dataset tests
the 24-endpoint Geometry eligibility/denial test (former M-19s)      <-- F9

These are DESIGN-complete in §3 and §15 but are NOT part of the G3 CI surface, because
G3 does not implement the importer or the offline spec type.
```

### 16.7 Process rules

```text
The CI migration is a SEPARATE reviewed diff from the runtime change.
No assertion is deleted without a replacement in the same diff.
No forbidden-token list is shortened to accommodate a refactor.
Launcher SHA pins are updated only after the new bytes are reviewed (D6).
```

---

## 17. Global safety ownership

Per **R2-16**. M7 remains **CLOSED**; this section restates it and corrects D-3.

```text
C — global ST3215 safety (never spec-parameterized, never weakenable by any spec)
    topology and identity        MATDOG_MOTOR_IDS, exact-set check, dispatch
    bus identity                 EXPECTED_BUS_SERIAL
    frame                        HOME_TICK 2048, 4096 ticks/rev, MAX_ANGLE_STEP 4095,
                                 MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS 96
    unsigned arithmetic          all wrap helpers; wrap math stays local
    envelope                     TORQUE_LIMIT 500, GOAL_SPEED 160, ACCELERATION 8
    guard                        GUARD_OVERSHOOT_TICKS 64
    protection                   HARD_CURRENT_ABORT_RAW 200, 70 C thermal contract,
                                 port.rs thermal supervisor, forced torque OFF
    liveness                     5 s / 2 s / 3 s
    supervision                  MotorObservation, per-frame safety, drift cap 16
    authority                    arming env, RAM allowlist, armed goal gate, port gate,
                                 isolated-HIP block
    Station / serial             Station sole serial owner; process ownership; identity
    observer                     external observer must not reconstruct motion policy
    launcher                     pin-ENFORCEMENT MECHANISM
    terminal                     global verified torque OFF, abort path, operator stop
    command scoping              app_start_id / nonce / counter
    ID 115 (C aspect)            EEPROM separation: the freeze path is a SEPARATE binary,
                                 requiring --commit LF_FREEZE_COMMIT, with Station stopped

D — historical LF-only evidence
    LF witness ticks + tolerance 24 (via the oracle, §13)
    LF arm sentinels (opaque historical)
    LF historical poses 341 / 967 / 1024 / -990
    the witness TERM of the freeze gate
    MATDOG_LF_PROFILE_V1 name and bytes (OPEN-3)
    LF total operation count 58
    the specific reviewed launcher pin VALUES (release facts, not spec data)
    robot-dog approved LF angles; physical evidence sha 6eae3201…
    ID 115 (D aspect)            the LF-specific motor list LF_MOTORS [11,12,13] and the
                                 LF staging/evidence content of the freeze transaction

A — generic engine behavior      grammar, detector, search, corridors, backoff, restore
                                 discipline, role derivation, diagnostics, progress
                                 derivation
B — per-leg data                 joint identities, motor IDs, directions, per-endpoint
                                 limits, calibration order, prerequisite/parking/restore
                                 references, provenance references
```

---

## 18. Contact semantics (CLOSED — unchanged)

Per **R2-17**. Review #2 found no regression; this text is unchanged in substance.

```text
FINAL ACCEPTED CONTACT -> STOP ADVANCING IMMEDIATELY
```

```text
Stage 1  ContactState :: FreeMotion | ContactSuspected | ContactConfirmed
                       | EarlyStall | HardAbort
Stage 2  ContactAcceptance :: Accepted { position }   -> stop_pressure(); return
                            | FrictionPlateauContinue -> bounded pre-final continuation

INV-D1a  Accepted is always followed immediately by stop_pressure() and return.
INV-D1b  FrictionPlateauContinue occurs ONLY on a fine pass (scout is Some).
INV-D1c  It is bounded by the mechanical guard, the adaptive acceptance corridor and the
         coarse scout depth — itself a real prior hardware contact.
INV-D1d  Thresholds unchanged, including FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS = 8.
```

Outside-predicted-band contact in future hardware phases:

```text
stop_pressure immediately -> controlled verified backoff -> record the disagreement
-> fail closed -> verified cleanup / global torque OFF

Never continue probing toward a historical LF span. The only preserved continued-probing
behavior is the exact immutable LF pre-final fine-pass friction/chamfer qualification.
Any same-leg anchored prediction intersects the existing corridor, never widens it, and
fails before motion on an empty intersection.
```

### 18.1 Oracle hierarchy and q0 separation (unchanged)

```text
1  behavioral regression oracle   immutable release f87dd1fb… + its tests
2  runtime LF contact witness     lf_v25_oracle() ticks + tolerance 24
3  physical hardware evidence     6eae3201… (and the generated reconciliation d7fa04e2…)
4  README tick tables             documentary only

physical encoder scale | q0 offset | geometry/endstop mismatch | fitted affine
remain four distinct recorded quantities. The fitted affine never erases the raw
disagreement, including the three DISAGREES rows of §2.
```

---

## 19. Single-session only

Per **R2-18**.

```text
SES-1  The M42 participant/parking role is valid only inside ONE selected session.
SES-2  Phase 2A makes NO concurrency claim.
SES-3  No concurrent multi-leg session support is authorized.
SES-4  Future concurrency requires separate resource arbitration and architecture review.
```

Current startup signals a previous task to stop without proving completion before
another begins, so no concurrency-safety property may be inferred from today's behavior.

---

## 20. G3 scope and prohibitions

Scope is tightened per **R3-13**: G3 implements only what LF preservation requires.

```text
G3 IS        LF generic-engine extraction and LF behavioral preservation. Nothing else.

G3 IMPLEMENTS (LF-only surface — F10.1)
             ONE LegSessionStateMachine with the §6 grammar
             the three exact LF legacy modes                                  (§6.2)
             the SEALED ArmableLfSessionSpec brand + validate_lf_v25          (§4.4, §4.5)
             the LF V25 oracle                                                (§13)
             the pre-session Entry context and the Session context            (§8.0)
             the TWELVE intent-specific immediate motion operations           (§8.2)
             the private GoalPosition construction boundary + reviewed callers (§8.3)
             LF behavioral regression tests preserving §7, §9, §10, §11, §14
             the required CI migration                                        (§16)

G3 DOES NOT IMPLEMENT (design-complete here; deferred — F10.2, R3-13)
             OfflineLegCalibrationSpec  (prefer NOT to implement unused offline
                 types; only inert design scaffolding if strictly required — F10.3)
             the Geometry V5 importer / parser
             the TrustedGeometryManifest / ExpectedGeometryV5DW4 verifier
             any RF/RH/LH engine or runtime code
             DirectGeometryTarget
             non-LF hardware calibration
             concurrent sessions
             the 24-endpoint Geometry eligibility/denial test (former M-19s)  (F9)

G3 MUST NOT  implement any Geometry V5 parser, importer or trust-root verifier (R3-12.6)
             implement DirectGeometryTarget or any direct-geometry command path (R2-6, R3-4.4)
             introduce ANY motion-authority token, capability object, nonce, epoch or
               session-token machinery                                        (R3-3)
             accept a caller-selected motor, tick, pose, step, scout flag or evidence
               record as a motion-operation argument                          (F3, DER-3)
             merge the legacy torque-OFF HOME prime with the torque-ON HOME
               reassertion                                                    (F1.3)
             read any motion-bearing field from a raw spec after branding     (F5, BRAND-1)
             implement a runtime Geometry eligibility/dataset test            (F9)
             remove or deprecate any supported LF runtime mode, in particular
               LF_UPPER_M12_MIN/MAX and LF_LOWER_M11_MIN/MAX                  (R3-1)
             admit isolated LF Hip to hardware                                (ARM-3r)
             change any detector threshold or constant value
             remove, retune or generalize the friction/chamfer qualification  (D1, R3-15.2)
             create per-leg engines, validators or corridors
             retain run_profile as a second physical executor                 (R3-2.2)
             register any RF/RH/LH value in the runtime arm resolver          (ARM-1)
             let an offline object reach the engine or the sealed brand       (R3-6.5)
             let hardware evidence alter a V5 target domain                   (R1)
             let an external PASS create eligibility                          (R1)
             treat UnsignedTick as command authority                          (AUTH-1)
             apply Full-mode all-12 normalization to HipPair or Single        (R3-11)
             reduce Full-mode all-12 normalization or its 10-tick predicate   (R2-7)
             substitute 967 into a full-session Hip MAX hold                  (R2-8.4)
             use scout = Some in a HipPair fine pass                          (§6.2, W11)
             normalize the three mode traces for architectural elegance       (R3-2.4)
             weaken or delete any CI gate, or claim an unsupported negative
               compile-time test                                              (D5, R3-7)
             re-hash any SHA pin mechanically                                 (D6)
             depend on matdog_leg_fk_live.py or the VISUAL_ZERO / DIGITAL_ZERO
               status contract                                                (D9/R13.3)
             change the established digital zero for any reason
             claim or rely on concurrent multi-leg sessions                   (R2-18)
             touch main, robot-dog or the historical RF worktree
             perform any hardware, serial or EEPROM action
```

---

## 21. Residual items

```text
OPEN-1..OPEN-5   all resolved (R1, revision-1 decision, R2-6, R2-18).
NOTE-A           resolved by R2-6: predicate documented, capability NOT implemented.
NOTE-B           resolved by R2-18: single-session only.
D-1..D-4         revision-2 documentation defects, corrected in §16.4, §21, §17, §15.4.
E-1..E-4         revision-3 documentation defects, corrected as below.
```

D-2 correction: OPEN-2 (`"LF_LEG_STATE_MACHINE"` remains an opaque historical
compatibility sentinel; no non-LF runtime sentinels in Phase 2A) is resolved by the
**OPEN-2 ruling of the revision-1 review decision**, not by ruling R2 of that document.

E-corrections applied in revision 2.1:

```text
E-1  §16.4 now states nine test names preserved and only T-1 renamed
     (revision 2 said "eight preserved, two renamed").
E-2  §9 Preflight row now distinguishes errors before session creation from errors
     after inspect_lf_native_session_entry(), which occur with a live session.
E-3  §16 is self-contained; it no longer imports rows "as written in revision 1".
E-4  Offline-importer CI rows are assigned to the later non-LF gate (§16.6).
     The false Rust wording "a MotionGrant cannot be stored" disappears with the
     construct itself (R3-3).
```

No open architectural question remains in this contract. Items requiring a future
separate gate, and explicitly out of Phase 2A / G3 scope:

```text
direct Geometry V5 motion authority                  (R2-6.4)
concurrent multi-leg sessions                        (R2-18.4)
a generic non-LF machine record / schema             (OPEN-3)
non-LF hardware validation                           (future RF -> RH -> LH phases)
the Geometry V5 importer and trust-root verifier     (R3-12.6)
OfflineLegCalibrationSpec and RF/RH/LH runtime code  (R3-13)
```

---

## 22. Executor position

```text
P2A-G2 CONTRACT: REVISION 2.2d COMPLETE
IMPLEMENTATION PERFORMED: NONE
R1 .. R14, R2-1 .. R2-18, R3-1 .. R3-15, F1 .. F10, AG-1 .. AG-5, CG-1 .. CG-8,
CI-C1 .. CI-C3,
D-1 .. D-4, E-1 .. E-4 and G-1 .. G-6: all addressed in the normative body above.
```

Revision 2.2 / 2.2a / 2.2b / 2.2c / 2.2d closure summary:

```text
SOURCE WRITE PATH COUNT                  23
CONTRACT WRITE ROW COUNT                 23
MATCH                                    YES                §7

legacy torque-OFF prime / torque-ON reassertion  SPLIT      W2 / W3, F1
pre-session Entry context                DEFINED            §8.0, F2
motion inputs                            DERIVED, not chosen §8.4, F3
GoalPosition construction boundary       SEALED + 7-stage scan §8.3, M-27s, F4
LF motion-bearing fields                 DERIVED internally §4.5, F5
operational increments                   58 / 20 / 16       §14.2-§14.4
complete external streams                60 / 22 / 18       §14.0, F6
CI roots                                 EXPLICIT           §16.0b, F7
rejection matrix                         EXPANDED           M-28s, F8
M-19s                                    REMOVED from G3    §16.2, §16.6, F9
G3 surface                               LF-ONLY            §20, F10

GoalPosition construction sites          1                  CG-1, §8.3
raw constructor direct callers           2 policy writers   CG-2
engine operation callers                 12                 CG-5
startup writer coverage                  W1..W3 only        CG-3
normal writer coverage                   W4..W23 only       CG-4
policy writers kept distinct             different motor sets CG-7
terminal safety outside the graph        TorqueEnable only  CG-8

CG-1 counts write-construction sites     not enum refs      CI-C1
#[cfg(test)] excluded from counts        port.rs :1838      CI-C1
SCOPE-B anchors named, kept separate     mod.rs / port.rs   CI-C1
Python dynamic-import scope              all 8 files        CI-C2
K-11 observer policy scope               1 file             CI-C2
live A-row references                    0                  CI-C3

four LF single-contact runtime modes     PRESERVED          §5.2, ARM-2r
isolated LF Hip                          still BLOCKED      ARM-3r
RF/RH/LH runtime tokens                  still ABSENT       ARM-1
motion-authority token                   NONE EXISTS        §8, R3-3
ArmableLfSessionSpec                     SEALED             §4.4
provenance trust root                    FULLY SPECIFIED    §3.3
Geometry importer                        OUTSIDE G3         §3.3, §16.6, §20
```

STOP. G3 is not started.

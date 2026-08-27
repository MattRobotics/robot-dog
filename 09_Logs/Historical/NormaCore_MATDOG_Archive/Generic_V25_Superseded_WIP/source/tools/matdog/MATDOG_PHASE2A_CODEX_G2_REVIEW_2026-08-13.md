Worked for 23m 44s

I’ll read the attached review brief in full, then inspect the repository and carry out the requested closeout work. I’ll split independent verification into parallel tracks once I’ve identified the exact requirements.

Read files

I’ve parsed the mandate. I’m starting with the mandatory integrity gate; if `base..HEAD` contains anything beyond the three specified documents, the review must stop immediately as a BLOCKER. No repository or hardware state will be changed.

Ran git branch --show-current git rev-parse HEAD git rev-parse HEAD^ git merge-base HEAD 4a8ed6337261553b79c928975808d294c9ca723b git status --short --branch git diff --name-only 4a8ed6337261553b79c928975808d294c9ca723b..HEAD sha256sum tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md tools/matdog/MATDOG\_PHASE2A\_G1\_GATE\_DECISION\_2026-08-13.md tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md

The integrity gate passed: the branch, parent, merge-base, clean status, and exact three-file diff all match the brief. I’m now applying the required independence rule. Three parallel read-only audits will cover LF behavior/safety, CI and ownership, and Geometry V5 plus the historical RF anti-pattern; none will consult the proposed G2 design during this pass.

Pass a lfPass a ci ownershipPass a geometry rfstarted working

Read files, ran commands

Pass A is establishing a strong separation that the review will enforce: Geometry V5’s clearance verdict is explicitly non-authorizing, its target-domain classification is independent, and LF hardware evidence is scoped “LF only” with mirroring forbidden. The immutable LF source also has a narrow exception: a detector-level fine-pass plateau candidate may be bypassed before final qualification, but every final accepted contact immediately calls pressure-stop and returns. I’ll test whether G2 preserves that exact boundary rather than generalizing it.

Read files

The independent baseline is now sufficiently reconstructed to begin Pass B without relying on Claude’s conclusions. I’m reading the three documents in order and will compare their claims against the source-derived constraints; the parallel auditors are continuing to sharpen exact evidence and ownership disagreements.

Read files

Context automatically compacted

Pass A is complete against the immutable LF, Geometry V5, CI, and historical RF sources. I’m now in Pass B: testing the G2 contract against that independent baseline, with special attention to authority separation, offline-only enforcement, LF trace equivalence, and CI non-weakening.

Read files

Pass b falsifyPass a geometry rffinished

Read files

Doc qualityfinished

Read files

The falsification pass has found several contract-level issues, not repository-integrity issues. The most consequential are: OPEN-1 currently launders an external `UNRESOLVED` result through hardware evidence; the proposed executable token proves only numeric range, not target authority; and “offline-only” non-LF specs are still placed on the runtime arming path. I’m cross-checking LF representability and every CI migration row before assigning final severities.

Read files, ran commands

The document checks are complete: G0 has all sections 1–15, the master table is exactly IDs 01–120, and AMB-1 through AMB-9 are present. G1 accurately records its rulings overall, but G2 silently reverses several of them—most notably the immutable Geometry V5 target domain and global Station/bus ownership. I’m doing the final read-only integrity recheck now.

Read files, ran a command

# MATDOG PHASE 2A — CODEX INDEPENDENT G2 REVIEW

## 1. Repository integrity

Integrity preflight and final recheck both passed.

- Branch: `matdog/generic-v25-full-leg-engine`
- HEAD: `2047c11ad9498af7f14061eafe2e631cb4bc7380`
- HEAD parent: `4a8ed6337261553b79c928975808d294c9ca723b`
- Merge-base against expected base: `4a8ed6337261553b79c928975808d294c9ca723b`
- Status: clean; exact output was `## matdog/generic-v25-full-leg-engine`
- `base..HEAD` contains exactly the three authorized Phase 2A documents.
- No runtime, test, workflow, runner, observer, or hardware code changed.

SHA256:

- G0/G1 report: `37646f2eaf0f56af558b2450ada7908c44b8b1fdb394f3095fb99c25c651bc17`
- G1 decision: `89df3920461d347d372f30fb44aa0eefb016543f45b4f5615d8b7309fd51433d`
- G2 contract: `912158b08425f879edf1ff41096617440f4a0f3282ec52c6b05372ff4c2428a7`

The historical RF worktree remains in its expected evidence state: branch `matdog/rf-calibrator-from-lf-v25`, with only the pre-existing modified `matdog.rs` and `matdog_test.rs`.

## 2. Independent baseline

Pass A was completed before relying on G0/G1/G2 conclusions.

| OwnerIndependent conclusion |                                                                                                                                                                                                                                                    |
| --------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| A — behavior                | One engine owns transition grammar, detector/search/backoff mechanics, role derivation, hold lifecycle, LIFO restore, diagnostics, progress mechanics, and the accepted-contact stop rule.                                                         |
| B — per-leg data            | Each leg independently supplies motor mappings, directions, endpoint geometry, contact ordering, parking and prerequisite poses. No mirror/default/shared-limit constructor is authoritative.                                                      |
| C — global safety           | Exact topology, unsigned range, torque/speed/acceleration, current/temperature/freshness, RAM allowlist, command matching, operator stop, q0 protection, verified all-12 torque-off, port/serial authority, and EEPROM separation are non-tunable. |
| D — LF evidence             | LF witness ticks, tolerance, immutable behavior trace, 58-step contract, LF V1 schema, physical evidence, and historical prerequisite values remain LF-only.                                                                                       |

A safe architecture additionally requires:

- Geometry target domain, hardware-contact evidence, bounded-probe permission, and motion authorization to remain separate.
- A validated/branded spec before any command capability can be created.
- Engine-owned lifecycle grammar; per-leg data cannot encode a second state machine.
- LF’s bounded fine-pass friction/chamfer qualification, followed by immediate stop after final acceptance, preserved exactly.
- Non-LF Phase 2A data to have no runtime arming or motion path.
- External geometry values to be bound to their exact source records and semantic provenance before runtime consumption.

## 3. Findings, ordered BLOCKER → MAJOR → MINOR → NOTE

### BLOCKER — B1: OPEN-1 launders geometry and policy results into direct executability

Canonical Geometry V5 has only eight executable-domain targets. For LF, only Upper min/max are executable; both Hip endpoints and both Lower endpoints are diagnostic outside the URDF domain. G2 nevertheless makes all six LF endpoints `TargetDomain::Executable` because hardware evidence exists.

Its admission formula also explicitly permits `external_policy == Unresolved` when hardware validation is present. This violates all three required permanent rules:

- hardware validation does not create source-domain eligibility;
- external-policy PASS cannot create eligibility;
- external-policy UNRESOLVED prevents a Geometry-V5-derived direct target.

The two highlighted endpoints, `lf_hip_joint:min` and `lf_lower_leg_joint:min`, cannot be represented safely by the proposed rule. The same problem also affects LF Hip max and Lower max: external-policy PASS still cannot turn their diagnostic V5 targets into direct targets.

### BLOCKER — B2: `ExecutableTick` is only a numeric range wrapper

The constructor proves `0..4095`, but proves nothing about geometry domain, external-policy verdict, provenance, operation purpose, or motion authority.

G2 claims a diagnostic value would require a second constructor, while also requiring guards, baselines, prerequisites, staged q0, and backoff calculations to call the same raw `ExecutableTick::new(i32)`. Any code in the defining module can wrap an arbitrary diagnostic value without adding a constructor.

Further, `validate()` returns `Result<(), String>` rather than a branded validated object, while the engine consumes a raw `&LegCalibrationSpec`. The type signatures therefore do not prove validation occurred before target construction.

### BLOCKER — B3: “Offline-only” non-LF specs are registered on the runtime arming path

G2 adds `RF_LEG_STATE_MACHINE` to the `MATDOG_ARM_ENV` resolver but retains only the current isolated-HIP hardware block. `hardware_witness == None` prevents staging and EEPROM after measurement; it does not prevent torque enable, prerequisite motion, or contact probing before that point.

Phase 2A non-LF specs must not be present in the runtime arm resolver. They need a separate offline-only API with no command, serial, torque, or port capability.

### MAJOR — M1: `SessionPlan` is an unchecked per-leg state machine encoded as data

The spec supplies arbitrary phase order, active IDs, hold sets, predecessor constraints, and terminal phases. Validation does not require:

- Preflight first;
- a fixed lifecycle grammar;
- Diagnostics before staged return;
- one terminal `Cleanup → TorqueOff`;
- no successor after cleanup;
- contact active ID equal to the referenced joint’s motor;
- held-set consistency with completed operations.

A singular Rust executor does not satisfy the one-engine requirement if each leg can supply an arbitrary transition table.

`allowed_motor_ids` has the same problem: validation permits any subset of the twelve motors containing the leg’s three. A spec can add unrelated motors and then use them in phases. The legal participant set must be derived as exactly the leg’s three motors plus its provenance-bound optional parking motor.

### MAJOR — M2: The proposed LF plan cannot represent `InitialRecovery`

`SessionPhase.active_motor_id` permits one optional motor and INV-11 requires exactly one active motor per phase. The LF table requires `11/12/13` for `InitialRecovery`, matching the immutable engine’s eligible set.

The design must either make recovery a structural engine operation that sequentially iterates the leg joints while preserving one-active-at-a-time, or model an eligible set separately from the current active motor.

### MAJOR — M3: Standalone prerequisites and full-session holds are conflated

`PrerequisitePlan.targets` are described as static holds. The LF table gives Upper contact M42, LF Hip at home, and LF Lower at home. In the immutable full LF session, only M42 is held during Upper contact; Hip and Lower remain passive torque-off at home.

The contract needs distinct representations for:

- a standalone-contact prerequisite hold;
- a full-session accumulated hold;
- a torque-off joint required merely to remain at home.

Otherwise a direct implementation changes immutable LF torque roles.

### MAJOR — M4: Provenance is metadata-shaped but not value-bound

The proposed validation checks hash formatting and sibling-string equality, but does not bind:

- endpoint record ID to leg, joint, side, value, or target domain;
- parking/prerequisite values to specific V5 records;
- external-policy verdict to its policy artifact, threshold, semantic SHA, and source SHA;
- `HardwareWitness.evidence_sha256` to the provenance hardware-evidence field.

Correct current hash strings can therefore accompany stale or invented values and still pass the documented invariants. “Authored by review” is not runtime provenance verification.

### MAJOR — M5: `HardwareWitness` is not structurally LF-only

A private plain struct remains constructible anywhere in its module. `Option<HardwareWitness>` does not itself prevent `Some` on another leg, and the engine accepts an unbranded raw spec.

The claimed fourth barrier also does not exist: G2 says M-11 checks non-LF witness/span adjacency, but M-11 only forbids per-leg engine and validator names.

The witness should be an opaque LF-branded authority, keyed by `JointKind` rather than positional array order, and consumable only through a validated LF spec.

### MAJOR — M6: CI migration is incomplete and weaker than claimed

The required assertion-by-assertion mapping is not complete:

- M-9 bundles ten exact test-name assertions without listing old name, new name, and individual proof.
- M-10 changes the staged-return locator despite K-10 calling it verbatim, and does not clearly retain checks for all three staged joints.
- `MATDOG_LF_PROFILE_V1|joint=` and the Python affine-serializer assertions have no explicit migration row.
- M-8 does not require its truth table to exercise the production evidence/staging path.
- M-12’s decimal-literal ban cannot prevent imported, computed, renamed, or differently encoded LF spans.
- M-13 checks the flawed range wrapper, not target authority.
- M-14 checks hash-token presence, not record/value binding.
- Current forbidden-source scanning and rustfmt cover an explicit file list; newly split calibrator modules could evade them.

The EEPROM-forbidden list and observer forbidden-token list are correctly retained and should remain verbatim.

### MAJOR — M7: Global Station/bus ownership is reassigned to per-spec evidence

G0 ID 120 is C+D: Station/bus ownership and torque enforcement are global C, while LF’s `58` is D. G2 places `EXPECTED_BUS_SERIAL` alongside the LF step count under D as “per-spec later.” Bus serial, torque-limit enforcement, and Station ownership must not become leg-spec data.

G2 also omits ID 118 from its claimed complete ownership mapping and moves ID 119, the reviewed launcher/pin enforcement boundary, from C to D.

### MINOR — N1: Progress count is arbitrary spec data

`SessionPlan.total_steps` is not derived from the expanded operations. Current LF progress is structurally `16 + 6×7 = 58`, while `mark_done()` publishes `58/58` without asserting that 58 real increments occurred.

The engine should derive the count and refuse DONE unless the executed count equals it; LF must additionally assert exactly 58 and identical phase strings.

### MINOR — N2: Non-LF specs remain placeholders

G2 promises four independent literal specs but gives `<V5>` and “from Geometry V5” placeholders rather than reviewable RF/RH/LH endpoint, prerequisite, parking, domain, and phase datasets. This prevents a complete symmetry/provenance review.

### MINOR — N3: Documentation defects

- G2 points OPEN-1 to §11, but it is in §12.
- G1’s D9 search-count claim says one documentation hit; the stated expression returns five documentation hits at the base. The material conclusion—zero runtime live-FK dependency—remains correct.
- G2 does not explicitly carry D9 into its G3 prohibitions.

### NOTE — N1: Contact semantics are captured correctly at design level

G2 explicitly distinguishes detector-level confirmation, bounded fine-pass friction/chamfer continuation, and final accepted contact. It requires `stop_pressure()` and return immediately after final acceptance. This matches immutable LF behavior, subject to production-path regression tests.

### NOTE — N2: Oracle separation is otherwise correct

G2 does not rewrite the behavioral release oracle, runtime witness ticks, physical LF evidence, or README documentation to force numerical agreement. Fixed-scale disagreement remains diagnostic while affine q0 remains the staging value.

## 4. OPEN-1 ruling

The proposed three fields are insufficient because their admission rule still uses hardware evidence to upgrade direct-target eligibility.

Minimal semantic correction:

1. Preserve the Geometry V5 source domain immutably:
   - `ExecutableUrdfDomain`
   - `DiagnosticOutsideUrdfLimits`
2. Keep external policy as an independently provenance-bound, deny-only result:
   - PASS may retain an already eligible target.
   - FAIL or UNRESOLVED denies Geometry-V5-derived direct targeting.
   - PASS never creates eligibility.
3. Keep hardware contact validation as evidence only. It neither changes the V5 target domain nor creates motion authority.
4. Add a separate opaque bounded-probe authority. LF’s immutable historical behavior may receive this capability for incremental, detector-controlled search within its reviewed guard/corridor. It must not yield a directly commandable V5 endpoint.
5. Distinguish purpose-specific command capabilities:
   - direct target;
   - bounded probe step;
   - home normalization;
   - prerequisite/parking move;
   - backoff;
   - staged affine q0.

`lf_hip_joint:min` and `lf_lower_leg_joint:min` then remain diagnostic and externally unresolved as direct Geometry targets, retain historical hardware evidence, and remain probeable only under the LF-specific bounded-probe authority.

## 5. OPEN-2..OPEN-5 rulings

| Open itemRulingDecision before G3Minimal resolution |       |                                       |                                                                                                                                                                             |
| --------------------------------------------------- | ----- | ------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| OPEN-2                                              | NOTE  | Yes—record the compatibility decision | Preserve `"LF_LEG_STATE_MACHINE"` as an opaque historical sentinel. Its spelling does not imply a duplicate Rust type and must not justify adding non-LF runtime sentinels. |
| OPEN-3                                              | MINOR | Yes—explicitly confirm the deferral   | Preserve LF V1 byte-for-byte and require an LF-branded accepted outcome at the serializer boundary. A generic record may remain deferred.                                   |
| OPEN-4                                              | NOTE  | No additional design decision         | Keep `GUARD_OVERSHOOT_TICKS = 64` global, non-spec-tunable, and CI-pinned.                                                                                                  |
| OPEN-5                                              | NOTE  | No additional design decision         | Keep LIFO restore engine-owned. Any future non-LIFO requirement needs a separate architecture review.                                                                       |

## 6. A/B/C/D disagreements

The principal ownership corrections are:

- Transition grammar, valid lifecycle, required-hold derivation, cleanup suffix, and single-active enforcement are A/C—not arbitrary B data.
- B may supply geometry-dependent operation order, but the engine must compile and validate it against a fixed grammar.
- Motor identity comes from `JointSpec`; it should not be repeated as an independently authoritative phase field.
- Standalone prerequisites, session holds, and passive-at-home requirements need separate roles.
- `total_steps` is derived A behavior; exact LF `58` is D evidence.
- Bus serial, Station-process ownership, observer boundaries, torque limit, and launcher pin enforcement remain C.
- LF witness values and LF V1 remain D.
- ContactProfile’s proposed A/B split is directionally correct, but the replacement must also separate immutable session identity from ephemeral active-contact state.

## 7. Geometry/provenance/executability assessment

The current boundary fails closed only numerically, not semantically.

A corrected chain should be:

```
trusted source manifest
→ record/value/domain/policy provenance verification
→ ValidatedLegSpec
→ purpose-specific authority
→ AuthorizedGoal
→ unsigned GoalPosition
```

Required properties:

- Validation consumes a raw spec and returns the only spec type accepted by the engine.
- A range-checked tick is not itself command authority.
- Provenance binds exact endpoint and parking records, not merely aggregate hashes beside manually copied values.
- The V5 policy artifact identity, threshold, semantic SHA, and source identity accompany each imported verdict.
- Geometry V5’s PASS remains clearance evidence only; its canonical artifact explicitly grants zero motion authorizations.
- A diagnostic geometry contact may constrain a bounded probe envelope, but cannot become a direct goal.
- Provenance/domain checks occur before any command-capable token is minted.

## 8. LF behavioral-preservation assessment

Preserved correctly in intent:

- one behavioral oracle at immutable release `f87dd1…`;
- LF-only runtime witness ticks and tolerance;
- separate physical evidence;
- coarse scout discarded from metrology;
- two fine passes and circular midpoint;
- bounded friction/chamfer qualification;
- final accepted contact immediately stops;
- affine+witness controls staging;
- raw fixed-scale disagreement remains visible;
- exact M13 → M11 → M12 staged return followed by M42 restoration;
- global verified torque-off;
- LF V1 compatibility.

Not yet safely representable:

- `InitialRecovery` active eligibility;
- standalone versus full-session prerequisite roles;
- fixed engine lifecycle and cleanup grammar;
- exact real progress count and DONE assertion;
- LF-only witness and serializer authority.

G3 cannot claim LF equivalence until replay tests prove the exact transition trace, roles, corridors, contact outcomes, phase strings, 58 actual increments, record bytes, and cleanup from every failure state.

## 9. CI migration assessment

The CI design must be revised before implementation:

- Provide a literal four-column mapping for every changed assertion, including all ten test names.
- Retain explicit proof that all three staged joints use affine q0 and none uses fixed-scale q0.
- Add an explicit LF V1/schema migration row and retain the Python affine-serializer checks.
- Exercise the production evidence/staging function in the affine×witness truth table.
- Replace literal-span bans with typed ownership tests proving no non-LF spec or calculation can consume LF witness/span data.
- Test that an invalid raw spec cannot be passed to the engine.
- Test each provenance field and each record value with independent mutation rejection.
- Scan and format every new MATDOG calibrator module.
- Preserve the EEPROM and observer forbidden-token lists verbatim.
- Extend observer enforcement to imported helpers so motion policy cannot be moved out of the scanned file.

## 10. Historical RF anti-pattern assessment

The historical RF worktree remains evidence-only and untouched.

Confirmed prohibited patterns:

- duplicated `RfSessionStateMachine`, RF roles, validators, and corridors;
- LF spans `935/2004/1435` used as RF prediction/witness authority;
- RF acceptance derived from LF span agreement;
- confirmed contact outside the predicted RF region causing another advancing command.

The only reusable concept is:

```
same-leg current-session anchor
+ same-leg pinned model-derived span
+ intersection with the existing probe corridor
+ never widen
+ empty intersection fails before motion
```

Additional mandatory rule: any real confirmed contact stops pressure immediately. If it lies outside the predicted band, the engine backs off, records the disagreement, fails closed, and performs verified cleanup. The only continued-probing exception remains LF’s exact bounded pre-final fine-pass friction/chamfer qualification.

## 11. Required G2 corrections

Before G3:

1. Replace OPEN-1 admission with separate immutable domain, policy, hardware-evidence, probe-authority, and motion-authority concepts.
2. Replace `ExecutableTick` with a numeric range type plus purpose-specific opaque `AuthorizedGoal`.
3. Make validation return a branded `ValidatedLegSpec`; raw specs cannot reach the engine.
4. Remove RF/RH/LH from runtime arm resolution in Phase 2A.
5. Make lifecycle, cleanup, role derivation, and legal participant derivation engine-owned.
6. Correct the `InitialRecovery` representation.
7. Separate standalone prerequisites, session holds, and passive-home requirements.
8. Bind every imported value and policy verdict to an independently trusted record manifest before runtime use.
9. Make LF witness and LF V1 serializer inputs structurally LF-only.
10. Derive progress totals and require actual LF execution to equal 58 before DONE.
11. Replace the incomplete CI migration table with assertion-by-assertion, production-path proofs.
12. Correct the A/B/C/D mapping for IDs 118–120 and retain global Station/bus ownership.
13. Add the explicit outside-predicted-band contact stop/backoff/fail rule.
14. Supply complete independent RF/RH/LH offline datasets for review.

## 12. Exact evidence supporting each finding

- B1: [G1 target-domain ruling (line 151)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G1\_GATE\_DECISION\_2026-08-13.md:151), [G1 permanent semantics (line 349)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G1\_GATE\_DECISION\_2026-08-13.md:349), [G2 hardware upgrade (line 242)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:242), [G2 admission formula (line 261)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:261), [canonical V5 domains (line 203)]\(/home/matteo-manicardi/MATDOG/worktrees/robot-dog-geometry-v5/06\_Software/Matdog\_Core/calibration/MATDOG\_GEOMETRY\_COMPILER\_V5\_FINAL\_ARCHITECTURE\_2026-08-11.md:203), [LF policy records (line 14)]\(/home/matteo-manicardi/MATDOG/worktrees/robot-dog-geometry-v5/09\_Logs/Validation\_Reports/Geometry\_Compiler/2026-08-11\_132758\_MATDOG\_GEOMETRY\_V5\_G12\_FINAL\_EXTERNAL\_SAFETY\_POLICY.md:14).
- B2: [range-only constructor (line 124)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:124), [claimed sole path (line 195)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:195), [other raw constructor callers (line 694)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:694), [unbranded validation (line 359)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:359), [raw engine input (line 491)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:491).
- B3: [non-LF runtime sentinel (line 708)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:708), [witness-based offline inference (line 920)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:920), [current arm resolver (line 511)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:511), [current runtime dispatch (line 2325)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:2325).
- M1/M2: [SessionPhase and SessionPlan (line 528)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:528), [transition rule (line 551)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:551), [insufficient INV-11 (line 381)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:381), [LF recovery table (line 872)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:872), [immutable recovery eligibility (line 1166)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:1166).
- M3: [G2 prerequisite holds (line 333)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:333), [G2 LF prerequisite table (line 854)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:854), [immutable full-session held sets (line 1109)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:1109), [immutable Upper execution (line 2878)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:2878).
- M4/M5: [provenance types (line 729)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:729), [provenance invariants (line 766)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:766), [review-only admission (line 814)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:814), [policy identity and zero authorization (line 3)]\(/home/matteo-manicardi/MATDOG/worktrees/robot-dog-geometry-v5/09\_Logs/Validation\_Reports/Geometry\_Compiler/2026-08-11\_132758\_MATDOG\_GEOMETRY\_V5\_G12\_FINAL\_EXTERNAL\_SAFETY\_POLICY.md:3), [HardwareWitness (line 423)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:423), [claimed barriers (line 445)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:445), [actual M-11 (line 973)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:973).
- M6: [G2 CI migration (line 959)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:959), [current exact source assertions (line 84)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/.github/workflows/matdog-native-calibrator-check.yml:84), [exact test names (line 105)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/.github/workflows/matdog-native-calibrator-check.yml:105), [three-joint staged-return proof (line 152)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/.github/workflows/matdog-native-calibrator-check.yml:152), [observer boundary (line 39)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/.github/workflows/matdog-native-observer-check.yml:39).
- M7/N1: [G0 ownership IDs 109–120 (line 458)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md:458), [G2 C mapping (line 1080)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:1080), [G2 D mapping (line 1089)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:1089), [arbitrary total (line 541)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:541), [current \`mark\_done\` (line 4732)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:4732).
- Contact/oracles: [G1 oracle hierarchy (line 171)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G1\_GATE\_DECISION\_2026-08-13.md:171), [runtime witness (line 1979)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:1979), [G2 contact distinction (line 590)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:590), [immutable immediate branch (line 4013)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:4013), [immutable adaptive branch (line 4112)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:4112).
- Historical RF: [LF spans as RF authority (line 2665)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator/software/drivers/st3215/src/auto\_calibrate/matdog.rs:2665), [continued motion after confirmed contact (line 5045)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator/software/drivers/st3215/src/auto\_calibrate/matdog.rs:5045), [canonical stop/fail rule (line 382)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_CALIBRATION\_CANONICAL\_HANDOFF\_2026-08-07.md:382).
- Document quality: [G0 section 1 (line 5)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md:5), [G0 section 15 (line 966)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md:966), [master table (line 343)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md:343), [AMB-1 through AMB-9 (line 863)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md:863).

VERDICT = BLOCKED

Final integrity recap:

- Branch: `matdog/generic-v25-full-leg-engine`
- HEAD: `2047c11ad9498af7f14061eafe2e631cb4bc7380`
- Base / parent: `4a8ed6337261553b79c928975808d294c9ca723b` / `4a8ed6337261553b79c928975808d294c9ca723b`
- Merge-base: `4a8ed6337261553b79c928975808d294c9ca723b`
- Git status: `## matdog/generic-v25-full-leg-engine`
- Changed files:
  - `tools/matdog/MATDOG_PHASE2A_G0_G1_EXECUTOR_REPORT_2026-08-13.md`
  - `tools/matdog/MATDOG_PHASE2A_G1_GATE_DECISION_2026-08-13.md`
  - `tools/matdog/MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md`
- SHA256:
  - `37646f2eaf0f56af558b2450ada7908c44b8b1fdb394f3095fb99c25c651bc17`
  - `89df3920461d347d372f30fb44aa0eefb016543f45b4f5615d8b7309fd51433d`
  - `912158b08425f879edf1ff41096617440f4a0f3282ec52c6b05372ff4c2428a7`

FILES MODIFIED BY CODEX = NONE
COMMITS CREATED = NONE
PUSHES = NONE
HARDWARE ACTIONS = NONE

11:57 AM

Full access
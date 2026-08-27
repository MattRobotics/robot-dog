Worked for 16m 44s

I’ll read the new review brief first, then establish its exact integrity and independence requirements before inspecting any revised design material. I’ll keep this pass read-only unless the attached instructions explicitly say otherwise.

Read MATDOG PHASE 2A — CODEX INDEPENDENT G2 REVIEW #2 REVISION-1 CLOSURE / REGRESSIO…

I’ve parsed the brief. I’m starting with the mandatory checkpoint only: parentage, clean status, exact Revision-1 file scope, byte identity of G0/G1, and the archived Review #1 hash. If any of those fail, I’ll stop without reviewing the revised architecture.

Ran set -euo pipefail prev=2047c11ad9498af7f14061eafe2e631cb4bc7380 base=4a8ed6337261553b79c928975808d294c9ca723b printf 'branch\t%s\n' "$(git branch --show-current)" printf 'head\t%s\n' "$(git rev-parse HEAD)" printf 'parent\t%s\n' "$(git rev-parse HEAD^)" printf 'merge\_base\t%s\n' "$(git merge-base HEAD "$base")" printf 'status\_begin\n' git status --short --branch printf 'status\_end\n' printf 'revision\_name\_status\_begin\n' git diff --name-status "$prev"..HEAD printf 'revision\_name\_status\_end\n' printf 'base\_name\_status\_begin\n' git diff --name-status "$base"..HEAD printf 'base\_name\_status\_end\n' printf 'g0\_prev\_blob\t%s\n' "$(git rev-parse "$prev\:tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md")" printf 'g0\_head\_blob\t%s\n' "$(git rev-parse "HEAD\:tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md")" printf 'g1\_prev\_blob\t%s\n' "$(git rev-parse "$prev\:tools/matdog/MATDOG\_PHASE2A\_G1\_GATE\_DECISION\_2026-08-13.md")" printf 'g1\_head\_blob\t%s\n' "$(git rev-parse "HEAD\:tools/matdog/MATDOG\_PHASE2A\_G1\_GATE\_DECISION\_2026-08-13.md")" printf 'review\_archive\_sha256\t' sha256sum tools/matdog/MATDOG\_PHASE2A\_CODEX\_G2\_REVIEW\_2026-08-13.md printf 'all\_phase\_docs\_sha256\_begin\n' sha256sum tools/matdog/MATDOG\_PHASE2A\_\*2026-08-13.md printf 'all\_phase\_docs\_sha256\_end\n'

Integrity passes exactly: current HEAD is a single child of the previously reviewed commit, the worktree is clean, Revision-1 changes only the two new review documents plus the revised G2 contract, G0/G1 blobs are unchanged, and the archived Review #1 matches the supplied independent SHA256. I’m now reading the complete revised contract and decision—not just their diff—and running three independent falsification tracks in parallel.

R1 authority provenanceR1 engine lf offlineR1 ci data docsstarted working

Read files

The complete revised contract is now read. The high-level five-axis correction is materially better, but I’m testing whether the new capability layer can actually express the immutable LF command path. Early checks have exposed two areas needing exact source comparison: goal purposes omitted from the enum, and a possible LF Hip-Max prerequisite mismatch between standalone and full-session behavior.

Ran commands

Context automatically compacted

The revised contract closes much of the original conceptual gap, but the adversarial pass has now exposed several structural contradictions—most notably replayable goal capabilities, an offline API returning the runtime-capable type, and an LF Hip-Max hold value that disagrees with the frozen full-session choreography. I’m validating those against the exact artifacts and source before assigning closure statuses.

Read files, ran commands

R1 engine lf offlineR1 authority provenanceR1 ci data docsfinished

Read files, ran commands

The closure decision is now clear: Revision 1 cannot safely close G2. I found independent runtime-path, capability-lifetime, provenance, and LF-equivalence failures; I’m doing the mandated final integrity/hash recheck so the report can distinguish design blockers from repository-integrity status.

Ran git branch --show-current git rev-parse HEAD git rev-parse HEAD^ git merge-base HEAD 4a8ed6337261553b79c928975808d294c9ca723b git status --short --branch git diff --name-status 4a8ed6337261553b79c928975808d294c9ca723b..HEAD git diff --name-status 2047c11ad9498af7f14061eafe2e631cb4bc7380..HEAD git rev-parse 2047c11ad9498af7f14061eafe2e631cb4bc7380\:tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md HEAD\:tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md git rev-parse 2047c11ad9498af7f14061eafe2e631cb4bc7380\:tools/matdog/MATDOG\_PHASE2A\_G1\_GATE\_DECISION\_2026-08-13.md HEAD\:tools/matdog/MATDOG\_PHASE2A\_G1\_GATE\_DECISION\_2026-08-13.md sha256sum tools/matdog/MATDOG\_PHASE2A\_G0\_G1\_EXECUTOR\_REPORT\_2026-08-13.md tools/matdog/MATDOG\_PHASE2A\_G1\_GATE\_DECISION\_2026-08-13.md tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md tools/matdog/MATDOG\_PHASE2A\_CODEX\_G2\_REVIEW\_2026-08-13.md tools/matdog/MATDOG\_PHASE2A\_G2\_REVIEW\_DECISION\_2026-08-13.md sha256sum /home/matteo-manicardi/.codex/attachments/870b629e-dcdb-489b-8cb4-5b72b0f3c6ec/pasted-text.txt /home/matteo-manicardi/.codex/attachments/bfd83292-4144-4bdc-bddd-51fb9fa2b094/pasted-text.txt

# MATDOG PHASE 2A — CODEX INDEPENDENT G2 REVIEW #2

## 1. Repository integrity

Mandatory preflight and final recheck both passed.

- Branch: `matdog/generic-v25-full-leg-engine`
- Previous reviewed HEAD: `2047c11ad9498af7f14061eafe2e631cb4bc7380`
- Current HEAD: `f4c5a500697d04e4c45641385170a4a630a0f38a`
- HEAD parent: `2047c11ad9498af7f14061eafe2e631cb4bc7380`
- Canonical base: `4a8ed6337261553b79c928975808d294c9ca723b`
- Merge-base: `4a8ed6337261553b79c928975808d294c9ca723b`
- Status: clean; exact output was `## matdog/generic-v25-full-leg-engine`
- G0/G1 report blob at previous/current HEAD: identical, `275499ea32b512c88ee3f4ba70f19f15c2552707`
- G1 decision blob at previous/current HEAD: identical, `536fddef5427e51c9660458a422713755793812d`
- Archived Review #1 SHA256: exact expected `c51fb5d74c1752302e886febacde213c6748b70078ecc898ccd97efe5b9fbf6d`

Repository integrity is not the reason G2 fails closure.

## 2. Revision-1 changed files and hashes

`previousHEAD..HEAD` contains exactly the three expected files:

- Added [MATDOG\_PHASE2A\_CODEX\_G2\_REVIEW\_2026-08-13.md]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_CODEX\_G2\_REVIEW\_2026-08-13.md): `c51fb5d74c1752302e886febacde213c6748b70078ecc898ccd97efe5b9fbf6d`
- Modified [MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md): `f22bdbc8d7a5dfdae7815fe04316253e2f9c47eeba091ca1c8a1ff6c229d6c27`
- Added [MATDOG\_PHASE2A\_G2\_REVIEW\_DECISION\_2026-08-13.md]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_REVIEW\_DECISION\_2026-08-13.md): `7a8590437cefacae49f7e02c918602642de6a69caca29152669b374aea2670b2`

No unexpected Revision-1 file changed.

## 3. Finding closure matrix

| FindingStatusClosure result |                   |                                                                                                                                                                                  |
| --------------------------- | ----------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| B1                          | CLOSED            | The five authority axes and `Executable && Pass` direct-target rule are now semantically correct.                                                                                |
| B2                          | PARTIALLY\_CLOSED | Raw/validated and range/authority types are separated, but goal capabilities are forgeable, replayable, incompletely scoped, and cannot represent the immutable LF command path. |
| B3                          | STILL\_OPEN       | The resolver retains non-LF single-contact arm values, and the offline API returns the same type accepted by the runtime engine.                                                 |
| M1                          | PARTIALLY\_CLOSED | Fixed grammar closes the arbitrary phase-table defect, but participant identity remains spoofable and the standalone runtime path is outside that grammar.                       |
| M2                          | REGRESSED         | The eligible-set distinction is fixed, but all-12 q0 normalization is reduced to three motors and its effective tolerance changes.                                               |
| M3                          | PARTIALLY\_CLOSED | Three role kinds exist, but standalone/full-session roles and exact LF hold targets/timing are still not representable.                                                          |
| M4                          | PARTIALLY\_CLOSED | Per-record structures were added, but several bindings are unimplementable or incomplete and the wrong C/W1 artifact materialization is called canonical.                        |
| M5                          | PARTIALLY\_CLOSED | The oracle is LF-named, opaque, and joint-keyed, but an LF-labelled hybrid spec can still combine other-leg motors with LF evidence.                                             |
| M6                          | PARTIALLY\_CLOSED | Migration rows are much better, but several proposed checks are incomplete, impossible at the stated layer, or weaker than claimed.                                              |
| M7                          | CLOSED            | Global C ownership, including IDs 118–120, is restored and is not spec-parameterized.                                                                                            |
| N1                          | PARTIALLY\_CLOSED | Total is no longer spec data and DONE equality is required, but the actual 16 non-contact operations are not derived from the documented trace.                                  |
| N2                          | PARTIALLY\_CLOSED | Canonical endpoint/parking tables are present, but the non-LF objects are not complete constructible specs and parking evidence is generalized unsafely.                         |
| N3                          | CLOSED            | The original OPEN-1, live-FK count, and G3 prohibition defects are corrected.                                                                                                    |

## 4. Detailed evidence for findings not CLOSED

### B2 — PARTIALLY\_CLOSED

- Original problem: `ExecutableTick` proved only numeric range, raw specs reached the engine, and arbitrary ticks could acquire command authority.
- Revision mechanism: [§4 (line 452)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:452) introduces `RawLegCalibrationSpec → ValidatedLegSpec`; [§5 (line 520)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:520) makes `UnsignedTick` range-only and the command signature accept `AuthorizedGoal`.
- Attempted falsification succeeded:
  1. `AuthorizedGoal` has private fields but no sealed authority module. Code in its defining Rust module can directly construct `AuthorizedGoal { purpose, motor_id, tick }`, including a diagnostic `DirectGeometryTarget`, without adding the claimed new constructor.
  2. It derives `Clone + Copy` and carries no session ID, phase, operation, epoch, nonce, or expiry. A once-valid probe, backoff, prerequisite, or staging token can be replayed after state changes.
  3. The command method accepts only the token. It cannot recheck “just-recorded contact,” “one current probe step,” or current-phase authorization.
  4. `PrerequisiteOrParking` requires a record-bound value, but LF historical poses are explicitly not V5 records. No sealed LF-history binding path is defined.
  5. `DirectGeometryTarget` depends on an undefined “explicit session motion authorization”; there is no opaque authorization producer or lifetime.
  6. The six-purpose vocabulary cannot express immutable LF commands. Current LF primes GoalPosition to the present position before torque-on, commands a moving-baseline target, and stops pressure at the present position ([matdog.rs (line 3875)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:3875), [matdog.rs (line 3903)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:3903), [matdog.rs (line 4249)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:4249)). None has a valid stated purpose.
  7. `BoundedProbeStep` requires each goal to be inside the adaptive acceptance corridor, but immutable coarse probing starts near HOME and advances through positions outside the narrow endpoint corridor ([matdog.rs (line 800)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:800), [matdog.rs (line 3980)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:3980)). Literal enforcement makes LF probing unreachable.
- Remaining risk: arbitrary or stale motion authorization, plus an LF implementation that cannot preserve the frozen command trace.

### B3 — STILL\_OPEN

- Original problem: RF/RH/LH Phase 2A data remained reachable from runtime arming and motion.
- Revision mechanism: [§8 (line 843)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:843) removes full-leg non-LF sentinels and introduces an `offline` module.
- Attempted falsification succeeded:
  1. The contract explicitly retains all 24 single-contact values in the runtime resolver. Current `all_profiles()` constructs six per leg ([matdog.rs (line 499)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:499)), so this retains 18 RF/RH/LH resolver entries.
  2. The hardware gate blocks only isolated Hip profiles ([matdog.rs (line 536)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:536)). Twelve non-LF Upper/Lower profiles remain physically dispatchable through `run_profile` ([matdog.rs (line 2325)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:2325)).
  3. CI row M-16 would pin that unsafe 24-value resolver set.
  4. `offline::validated_spec()` returns the same `ValidatedLegSpec` that [§4 (line 484)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:484) names as the engine’s only input. It is not an inert offline-branded type.
- Remaining risk: actual RF/RH/LH serial, torque, prerequisite, normalization, and probe motion remains architecturally reachable.

### M1 — PARTIALLY\_CLOSED

- Original problem: arbitrary phases, active IDs, hold sets, participants, cleanup placement, and terminal placement could be encoded as spec data.
- Revision mechanism: [§6.1–6.3 (line 614)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:614) provides an engine-owned grammar and derives participants.
- Attempted falsification:
  - Arbitrary phase and terminal placement could no longer be encoded. That part is closed.
  - Validation does not bind `(Leg, JointKind)` to the canonical motor ID, name, direction, and deltas. Participants are derived from raw `JointSpec.motor_id`, so an LF-labelled raw spec can substitute other-leg motors.
  - The parking motor need only be outside the leg and accompany a non-empty configuration. It is not bound to the canonical joint named in that configuration.
  - `RestoreData` remains an undefined spec field despite the assertion that restore order is entirely engine-owned.
  - The retained standalone `run_profile` path is not represented by the full-session grammar, leaving an alternate physical executor/model.
- Remaining risk: unrelated motors can still enter the participant set, and “one engine” is not true across the retained standalone path.

### M2 — REGRESSED

- Original problem: one fixed `active_motor_id` could not represent LF InitialRecovery’s three eligible motors with one active at a time.
- Revision mechanism: [§6.4 (line 686)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:686) separates `{11,12,13}` eligibility from the currently active motor.
- Attempted falsification found a broader behavioral regression:
  - Immutable LF verifies global torque-off and normalizes every one of the 12 MATDOG motors before creating the LF session ([matdog.rs (line 2850)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:2850)).
  - The normalization loop is over `MATDOG_MOTOR_IDS`, not only M11/M12/M13 ([matdog.rs (line 3698)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:3698)).
  - Immutable recovery moves when distance exceeds 10 ticks or the historical `>16/speed>4` predicate applies. Revision 1 uses only `>16/speed>4`.
  Falsification examples: a displaced M21 is normalized by immutable LF but ignored by Revision 1; M11 at HOME+11 moves in immutable LF but is skipped by Revision 1.
- Remaining risk: session entry can begin from a topology state the frozen LF path rejects.

### M3 — PARTIALLY\_CLOSED

- Original problem: standalone held prerequisites, accumulated full-session holds, and passive torque-off-at-home requirements were conflated.
- Revision mechanism: [§6.5 (line 709)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:709) defines the three distinct kinds.
- Attempted falsification found:
  1. Each prerequisite has one fixed `kind`, and the LF dataset supplies only full-session kinds. Retained standalone profiles need every prerequisite torque-on and actively held ([matdog.rs (line 3473)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:3473)).
  2. Revision 1 gives full-session Hip Max M12 = `UPPER_85_DELTA`/967 ([G2:954 (line 954)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:954)). Immutable full LF forces M12 = `UPPER_90_DELTA`/1024 for both Hip sides ([matdog.rs (line 465)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:465), [matdog.rs (line 2931)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:2931)).
  3. The phase table lists M11/M12 as both active and held during their staged returns. Immutable LF removes each from held state before moving, then re-adds it afterward ([matdog.rs (line 3030)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:3030)).
  4. Revision 1 says Cleanup has an empty held set. Immutable LF enters Cleanup, verifies all-12 torque-off, and only then clears holds ([matdog.rs (line 4411)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/auto\_calibrate/matdog.rs:4411)).
- Remaining risk: changed torque roles, a wrong Hip-Max clearance pose, or an unrepresentable standalone/full-session split.

### M4 — PARTIALLY\_CLOSED

- Original problem: correct hashes could sit beside invented values because provenance was metadata-shaped.
- Revision mechanism: [§3 (line 240)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:240) adds artifact and record bindings; IMP-1–IMP-13 are fail-closed.
- Attempted falsification still succeeds:
  1. `TrustedManifest` is referenced but never defined. [§3.5 (line 427)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:427) simultaneously requires artifact reading and says G3 has no parser or reader. The independent comparison source for compiled literals is unspecified.
  2. `EndpointRecordBinding.artifact` is the endpoint profile, but `target_domain` is not present there. It lives in `PATH_PARKING.plans[].target_domain`. IMP-7 is unimplementable against its named artifact.
  3. The contract calls C/W1 canonical. The canonical V5 architecture explicitly designates D/W4 as deployment output and C/W1 only as the determinism oracle ([V5 architecture:277 (line 277)]\(/home/matteo-manicardi/MATDOG/worktrees/robot-dog-geometry-v5/06\_Software/Matdog\_Core/calibration/MATDOG\_GEOMETRY\_COMPILER\_V5\_FINAL\_ARCHITECTURE\_2026-08-11.md:277)).
  4. Exact D/W4 file hashes are endpoint `dd8cb42c…`, parking `e561e7fb…`, combined `448ebcb3…`, manifest `0db86e63…`. Revision 1 instead pins C/W1 files `23ca4709…`, `65cd6d25…`, `a482dd28…`; the final policy itself consumes D/W4 parking.
  5. No invariant independently compares several declared fields: canonical index, declared limit, contact/path statuses, structural depth, parking target/outcome/configuration/DOF/candidate counts, policy verdict/clearance/kind/motion authorization.
  6. Policy and reconciliation artifact file identities are omitted. Their canonical JSON hashes are `82f00a94…` and `d7fa04e2…`.
  7. The LF evidence binding ambiguously mixes physical evidence identity with the generated reconciliation artifact.
- Remaining risk: wrong record identity, target domain, parking configuration, or policy result can still accompany credible neighboring hashes.

### M5 — PARTIALLY\_CLOSED

- Original problem: a generic witness field allowed non-LF specs to consume LF evidence.
- Revision mechanism: [§7 (line 770)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:770) withdraws generic `Option<HardwareWitness>` and introduces an opaque, `JointKind`-keyed LF oracle.
- Attempted falsification:
  - `HardwareContactEvidence::SupervisedPassRecorded` remains a generic enum; “LF only, today” is only a comment.
  - No validation invariant binds an LF label to LF’s canonical motor mapping and limits.
  - `lf_oracle()` returns `Some` based on “the LF spec.” An LF-labelled raw spec containing RF motor IDs can therefore derive RF participants while receiving LF oracle/probe authority.
  - The parking motor is not bound to the joint named by the parking configuration.
  - The oracle sketch contains one `HardwareEvidenceBinding`, although reconciliation has six joint/side records.
- Remaining risk: a hybrid LF-labelled spec can apply LF witness ticks, tolerance, and probe authority to non-LF motors.

### M6 — PARTIALLY\_CLOSED

- Original problem: migration rows were grouped, incomplete, and often grep-only.
- Revision mechanism: [§12 (line 1234)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:1234) now gives K-1–K-13, M-1–M-26, and T-1–T-10 individually.
- Attempted falsification:
  1. Important checks are now explicit: three affine staged joints, fixed-scale exclusion, LF V1 bytes, Python affine serialization, the production evidence truth table, EEPROM and observer scans, raw-spec rejection, and module coverage.
  2. M-20 does not concretely enumerate mutation rejection for every actual bound field, and several corresponding IMP invariants do not compare those fields.
  3. T-3 says the port RAM gate rejects a tick not carried by `AuthorizedGoal`. The port sees only the serialized command/register/value, after the Rust type has been erased ([port.rs (line 80)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/software/drivers/st3215/src/port.rs:80)). It cannot distinguish identical bytes by prior wrapper provenance.
  4. M-9/M-17 require compile-fail tests but specify no mechanism that proves the in-crate boundary rather than merely external privacy.
  5. M-16 pins the unsafe 24-profile runtime resolver.
  6. M-18 mandates all six variants, including the unused direct-motion capability and the incomplete purpose vocabulary.
  7. “Directory glob” and observer-import coverage do not define recursive module-graph completeness.
- Remaining risk: CI can pass while alternate construction paths, unbound provenance fields, or non-LF runtime entries survive.

### N1 — PARTIALLY\_CLOSED

- Original problem: `total_steps` was arbitrary spec data and DONE could jump to total.
- Revision mechanism: [§10.4 (line 1019)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:1019) derives total operations, requires executed equals derived, and pins LF to 58.
- Attempted falsification: the grammar does not enumerate the claimed 16 non-contact progress operations. Its visible non-contact nodes total 12. M-24 merely checks that a derivation function returns 58, not that 58 equals the actual emitted operation trace and historical strings.
- Remaining risk: an independently maintained or hard-coded 58 can satisfy CI while real increments differ.

### N2 — PARTIALLY\_CLOSED

- Original problem: RF/RH/LH data remained `<V5>` placeholders.
- Revision mechanism: [§11 (line 1063)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:1063) includes all 24 endpoint records, all 24 parking records, and RF/RH/LH summaries.
- Attempted falsification:
  - The endpoint, domain, policy, obstruction, depth, parking, and front/hind numeric values match canonical semantics; no mirror/default placeholder remains.
  - The per-leg summaries are not complete `RawLegCalibrationSpec`s: calibration/side order, six prerequisite sets, restore data, independent min/max joint literals, and full policy bindings are absent.
  - The summaries refer to current shared `JOINT_SPECS` constants despite the ownership claim that joint limits are independently supplied per leg.
  - A canonical endpoint-specific parking plan is promoted to one leg-global parking dependency held across the entire session. V5 validates RF M32 parking for `rf_upper_leg_joint:max`, not for every RF contact or combined session.
  - The provenance source points at C/W1 rather than the canonical D/W4 materialization.
- Remaining risk: non-LF validation can be claimed without complete independent specs, and endpoint-specific path evidence can be overgeneralized into unsafe session-wide geometry.

## 5. New findings

### BLOCKER — Goal capabilities are forgeable and stale

`AuthorizedGoal` is `Clone + Copy`, lacks session/phase/epoch binding, and is not placed behind a sealed constructor boundary. Dynamic one-operation authority can be forged or replayed.

### BLOCKER — The six-purpose model makes frozen LF execution unreachable

Present-position priming, moving baseline, early approach steps outside the acceptance corridor, and present-position pressure stop have no valid minting path. M-18 would freeze this incomplete enum in CI.

### BLOCKER — Offline and runtime use the same nominal type

`offline::validated_spec()` exports `ValidatedLegSpec`, which is also the engine input and owns goal-minting methods. Call-graph prose does not make the object inert.

### BLOCKER — Retained standalone values preserve a second runtime model

The contract says there is one fixed full-session engine while retaining 24 standalone runtime arm values. It never specifies how standalone behavior enters that grammar, and the existing physical `run_profile` path remains the apparent alternate executor.

### MAJOR — Canonical artifact chain is split between C and D

Revision 1 calls C/W1 canonical while the authoritative architecture and final policy identify D/W4 as deployment canonical. Semantic equality does not make the file identity or provenance chain interchangeable.

### MAJOR — Endpoint-specific parking is generalized into a session-wide hold

The generic grammar applies one parking configuration before all joint blocks and restores it last. Canonical V5 parking evidence is endpoint-scoped; LF’s whole-session M42 behavior is separate LF historical evidence.

### MAJOR — LF exactness regressions

All-12 normalization, the effective 10-tick recovery threshold, full-session Hip-Max M12=1024, staged hold removal/re-addition, and cleanup hold timing are not preserved.

### MAJOR — LF branding can be spoofed

An LF label is not validated against LF’s exact motors, directions, and limits, permitting a hybrid spec to acquire LF evidence/probe authority.

### MINOR — Documentation and ownership defects

- §12.3 says eight test names are preserved and two renamed; actually nine are preserved and only T-1 is renamed.
- OPEN-2 is incorrectly said to be resolved by R2.
- ID 115 is shown only under D although EEPROM separation also has a C component.
- “Four distinct hip contact values” should be “four distinct per-leg hip endpoint pairs.”

## 6. Authority-model assessment

The semantic model in B1 is now correct:

| LF endpointV5 domainPolicyHardware evidenceBounded LF probeDirect V5 goal |            |            |     |     |    |
| ------------------------------------------------------------------------- | ---------- | ---------- | --- | --- | -- |
| Hip min                                                                   | Diagnostic | UNRESOLVED | Yes | Yes | No |
| Hip max                                                                   | Diagnostic | PASS       | Yes | Yes | No |
| Lower min                                                                 | Diagnostic | UNRESOLVED | Yes | Yes | No |
| Lower max                                                                 | Diagnostic | PASS       | Yes | Yes | No |

Hardware evidence no longer changes target domain; PASS is deny-only; FAIL/UNRESOLVED deny direct targeting; diagnostic endpoints remain diagnostic.

The enforcement model is not safe. Forgeable/replayable goals, undefined session authorization, spoofable LF branding, and the shared offline/runtime type provide alternate authority paths around that correct semantic expression.

## 7. LF V25 representability assessment

Revision 1 reproduces the broad 18-state names and ordering, but not the exact frozen behavior.

It fails exact representability through:

- loss of all-12 startup normalization;
- changed effective recovery threshold;
- missing goal purposes for priming, baseline, early probing, and pressure stop;
- no contextual standalone/full-session prerequisite representation;
- Hip-Max M12=967 instead of the frozen full-session 1024;
- contradictory active/held roles during staged return;
- ambiguous cleanup hold-clear timing;
- no derivation of the actual 16 non-contact progress increments.

LF V25 equivalence is therefore not presently implementable from this contract.

## 8. RF/RH/LH offline dataset assessment

Positive findings:

- All 24 endpoint rows and all 24 path/parking rows are present.
- RF/RH/LH geometry differences remain visible.
- Numerical equality is not claimed as derivation authority.
- No non-LF hardware evidence is asserted.

Closure failures:

- The per-leg summaries are not complete constructible raw specs.
- Full per-record policy/provenance data is absent.
- C/W1 is used instead of deployment-canonical D/W4.
- Endpoint-specific parking is assigned a session-wide lifetime.
- The runtime resolver still contains non-LF single-contact values.

The data is useful review material, but not yet a safe offline-only specification boundary.

## 9. Provenance assessment

The desired chain is stated but not fully realized:

```
trusted canonical artifact
→ exact source-record comparison
→ raw spec
→ validated spec
```

Remaining defects prevent a correct hash beside an invented value from failing in every required case:

- no defined `TrustedManifest` or independent compiled-literal source;
- target domain attributed to an artifact that does not contain it;
- wrong canonical C/W1 file materialization;
- incomplete endpoint, parking, policy, and reconciliation field comparisons;
- missing policy/reconciliation file pins;
- ambiguous physical-evidence versus reconciliation identity.

M4 remains materially open.

## 10. CI migration assessment

Revision 1 is substantially better than Revision 0: individual rows exist, all ten historical test names appear, and the critical affine, LF V1, EEPROM, observer, staging, and production-evidence concepts are represented.

It is not ready to govern G3 because:

- provenance mutation coverage is incomplete;
- T-3 requests a provenance distinction unavailable at the serialized port boundary;
- compile-fail mechanisms are unspecified and can prove only privacy;
- M-16 pins non-LF runtime profiles;
- M-18 pins an incomplete and unsafe goal enum;
- recursive module/import coverage is not mechanically defined.

## 11. Contact-semantics assessment

No regression was found in the contact rule.

[G2 §15 (line 1424)]\(/home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25/tools/matdog/MATDOG\_PHASE2A\_G2\_GENERIC\_SPEC\_CONTRACT\_2026-08-13.md:1424) correctly preserves:

- detector-level `ContactConfirmed` as distinct from final acceptance;
- only the exact LF fine-pass friction/chamfer pre-final continuation;
- immediate `stop_pressure()` and return after final acceptance;
- no further forward probe after final acceptance;
- outside-model-band stop, verified backoff, disagreement record, fail-closed, and verified cleanup;
- no generalization of the LF exception to future legs.

## 12. NOTE-A ruling — DirectGeometryTarget

Designing the future eligibility predicate now is safe.

Implementing an unused `DirectGeometryTarget` runtime variant or minting function in G3 is not safe or necessary:

- Phase 2A grants zero session motion authorizations.
- No LF behavioral-equivalence path needs it.
- Non-LF work is required to remain offline.
- Current M-18/M-19 would nevertheless pressure G3 to implement the unused authority.
- The currently proposed unsealed goal type makes an otherwise unused variant directly forgeable.

Ruling: preserve the eligibility design and denial tests, but keep `DirectGeometryTarget` and its minting path unimplemented until a separately authorized phase requires direct geometry motion.

## 13. NOTE-B ruling — M42 and concurrency

The per-selected-spec M42 explanation is acceptable only as a single-session ownership statement.

It is not cross-leg arbitration and does not authorize concurrent calibration. Future concurrent sessions require a separate architecture review, resource ownership model, and verified start/stop serialization.

Current startup signals a previous task to stop without proving completion before another task begins, so no concurrency-safety claim may be inferred. Phase 2A remains single-session only.

## 14. Required changes before G3

1. Remove all RF/RH/LH values from Phase 2A runtime arm resolution; revise M-16 accordingly.
2. Use structurally distinct inert offline specs and an LF-only armable session type.
3. Define one engine grammar that explicitly covers retained LF standalone and full-session modes, or remove the alternate runtime path.
4. Seal goal construction and bind each dynamic capability to the active session, phase, operation, and epoch; prevent stale/replayed use.
5. Add authority paths for every frozen LF command operation, including priming, baseline, early approach, and pressure stop.
6. Do not implement `DirectGeometryTarget` in G3.
7. Restore exact LF all-12 q0 normalization and its effective tolerance.
8. Represent standalone versus full-session prerequisite roles separately; correct full-session Hip-Max M12 to 1024 and define staged/cleanup role timing.
9. Validate every leg/joint mapping and every parking motor against exact canonical identity before deriving participants or LF branding.
10. Use deployment-canonical D/W4 artifacts and bind policy/reconciliation file identities.
11. Define the trusted-manifest mechanism and independently compare every endpoint, parking, policy, and hardware field to its exact source record.
12. Keep V5 parking/path evidence endpoint-scoped; do not infer a whole-session hold from one endpoint plan.
13. Complete RF/RH/LH raw offline specs without shared-limit, mirror, or placeholder authority.
14. Derive progress from the actual expanded operation trace and prove LF’s 58 actual increments and phase strings.
15. Correct the incomplete/impossible CI rows and minor documentation defects.

VERDICT = BLOCKED

Final integrity recap:

- Branch: `matdog/generic-v25-full-leg-engine`
- Previous HEAD: `2047c11ad9498af7f14061eafe2e631cb4bc7380`
- Current HEAD: `f4c5a500697d04e4c45641385170a4a630a0f38a`
- Base/merge-base: `4a8ed6337261553b79c928975808d294c9ca723b`
- Git status: `## matdog/generic-v25-full-leg-engine`

`base..HEAD` changed files:

- `A tools/matdog/MATDOG_PHASE2A_CODEX_G2_REVIEW_2026-08-13.md`
- `A tools/matdog/MATDOG_PHASE2A_G0_G1_EXECUTOR_REPORT_2026-08-13.md`
- `A tools/matdog/MATDOG_PHASE2A_G1_GATE_DECISION_2026-08-13.md`
- `A tools/matdog/MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md`
- `A tools/matdog/MATDOG_PHASE2A_G2_REVIEW_DECISION_2026-08-13.md`

`previousHEAD..HEAD` changed files:

- `A tools/matdog/MATDOG_PHASE2A_CODEX_G2_REVIEW_2026-08-13.md`
- `M tools/matdog/MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md`
- `A tools/matdog/MATDOG_PHASE2A_G2_REVIEW_DECISION_2026-08-13.md`

Phase 2A document SHA256:

- G0/G1 report: `37646f2eaf0f56af558b2450ada7908c44b8b1fdb394f3095fb99c25c651bc17`
- G1 decision: `89df3920461d347d372f30fb44aa0eefb016543f45b4f5615d8b7309fd51433d`
- Revised G2 contract: `f22bdbc8d7a5dfdae7815fe04316253e2f9c47eeba091ca1c8a1ff6c229d6c27`
- Archived Review #1: `c51fb5d74c1752302e886febacde213c6748b70078ecc898ccd97efe5b9fbf6d`
- G2 review decision: `7a8590437cefacae49f7e02c918602642de6a69caca29152669b374aea2670b2`

FILES MODIFIED BY CODEX = NONE
COMMITS CREATED = NONE
PUSHES = NONE
HARDWARE ACTIONS = NONE
G3 STARTED = NO

12:54 PM
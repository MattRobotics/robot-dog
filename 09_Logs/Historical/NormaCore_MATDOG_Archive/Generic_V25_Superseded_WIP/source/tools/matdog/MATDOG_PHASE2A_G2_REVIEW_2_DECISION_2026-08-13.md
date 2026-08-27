# MATDOG — Phase 2A G2 Review #2 Gate Decision
## Architecture-owner rulings after independent Codex review #2 — 2026-08-13

## Status

```text
P2A-G0             = CLOSED / PASS
P2A-G1             = CLOSED / PASS
P2A-G2 REVISION 0  = BLOCKED
P2A-G2 REVISION 1  = BLOCKED
P2A-G2             = OPEN
P2A-G3             = NOT AUTHORIZED
```

The independent review #2 is archived verbatim at:

```text
tools/matdog/MATDOG_PHASE2A_CODEX_G2_REVIEW_2_2026-08-13.md
sha256 e2760ad4413b526427f73fd5020ba1720a36bf28b657a569042599652947d533
```

Reviewed at `f4c5a500697d04e4c45641385170a4a630a0f38a`. Codex modified nothing.

ChatGPT accepts the review #2 findings as materially valid, including the four new
BLOCKERs, the four MAJORs and the MINOR documentation defects, and the closure matrix:

```text
CLOSED             B1, M7, N3
PARTIALLY_CLOSED   B2, M1, M3, M4, M5, M6, N1, N2
REGRESSED          M2
STILL_OPEN         B3
```

This correction gate revises exactly one document:

```text
tools/matdog/MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md
```

All earlier evidence documents remain byte-identical.

---

## Independent verification performed for these rulings

Every factual claim below was re-derived from primary sources before ruling, not taken
from the review narrative.

### From immutable LF V25 source (`matdog.rs`, blob `65c16f3c…`, identical on the release ref)

```text
normalize_all_matdog_joints_to_q0()          loops over MATDOG_MOTOR_IDS — all 12
  recovery predicate (line 3708-3709):
      distance > STATIC_TOLERANCE_TICKS (10)  ||  lf_initial_recovery_needed(before)
      where lf_initial_recovery_needed = distance > 16 || speed > 4
  => the EFFECTIVE predicate is  distance > 10 || speed > 4
  => Revision 1's ">16 / speed>4" is wrong: M11 at HOME+11 moves in immutable LF
     and would have been skipped.

lf_hip_sequence_profile(side)  (line 465)     OVERWRITES prerequisites for BOTH sides:
      LH Upper  UPPER_30_DELTA    (341)
      LF Upper  UPPER_90_DELTA   (1024)
      LF Lower  LOWER_FOLDED_DELTA (-990)
  run_lf_state_machine (line 2934) calls it for Min AND Max.
  => full-session Hip MAX uses 1024, NOT 967.
  => hip_upper_clearance_delta()'s 967 belongs to the STANDALONE build_profile path only.

staged return (lines 3030-3050)               remove_held_target(N) BEFORE move_motor_to,
                                              upsert_held_target(N) AFTER stable arrival.
global_torque_off_verified() (line 4411)      transition(Cleanup) -> sync-write OFF to all 12
                                              -> verify all 12 -> THEN held_targets.clear()
                                              -> complete_verified_cleanup() -> TorqueOff.
  => holds persist through the torque-OFF write; Revision 1's "Cleanup held = {}" was wrong.

approach_with_scout (line 3997)               commands advance_tick(target, probe_sign, step)
                                              bounded by the GUARD, not by the contact
                                              acceptance corridor.
  => Revision 1's "each probe goal inside the adaptive acceptance corridor" makes LF
     probing unreachable. Confirmed.

prepare_motor (line 3884)                     GoalPosition := PRESENT POSITION before torque on.
acquire_moving_current_baseline (line 3909)   GoalPosition := profile.baseline_target_tick.
acquire_moving_current_baseline_forward (3309) GoalPosition := advance_tick(present, sign, 64).
stop_pressure (line 4250)                     GoalPosition := present position.
  => none of these four has a valid purpose in Revision 1's six-purpose enum. Confirmed.

progress: 16 non-contact next_phase() calls in run_lf_state_machine
          + 7 per contact side in measure_lf_contact_side_efficient x 6 sides = 42
          = 58 exactly.
```

### From canonical robot-dog artifacts (`origin/main` = `bd5aa8ed…`)

```text
D/W4 is the canonical DEPLOYMENT materialization; C/W1 is the determinism oracle.
The final external safety policy consumes the D/W4 parking artifact
(input_parking_artifact file_sha256 = e561e7fb…), confirming the D/W4 chain.

D/W4 run manifest content sha256   4e7172d473b5d19c79112252e50212565ef73505551bff993b6a0c927f2aaee7
D/W4 endpoint_profile  file sha256 dd8cb42c3b916d067f97a321c5ffcdfb013dde1f2f2a6ba71f73becff360dc0f
D/W4 parking_json      file sha256 e561e7fb98880843e590f4676e8559374c51d0e641102f89a803b3619722c4d7
D/W4 combined_profile  file sha256 448ebcb3ed56d7f5225e6f9906e2efeb622010d8ecc457fe9ad00bda6407b00c
final policy JSON      file sha256 82f00a9414df06676babed1101a73e1a9d7bfa126592cdd7a425f66d5f01f1cc
final LF reconciliation JSON       d7fa04e2b8cde6b1049d4c34c5ba15fa1febde8b2fbf9fd8f14853034b98cc9a

Semantic hashes are IDENTICAL between C/W1 and D/W4
(endpoint de205209…, parking 67c58430…, combined 0a772234…) — which is precisely why
semantic equality must NOT be treated as provenance identity. Revision 1 pinned the
C/W1 file hashes. Confirmed defect.

target_domain does NOT live in the endpoint profile. It lives in
PATH_PARKING.plans[].target_domain. Revision 1's IMP-7 named the wrong artifact.
Confirmed defect.
```

---

## R2-1 — Distinct offline and runtime nominal types

```text
R2-1.1  Phase 2A must distinguish OfflineLegCalibrationSpec from ArmableLfSessionSpec,
        or semantically equivalent NOMINALLY DISTINCT types.
R2-1.2  RF/RH/LH may produce ONLY offline objects.
R2-1.3  There must be NO public/general conversion
            OfflineLegCalibrationSpec -> ArmableLfSessionSpec
R2-1.4  In Phase 2A only the LF V25 reviewed boundary can produce an ArmableLfSessionSpec.
R2-1.5  The LF constructor must verify exact LF identity: leg, joint names, motor IDs,
        directions, limits, historical oracle identity, required session mode.
R2-1.6  An LF label alone is NEVER authority.
```

Revision 1's `offline::validated_spec()` returned `ValidatedLegSpec`, the same nominal
type the engine accepts and the same type that owned goal-minting methods. Call-graph
prose does not make an object inert; the type must.

## R2-2 — Runtime arm resolution is separate from profile enumeration

```text
R2-2.1  All 24 contact profiles MAY remain available for OFFLINE enumeration,
        geometry/spec validation and tests.
R2-2.2  They MUST NOT automatically be runtime-armable.
R2-2.3  No RF/RH/LH arm value is allowed in Phase 2A runtime resolution.
R2-2.4  Preserve only explicitly reviewed LF runtime compatibility sentinels required
        by immutable LF V25 behavior. At minimum review explicitly:
            LF_LEG_STATE_MACHINE
            LF_HIP_M13_MIN_MAX
R2-2.5  Any additional historical LF single-contact arm token must be justified
        individually against the immutable LF oracle before being kept runtime-armable.
R2-2.6  Do not preserve run_profile as an undocumented second physical executor.
```

Today `all_profiles()` yields six profiles per leg; `hardware_profile_allowed` blocks
only isolated HIP. Eighteen RF/RH/LH resolver entries exist and twelve non-LF
Upper/Lower profiles are physically dispatchable through `run_profile`. Revision 1's
CI row M-16 would have pinned that set. Both are rejected.

## R2-3 — One engine, explicit reviewed LF session modes

```text
R2-3.1  The target remains ONE LegSessionStateMachine.
R2-3.2  If immutable LF requires more than one session mode, model them explicitly
        inside the same engine grammar (e.g. LfFullLegSession, LfHipPairLegacy).
R2-3.3  Do NOT create separate engines.
R2-3.4  Do NOT leave a generic legacy run_profile motion path beside the engine.
```

## R2-4 — Dynamic goal authority must be non-forgeable and non-replayable

```text
R2-4.1  Do NOT use a freely constructible AuthorizedGoal { .. } in the engine module.
R2-4.2  Goal authority lives behind a nested private/sealed authority boundary.
R2-4.3  A dynamic goal capability MUST:
            NOT implement Clone
            NOT implement Copy
            be CONSUMED by the command operation
            be bound to current session identity
            be bound to current phase/state
            be bound to current operation/epoch
            be bound to motor
            be bound to purpose
            carry only an unsigned 0..=4095 tick
R2-4.4  Before command emission, engine state must verify the capability still belongs
        to the active session/phase/operation. A stale token FAILS CLOSED.
R2-4.5  A diagnostic or future direct-geometry target must not be forgeable by code
        elsewhere in the calibrator module.
```

Revision 1's `AuthorizedGoal` derived `Clone + Copy`, carried no session/phase/epoch,
had private fields but no sealed module, and was therefore both forgeable in-module and
replayable after state changes.

## R2-5 — Complete LF goal-intent inventory before defining enums

```text
R2-5.1  Do NOT invent a fixed purpose enum first.
R2-5.2  Re-read immutable LF V25 and inventory EVERY GoalPosition-producing operation.
R2-5.3  At minimum classify: prime at present position before torque enable; global
        q0/home normalization; prerequisite move; historical LF parking move;
        moving-baseline target; coarse probe advance; fine probe advance;
        stop-pressure at present position; backoff; transition/static hold move;
        staged affine q0; parking restore; and any additional actual LF
        GoalPosition-producing operation found in source.
R2-5.4  For every source call site state: source operation, required engine phase,
        required motor role, value source, corridor/guard, authority purpose,
        whether the token must be one-shot, regression witness/test.
R2-5.5  Only AFTER that inventory may the design define the minimum internal
        MotionIntent / authority vocabulary.
```

## R2-6 — Direct geometry target is design-only

```text
R2-6.1  The eligibility predicate for a future direct Geometry V5 target MAY remain
        documented.
R2-6.2  Do NOT require a DirectGeometryTarget enum variant, minting function or command
        path to be implemented in G3.
R2-6.3  Phase 2A grants zero direct Geometry motion authorization.
R2-6.4  Direct geometry execution requires a separate future gate.
```

NOTE-A from revision 1 is resolved by this ruling: keep the predicate and its denial
tests; do not implement the capability.

## R2-7 — Preserve exact LF startup

```text
R2-7.1  Immutable LF V25 requires, in order:
            verified exact MATDOG set
            verified global torque OFF
            normalization of ALL 12 MATDOG motors to q=0
            then LF session creation/recovery
R2-7.2  Do NOT reduce normalization to LF participant motors.
R2-7.3  Reconstruct and preserve the exact effective recovery predicates/tolerances
        from immutable source.
R2-7.4  Do NOT replace a 10-tick behavior with a 16-tick approximation.
```

The verified effective predicate is `distance > 10 || speed > 4`, applied to each of the
twelve canonical motors.

## R2-8 — LF prerequisites are context-specific

```text
R2-8.1  Do NOT assign one universal prerequisite role/value to an endpoint.
R2-8.2  Distinguish at least: standalone-contact prerequisites; full-session historical
        prerequisites; passive torque-OFF at-home requirements; accumulated
        full-session held targets.
R2-8.3  M42 historical full-session parking remains LF V25 evidence.
R2-8.4  LF HIP full-session MUST preserve M12 at UPPER_90_DELTA / 1024 for BOTH Hip
        contact sides. Do NOT substitute UPPER_85_DELTA / 967 into full-session Hip MAX.
R2-8.5  During staged return: remove the joint from held state BEFORE moving it, then
        re-add only after stable arrival.
R2-8.6  During Cleanup: preserve session/held state THROUGH verified global torque OFF,
        then clear holds and complete the TorqueOff terminal state.
```

## R2-9 — V5 path/parking evidence is endpoint-scoped

```text
R2-9.1  Geometry V5 endpoint parking/path evidence stays associated with the specific
        endpoint record that produced it.
R2-9.2  Do NOT promote an endpoint-specific parking plan into a whole-session parking
        hold.
R2-9.3  LF historical M42 whole-session parking is separate LF V25 historical behavior
        and is NOT derived from V5 endpoint parking.
```

V5 validates RF M32 parking for `rf_upper_leg_joint:max` only — not for every RF
contact and not for a combined session.

## R2-10 — Canonical V5 materialization is D/W4

```text
R2-10.1  Canonical deployment source is
             2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_*
R2-10.2  C/W1 is the determinism oracle ONLY.
R2-10.3  Semantic equality does NOT permit provenance identity substitution.
R2-10.4  Revise every canonical source/file/hash reference accordingly.
```

## R2-11 — Define the trusted manifest precisely

```text
R2-11.1  Do NOT leave TrustedManifest undefined.
R2-11.2  Design a TrustedGeometryManifest whose authority derives from the exact
         canonical D/W4 bundle manifest and exact artifact identities.
R2-11.3  State explicitly WHERE verification happens.
R2-11.4  For G3: NO Geometry V5 parser/importer is required. G3 is LF generic-engine
         extraction and LF behavioral preservation.
R2-11.5  For later non-LF offline spec work: the offline importer/validator must
         read/verify the canonical D/W4 artifact records before creating validated
         offline spec objects.
R2-11.6  Exact record binding must cover the source where each field ACTUALLY lives.
R2-11.7  Do NOT claim target_domain belongs to an artifact that does not contain it.
R2-11.8  Policy/reconciliation artifact identities must be independently pinned/bound.
R2-11.9  Distinguish the physical LF hardware evidence artifact from the generated LF
         geometry/hardware reconciliation artifact.
```

## R2-12 — LF hardware oracle must not be spoofable

```text
R2-12.1  LF V25 hardware/contact evidence is obtainable ONLY after exact LF canonical
         identity validation.
R2-12.2  A raw object labelled "LF" but containing RF/RH/LH motors, directions, joint
         names or limits MUST fail before any LF oracle/probe authority exists.
R2-12.3  The LF oracle remains opaque and JointKind-keyed.
R2-12.4  No generic HardwareContactEvidence value may grant authority merely because
         its enum variant says supervised/validated.
```

## R2-13 — Progress derives from the actual operation trace

```text
R2-13.1  Do NOT maintain an expected total independently.
R2-13.2  The engine-expanded operation trace is the source.
R2-13.3  The progress count is derived mechanically from exactly those operations that
         emit progress.
R2-13.4  DONE requires actual executed progress == mechanically derived expected progress.
R2-13.5  LF regression must prove 58 ACTUAL increments and the historical phase
         strings/order.
R2-13.6  Do NOT merely assert that a function returns 58.
```

## R2-14 — CI must test the correct layer

```text
R2-14.1  Do NOT require the serialized port layer to know whether identical command
         bytes originated from a typed authority object.
R2-14.2  Authority/provenance tests belong at the calibrator/engine boundary BEFORE
         type erasure into command bytes.
R2-14.3  The port layer continues to enforce its real byte-level/RAM/motor safety
         contract.
R2-14.4  Remove or redesign impossible assertions such as T-3.
R2-14.5  Do NOT freeze an incomplete MotionIntent enum in CI.
R2-14.6  Do NOT pin non-LF runtime resolver entries.
R2-14.7  Define how all new calibrator modules are RECURSIVELY included in source safety
         scans, rustfmt/check, and observer authority scans.
R2-14.8  Do NOT add a new test dependency merely to prove Rust privacy unless separately
         justified.
```

## R2-15 — Complete non-LF offline specs without creating runtime authority

```text
R2-15.1  RF/RH/LH design records must be complete enough for offline validation:
             independent joint identities, independent motor IDs, directions,
             independent limits, joint/contact ordering data, endpoint record
             references, target domains, external policy records, endpoint-scoped
             prerequisite/path/parking references, canonical D/W4 provenance.
R2-15.2  No shared-limit / mirror / default constructor is authoritative.
R2-15.3  The resulting type remains OFFLINE ONLY and is not accepted by the runtime
         engine.
R2-15.4  Do NOT infer a session-wide parking plan from an endpoint-specific V5 plan.
```

## R2-16 — Global safety ownership remains as previously corrected

```text
R2-16.1  M7 remains CLOSED.
R2-16.2  Station ownership, serial ownership, bus identity enforcement, observer
         boundary, launcher pin mechanism, torque/current/temperature/freshness,
         topology, RAM allowlist, global torque-off, operator stop, q0 protection and
         EEPROM separation remain C / global safety.
R2-16.3  Correct the documentation detail that ID 115 has BOTH a C safety-separation
         aspect and a D LF-specific evidence aspect.
```

## R2-17 — Contact rule remains closed

```text
R2-17.1  Do NOT alter the already-correct contact semantics.
R2-17.2  Preserve exactly: the narrow immutable LF pre-final fine-pass friction/chamfer
         qualification, then FINAL ACCEPTED CONTACT -> immediate stop_pressure -> no
         further forward motion.
R2-17.3  Outside predicted band in future hardware: stop, backoff, record disagreement,
         fail closed, verified global cleanup.
```

Review #2 found no regression in the contact rule. It stays as written.

## R2-18 — Single-session only

```text
R2-18.1  The M42 participant/parking role is valid only inside ONE selected session.
R2-18.2  Phase 2A makes NO concurrency claim.
R2-18.3  No concurrent multi-leg session support is authorized.
R2-18.4  Future concurrency requires separate resource arbitration and architecture
         review.
```

NOTE-B from revision 1 is resolved by this ruling. Current startup signals a previous
task to stop without proving completion, so no concurrency-safety claim may be inferred.

---

## Minor documentation defects accepted for correction

```text
D-1  §12.3 said "eight test names preserved, two renamed". Actual: nine preserved,
     only T-1 renamed.
D-2  OPEN-2 was incorrectly attributed as "resolved by R2"; it is resolved by the
     OPEN-2 ruling of the revision-1 review decision.
D-3  ID 115 must show BOTH its C aspect (EEPROM separation / separate-binary boundary)
     and its D aspect (LF-specific motor list and evidence).
D-4  "Four distinct hip contact values" must read "four distinct per-leg hip endpoint
     pairs".
```

---

## Scope of this correction gate

Authorized:

```text
archive review #2 verbatim
record these rulings
coherently rewrite tools/matdog/MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md
one local documentation-only commit
```

Not authorized:

```text
any .rs change            any .py runtime change
any test change           any workflow change
Station / robot-dog       historical RF worktree
push / PR                 hardware / serial / EEPROM
G3
```

Documents that must remain byte-identical:

```text
37646f2eaf0f56af558b2450ada7908c44b8b1fdb394f3095fb99c25c651bc17  G0/G1 report
89df3920461d347d372f30fb44aa0eefb016543f45b4f5615d8b7309fd51433d  G1 gate decision
c51fb5d74c1752302e886febacde213c6748b70078ecc898ccd97efe5b9fbf6d  Codex review #1
7a8590437cefacae49f7e02c918602642de6a69caca29152669b374aea2670b2  G2 review #1 decision
```

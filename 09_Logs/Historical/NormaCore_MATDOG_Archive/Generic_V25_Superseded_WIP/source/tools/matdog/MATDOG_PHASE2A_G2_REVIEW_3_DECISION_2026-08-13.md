# MATDOG — Phase 2A G2 Review #3 Gate Decision
## Architecture-owner rulings after independent Codex review #3 — 2026-08-13

## Status

```text
P2A-G0             = CLOSED / PASS
P2A-G1             = CLOSED / PASS
P2A-G2 REVISION 0  = BLOCKED
P2A-G2 REVISION 1  = BLOCKED
P2A-G2 REVISION 2  = BLOCKED
P2A-G2             = OPEN
P2A-G3             = NOT AUTHORIZED
```

Independent review #3 archived verbatim at:

```text
tools/matdog/MATDOG_PHASE2A_CODEX_G2_REVIEW_3_2026-08-13.md
sha256 77829ec1606835ca8d0ee0f86e986bb15d5971fe8e5e8f394e148f0c71cc7951
```

Reviewed at `f2115c57a6ec270ff9cf58ded9a8c37acf77f9dd`. Codex modified nothing.

This is a **TARGETED revision 2.1**, not another broad rewrite. The following
revision-2 sections are independently validated and are preserved:

```text
five-axis B1 semantics                      D/W4 deployment identity
external PASS deny-only semantics           hardware evidence non-authorizing
full-session all-12 startup normalization   effective distance>10 || speed>4
full-session M12=1024 Hip prerequisite      staged remove-before-move/re-add-after
cleanup held-target timing                  endpoint-scoped V5 parking
LF final-contact semantics                  DirectGeometryTarget excluded from G3
global C safety ownership
```

---

## Independent verification performed for these rulings

### LF single-contact runtime support (immutable source)

```text
build_profile  :441   arm_value = "{LEG}_{JOINT}_M{motor}_{SIDE}"
profile_for_arm_value :511-534   recognizes all 24 single-contact tokens
hardware_profile_allowed :536-543   blocks ONLY isolated HIP

matdog_test.rs :279-286
    profile_for_arm_value("LF_HIP_M13_MIN")   -> hardware_profile_allowed = Err
    profile_for_arm_value("LF_LOWER_M11_MIN") -> hardware_profile_allowed = Ok
matdog_test.rs :2493-2498
    profile_for_arm_value("LF_UPPER_M12_MIN") -> Ok
```

Confirmed: `LF_UPPER_M12_MIN/MAX` and `LF_LOWER_M11_MIN/MAX` are **supported immutable
V25 runtime hardware modes**. Revision 2's ARM-2 removed them without authorization.

### The three legacy progress traces (immutable source, exact)

```text
LfFullLegSession    run_lf_state_machine      16 direct + 7x6 = 58   total_steps=58 :2531
LfHipPairLegacy     run_lf_hip_min_max        20 direct              total_steps=20 :2454
LfSingleContactLegacy  run()                  16 direct              total_steps=16 :2385
```

### Mode-distinguishing behavior (immutable source)

```text
Full     baseline: acquire_moving_current_baseline_forward (RELATIVE)      :3294
         fine:     approach_with_scout(FINE, Some(coarse_scout_tick)) x2   :3259,:3267
HipPair  baseline: acquire_moving_current_baseline (ABSOLUTE)              :2760,:2793
         coarse:   approach(COARSE)  = scout None                          :2763,:2796
         fine:     approach(FINE)    = scout None  -- ONE fine pass/side    :2770,:2803
         between-side M13 HOME return                                       :2775-2779
Single   baseline: acquire_moving_current_baseline (ABSOLUTE)              :2656
         coarse:   approach(COARSE)  = scout None                          :2659
         fine:     approach_with_scout(FINE, Some(coarse_scout_tick)) x2   :2666,:2674
         LIFO prerequisite restore + post-restore settle                    :2690-2712
```

Codex is correct: **Hip-pair fine passes use `scout=None`.** Revision 2's TABLE A row A9
claimed `Some(...)` for Hip-pair, which would have added a friction/chamfer continuation
that immutable Hip-pair execution does not have.

### Startup scoping

```text
Full     wait_for_exact_motor_set -> global_torque_off_verified
         -> normalize_all_matdog_joints_to_q0 (ALL 12)                      :2851-2858
HipPair  wait_for_exact_motor_set -> global_torque_off_verified
         -> inspect_profile_entry -> recover_home_only_joints
         -> establish_prerequisites_restart_safe                            :2738-2752
Single   identical restart-safe entry shape                                 :2634-2648
```

Confirmed: all-12 normalization belongs to **Full only**. Applying it to HipPair or
Single would change their legacy entry choreography.

### HOME write paths are distinct

```text
recover_home_only_joints :3452-3459
    prepare_startup_home_recovery_motor  -> set_startup_home_goal_verified(m, HOME) :3787
    move_profile_entry_motor_to_target(..., startup_writer = TRUE) -> :3532
establish_prerequisites_restart_safe :3491-3498
    prepare_motor -> set_motor_goal_verified(m, present)                    :3884
    move_profile_entry_motor_to_target(..., startup_writer = FALSE) -> :3535
```

Confirmed: revision 2's A5 wrongly attributed the home-only recovery write to
prerequisite motion. They are different writers with different gates.

### Stop-pressure evidence sources differ

```text
fresh observation value   :4054 :4058 :4150 :4154   stop_pressure(m, observation.position)
saved accepted-contact    :3220 stop_pressure(m, minimum.second_tick)
                          :3245 stop_pressure(m, previous)
                          :2681 :2776 :2809 stop_pressure(m, <saved second tick>)
```

Confirmed: revision 2's A10 conflated two materially different evidence bindings.

### Provenance identities newly pinned

```text
D/W4 run manifest FILE sha256
    0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17     VERIFIED
LF reconciliation CONTENT sha256 (field `reconciliation_content_sha256`)
    0af31e9dbcae0aae22978faeaf12a668c8d402abf921d2d63cea49f11195e70c     VERIFIED
```

Both recomputed from robot-dog `origin/main` `bd5aa8ed…` and match exactly.

### Existing CI harness capability

```text
present:  cargo test --package st3215; inline Python source-shape scan;
          explicit-file rustfmt
absent:   trybuild, compiletest, compile-fail fixtures, negative-trait infrastructure
```

Confirmed: revision 2's M-17r and M-27 claimed negative compile-time properties that an
ordinary compiling unit test cannot establish.

---

## R3-1 — Preserve all currently supported LF V25 runtime modes

```text
R3-1.1  Revision 2 incorrectly removed four supported LF single-contact hardware modes.
R3-1.2  The one-engine runtime modes for Phase 2A are:
            LfFullLegSession
            LfHipPairLegacy
            LfSingleContactLegacy { joint: UpperOrLower, side: ContactSide }
R3-1.3  The historically supported single-contact combinations are exactly:
            LF_UPPER_M12_MIN   LF_UPPER_M12_MAX
            LF_LOWER_M11_MIN   LF_LOWER_M11_MAX
        They MUST remain runtime-supported through the single engine.
R3-1.4  LF_HIP_M13_MIN and LF_HIP_M13_MAX remain recognized historical profile data but
        remain hardware BLOCKED exactly as immutable V25 requires.
R3-1.5  RF/RH/LH runtime arming remains FORBIDDEN in Phase 2A.
R3-1.6  No behavior deprecation is authorized.
```

## R3-2 — One engine, three exact LF mode expansions

```text
R3-2.1  There is exactly one LegSessionStateMachine covering Full, HipPair and Single.
R3-2.2  There is no second run_profile executor.
R3-2.3  The three modes MUST preserve their distinct immutable behavior:

  Full     exact full-session startup and 58-operation trace
  HipPair  restart-safe legacy entry, shared prerequisites, absolute moving baseline,
           historical MIN/MAX choreography, historical fine behavior, between-side M13
           HOME return, restore behavior, exact 20-operation trace
  Single   restart-safe legacy entry, historical prerequisites, absolute moving baseline,
           coarse scout, two fine passes using the immutable scout behavior, HOME return,
           LIFO prerequisite restore, post-restore settle, exact 16-operation trace

R3-2.4  Do NOT normalize these three traces merely for architectural elegance.
```

## R3-3 — Remove first-class MotionGrant from the design

```text
R3-3.1  Revision 2's first-class MotionGrant capability is WITHDRAWN.
R3-3.2  Do NOT replace it with additional nonce/session/epoch token machinery.
R3-3.3  Phase 2A does not require a reusable motion-authority token.
R3-3.4  The safer, smaller design is IMMEDIATE ENGINE-OWNED AUTHORIZATION + EMISSION:
            UnsignedTick remains range proof only
            the raw GoalPosition sink is PRIVATE
            only intent-specific private engine operations may reach that sink
            each operation derives its evidence internally, checks current
              mode/state/motor/observation/guard immediately, and emits in the same call
R3-3.5  There is no "authorize now / store capability / emit later" path, and therefore
        no first-class motion token to clone, copy, store, replay, expire or revoke.
```

## R3-4 — Intent-specific engine motion API

```text
R3-4.1  Define the minimum engine-owned internal motion operations from immutable source.
        At minimum: home normalization prime; prime at present observation;
        prerequisite / historical parking move; moving baseline step; probe advance step;
        stop pressure at present / at accepted contact; backoff; static hold transition;
        staged affine q0; return home.
R3-4.2  Re-derive EVERY actual LF GoalPosition write from immutable source before
        freezing the final operation inventory.
R3-4.3  Corrections required in TABLE A:
            A2 is NOT a GoalPosition-producing operation — remove it from a write inventory
            separate every actual HOME write path instead of conflating them
            correct mode attribution for relative vs absolute baseline
            correct HipPair fine behavior (scout = None)
            separate fresh-observation stop-pressure writes from accepted-contact-value ones
            represent pre-session normalization explicitly
R3-4.4  No DirectGeometryTarget operation exists in G3.
```

## R3-5 — Raw sink call-site control

```text
R3-5.1  The lowest GoalPosition-writing helper is PRIVATE to the MATDOG engine boundary.
R3-5.2  Every normal GoalPosition write originates from one of the reviewed
        intent-specific engine methods.
R3-5.3  CI enumerates and checks the raw sink call sites; adding a new raw sink caller is
        a review-visible change.
R3-5.4  Verified global torque-off, hard abort, operator stop and equivalent terminal
        safety paths remain INDEPENDENT of normal GoalPosition authorization.
R3-5.5  If stop_pressure cannot safely perform its immediate pressure-release write, the
        engine falls through to the existing verified global torque-off safety path.
```

## R3-6 — Sealed LF armable brand

```text
R3-6.1  ArmableLfSessionSpec is created behind a nested module / private-constructor
        boundary, equivalent in strength to the intended sealed authority boundary.
R3-6.2  Safe code outside that private module cannot construct its inner representation.
R3-6.3  The ONLY safe producer is exact LF V25 validation, covering leg, joint names,
        motor IDs, directions, limits, parking identity when applicable, session mode and
        LF oracle identity.
R3-6.4  An LF string/enum label is never authority.
R3-6.5  Offline/non-LF objects have no conversion to the sealed LF armable brand.
```

## R3-7 — CI: do not claim impossible negative unit tests

```text
R3-7.1  Withdraw revision-2 claims that a normal compiling unit test proves invalid code
        cannot compile.
R3-7.2  No new trybuild/compiletest dependency is authorized.
R3-7.3  Use: compiler-enforced API/type signatures as the primary invariant; runtime
        tests for positive/rejection behavior that can actually execute; existing-tool
        source-shape CI as secondary structural evidence.
R3-7.4  For offline -> armable separation: assert the engine signature accepts only the
        sealed armable type; assert via source-shape scan that no public/general
        conversion implementation exists; runtime-test that validation rejects every LF
        identity mismatch.
R3-7.5  For private raw GoalPosition authority: source-shape CI enumerates every raw sink
        caller and refuses unreviewed callers.
R3-7.6  Do not claim more than these checks establish.
```

## R3-8 — CI file coverage must be mechanically simple

```text
R3-8.1  Do NOT require a custom Rust module-graph parser.
R3-8.2  For the MATDOG calibrator safety scan, conservatively inspect EVERY .rs file
        under the relevant auto_calibrate MATDOG directory tree. A new Rust file is
        therefore included automatically.
R3-8.3  For Python MATDOG observer/profile safety boundaries, use a conservative,
        explicitly-defined file-tree scan of the relevant tools/matdog Python scope.
R3-8.4  Do not claim full dynamic-import resolution. If dynamic import mechanisms are
        present in the protected scope, FAIL CLOSED and require separate review.
R3-8.5  Use only standard shell/Python/Rust tooling already present.
```

## R3-9 — Exact three legacy progress oracles

```text
R3-9.1  Document and derive:
            LfFullLegSession       = 58 actual progress emissions
            LfHipPairLegacy        = 20 actual progress emissions
            LfSingleContactLegacy  = 16 actual progress emissions
R3-9.2  For each mode produce a complete ORDERED oracle table.
R3-9.3  The oracle contains the EXACT externally emitted progress strings, including the
        dynamic profile-label prefix produced by next_phase(), not merely the inner
        phase argument.
R3-9.4  Document profile-label changes through each mode.
R3-9.5  Expected progress is derived from the expanded actual operation trace.
R3-9.6  No standalone numeric constant is sufficient evidence.
```

The emitted form is `format!("{}: {phase}", self.profile.label)` (`publish_progress`
:4744-4758), so the label prefix is part of the externally observable string.

## R3-10 — Correct TABLE A

```text
R3-10.1  Re-read all actual GoalPosition-producing source call paths.
R3-10.2  Produce a corrected table containing ONLY writes.
R3-10.3  Every row includes: actual source write path; session mode(s); state/phase;
         motor role; target-value origin; required observation/evidence; guard/corridor;
         engine-owned immediate motion operation; failure/cleanup behavior;
         regression witness.
R3-10.4  Do not count helpers that emit no GoalPosition.
R3-10.5  Do not merge write paths with materially different value/evidence sources merely
         to reduce row count.
```

## R3-11 — HipPair and Single startup must not inherit Full startup

```text
R3-11.1  All-12 q0 normalization belongs to immutable LfFullLegSession.
R3-11.2  Do NOT automatically apply it to HipPair or Single legacy sessions.
R3-11.3  HipPair and Single preserve their existing restart-safe profile-entry behavior
         exactly.
R3-11.4  The generic engine may share implementation primitives but must not change a
         mode's observable or physical choreography.
```

## R3-12 — Provenance trust-root completion, implementation deferred

```text
R3-12.1  Keep D/W4 as canonical deployment.
R3-12.2  Complete the DESIGN of an immutable expected trust root ExpectedGeometryV5DW4
         with exact expected artifact identities, including:
             run manifest FILE SHA
                 0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17
             run manifest CONTENT SHA
                 4e7172d473b5d19c79112252e50212565ef73505551bff993b6a0c927f2aaee7
             reconciliation CONTENT SHA
                 0af31e9dbcae0aae22978faeaf12a668c8d402abf921d2d63cea49f11195e70c
         plus endpoint, parking, combined, policy, reconciliation, URDF, compiler and the
         17 mesh identities already verified.
R3-12.3  Define ArtifactRef precisely: relative artifact identity/path; expected file SHA;
         optional expected semantic SHA; schema/role identity where applicable.
R3-12.4  The future importer compares bytes against this COMPILED EXPECTED TRUST ROOT.
         A bundle cannot authenticate itself merely because its internal hashes are
         self-consistent.
R3-12.5  Cross-bind compiler/URDF/17-mesh identities against the expected root.
R3-12.6  G3 MUST NOT IMPLEMENT THIS IMPORTER. The Geometry importer and the
         OfflineLegCalibrationSpec runtime representation remain for the later non-LF
         offline gate.
```

## R3-13 — Defer non-LF offline implementation from G3

```text
R3-13.1  The G2 contract may retain RF/RH/LH canonical data and future offline-spec design.
R3-13.2  G3 does NOT need to implement OfflineLegCalibrationSpec, the Geometry V5
         importer, TrustedGeometryManifest, or RF/RH/LH engine/runtime code.
R3-13.3  G3 implementation surface is limited to what LF preservation requires.
R3-13.4  This reduces risk and does not weaken the later G5 design obligation.
```

## R3-14 — Session participants ≠ V5 endpoint path dependencies

```text
R3-14.1  Runtime session participants are derived from the SELECTED LF session mode.
R3-14.2  Offline V5 endpoint parking/path participants remain properties of their
         endpoint records only.
R3-14.3  A V5 endpoint-local parking dependency MUST NOT automatically enter a
         session-wide participant set.
```

This resolves the residual NM2 contradiction: revision 2's §6.4 unioned a parking motor
whenever any parking reference existed, while RF correctly declared participants
`{21,22,23}` despite endpoint #9 carrying an M32 parking plan.

## R3-15 — Contact and global safety remain closed

```text
R3-15.1  Do not alter: FINAL ACCEPTED CONTACT -> stop_pressure immediately -> no further
         forward probe.
R3-15.2  Do not generalize the narrow immutable LF pre-final friction/chamfer
         qualification.
R3-15.3  Global torque-off, hard abort, operator stop and terminal cleanup remain global
         C safety behavior, independent from normal per-goal authorization.
```

---

## Minor documentation defects accepted for correction

```text
E-1  "A MotionGrant cannot be stored" was false Rust wording. The whole construct is
     withdrawn under R3-3, so the wording disappears with it.
E-2  TABLE B overstated session nonexistence throughout Preflight: an error after
     inspect_lf_native_session_entry() and during transition/role verification occurs
     after the session object exists.
E-3  §16.4 imported rows "as written in revision 1" although revision 2 supersedes
     revision 1 in full; the rows must be restated self-containedly.
E-4  Offline-importer CI rows must be assigned to the later non-LF gate, since G3
     prohibits that importer.
```

---

## Scope of this correction gate

Authorized:

```text
archive review #3 verbatim
record these rulings
TARGETED revision of tools/matdog/MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md
one local documentation-only commit
```

Not authorized:

```text
another broad architecture rewrite      any .rs / .py runtime / test / workflow change
Station / robot-dog                     historical RF worktree
push / PR                               hardware / serial / EEPROM
G3
```

Documents that must remain byte-identical:

```text
37646f2eaf0f56af558b2450ada7908c44b8b1fdb394f3095fb99c25c651bc17  G0/G1 report
89df3920461d347d372f30fb44aa0eefb016543f45b4f5615d8b7309fd51433d  G1 gate decision
c51fb5d74c1752302e886febacde213c6748b70078ecc898ccd97efe5b9fbf6d  Codex review #1
7a8590437cefacae49f7e02c918602642de6a69caca29152669b374aea2670b2  G2 review #1 decision
e2760ad4413b526427f73fd5020ba1720a36bf28b657a569042599652947d533  Codex review #2
4f1c279fb0f91aaa512194aca8c2f9aaaefa6fa548b0cf8210d33b2ad5c5752c  G2 review #2 decision
```

# MATDOG — Phase 2A G2 Final Review Decision
## Architecture-owner rulings after the Codex final G2 review — 2026-08-13

## Status

```text
P2A-G0 = CLOSED / PASS
P2A-G1 = CLOSED / PASS
P2A-G2 = OPEN
P2A-G3 = NOT AUTHORIZED
```

Final review archived verbatim at:

```text
tools/matdog/MATDOG_PHASE2A_CODEX_FINAL_G2_REVIEW_2026-08-13.md
sha256 cb0e938b42266b398df58fac50007d2e00d59dbd087f6da230369755c220ae2a
```

Reviewed at `168bf108033a6b528d3c8c54a6dbfc0a20dea930`. Codex modified nothing.

This is a **surgical revision 2.2**. The architecture is not reopened. Sections the
final review closed are preserved unchanged:

```text
A  LF single-contact runtime surface        CLOSED
B  one engine / no second executor          CLOSED
E  sealed ArmableLfSessionSpec (nominal)    CLOSED
I  D/W4 provenance trust-root design        CLOSED
L  contact and global-safety preservation   CLOSED
```

Only findings C, D, F, G, H, J, K and the MINOR documentation defects are corrected.

---

## Independent verification performed for these rulings

### F1 — the missing 23rd write path

```text
recover_home_only_joints :3430   (HipPair and Single legacy entry ONLY)
  :3452  prepare_startup_home_recovery_motor
             :3773-3785  TorqueLimit / Acc / GoalSpeed
             :3787       GoalPosition = HOME_TICK      <-- WRITE, torque OFF
             :3789       set_startup_home_torque_verified(true)   torque ON
  :3453  move_profile_entry_motor_to_target(.., startup_writer = true)
             :3532       GoalPosition = HOME_TICK      <-- WRITE, torque ON
```

Two physical writes with **different torque state and different preconditions**.
Revision 2.1 gave them one row (W2) whose precondition required torque OFF, so the
second write had no valid operation. Confirmed blocker.

By contrast the Full path writes only once:

```text
normalize_all_matdog_joints_to_q0 :3698
  :3720  prepare_startup_home_recovery_motor  -> :3787 WRITE, torque OFF
  :3721  move_startup_home_motor_to_q0 :3651  -> dwells on the existing goal, NO WRITE
```

The inventory therefore splits into three startup rows and totals **23**.

### F6 — increments versus the complete external stream

```text
publish_progress :4744   emits format!("{}: {phase}", self.profile.label)

step-zero preflight events (current = 0, status InProgress):
  :2386  run_profile              "MATDOG native profile preflight"
  :2455  run_lf_hip_min_max       "LF HIP MIN+MAX shared-geometry preflight"
  :2532  run_lf_full_calibration  "single-session LF native calibration preflight"

terminal event:
  :4761  mark_done -> publish_progress(total_steps, "completed", CalibrationStatus::Done)
```

```text
operational increments (next_phase)    Full 58   HipPair 20   Single 16
+ step-zero preflight                       1          1           1
+ terminal completed                        1          1           1
= complete successful external stream  Full 60   HipPair 22   Single 18
```

Confirmed. Revision 2.1 called the 58/20/16 list "the actual emitted strings", which is
inaccurate for an unfiltered external observer.

### Operation count

With F1's new torque-on reassertion operation the reviewed set is **twelve**, not ten or
eleven:

```text
 1 home_normalization_prime          W1  W2
 2 home_reassert_torque_on           W3        <-- new, F1
 3 prime_at_present                  W4
 4 prerequisite_or_parking_move      W5  W6
 5 moving_baseline_step              W7  W8
 6 probe_advance_step                W9  W10 W11
 7 stop_pressure_at_observation      W12
 8 stop_pressure_at_recorded_contact W13
 9 backoff_step                      W14
10 static_hold_transition            W15 W16
11 staged_affine_q0                  W17 W18 W19
12 return_home                       W20 W21 W22 W23
```

---

## F1 — Exact 23 GoalPosition authority/evidence paths

```text
F1.1  Revision 2.1's 22-row inventory is incomplete. The immutable source requires 23
      materially distinct paths.
F1.2  Split legacy home-only recovery into:
          A. torque-OFF HOME prime  — prepare_startup_home_recovery_motor / :3787
          B. torque-ON HOME reassertion — move_profile_entry_motor_to_target(
                                          startup_writer = true) / :3532
F1.3  Do NOT merge them: their torque state and authority preconditions differ.
F1.4  The final source-backed inventory contains all 23 and only real GoalPosition
      writes.
```

## F2 — Pre-session entry context

```text
F2.1  Full-session normalization happens before the LF session-state object exists.
F2.2  Define an internal engine Entry/Preflight context sufficient to authorize the
      startup HOME writes.
F2.3  This is ENGINE STATE, not a transferable motion token.
F2.4  Full and legacy modes retain their distinct entry behavior.
```

## F3 — Motion methods derive authority-relevant inputs internally

```text
F3.1  Intent-specific engine operations must not accept arbitrary caller-selected motion
      authority inputs.
F3.2  Current motor, target/reference, probe step, scout policy, contact value and
      staged evidence are DERIVED from the engine's current grammar node and current
      validated LF mode.
F3.3  Fresh observation values may enter only after being rebound to: the exact expected
      motor; the latest observation identity/stamp; freshness; the current
      operation/verdict.
F3.4  Examples:
          probe advance derives coarse/fine step from the current sub-operation;
          static hold derives its exact historical pose from current grammar state;
          staged q0 derives the current joint's accepted evidence from this session;
          return-home derives the exact currently selected restore/probe motor.
F3.5  A caller must not be able to select an off-trace target merely by calling a
      correctly-named method with different arguments.
```

## F4 — Seal the lowest GoalPosition write boundary

```text
F4.1  Identify the TRUE lowest MATDOG GoalPosition construction/emission boundary.
F4.2  Normal GoalPosition writes reach it only from the reviewed immediate engine
      motion operations.
F4.3  CI must conservatively detect: every direct RamRegister::GoalPosition
      construction; every helper capable of producing a GoalPosition write; every caller
      of the raw sink.
F4.4  A new direct/third writer FAILS the source-shape gate until reviewed.
F4.5  Global torque-off, hard abort, operator stop and terminal cleanup remain
      independent safety paths.
```

## F5 — Validate or internally derive every LF motion-bearing field

```text
F5.1  The sealed brand cannot validate only leg / joint names / motor IDs / directions /
      limits / parking / oracle identity.
F5.2  It must EXACTLY validate, or NOT expose as caller-supplied data:
          mode-specific joint order        side order
          prerequisite identities          prerequisite exact historical poses
          parking identity/lifetime        restore identities/order
          historical LF pose source        LF oracle binding
F5.3  Preferred G3 design: DERIVE immutable LF choreography internally from the sealed
      LF mode instead of accepting redundant motion-bearing raw fields.
F5.4  A modified RawLegCalibrationSpec must not be able to alter LF runtime motion after
      receiving the armable brand.
```

## F6 — Exact progress semantics

```text
F6.1  Keep two explicitly different quantities.

      OPERATIONAL PROGRESS INCREMENTS      Full 58   HipPair 20   Single 16
          the InProgress operations counted by the engine trace

      COMPLETE SUCCESSFUL EXTERNAL STREAM  Full 60   HipPair 22   Single 18
          additionally includes the exact step-zero/preflight external update and the
          terminal completed update

F6.2  Define the precise filtering predicate used by regression tests.
F6.3  Do NOT call the 58/20/16 list "all externally emitted updates".
F6.4  For each mode preserve exact ordered strings, labels and dynamic prefixes.
```

## F7 — CI roots must be explicit and mechanically executable

```text
F7.1  Do not say merely "relevant tree". Define exact conservative scan scopes using
      existing tools only.
F7.2  Rust: scan every *.rs under the explicitly named MATDOG auto-calibrate
      implementation scope used by G3, with explicit treatment/exclusion for test
      fixtures containing deliberate forbidden-token strings.
F7.3  Separately enumerate and check every GoalPosition construction and every sink
      caller.
F7.4  Python: define the exact tools/matdog Python file-tree scope protected by the
      MATDOG observer/profile boundary.
F7.5  Do not claim dynamic import resolution. On encountering a prohibited dynamic import
      mechanism in the protected scope, FAIL CLOSED and require review.
F7.6  Explicitly retain the existing: Python source-shape gate; cargo test --package
      st3215; Python unittests; rustfmt; observer boundary checks; EEPROM forbidden-token
      checks; Station release build.
```

## F8 — Expanded executable rejection test matrix

```text
F8.1  For every immediate engine motion operation, G3 tests exercise relevant rejections
      for: wrong mode; wrong state/sub-operation; wrong motor; wrong historical/reference
      pose; wrong coarse/fine step; stale/wrong observation where applicable;
      wrong-session staged evidence; guard violation; unsigned range violation.
F8.2  Only checks mechanically executable with the existing harness may be claimed.
```

## F9 — Defer M-19s completely

```text
F9.1  Remove the 24-endpoint Geometry eligibility/dataset runtime test from G3.
F9.2  The predicate may remain documented.
F9.3  Implementation and tests belong with the future Geometry V5 offline importer gate.
F9.4  G3 has NO Geometry V5 dataset or importer requirement.
```

## F10 — G3 implementation surface is LF-only

```text
F10.1  G3 requires only: ONE LegSessionStateMachine; three exact LF legacy modes; the
       sealed ArmableLfSessionSpec; the LF V25 oracle; immediate intent-specific motion
       operations; LF behavioral regression tests; the required CI migration.
F10.2  G3 does NOT implement: OfflineLegCalibrationSpec (unless strictly required as
       inert design scaffolding); the Geometry importer; a TrustedGeometryManifest
       parser; RF/RH/LH runtime; DirectGeometryTarget; non-LF hardware calibration;
       concurrent sessions.
F10.3  Prefer NOT to implement unused offline types in G3.
```

---

## Documentation defects accepted for correction

```text
G-1  "ten intent-specific engine operations" while eleven were listed; with F1 the
     correct count is TWELVE.
G-2  "adding an eleventh intent" when eleven already existed; now "a thirteenth".
G-3  stale `M-27r` reference in §8.3 (the row is `M-27s`).
G-4  stale A-row references (A1/A3/A6/A7/A8/A10) in the §7 closing note, which must use
     W-numbers.
G-5  §16.0 availability inventory omitted the existing Python unittest step and the
     Station release build step.
G-6  `ArtifactRef.expected_role` semantics need clarification for artifacts that carry no
     top-level role field: the role is the reviewer-assigned slot in the expected trust
     root, not a value read from the artifact.
```

---

## Required end state for revision 2.2

```text
source GoalPosition inventory = 23/23
separate legacy torque-off HOME prime and torque-on HOME reassertion
no caller-selected off-trace target authority
true raw GoalPosition sink sealed and review-visible
all LF motion-bearing brand data validated or internally derived
58/20/16 operational increments distinguished from 60/22/18 external streams
CI scopes and tests mechanically executable
M-19s absent from G3
G3 implementation scope LF-only
```

---

## Scope of this correction gate

Authorized:

```text
archive the final review verbatim
record these rulings
SURGICAL revision of tools/matdog/MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md
one local documentation-only commit
```

Not authorized:

```text
reopening the architecture       any .rs / .py runtime / test / workflow change
Station / robot-dog              historical RF worktree
push / PR                        hardware / serial / EEPROM
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
77829ec1606835ca8d0ee0f86e986bb15d5971fe8e5e8f394e148f0c71cc7951  Codex review #3
475267d86fab6777fc89c2dfd7a48c6512e3d0092ed54551734d673792f82e80  G2 review #3 decision
```

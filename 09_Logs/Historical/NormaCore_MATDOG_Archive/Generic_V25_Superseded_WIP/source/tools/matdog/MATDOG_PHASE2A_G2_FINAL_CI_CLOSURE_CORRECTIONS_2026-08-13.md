# MATDOG — Phase 2A G2 Final CI-Closure Corrections
## Resolving the two CI MAJORs and one documentation MINOR — 2026-08-13

## Status

```text
P2A-G0 = CLOSED / PASS
P2A-G1 = CLOSED / PASS
P2A-G2 = OPEN — CI feasibility was the only FAIL in the final closure check
P2A-G3 = NOT AUTHORIZED
```

Final independent closure check result:

```text
repository integrity      PASS      write inventory            PASS
12-operation surface      PASS      12 -> 2 -> 1 call graph    PASS
LF runtime compatibility  PASS      58/20/16 traces            PASS
60/22/18 streams          PASS      sealed LF authority        PASS
terminal safety           PASS      G3 LF-only scope           PASS
CI feasibility            FAIL      <- the only failure

NEW BLOCKER = 0   NEW MAJOR = 2   NEW MINOR = 1
```

**Nothing that passed is reopened.** Unchanged: 23 write paths; 12 engine operations;
2 policy writers; 1 raw constructor; LF modes, traces, constants, contact semantics,
provenance, global safety and G3 scope.

Applies to `MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md` at revision 2.2b
(`sha256 e759a48635fd95bc9bb913dca6a5e2941a6c9fc61591d085e3b5956f2b2764f4`,
HEAD `89aa0a41c017e6e0187d5540f6c2becaa3c4404f`).

---

## CI-C1 — M-27s production scope (MAJOR)

**Finding.** M-27s did not mechanically define its Scope-B production locations or its
`#[cfg(test)]` exclusion. A literal `RamRegister::GoalPosition` scan is ambiguous, so
"production count == one" could not be evaluated.

### Source inspection performed before editing

```text
software/drivers/st3215/src/auto_calibrate/matdog.rs   #[cfg(test)] at :4946 (and inner
                                                        :405 :861 :871 :1792 :1803)
    :576   RamRegister::GoalPosition   element of the allowed-register array in
                                       ram_write_allowed_for_profile   -- NOT a construction
    :606   Some(RamRegister::GoalPosition) =>   match arm in the same gate -- NOT a construction
    :2256  RamRegister::GoalPosition   element of is_allowed_matdog_ram_register
                                       matches! list                    -- NOT a construction
    :3802  write_startup_home_ram_verified(motor_id, RamRegister::GoalPosition,
                                           target.to_le_bytes().to_vec())   -- WRITE CONSTRUCTION
    :4382  write_motor_ram_verified(motor_id, RamRegister::GoalPosition,
                                    target.to_le_bytes().to_vec())          -- WRITE CONSTRUCTION

software/drivers/st3215/src/port.rs                    #[cfg(test)] at :1838
    :2016 :2066 :2076 :2086   protocol::RamRegister::GoalPosition.address()
                              ALL FOUR are AFTER :1838, i.e. ALL TEST-ONLY

software/drivers/st3215/src/auto_calibrate/mod.rs
    zero GoalPosition occurrences
```

Two consequences the contract had not captured:

```text
(1) port.rs's four GoalPosition occurrences are entirely inside its #[cfg(test)] block.
    A literal scan of port.rs would count four "constructions" that are test fixtures.
(2) In matdog.rs, only two of five production occurrences are write constructions.
    The other three are legitimate RAM-allowlist / armed-goal-gate references that a
    correct G3 implementation must keep. A gate counting bare enum occurrences would
    fail a correct implementation.
```

**Ruling.**

```text
CI-C1.1  The CG-1 production construction count applies to MATDOG production SCOPE A
         only, and counts WRITE-CONSTRUCTION SITES, not bare enum occurrences.
         A write-construction site is a call passing RamRegister::GoalPosition together
         with a value payload to a RAM-write helper.
         Enum references inside register allowlists and match arms are NOT construction
         sites and are expected to remain.
CI-C1.2  Structurally exclude from production counts: matdog_test.rs; every
         #[cfg(test)] module/block; any other demonstrably test-only code.
         The exclusion is ONLY for the production construction/caller count. Tests
         remain in cargo test, rustfmt and the structural/runtime regressions.
CI-C1.3  Required invariant unchanged: MATDOG production GoalPosition construction
         sites = exactly 1 (the single raw constructor of CG-1).
CI-C1.4  SCOPE B is NOT part of that constructor count. port.rs test GoalPosition
         constructions must not affect CG-1.
CI-C1.5  SCOPE B has a SEPARATE purpose: verifying the existing MATDOG driver/port
         bridge and armed port authority boundary. Its exact production anchors,
         confirmed present in current source, are:

             software/drivers/st3215/src/auto_calibrate/mod.rs
                 mod matdog;                                  (1 occurrence)
                 matdog_calibrator_is_armed                   (1)
                 matdog_armed_ram_write_allowed               (1)
                 matdog_ram_write_allowed_for_arm_value       (1)

             software/drivers/st3215/src/port.rs
                 matdog_command_allowed_with                  (4)
                 matdog_armed_command_allowed                 (5)
                 "native MATDOG profile arming is active"     (1)

         These are exactly the existing K-6 / K-7 assertions and are retained verbatim.
CI-C1.6  M-27s must prove items 1-6 over SCOPE A production, and item 7 (the SCOPE-B
         anchors) SEPARATELY. Item 7 must not be folded into the occurrence count.
```

---

## CI-C2 — Split Python scopes (MAJOR)

**Finding.** The contract conflated the broad `tools/matdog` dynamic-import scan with
K-11's observer-policy scan. Applying K-11's forbidden-token list to all
`tools/matdog/*.py` would false-fail legitimate runner and test files.

### Source inspection performed before editing

```text
.github/workflows/matdog-native-observer-check.yml

  reads exactly two files:
      observer = tools/matdog/matdog_native_observer_contract.py
      launcher = tools/matdog/matdog_v42_pinned_launcher.py

  required tokens  -> checked against  combined = observer + "\n" + launcher
  forbidden tokens -> checked against  observer ONLY
      found = [token for token in forbidden_observer if token in observer]

  workflow path triggers additionally list the two matching test files.

tools/matdog/*.py  (8 files)
      matdog_headless_auto_calibrate.py        test_matdog_headless_auto_calibrate.py
      matdog_lf_profile.py                     test_matdog_lf_profile.py
      matdog_native_observer_contract.py       test_matdog_native_observer_contract.py
      matdog_v42_pinned_launcher.py            test_matdog_v42_pinned_launcher.py
```

The forbidden-token list contains `M42`, `HOME_TICK`, `EXPECTED_ACTIVE_TORQUE_LIMIT` and
similar. Those legitimately appear in the headless runner and in tests; only the observer
module is required to be free of them.

**Ruling.**

```text
CI-C2.1  PYTHON DYNAMIC-IMPORT SAFETY ROOT (broad)
             tools/matdog/*.py   — all eight production and test files
         This broad scope checks ONLY the dynamic-code/import mechanisms the contract
         actually bans: __import__, importlib, exec(, eval(.
         Any occurrence FAILS CLOSED and requires review. No import resolution claimed.

CI-C2.2  K-11 OBSERVER POLICY ROOT (narrow, forbidden tokens)
             tools/matdog/matdog_native_observer_contract.py   — ONLY this file
         This is literally what the existing workflow enforces.

CI-C2.3  The observer REQUIRED-token check additionally spans
             matdog_native_observer_contract.py + matdog_v42_pinned_launcher.py
         exactly as the existing workflow does.

CI-C2.4  K-11 is NOT applied to runner files, test files, fixtures or unrelated MATDOG
         Python utilities. They remain covered by the dynamic-import safety root, by
         their own unit tests, and by their existing checks.

CI-C2.5  M-26s must state which Python scope it checks. K-11 must state its exact
         observer-production scope. The whole tools/matdog tree must not be described
         as "the observer boundary".

CI-C2.6  No new Python dependency and no import resolver.
```

---

## CI-C3 — Stale A-row references and legacy W2+W3 precision (MINOR)

### A-row mapping, derived from §7 and source — not from numbering

Revision 2.1's TABLE A used `A`-numbers; revision 2.2 replaced it with `W1..W23`. Live
normative prose still carried A-numbers. The mapping was derived by matching each row's
operation description:

```text
A3  prime at present position (prepare_motor)        -> W4    prime_at_present
A4  whole-session parking move                       -> W5    prerequisite_or_parking_move
A5  standalone prerequisite establish                -> W6    prerequisite_or_parking_move
A12 static hold Upper -> horizontal                  -> W15   static_hold_transition
A13 static hold Lower -> folded                      -> W16   static_hold_transition
A14 staged affine q0 Hip                             -> W17   staged_affine_q0
A15 staged affine q0 Lower                           -> W18   staged_affine_q0
A16 staged affine q0 Upper                           -> W19   staged_affine_q0
A17 parking restore to home                          -> W20   return_home
```

```text
CI-C3.1  Every obsolete A-row reference in LIVE normative text is replaced by the correct
         current W-row and, where useful, the §8.2 engine-operation name.
CI-C3.2  Historical NON-NORMATIVE revision records may retain A-row wording only where
         clearly identified as historical.
```

### Legacy startup W2 + W3

§10.1 described `recover_home_only_joints()` as "startup-home writer (W2)", which is
incomplete after the F1 split. The live legacy-entry description must preserve **both**
writes:

```text
W2   torque-OFF HOME prime      (prepare_startup_home_recovery_motor -> :3787)
     then torque enable          (:3789)
W3   torque-ON HOME reassertion  (move_profile_entry_motor_to_target(startup_writer=true)
                                  -> :3532)
before the legacy home recovery is considered complete.
```

The immutable choreography is not changed.

---

## Mandatory self-check before commit

```text
CORE ARCHITECTURE UNCHANGED
    write paths 23 | engine operations 12 | policy writers 2 | raw constructor 1
    LF modes / traces / safety unchanged

CI CLOSURE
    CG-1 production count applies only to MATDOG production SCOPE A
    #[cfg(test)] material structurally excluded from production occurrence counts
    port.rs test GoalPosition constructors do not affect CG-1
    SCOPE-B production bridge/port anchors named EXACTLY from current source
    SCOPE B assertions separate from the raw-constructor count

PYTHON CLOSURE
    broad dynamic-import scope explicitly named
    K-11 exact observer-production file explicitly named
    runner / tests / fixtures not subjected to K-11
    existing Python unit tests and observer checks retained

DOC CLOSURE
    zero obsolete A-row references in LIVE normative lifecycle prose
    §10.1 explicitly describes W2 -> torque ON -> W3
    remaining A-row mentions explicitly NON-NORMATIVE

no code / test / workflow / runtime file changed
```

---

## Scope

Authorized: create this record; correct the contract for CI-C1, CI-C2, CI-C3; one local
documentation-only commit.

Not authorized: reopening anything that passed; any `.rs` / `.py` / test / workflow
change; Station, robot-dog or the historical RF worktree; push; PR; hardware, serial or
EEPROM; G3.

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
cb0e938b42266b398df58fac50007d2e00d59dbd087f6da230369755c220ae2a  Codex final review
4afa6cbbe17ada225ebb31b013815aa3f7f71127b12fdd84458e9240ee5e0af7  G2 final review decision
9439d6ace8c561f160028921dd1e70199009ed6b333547f41a234d9f8edcf862  architecture-gate corrections
bdfd8447eddc922900b8f3710c479bab1c2d5fbff6228dd8304198ce2ec2eea7  final call-graph correction
```

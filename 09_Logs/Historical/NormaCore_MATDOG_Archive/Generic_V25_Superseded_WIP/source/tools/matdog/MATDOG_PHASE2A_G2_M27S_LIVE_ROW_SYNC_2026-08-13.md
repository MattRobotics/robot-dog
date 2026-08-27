# MATDOG — Phase 2A G2 M-27s Live-Row Synchronization
## Revision 2.2d — 2026-08-13

## Status

```text
P2A-G0 = CLOSED / PASS
P2A-G1 = CLOSED / PASS
P2A-G2 = OPEN
P2A-G3 = NOT AUTHORIZED
```

**No new design ruling.** This record documents a synchronization defect and its
correction. CI-C1's ruling was and remains correct; CI-C2 and CI-C3 are accepted and
unchanged.

Applies to `MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md` at revision 2.2c
(`sha256 a16eac0377e1a036de7388669dec385a078275ad13075483cf1d2bd1ac9bf51c`,
HEAD `9f29152a8b9e147a3afb98caa7eafe3f6d06e4f0`).

---

## The defect

Revision 2.2c's CI-C1 ruling was correctly stated in four places:

```text
§0.8                                                    correct
§8.3 CG-1                                               correct
§16.0b                                                  correct
MATDOG_PHASE2A_G2_FINAL_CI_CLOSURE_CORRECTIONS_...md    correct
```

But the **live normative M-27s row in §16.3** was never actually updated. At revision
2.2c it still read, in substance:

```text
"Seven-stage conservative scan over SCOPE A plus only the exact SCOPE B bridge/port
 locations required for the armed port boundary: (a) enumerate every
 RamRegister::GoalPosition occurrence — the PRODUCTION count must be exactly one ..."
```

That is exactly the formulation the final closure check blocked:

```text
- it scans SCOPE A and SCOPE B together;
- it counts every RamRegister::GoalPosition occurrence, not write-construction sites;
- "production count == one" is therefore unevaluable, because port.rs's four occurrences
  are #[cfg(test)] fixtures and three of matdog.rs's five production occurrences are
  allowlist / match-arm references that a correct implementation must keep.
```

## Cause

The revision-2.2c edit located its target with a **first-match** search for the literal
`| M-27s |`. The first such row in the file is not the §16.3 gate row — it is a cell in
the **§0.7 revision-record table** added by revision 2.2b:

```text
line  149   §0.7 revision-record table row      <- first match, overwritten by mistake
line 2182   §16.3 live normative M-27s row      <- the intended target, left untouched
```

So the corrected wording was written into the historical record table, and the live gate
kept the superseded text. Two rows were wrong as a result: the record row lost its short
description, and the live row was never synchronized.

## Correction applied in revision 2.2d

Both rows were located **by line index after asserting their ordering relative to the
`### 16.3` heading**, not by first-match.

```text
§16.3 live M-27s row   replaced with wording mechanically identical in meaning to CI-C1
                       and §16.0b:
                           stages 1-6 over SCOPE A PRODUCTION ONLY
                           matdog_test.rs + every #[cfg(test)] block structurally excluded
                           WRITE-CONSTRUCTION SITES only — a call passing
                             RamRegister::GoalPosition together with a value payload to a
                             RAM-write helper; bare enum references in allowlists,
                             matches!, match arms and register-policy predicates are NOT
                             construction sites
                           required production count = exactly ONE raw constructor/emitter
                           direct callers = exactly TWO private policy writers
                           policy-writer caller union = exactly the TWELVE §8.2 operations
                           startup-home writer covers only W1..W3
                           normal armed-goal writer covers only W4..W23
                           any second raw construction site, direct bypass, third policy
                             writer, third raw caller or unreviewed policy-writer caller FAILS
                           STAGE 7 — SCOPE B, SEPARATE, not part of the occurrence/count
                             logic; the exact §16.0b anchors and the existing K-6/K-7
                             assertions are retained; port.rs's four #[cfg(test)]
                             GoalPosition occurrences have ZERO effect on CG-1 / M-27s
                           the gate must NOT assert that the twelve operations directly
                             call the raw constructor

§0.7 record row        restored to its original short description
§22 summary line       "SEALED + 3-stage scan" corrected to "SEALED + 7-stage scan"
version markers        2.2c -> 2.2d
```

## What was NOT touched

```text
W1..W23                the twelve §8.2 operations       the two policy writers
the one-constructor architecture                        LF modes and traces
58/20/16               60/22/18                         authority model
global safety          Python scopes (CI-C2)            K-11
G3 scope               provenance                       contact semantics
CI-C3 A-row and W2/W3 corrections
```

## Self-check performed

```text
live §16.3 row says WRITE-CONSTRUCTION SITES, not every enum occurrence      verified
stages 1-6 are SCOPE A PRODUCTION ONLY                                       verified
#[cfg(test)] structurally excluded                                           verified
raw constructor count = 1                                                    verified
raw direct callers = 2 policy writers                                        verified
engine caller union = 12                                                     verified
startup coverage = W1..W3 ; normal coverage = W4..W23                        verified
SCOPE B is a SEPARATE stage 7                                                verified
port.rs #[cfg(test)] occurrences cannot affect CG-1                          verified
no other normative M-27s formulation contradicts this                        verified
CI-C2 unchanged ; CI-C3 unchanged                                            verified
no code / test / workflow file changed                                       verified
```

## Documents that must remain byte-identical

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
7055ae3d52a109e0c29f991c76e2ec86f6facc655c04782d5180b0a9e4bf95cd  final CI-closure corrections
```

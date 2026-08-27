# MATDOG — Phase 2A G2 Architecture-Gate Corrections
## Consistency findings from the ChatGPT integral architecture-gate read — 2026-08-13

## Status

```text
P2A-G0 = CLOSED / PASS
P2A-G1 = CLOSED / PASS
P2A-G2 = OPEN — substantive Revision-2.2 architecture ACCEPTED FOR FINAL REVIEW
P2A-G3 = NOT AUTHORIZED
```

This is **not** an architecture reopen. The Revision-2.2 substantive design is accepted
for final review. A small set of internal-consistency and CI-mechanical defects must be
corrected first so the final Codex review is not spent on them.

Applies to `MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md` at revision 2.2
(`sha256 7e45f0326ce3235332dd3c9c915dbda81b7cf1e3e4c5069a07ba514857f313b4`,
HEAD `df1ffb068af03ec457289d018a6f02b291fea6b0`).

**Explicitly NOT changed by this correction:** W1..W23; the twelve engine operations;
the Full/HipPair/Single traces; 58/20/16 versus 60/22/18; the LF runtime token surface;
the sealed LF brand; contact semantics; global safety ownership; the D/W4 hashes; any
provenance value; the LF-only G3 scope.

---

## AG-1 — Canonical deployment name is D/W4, not D/W5

**Ruling.** The canonical deployment materialization is
`2026-08-11_131818_..._REMEDIATION_BENCHMARK_D_W4_*`. C/W1 remains the determinism
oracle. Correct all normative `D/W5` references to `D/W4`, and clarify historical
references so no reader can infer that a W5 materialization exists.
`ExpectedGeometryV5DW4` remains the correct conceptual name. **No artifact hash changes.**

### Independent verification

```text
Materializations that actually exist on robot-dog origin/main bd5aa8ed…:
    C_W1     determinism oracle
    D_W4     canonical deployment
    (no W2, W3, W5 or any other W-index exists)

Occurrences of "D/W5" in the revision-2.2 contract: 13
Every D/W4 file hash in the contract is UNCHANGED and CORRECT:
    0db86e63  4e7172d4  dd8cb42c  e561e7fb  448ebcb3  82f00a94  d7fa04e2  0af31e9d
```

### Root cause, recorded for audit

The revision-2.2 TABLE A renumbering applied `\bW{i}\b -> W{i+1}` for `i = 22 … 3`. At
`i = 4` the pattern `\bW4\b` also matched the `W4` inside the literal `D/W4`, because `/`
is a word boundary. The label was rewritten; the hashes contain no `W` token and were
untouched. `C/W1` was never in the rewritten range and is unaffected.

This is a purely textual defect with no semantic consequence, but it must be corrected
because a reader could otherwise infer a non-existent W5 bundle.

---

## AG-2 — `ArtifactRef.expected_role` is a trust-root slot, not artifact content

**Ruling.** The existing `ArtifactRef` documentation is authoritative: `expected_role` is
the reviewer-assigned slot that artifact occupies in the compiled expected trust root. It
is **not** read from the artifact and **not** compared against a top-level role field.

`IMP-3b` currently reads as though both `expected_schema` **and** `expected_role` are
checked against artifact-declared values. Correct it to:

```text
expected_schema, when Some, is checked against the artifact-declared schema;
expected_role selects/binds the expected ArtifactRef slot in ExpectedGeometryV5DW4;
expected_role is NOT asserted to equal artifact content.
```

Do not weaken file SHA, semantic SHA, path, schema, or per-field record binding.

---

## AG-3 — G3 runtime never accepts a V5 record-bound pose

**Ruling.** §4.5 BRAND-3 is authoritative: LF G3 runtime poses are
`PoseReference::LfHistoricalV25`; V5 record-bound poses are offline / future-gate data
only.

§11.5 currently says `prerequisite_or_parking_move` "accepts either a V5 record-bound
pose (offline analysis) or an LF-historical pose (LF runtime)". That contradicts BRAND-3
and §8.4/§8.6, where the operation derives its pose from the current grammar node.

Required distinction:

```text
G3 runtime        prerequisite_or_parking_move derives ONLY the exact LF historical pose
                  required by the sealed LF mode.

Future offline    V5 record-bound poses may be represented as inert offline records,
Geometry design   with NO engine consumer and NO motion authority.
```

There must be no apparent V5-pose → G3-runtime path anywhere in the document.

---

## AG-4 — The MATDOG forbidden-token scan must not cover unrelated generic calibrators

**Ruling.** Revision 2.2's Rust safety root
`software/drivers/st3215/src/auto_calibrate/**/*.rs` is too broad. That directory holds
unrelated generic robot calibrators that legitimately use EEPROM functionality, so an
all-directory forbidden-token scan would false-fail on pre-existing non-MATDOG code.

### Independent verification

```text
forbidden-token occurrences in NON-MATDOG files in that directory:
    calibrator.rs   29   incl. :5   use crate::protocol::{RamRegister, EepromRegister};
                             :389  RamRegister::Lock.address()
                             :424  EepromRegister::Offset.address()   <- real EEPROM write
    elrobot.rs       2   incl. :4   use crate::protocol::EepromRegister;
    so101.rs         3   incl. :4   use crate::protocol::{RamRegister, EepromRegister};

Current directory contents: calibrator.rs elrobot.rs matdog.rs matdog_test.rs mod.rs so101.rs
Current MATDOG namespace:   matdog.rs matdog_test.rs
```

The claim is correct and the defect is real: the revision-2.2 gate would fail on day one.

### Required two-scope definition

```text
A. MATDOG IMPLEMENTATION SAFETY SCOPE
       software/drivers/st3215/src/auto_calibrate/matdog.rs
       software/drivers/st3215/src/auto_calibrate/matdog_*.rs
       software/drivers/st3215/src/auto_calibrate/matdog/**/*.rs

   Every new G3 MATDOG Rust implementation module MUST live inside that namespace.
   A MATDOG implementation file outside it FAILS the gate / requires review rather than
   silently escaping the scan.

   matdog_test.rs
       remains included in rustfmt and cargo test;
       may be excluded ONLY from forbidden-token LITERAL scanning, because it
         intentionally contains those strings;
       still participates in structural and runtime tests.

B. GLOBAL BRIDGE / PORT SCOPE
       software/drivers/st3215/src/auto_calibrate/mod.rs
       software/drivers/st3215/src/port.rs
   These retain their existing specific MATDOG bridge/port assertions.
```

```text
The MATDOG EEPROM-forbidden implementation-token scan MUST NOT be applied to
calibrator.rs, elrobot.rs or so101.rs unless a future separately-reviewed invariant
explicitly requires it.

The GoalPosition construction / raw-sink scan targets scope A plus only the exact
bridge/port locations required for the armed port boundary.
```

---

## AG-5 — Do not overclaim staleness

**Ruling.** Correct any normative sentence equivalent to "staleness is impossible by
construction" where it could imply that observations or evidence cannot themselves become
stale.

Required precise statement:

```text
There is no AUTHORIZE-NOW / EMIT-LATER staleness window, because authorization and the
write happen in one engine operation.

Observation and evidence freshness remain an explicit runtime invariant and MUST be
checked immediately before the write, per DER-1 and §8.6.
```

No semantic behavior change.

---

## Optional clarity fix — §6.3

Where §6.3 describes generic/per-leg `CalibrationOrder`, prerequisite, parking and restore
data, clarify that it is future generic/offline design data; that the G3 LF runtime does
**not** consume those raw motion-bearing fields; and that sealed LF runtime choreography
is derived internally from `LfSessionMode` under §4.5. §6 is not otherwise changed.

---

## Required self-check before commit

```text
ZERO normative "D/W5" references remain
D/W4 used consistently for canonical deployment
ExpectedGeometryV5DW4 unchanged in meaning
IMP-3b no longer treats expected_role as artifact-declared content
ZERO normative G3 runtime path accepts a V5 record-bound pose
MATDOG EEPROM-forbidden scan excludes calibrator.rs / elrobot.rs / so101.rs
every new G3 MATDOG implementation source is mechanically required to reside inside
    the defined MATDOG implementation scope
observation/evidence freshness remains explicitly checked
W1..W23 semantically unchanged
no code / runtime / test / workflow file changed
```

---

## Scope of this correction gate

Authorized:

```text
create this record
consistency-correct tools/matdog/MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md
one local documentation-only commit
```

Not authorized:

```text
architecture reopen              any .rs / .py runtime / test / workflow change
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
cb0e938b42266b398df58fac50007d2e00d59dbd087f6da230369755c220ae2a  Codex final review
4afa6cbbe17ada225ebb31b013815aa3f7f71127b12fdd84458e9240ee5e0af7  G2 final review decision
```

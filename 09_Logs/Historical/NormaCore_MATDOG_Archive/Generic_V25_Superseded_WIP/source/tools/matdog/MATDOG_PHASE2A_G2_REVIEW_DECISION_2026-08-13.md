# MATDOG — Phase 2A G2 Review Decision
## Architecture-owner rulings after independent Codex review — 2026-08-13

## Status

```text
P2A-G0 = CLOSED / PASS
P2A-G1 = CLOSED / PASS
P2A-G2 = BLOCKED / REVISION REQUIRED
P2A-G3 = NOT AUTHORIZED
```

Gate authority: ChatGPT (architecture/gate owner) + Matteo.
Independent adversarial reviewer: Codex.
Executor: Claude Code Opus.

The independent review is archived verbatim, unmodified, at:

```text
tools/matdog/MATDOG_PHASE2A_CODEX_G2_REVIEW_2026-08-13.md
sha256 c51fb5d74c1752302e886febacde213c6748b70078ecc898ccd97efe5b9fbf6d
```

Accepted in full:

```text
BLOCKERS  B1, B2, B3
MAJORS    M1, M2, M3, M4, M5, M6, M7
MINORS    N1, N2, N3
NOTES     both accepted as positive preserved behavior
```

Codex reviewed at `2047c11ad9498af7f14061eafe2e631cb4bc7380` and modified nothing.

The revision applies to exactly one document:

```text
tools/matdog/MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md
```

`MATDOG_PHASE2A_G0_G1_EXECUTOR_REPORT_2026-08-13.md` and
`MATDOG_PHASE2A_G1_GATE_DECISION_2026-08-13.md` remain byte-identical. G1's rulings
D1–D9 are unchanged and remain binding; where the original G2 contract contradicted
them, G2 was wrong, not D1–D9.

---

## Evidence base for these rulings

The rulings below are grounded in the canonical post-V5 robot-dog artifacts read from
`MattRobotics/robot-dog` `origin/main` = `bd5aa8edbd903531885a3439b29a7303009e838b`.

Canonical remediated V5 bundle (benchmark C/W1, `2026-08-11_131818`):

```text
run manifest content sha256   9b13dd758f6a8bcf3da24bdf0619303705c2ad8cc44187040a4a846d211d1a46
endpoint semantic sha256      de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e
parking  semantic sha256      67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139
combined semantic sha256      0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51

endpoint_profile file sha256  23ca4709385019466cfb5782a7e0e6f2053fd0d632b67db6c94bd00c05f5e147
parking_json     file sha256  65cd6d25e28c6547b9cdbdf267eafc6b6452f34347415cb5e20dfbe3798047ca
combined_profile file sha256  a482dd28fc97ec387f9a6fd03c501ea7cf5045dc97a384fbb5fe9bcdd23d73fa

URDF     03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf
         sha256 3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59
compiler source_combined_sha256
         e4eea175e90131b1d8e55ae381fe8e88d6f966c0f87d4eee5c064d0a6967737f
collision meshes: 17 links, each with its own sha256
```

Final external safety policy (`2026-08-11_132758`):

```text
policy_id                MATDOG_REFERENCE_MINIMUM_CLEARANCE
schema                   matdog.geometry_safety_policy.v1
threshold_m              0.003
safety semantic sha256   e5cb2a4c33082c59c6f5d381f90fcc19680cc899e326d13c9c9129932bde5d08
policy source sha256     8b124ef45934cb318af6ce4bcb873ce078b505dfa74ad017e6394b4f51b674f6
geometry_mutated         false
motion_authorization_granted_count   0
PASS 16 / FAIL 0 / UNRESOLVED 8 / endpoints 24
urdf_executable_target_count         8
diagnostic_target_outside_urdf_limits_count  16
clearance_pass_but_target_outside_urdf_limits_count  8
```

Final LF hardware reconciliation (`2026-08-11_132758`):

```text
schema                     matdog.geometry_hardware_reconciliation.v1
hardware evidence sha256   6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4
reconciler source sha256   111da4045c983c4f3a5acd0061dbbe3c7b77fbda7e120693a927f5f0a40f5dfe
agreement threshold        2.000 deg
AGREES 3 / DISAGREES 3 / NO_GEOMETRIC_CONTACT 0
```

### The two facts that decide B1

**Fact 1 — only 8 of 24 V5 endpoints are in the executable URDF domain.**

```text
EXECUTABLE_URDF_DOMAIN                        8   (the four *_upper_leg_joint min+max)
DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS      16   (all hip and all lower endpoints)
```

For LF specifically: `lf_upper_leg_joint:min` and `lf_upper_leg_joint:max` are
executable; **both hip endpoints and both lower endpoints are diagnostic outside URDF
limits.** The original G2 contract declared all six LF endpoints `Executable` on the
strength of hardware evidence. That is precisely the laundering B1 identifies.

**Fact 2 — on exactly those diagnostic LF endpoints, geometry and hardware disagree.**

```text
lf_hip_joint:min        geometry -46.012   hardware -42.803   delta -3.209   DISAGREES
lf_hip_joint:max        geometry +45.223   hardware +39.375   delta +5.848   DISAGREES
lf_upper_leg_joint:min  geometry -52.133   hardware -53.525   delta +1.393   AGREES
lf_upper_leg_joint:max  geometry +121.875  hardware +122.607  delta -0.732   AGREES
lf_lower_leg_joint:min  geometry -92.074   hardware -91.846   delta -0.229   AGREES
lf_lower_leg_joint:max  geometry +38.180   hardware +34.277   delta +3.902   DISAGREES
```

The three `DISAGREES` endpoints are diagnostic. Had the original G2 rule shipped, the
engine would have been permitted to command a V5 geometry target 3.2°, 5.8° or 3.9°
beyond where the hardware actually stops. **Hardware evidence is the reason those
endpoints must stay diagnostic, not a reason to promote them.** B1 is not a
theoretical objection.

---

## R1 — B1 ACCEPTED: authority axes must remain orthogonal

Five independent axes. No axis may upgrade another.

```text
A. Geometry V5 target domain            IMMUTABLE, imported, never recomputed
      EXECUTABLE_URDF_DOMAIN
      DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS

B. External safety-policy verdict       DENY-ONLY
      PASS / FAIL / UNRESOLVED

C. Hardware-contact validation evidence EVIDENCE ONLY

D. Bounded calibration probe/search authority

E. Direct-goal motion authority
```

Binding rules:

```text
R1.1  HardwareValidation::Validated MUST NOT change the Geometry V5 target domain.
R1.2  ExternalPolicyVerdict::Pass is deny-only retention evidence.
      It MUST NOT create executability.
R1.3  FAIL or UNRESOLVED MUST prevent a Geometry-V5-derived direct target.
R1.4  Hardware evidence MUST NOT create direct geometry-target authority.
R1.5  Historical LF V25 bounded probing is a SEPARATE capability from direct
      Geometry V5 endpoint execution.
R1.6  LF HIP and LOWER V5 endpoints remain DIAGNOSTIC outside the URDF domain
      even where LF hardware contacts historically exist.
```

Direct-target eligibility is therefore:

```text
eligible_for_direct_geometry_target
    = (domain == EXECUTABLE_URDF_DOMAIN)
      AND (policy_verdict == PASS)

Hardware evidence appears nowhere in this expression.
```

LF V25's historically probed hip/lower endpoints remain reachable only through the
separate bounded-probe capability, which is detector-controlled, incremental, and
guard/corridor-bounded — never a direct commandable endpoint.

---

## R2 — B2 ACCEPTED: range is not authority

The conceptual `ExecutableTick` boundary is withdrawn.

```text
UnsignedTick may prove ONLY:   0 <= tick <= 4095
It MUST NOT imply motion authority.
```

Required structure:

```text
RawLegCalibrationSpec
      -> validate(trusted manifest, provenance, domain, policy)
      -> ValidatedLegSpec        <-- the ONLY spec type the engine accepts

Command-capable values are purpose-specific opaque authority objects:

AuthorizedGoal { purpose, motor, tick }

purpose ∈ {
    HOME_NORMALIZATION
    PREREQUISITE_OR_PARKING
    BOUNDED_PROBE_STEP
    BACKOFF
    STAGED_AFFINE_Q0
    DIRECT_GEOMETRY_TARGET
}
```

```text
R2.1  The command layer MUST NOT accept a raw UnsignedTick as authority.
R2.2  Provenance, domain and policy checks occur BEFORE any authority object
      is minted.
R2.3  A raw spec MUST have no runtime consumer.
R2.4  Each purpose has its own minting function with its own preconditions.
      DIRECT_GEOMETRY_TARGET is the only purpose gated on R1's eligibility
      expression, and no LF endpoint currently qualifies for it in Phase 2A.
```

---

## R3 — B3 ACCEPTED: non-LF Phase 2A is offline only

```text
R3.1  RF/RH/LH specifications MUST NOT be registered in MATDOG_ARM_ENV or any
      runtime arming resolver in Phase 2A.
R3.2  No RF/RH/LH Phase 2A spec may obtain serial, torque, probe, port-command
      or Station-motion capability.
R3.3  They are offline data/spec/test objects only.
R3.4  Preserve the historical LF arming sentinel required for immutable LF
      regression compatibility.
R3.5  Do NOT create RF/RH/LH runtime sentinel strings during Phase 2A.
```

`hardware_witness == None` was never a sufficient control: it blocks staging and
EEPROM *after* measurement, but does nothing to prevent torque enable, prerequisite
motion or contact probing *before* it. The offline API must be structurally separate
from the arming resolver.

---

## R4 — M1 ACCEPTED: engine owns lifecycle grammar

`SessionPlan` must not be an arbitrary transition table.

The engine owns:

```text
legal lifecycle grammar
single-active enforcement
Diagnostics placement
staged-return grammar
Cleanup -> TorqueOff terminal suffix
universal fail-to-Cleanup
role derivation
legal participant derivation
```

Per-leg data may supply only geometry-dependent calibration operation data:

```text
joint order
contact-side order
prerequisite references
parking requirement
restore data
```

within the fixed grammar.

```text
R4.1  Participant motors MUST be DERIVED from the leg's actual joint mapping
      plus a provenance-bound optional parking dependency.
R4.2  No arbitrary allowed_motor_ids set.
```

---

## R5 — M2 ACCEPTED: InitialRecovery

```text
R5.1  InitialRecovery is a structural engine operation.
R5.2  It may inspect/recover multiple eligible leg joints.
R5.3  Actual motion remains ONE active motor at a time.
R5.4  Do NOT encode InitialRecovery as one SessionPhase with one fixed active ID.
```

The immutable engine permits `11 | 12 | 13` as the eligible set in that state while
moving them one at a time. Eligibility and current-active are different concepts and
must be modeled separately.

---

## R6 — M3 ACCEPTED: prerequisite role separation

At least three distinct representations are required:

```text
1. standalone-contact held prerequisite       (torque ON, actively held)
2. full-session accumulated held target       (promoted after a completed move)
3. passive torque-OFF at-home requirement     (torque OFF, must remain in corridor)
```

```text
R6.1  Do NOT conflate these into one prerequisite target list.
R6.2  The LF V25 runtime torque roles must be representable EXACTLY.
```

In the immutable full LF session, during Upper contact only M42 is held; LF Hip and
LF Lower are passive torque-OFF at home. A single "static holds" list would silently
change those torque roles.

---

## R7 — M4 ACCEPTED: value-bound provenance

Provenance may not be metadata-shaped. The validated import boundary must bind each
imported value to its canonical source record.

Per endpoint / prerequisite / parking record, bind at least:

```text
artifact identity / content SHA
record ID
leg
joint
side
numeric value
target-domain classification
semantic SHA where applicable
```

External policy additionally binds:

```text
policy source / content SHA
threshold
record identity
verdict
```

```text
R7.1  Hardware evidence SHA must bind to the actual LF evidence artifact
      (6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4).
R7.2  Correct hashes placed beside manually invented or stale values MUST fail.
R7.3  "Authored by review" is not runtime provenance verification.
```

---

## R8 — M5 ACCEPTED: LF hardware witness is structurally LF-only

```text
R8.1  Do NOT put a generic Option<HardwareWitness> into LegCalibrationSpec.
R8.2  The LF hardware witness belongs to an LF-only oracle/evidence boundary:
      a dedicated opaque LF V25 oracle object/module.
R8.3  Key it explicitly by JointKind, not by positional array order.
R8.4  RF/RH/LH must have NO constructor or path capable of consuming LF witness
      ticks or tolerance.
```

The original claim of a "fourth barrier" via CI item M-11 was false: M-11 forbade only
per-leg engine and validator names. Codex is correct.

---

## R9 — M6 ACCEPTED: CI migration must be complete

```text
R9.1   For EVERY changing assertion record, individually and not grouped:
           old assertion -> protected invariant -> new assertion -> proof of
           non-weakening.
R9.2   Include all ten exact test-name assertions individually.
R9.3   Explicitly retain proof that all THREE staged LF joints use affine q0 and
       that none uses fixed-scale q0.
R9.4   Explicitly cover MATDOG_LF_PROFILE_V1 and the Python serializer checks.
R9.5   The affine x witness truth table must exercise the PRODUCTION
       evidence/staging path, not a test-local reimplementation.
R9.6   Replace weak literal-only LF-span bans with ownership/type tests.
R9.7   Test that a RawLegCalibrationSpec cannot reach the engine.
R9.8   Define independent provenance mutation-rejection tests for EVERY bound field.
R9.9   All future MATDOG calibrator modules must be included in safety source
       scans and rustfmt/check coverage.
R9.10  The EEPROM forbidden-token list is retained VERBATIM.
R9.11  The observer forbidden-token boundary is retained VERBATIM and must also
       cover imported helpers.
```

---

## R10 — M7 ACCEPTED: Station/bus ownership remains global

Corrected ownership for G1 IDs 118–120:

```text
ID 118  observer policy / motion-authority boundary            = C
ID 119  launcher pin-enforcement MECHANISM                     = C
        the specific historically reviewed pinned SHA values   = evidence /
                                                                 current release fact
ID 120  Station process ownership                              = C
        serial ownership                                       = C
        bus identity enforcement (EXPECTED_BUS_SERIAL)         = C
        torque-limit enforcement (EXPECTED_ACTIVE_TORQUE_LIMIT)= C
        LF total step count 58                                 = D regression evidence
```

```text
R10.1  Do NOT move Station/bus ownership into LegCalibrationSpec.
R10.2  Pinned SHA values are release facts, NOT per-leg specification data.
```

---

## R11 — N1 ACCEPTED: progress is derived

```text
R11.1  The generic progress count is DERIVED from the expanded engine operation
       trace, never supplied as spec data.
R11.2  DONE must require actual executed progress to EQUAL the derived expected
       count.
R11.3  LF regression additionally requires exactly 58 increments and preserved
       historical phase strings.
```

---

## R12 — N2 ACCEPTED: complete non-LF offline datasets

```text
R12.1  The revised G2 contract MUST NOT contain RF/RH/LH <V5> placeholders.
R12.2  Read the canonical robot-dog Geometry V5 final artifacts from the verified
       post-V5 state. Use canonical final/remediated artifacts only.
R12.3  Document complete independent RF/RH/LH offline design datasets:
           endpoint record
           target domain
           external-policy verdict
           prerequisite reference
           parking/path reference
           provenance source
R12.4  Do NOT convert any of these records into runtime authorization.
R12.5  Current numerical equality between legs is NEVER authority for deriving
       one leg from another.
```

The canonical data independently confirms R12.5 is a real constraint, not a
formality. Declared URDF limits are equal across legs, but the measured geometric
contacts and the parking plans are not:

```text
hip:min geometric contact   LF -0.803056   RF -0.789284   RH -0.788125   LH -0.803056
hip:max geometric contact   LF +0.789284   RF +0.803056   RH +0.803056   LH +0.788125

lower:min parking           LF  lf_upper_leg_joint = 1.119920 rad (64.1667 deg)
                            RF  rf_upper_leg_joint = 1.119920 rad (64.1667 deg)
                            RH  rh_upper_leg_joint = 1.628974 rad (93.3333 deg)
                            LH  lh_upper_leg_joint = 1.628974 rad (93.3333 deg)

upper:max parking           LF  lh_upper_leg_joint = 0.610865 rad (35.0000 deg)
                            RF  rh_upper_leg_joint = 0.610865 rad (35.0000 deg)
                            RH  NOT_NEEDED
                            LH  NOT_NEEDED
```

Four distinct hip contact values exist. Front legs require a contralateral-rear
parking joint for `upper:max`; hind legs do not. Front and hind lower-min parking
differ by 29.17°. **FRONT ≠ HIND is a measured property of this robot.**

---

## R13 — N3 ACCEPTED: document corrections

```text
R13.1  Fix the OPEN-1 section cross-reference (it was §12, not §11).
R13.2  Correct the live-FK search-count statement while PRESERVING the verified
       conclusion: NO_DEPENDENCY_VERIFIED for the Phase 2A runtime.
R13.3  Explicitly carry D9 / live-FK isolation into the G3 prohibitions.
```

The G1 D9 text said the sweep returned one documentation hit; the stated expression
returns five documentation hits at the base commit. The material conclusion — zero
runtime live-FK dependency in norma-core — is unaffected and remains verified. G1 is
not edited; the correction is recorded here and carried into the revised G2.

---

## R14 — OUTSIDE-PREDICTED-BAND CONTACT

For future hardware phases:

```text
FINAL ACCEPTED CONTACT -> STOP PRESSURE IMMEDIATELY
```

If a real contact lies outside the model-predicted band:

```text
stop_pressure immediately
controlled verified backoff
record the disagreement
fail closed
verified cleanup / global torque off
```

```text
R14.1  Do NOT continue probing toward a historical LF span.
R14.2  The ONLY preserved continued-probing behavior is the exact immutable
       LF V25 bounded PRE-FINAL fine-pass friction/chamfer qualification
       already accepted in D1.
```

D1 is unchanged. R14 constrains what may be added around it, and forecloses the
historical RF worktree's pattern of issuing a further advancing command after a
confirmed contact fell outside the predicted region.

---

## OPEN-1 through OPEN-5 — final rulings

**OPEN-1 — RESOLVED by R1.** The three-field split proposed in the original G2 is
rejected: its admission rule still used hardware evidence to upgrade direct-target
eligibility. The corrected model is five orthogonal axes with a separate bounded-probe
authority. `lf_hip_joint:min` and `lf_lower_leg_joint:min` remain diagnostic and
externally unresolved as direct geometry targets, retain their historical hardware
evidence, and remain probeable only under the LF-specific bounded-probe authority.

**OPEN-2 — ADOPTED.** Preserve `"LF_LEG_STATE_MACHINE"` as an opaque historical LF
compatibility sentinel. Its spelling must not imply a duplicate Rust type and must not
justify adding non-LF runtime sentinels in Phase 2A.

**OPEN-3 — ADOPTED.** Preserve `MATDOG_LF_PROFILE_V1` byte-for-byte. The serializer
boundary requires LF-branded accepted evidence. A generic record/schema remains
deferred unless separately reviewed.

**OPEN-4 — ADOPTED.** `GUARD_OVERSHOOT_TICKS = 64` remains global, non-spec-tunable
and CI-pinned.

**OPEN-5 — ADOPTED.** LIFO restore remains engine-owned. Any future non-LIFO
requirement requires a new architecture review.

---

## Scope of this correction gate

Authorized:

```text
archive the Codex review verbatim
record these rulings
revise tools/matdog/MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md
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

The G0/G1 report and the G1 gate decision remain byte-identical:

```text
37646f2eaf0f56af558b2450ada7908c44b8b1fdb394f3095fb99c25c651bc17  G0/G1 report
89df3920461d347d372f30fb44aa0eefb016543f45b4f5615d8b7309fd51433d  G1 gate decision
```

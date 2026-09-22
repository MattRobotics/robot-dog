# MATDOG calibration bootstrap — current-source audit and geometry contract

**Audit date:** 2026-09-22 · **Branch:** `feat/controller-calibration-bootstrap-v1`
**Parent:** `feat/controller-safe-actuator-layer-v1` @ `988c84a`

This phase connects components that already exist. It builds no new architecture, and it
does **not** port the LF V25 18-state sequence as the calibration runtime. Its purpose is
the shortest safe path from here to a first current calibration and a stand-up.

```text
CURRENT URDF + CURRENT COLLISION MESHES
        ↓  Geometry Compiler V5          REUSED, provenance re-verified
CURRENT CALIBRATION GEOMETRY PROFILE     generated, compact, versioned
        ↓
manual placement at nominal URDF q=0     operator, no motion
        ↓
read-only q0 capture, torque OFF         TO_IMPLEMENT
        ↓
current direction verification            TO_IMPLEMENT
        ↓
raw encoder ↔ URDF q transform            TO_IMPLEMENT
        ↓
geometry-defined parking / path           REUSED from V5
        ↓
controlled contact probing                TO_IMPLEMENT
        ↓
MODEL ↔ REAL comparison → candidate → explicit acceptance → current safe limits
```

No historical replay may skip to `PROMOTED`. No hardware action was performed.

---

## 1. CURRENT SOURCE TRUTH

| Source | Classification |
|---|---|
| `06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml` | **VERIFIED CURRENT** — physical unit → joint → bus id |
| `MATDOG_JOINT_CALIBRATION.yaml` → `calibration_reset:` | **VERIFIED CURRENT** — the authoritative state block |
| `MATDOG_JOINT_CALIBRATION.yaml` → joint entries below the reset header | **STALE** — `all_joint_data_below_is_stale: true` |
| `03_CAD/URDF/matt_robodog_rev00/` + `SHA256SUMS.txt` | **VERIFIED CURRENT** |
| Geometry Compiler V5 canonical bundle (§4) | **VERIFIED CURRENT → REUSED** |
| `09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md` | **VERIFIED CURRENT** — the reset's own authority |
| `archive/2026-08-29/full-leg-calibrator-v1-h0` (LF V25) | **HISTORICAL ORACLE** — behaviour only, §10 |
| `rear_parking_pose`, `front_leg_dependencies`, `zero_encoder_final`, historical `direction`, historical `q0` | **SUPERSEDED** — §9 |

The current state block, unchanged:

```text
state:                          CALIBRATION_RESET_PENDING_FULL_RECALIBRATION
effective:                      2026-08-27
all_joint_data_below_is_stale:  true
hardware_motion_authorized:     false
```

The reset document settles the question this phase depends on, in its own words:

> Geometry Compiler / CAD geometry is **distinct** from the calibration of the physical
> assembly. The URDF, meshes and derived geometric endpoints are unaffected by reassembly —
> they describe the design, not the build. Geometry remains valid; **calibration does not**.

---

## 2. CURRENT SERVO BASELINE — **VERIFIED CURRENT**

All 17 units, 2026-08-27 campaign, profile `MATDOG_C018_V1`, `PositionOffset = 0`
everywhere, raw centre 2048 ±1 tick, cold verify PASS. The twelve leg joints:

| Joint | Unit | Bus | URDF `motorId` | Raw centre | Err |
|---|---|---:|---:|---:|---:|
| LF_HIP | M22 | 13 | 13 | 2048 | 0 |
| LF_UPPER | ELR01 | 12 | 12 | 2048 | 0 |
| LF_LOWER | M33 | 11 | 11 | 2049 | +1 |
| RF_HIP | NEW01 | 23 | 23 | 2047 | −1 |
| RF_UPPER | ELR03 | 22 | 22 | 2049 | +1 |
| RF_LOWER | NEW03 | 21 | 21 | 2048 | 0 |
| RH_HIP | NEW06 | 33 | 33 | 2047 | −1 |
| RH_UPPER | ELR02 | 32 | 32 | 2049 | +1 |
| RH_LOWER | NEW05 | 31 | 31 | 2049 | +1 |
| LH_HIP | M43 | 43 | 43 | 2049 | +1 |
| LH_UPPER | M42 | 42 | 42 | 2048 | 0 |
| LH_LOWER | M41 | 41 | 41 | 2049 | +1 |

**The URDF's `motorId` equals the allocation's `bus_id` for all twelve.** That cross-check is
name-independent and the exporter refuses to emit a table where it fails.

The identity trap, concretely: unit **M11** — whose calibration the LF V25 archive records —
is now `NECK_PITCH` on bus 52. It is not in a leg at all.

---

## 3. CURRENT URDF / MESH PROVENANCE — **VERIFIED CURRENT**

```text
03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf
  3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59
```

Identical to the hash `MATDOG_JOINT_CALIBRATION.yaml` declares **and** to the hash the V5 run
was gated on. `sha256sum -c SHA256SUMS.txt`: **36/36 OK, 0 FAILED**.

12 revolute leg joints, limits `hip ±45°`, `upper −52.5°…+122.5°`, `lower −92°…+37.5°`.

---

## 4. GEOMETRY V5 STATUS — **REUSED, not regenerated**

Canonical bundle: `2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4`.

Re-verified against the current repository, every hash recomputed rather than trusted:

| Checked | Result |
|---|---|
| 18 gated inputs (URDF + 17 collision meshes) | **0 mismatches** |
| 9 canonical semantic compiler sources | **0 mismatches** |
| 11 execution sources (incl. the non-canonical G4 oracle) | **0 mismatches** |
| 8 published bundle artifacts | **0 mismatches** |
| Frozen G4 reference profile | match |

The bundle's own gates: `24/24` endpoint coverage, `24/24` endpoint↔parking consistency,
`12/12` q0 active revolute pairs separated, determinism C vs D **PASS**, 119,696 collision
triangles, 94 evaluated 1-DOF candidates, **0** requiring 2-DOF.

**Decision (B2): REUSE.** The canonical bundle was produced from the exact geometry the robot
has now. Regenerating would burn ~28 minutes of compute to reproduce identical output, and
patching a value by hand is forbidden.

`CANONICAL V5 != G4 REPLAY` is preserved: the G4 oracle is excluded from the canonical
semantic source manifest and the compiler has no parameter that could accept a G4 profile.

---

## 5. 24 ENDPOINT DOMAIN CLASSIFICATION

```text
EXECUTABLE_URDF_DOMAIN                     8
DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS   16
geometric contact found                   24 / 24
path obstructed                            6
```

| # | Endpoint | Domain | Parking | Contact | URDF limit |
|---:|---|---|---|---:|---:|
| 0 | `lf_hip:min` | DIAGNOSTIC | NOT_NEEDED | −46.012° | −45.0° |
| 1 | `lf_hip:max` | DIAGNOSTIC | NOT_NEEDED | +45.223° | +45.0° |
| 2 | `lf_upper:min` | **EXECUTABLE** | NOT_NEEDED | −52.133° | −52.5° |
| 3 | `lf_upper:max` | **EXECUTABLE** | **1-DOF** | +121.875° | +122.5° |
| 4 | `lf_lower:min` | DIAGNOSTIC | **1-DOF** | −92.074° | −92.0° |
| 5 | `lf_lower:max` | DIAGNOSTIC | NOT_NEEDED | +38.180° | +37.5° |
| 6 | `rf_hip:min` | DIAGNOSTIC | NOT_NEEDED | −45.223° | −45.0° |
| 7 | `rf_hip:max` | DIAGNOSTIC | NOT_NEEDED | +46.012° | +45.0° |
| 8 | `rf_upper:min` | **EXECUTABLE** | NOT_NEEDED | −52.133° | −52.5° |
| 9 | `rf_upper:max` | **EXECUTABLE** | **1-DOF** | +121.875° | +122.5° |
| 10 | `rf_lower:min` | DIAGNOSTIC | **1-DOF** | −92.074° | −92.0° |
| 11 | `rf_lower:max` | DIAGNOSTIC | NOT_NEEDED | +38.180° | +37.5° |
| 12 | `rh_hip:min` | DIAGNOSTIC | NOT_NEEDED | −45.156° | −45.0° |
| 13 | `rh_hip:max` | DIAGNOSTIC | NOT_NEEDED | +46.012° | +45.0° |
| 14 | `rh_upper:min` | **EXECUTABLE** | NOT_NEEDED | −52.133° | −52.5° |
| 15 | `rh_upper:max` | **EXECUTABLE** | NOT_NEEDED | +121.875° | +122.5° |
| 16 | `rh_lower:min` | DIAGNOSTIC | **1-DOF** | −92.074° | −92.0° |
| 17 | `rh_lower:max` | DIAGNOSTIC | NOT_NEEDED | +38.180° | +37.5° |
| 18 | `lh_hip:min` | DIAGNOSTIC | NOT_NEEDED | −46.012° | −45.0° |
| 19 | `lh_hip:max` | DIAGNOSTIC | NOT_NEEDED | +45.156° | +45.0° |
| 20 | `lh_upper:min` | **EXECUTABLE** | NOT_NEEDED | −52.133° | −52.5° |
| 21 | `lh_upper:max` | **EXECUTABLE** | NOT_NEEDED | +121.875° | +122.5° |
| 22 | `lh_lower:min` | DIAGNOSTIC | **1-DOF** | −92.074° | −92.0° |
| 23 | `lh_lower:max` | DIAGNOSTIC | NOT_NEEDED | +38.180° | +37.5° |

**The eight executable endpoints are exactly the eight upper-leg ones.** For every hip and
every lower leg the mechanism contacts *beyond* the declared URDF limit — by 1.0° at the hips
and 0.07° at the lower legs. Those sixteen endpoints are evidence about where the mechanism
stops. They are not places the robot may be commanded to.

Consequence, stated plainly because it shapes the whole calibration plan: **a current contact
calibration of the hip and lower joints cannot use these geometric endpoints as motion
targets.** Reaching them requires an explicit calibration-specific geometry safety plan that
does not exist yet, and cannot be inferred from a diagnostic endpoint.

---

## 6. EXACT 6 PARKING / OBSTRUCTED PATHS

All six resolved with a single auxiliary joint; the compiler needed no 2-DOF plan anywhere.

| Endpoint | Domain | Blocking pair | Relation | Auxiliary joint | Parked at |
|---|---|---|---|---|---:|
| `lf_upper:max` | EXECUTABLE | `lf_foot` ↔ `lh_foot` | **cross_branch** | `lh_upper_leg` | **+35.000°** |
| `rf_upper:max` | EXECUTABLE | `rf_foot` ↔ `rh_foot` | **cross_branch** | `rh_upper_leg` | **+35.000°** |
| `lf_lower:min` | DIAGNOSTIC | `lf_hip` ↔ `lf_lower_leg` | same_branch | `lf_upper_leg` | **+64.1667°** |
| `rf_lower:min` | DIAGNOSTIC | `rf_hip` ↔ `rf_lower_leg` | same_branch | `rf_upper_leg` | **+64.1667°** |
| `rh_lower:min` | DIAGNOSTIC | `rh_hip` ↔ `rh_lower_leg` | same_branch | `rh_upper_leg` | **+93.3333°** |
| `lh_lower:min` | DIAGNOSTIC | `lh_hip` ↔ `lh_lower_leg` | same_branch | `lh_upper_leg` | **+93.3333°** |

**On the remembered "rear-leg / front-leg clearance" behaviour** — it is real, but it
describes **2 of the 6**, not all six. Only `lf_upper:max` and `rf_upper:max` are cross-leg:
swinging a **front** upper leg to its maximum brings that front foot into the **rear** foot on
the same side, and the plan lifts the **rear** upper leg out of the way. The other four are
*intra-leg* self-collisions — a leg's own hip meeting its own lower leg — parked by that same
leg's upper joint. The rear legs' own `upper:max` endpoints need no parking at all, because at
q=0 the front legs are not in their way.

**The legacy 30° / 50° / 85° / 90° prerequisites are superseded.** The current compiler found
**35.000°, 64.1667° and 93.3333°**. Its own source records why the old values were never core
truth: *"default context empty proves that +50/+90 degree legacy prerequisites are not
mandatory core truth."* The static audit fails the build if a parking pose drifts back to
within 1 mrad of any legacy value.

---

## 7. 8 UNRESOLVED STATUS — **still current, and still not PASS**

Policy `MATDOG_REFERENCE_MINIMUM_CLEARANCE`, threshold 3 mm, applied as a separate offline
consumer that does not mutate geometry.

```text
PASS                                      16
UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD     8
FAIL                                       0
motion_authorization_granted               0
clearance PASS but outside URDF limits     8
```

The eight: `lf_hip:min`, `lf_lower:min`, `rf_hip:max`, `rf_lower:min`, `rh_hip:max`,
`rh_lower:min`, `lh_hip:min`, `lh_lower:min`.

**All eight are DIAGNOSTIC endpoints**, so the repository's existing claim — zero FAIL and
zero UNRESOLVED among the eight `EXECUTABLE_URDF_DOMAIN` endpoints — is re-verified from the
artifact rather than quoted. The policy's own rule is retained verbatim: *"motion
authorization: not evaluated or granted"*.

Note the trap the numbers set: **eight endpoints pass the clearance policy while lying outside
the URDF limits.** `isExecutable()` therefore requires both conditions, and the mutation suite
proves that dropping either one is caught.

---

## 8. CALIBRATION GEOMETRY PROFILE CONTRACT

```text
URDF + collision meshes
      ↓ Geometry Compiler V5                    offline, unchanged
canonical bundle JSON                           09_Logs, unchanged
      ↓ matdog_calibration_geometry_export.py   pure reduction
src/actuator/CalibrationGeometryProfileData.h   generated constexpr tables
      ↓
Controller                                      verifies provenance, executes
                                                prevalidated primitives only
```

**The existing artifact format cannot be consumed directly** — the canonical bundle is 286 KB
+ 215 KB + 71 KB of JSON carrying worker telemetry, cgroup counters and per-path sample
hashes. The exporter is the smallest deterministic adapter: it computes no collision, no
forward kinematics and no contact search, re-verifies every input hash before rendering, and
refuses to emit anything on drift. Rounding into integer micro-radians introduces at most
**4.7 × 10⁻⁷ rad** against the compiler's own 10⁻⁴ rad bisection resolution.

**No mesh, no FK and no collision maths reach the ESP32-S3.** A generated table also means the
profile cannot be swapped under a running image; regenerating geometry costs a rebuild, which
is the right price for data that will one day authorise motion.

The profile preserves: URDF SHA, mesh manifest SHA, endpoint semantic SHA, parking semantic
SHA, safety-policy semantic SHA, servo-allocation SHA, joint/leg/contact identity keyed by
physical unit, target-domain classification, the contact bracket, the declared URDF limit,
parking outcome, the auxiliary joint and its exact parked pose, and the per-joint bootstrap
envelope. `12` joint records, `24` endpoint records, ~13 KB of source.

Not preserved, because nothing consumes them on the device: per-path sample hashes, clearance
distances, blocking link-pair names, and the 2-DOF search space that came back empty.

---

## 9. q0 BOOTSTRAP CONTRACT

```text
operator manually aligns the robot to the nominal URDF q=0 pose
Torque OFF
read raw positions            read-only, no write of any kind
capture q0 candidates
```

No automatic motion is used to find q0. Each candidate binds: joint identity, **physical
unit**, bus id *as transport metadata only*, raw tick, geometry/profile provenance, session id
and evidence state. A bus-id-only association is not expressible — `JointIdentity` has no bus
id field.

`2048` is a **sanity prior**, never an imposed q0: `JointTransform::q0_tick` defaults to 0 with
`present == false`, and a transform carrying 2048 without provenance is refused exactly like
one carrying any other number.

### The sanity window — **BLOCKED on one mechanical measurement**

The window is deliberately **not invented here**. The dominant failure mode is not a few
degrees of alignment error; it is a **one-tooth spline misindex**, and the window has to be
narrow enough to catch one:

```text
one_tooth_shift_ticks = 4096 / spline_tooth_count
q0_sanity_window      < one_tooth_shift_ticks / 2
```

What the repository proves today:

| Input | Status |
|---|---|
| Provisioned raw centre, 2048 ±1 tick, all 17 units | **VERIFIED CURRENT** |
| Output splines physically separated from horns and links | **VERIFIED CURRENT** |
| Assembly stack-up, bushings, screws, backlash, fit clearances | **UNKNOWN** — V5 `geometry_unknowns: NOMINAL_COLLISION_GEOMETRY_ONLY` |
| Adjacent-pair hardstop local sensitivity | **NOT COMPUTED** — V5 `geometry_unknowns` |
| **ST3215 output spline tooth count / horn indexing pitch** | **ABSENT FROM THE REPOSITORY** |

Every candidate window I could write down would be a number with no provenance, which is the
one thing this project consistently refuses. **One operator-supplied measurement closes it:**
the tooth count of the ST3215 output spline. With it the window follows from the inequality
above; without it any figure is an assertion.

A deviation beyond the window must **fail preflight and require mechanical inspection**. It
must never be compensated by writing EEPROM `PositionOffset` — the YAML's `forbidden:` list
and the reset document both prohibit exactly that.

---

## 10. DIRECTION VERIFICATION CONTRACT

LF V25's `direction` was **specification data used arithmetically**, never a measured witness.
The URDF's `motorDirection` is the same kind of thing. Both are what a current measurement is
compared *against*; neither may seed one. `JointTransform::direction` defaults to `0`, meaning
*not measured*, and a transform with direction 0 is refused.

```text
q0 captured
+ a small geometry-approved excursion
+ observed raw delta / physical sign
+ comparison against URDF +q
= current direction evidence
```

### How V5 supplies the envelope — answered from the existing artifacts

No new compiler run was needed. Each canonical endpoint search already sweeps **one joint from
q=0 with every other joint at q=0**, which is exactly the pose family a direction-verification
move lives in. The nearest proven-clear excursion on each side is therefore already in the
bundle: the path-obstruction bracket where the path is obstructed before contact, otherwise
the contact bracket.

The envelope is the **symmetric** minimum over both sides — and symmetric is not a
convenience, it is the entire safety argument. The move is commanded as a raw **tick delta**
before the joint's sign is known, so the same magnitude must be proven clear whichever way it
turns out to turn. Tick magnitude needs neither q0 nor direction; only the offset and the sign
are unknown.

| Joint | min side | max side | **symmetric half-span** |
|---|---:|---:|---:|
| hip (LF/RF) | 46.008° / 45.219° | 45.219° / 46.008° | **45.219°** |
| hip (RH/LH) | 45.152° / 46.008° | 46.008° / 45.152° | **45.152°** |
| upper leg (all four) | 52.129° | 73.280° or 121.871° | **52.129°** |
| lower leg (all four) | 84.270° | 38.176° | **38.176°** |

Tightest of all: **38.176°** on the lower legs — about 434 ticks. Whatever excursion the
operator approves, it will sit far inside geometry the compiler sampled and found clear.

The envelope bounds the geometry. **It does not set the excursion.** Choosing that is a
mechanical judgement about spline fit and placement error, and it shares the missing input of
§9, so `direction_verify_tick_budget` defaults to `0` = **not authorised**. Geometry and
budget are independent gates; neither can grant what the other refuses.

Operational safe limits are **not** used — they do not exist. Historical LF limits are **not**
used — they describe the previous installation.

**Not executed on hardware in this phase.**

---

## 11. CALIBRATION BOOTSTRAP AUTHORIZATION MODEL

Three routes through `SafeActuatorPolicy`, mutually exclusive by construction:

| Operation | Authorised by | Target | Today |
|---|---|---|---|
| `POSITION_COMMAND` | accepted joint bounds | tick | `REJECT_NO_ACCEPTED_LIMITS` |
| `DIRECTION_VERIFY` | bootstrap envelope **and** session budget | signed tick delta | `REJECT_NO_ENVELOPE_BUDGET` |
| `CALIBRATION_CONTACT_PROBE` | endpoint plan **and** transform | angle | `REJECT_NO_ACCEPTED_TRANSFORM` |
| `CALIBRATION_AUXILIARY_MOVE` | endpoint plan **and** transform | angle | `REJECT_NO_ACCEPTED_TRANSFORM` |

The operation model is the minimum the V5 artifacts justify. Each endpoint plan contains at
most three motion roles — park-in, task path, park-out — and parking always moves a *different*
joint from the one being calibrated. `CALIBRATION_BACKOFF` and `CALIBRATION_RESTORE` were
**not** added: both retreat along a corridor the same plan already validated, so they are the
same intent with a different target, and adding classes for them would be the mechanical
expansion the handoff warns against. `CALIBRATION_PREREQUISITE_MOVE` is the same thing as the
auxiliary move; one class covers both.

Every geometry-authorised operation first requires: a bound profile, **matching provenance on
all six hashes**, a live `CALIBRATION` lease and generation, a compatible `OperatingMode`, and
an **active non-replay** calibration session.

Fail-closed rules that are not obvious and are each tested:

- an **obstructed** plan refuses until *its own* auxiliary is parked — another endpoint's
  parking does not satisfy it;
- a **direct** plan refuses while *anything* is parked, because the compiler validated every
  direct path with all other joints at q=0;
- an auxiliary may go only to the compiler's parked pose or back to q=0 — the two
  configurations it validated, and no value between;
- a probe past the geometric contact is travelling into the mechanism, and is refused even
  while still inside the URDF limit;
- `reset()` drops the session, the transforms **and** the parked flag: after a fault the
  policy cannot know where the robot is standing.

`SAFE_OFF` remains outside all of it, structurally: no operation class can name a torque
removal, and the static audit forbids `ServoBus` from referencing this layer at all.

---

## 12. LF V25 — BEHAVIOR RETAINED vs DATA REJECTED

Every numeric constant classified before reuse, as the handoff requires. **Only class A may
transfer as behaviour.**

| Constant | Value | Class | Disposition |
|---|---:|---|---|
| coarse scout → backoff → fine ×2 → repeatability | — | **A** | **RETAINED** as behaviour |
| `ContactConfirmed` ⇒ stop advancing | — | **A** | **RETAINED** |
| EarlyStall / HardAbort / timeout taxonomy | — | **A** | **RETAINED** |
| telemetry freshness, single-owner readback | — | **A** | **RETAINED** |
| cleanup and restore philosophy | — | **A** | **RETAINED** |
| global SAFE_OFF / torque-OFF behaviour | — | **A** | **RETAINED** (already shipped) |
| `FINE_CONTACT_SCOUT_LAG_TOLERANCE = FINE_STEP` | relation | **A** | **RETAINED** as a relation |
| `COARSE_STEP_TICKS` | 64 | **C** | **REJECTED** — needs current runtime policy |
| `FINE_STEP_TICKS` | 8 | **C** | **REJECTED** |
| `BACKOFF_TICKS` | 96 | **C** | **REJECTED** |
| `STATIC_TOLERANCE_TICKS` | 10 | **C** | **REJECTED** |
| `OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS` | 16 | **C** | **REJECTED** |
| `PROBE_HOME_TOLERANCE_TICKS` | 16 | **C** | **REJECTED** |
| `REPEATABILITY_TOLERANCE_TICKS` | 16 | **C** | **REJECTED** |
| `ADAPTIVE_FINE_SCOUT_TICKS` | 32 | **C** | **REJECTED** |
| `LF_CONTACT_WITNESS_TOLERANCE_TICKS` | 24 | **D** | **ORACLE ONLY** |
| `AFFINE_SCALE_MIN/MAX_PERMILLE` | 850 / 1150 | **D** | **ORACLE ONLY** |
| measured q0 | 2067 / 2040 / 2074 | **D** | **ORACLE ONLY** |
| `direction`, `PositionOffset`, bus-id identity, joint limits | — | **D** | **ORACLE ONLY** |
| speed, acceleration, `TorqueLimit`, current thresholds, timings | — | **C/D** | **REJECTED** — the new execution contract must define and verify its own low-energy runtime settings |
| EEPROM freeze, hardcoded parking/prerequisites, the 18-state sequence | — | **D** | **SUPERSEDED** |

Nothing from class B (geometry data) is taken from LF V25 at all: geometry comes from the V5
profile.

The operator recalls two servos in the old LF campaign having non-uniform speed configuration.
That is a reason to **audit** old motion constants, not a source for new ones — and it is why
every runtime motion parameter above is class C.

---

## 13. REUSED / STALE / SUPERSEDED COMPONENTS

**REUSED, unchanged:** Geometry Compiler V5 (all 11 sources, all 8 artifacts); `ServoBus`;
`ServoCensus` / `ServoPopulation`; `ActuatorAuthority` arbiter with lease, generation and
inhibit; `CalibrationManager` session lifecycle; `SafeActuatorPolicy` transaction core;
`CalibrationDomain` identity, evidence lifecycle and provenance predicates; the Wi-Fi/OTA
runtime untouched.

**ADDED, minimal:** one offline exporter; one pure profile type plus its generated table; two
operation classes; one transform table; one bootstrap context.

**NOT created:** no second geometry engine, no second `ServoBus`, no second UART, no second
scheduler, no second calibration state machine, no Station architecture, no runtime collision
or FK on the device.

**STALE / SUPERSEDED:** the pre-reset joint entries in `MATDOG_JOINT_CALIBRATION.yaml`;
`rear_parking_pose` and `front_leg_dependencies`; the 30/50/85/90 prerequisite poses;
pre-2026-08-27 digital zero and `PositionOffset` values; LF V25 numeric evidence; the 18-state
LF sequence as an architecture.

---

## 14. SAFE ACTUATOR POLICY CHANGES

`POSITION_COMMAND` is **not weakened**. Its route is unchanged, it still requires an accepted
bound on the current machine, and a bound geometry profile plus a live session plus an
approved envelope plus an accepted transform together still leave it at
`REJECT_NO_ACCEPTED_LIMITS`. The audit fails the build if `POSITION_COMMAND` is ever routed
through either geometry path, and a mutation proves it.

Added: two operation classes (§11), 14 new refusal reasons, `JointTransformTable`,
`CalibrationBootstrapContext`, and the geometry binding. `reset()` now also clears transforms,
the session and the parked flag.

Changed: `CALIBRATION_CONTACT_PROBE` moved from the accepted-limits route to the endpoint-plan
route. It is strictly more constrained than before — it now additionally requires a bound
profile, matching provenance, a live session, an executable endpoint, a satisfied parking
state and an accepted transform.

---

## 15. DEFAULT BUILD WRITE REACHABILITY

```text
Unchanged: torque OFF only, inside ServoBus::safeOff().
```

No `ServoBus` write adapter, no Torque ON, no goal position, no `SyncWrite`, no EEPROM access,
no `PositionOffset` write, no ID write, no provisioning, no calibration motion, no DALY write.
`MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED` still defaults to `0`.

```text
CURRENT HARDWARE MOTION AUTHORIZATION   BLOCKED
```

---

## 16. NEXT SINGLE HARDWARE GATE

**Robot-powered read-only preflight — formal 12/12 leg population, identity and profile
verification.** Nothing else, and it is read-only: `Ping` and register reads, `SAFE_OFF`
available throughout, no torque, no motion, no write.

It needs no new firmware capability — `ServoCensus` and `ServoPopulation` already do it — and
it is the gate that turns the last formal `6/12` into a current result.

Only after it passes, in order, each its own authorised session:

1. manual URDF q=0 pose, torque OFF;
2. read-only q0 capture → q0 candidates;
3. **the q0 sanity window** (§9) — *blocked until the spline tooth count is supplied*;
4. one micro direction verification, budget explicitly approved;
5. one-joint bootstrap actuator transaction validated;
6. staged contact calibration — upper legs first, since they are the only executable endpoints;
7. current safe limits;
8. stand-up.

Two items must be closed by a human before step 4, and neither is a software task:

- the **ST3215 output spline tooth count**, which sets the q0 sanity window;
- the **low-energy runtime settings** (goal speed, acceleration, torque limit) for the new
  execution contract, which must be defined and verified rather than inherited from LF V25.

---

## 17. Offline result

No hardware action of any kind. No flash, no servo write, no torque, no goal position, no
EEPROM access, no provisioning, no DALY write, no push, no merge.

| Gate | Result |
|---|---|
| `static_audit.py` | **PASS** — 68 source files |
| Safe Actuator / geometry mutation suite | **PASS** — 30 targeted mutations, each caught with the expected reason |
| Host suite, calibration geometry | **PASS** — 354 checks, 0 failures |
| Host suite, write policy | **PASS** — 310 checks, 0 failures |
| Host suite, total | **PASS** — 4876 checks, 0 failures across 9 suites |
| Geometry export `--check` | **PASS** — the committed table matches the canonical bundle |
| `USB_ONLY` clean build (pinned FQBN) | **PASS** — 973447 bytes flash, 51812 bytes static RAM |
| Flash / RAM delta vs `988c84a` | **0 / 0 bytes** |

The zero delta again: the profile tables are `constexpr` and no runtime translation unit
references them yet, so the linker discards them. The shipped image gained no write path and
no footprint.

```text
CURRENT HARDWARE MOTION AUTHORIZATION   BLOCKED
DEFAULT BUILD WRITE REACHABILITY        torque OFF only, unchanged
```

---

## 18. Related

- [`SAFE_ACTUATOR_LAYER.md`](SAFE_ACTUATOR_LAYER.md) — the write-surface audit and the policy core
- [`CALIBRATION_SOURCE_PRECEDENCE.md`](CALIBRATION_SOURCE_PRECEDENCE.md) — source precedence and the EEPROM boundary
- [`DEVELOPMENT_GATES.md`](DEVELOPMENT_GATES.md) — the calibration gate
- [`../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md`](../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)
- [`../../06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_V5_FINAL_ARCHITECTURE_2026-08-11.md`](../../06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_V5_FINAL_ARCHITECTURE_2026-08-11.md)

# G2 — contact-space stand and motion foundation

## Checkpoint

G2 implements pure C++ physical contact FK/IK, deterministic C4-A stand
planning, a 51-frame contact-locked low-to-stand generator, semantic 12-joint
frames and the OFF/IDLE/STAND_TRANSITION/STAND/STOPPING lifecycle.

- Worktree: `/home/matteo-manicardi/MATDOG/worktrees/robot-dog-gait-engine`
- Branch: `feat/gait-engine-offline-v1`
- Accepted G1 starting HEAD: `a543a3590c3e575ea00a918fe32a700bf2a4ea75`
- Original main base: `bff2ae4517d88fc437213f3bb53427b800b784e7`
- G1's public header/semantic contract is unchanged. No G1 defect was found.
- No WALK/TROT, actuator integration, hardware timing or rate limits were added.

## Model and inverse boundary

The representation is generated from the current audited
`MATDOG_FOOT_CONTACT_GEOMETRY.yaml`, validated against the canonical URDF by
the existing Python binding checker. The shared cylinder has center
`(0,0,0.0149)` m in foot_link, axis `(0,1,0)`, radius 0.0149 m and a conservative
central support strip 0.0099 m wide. The real URDF distal X/Z eccentricity and
left/right ±1.5 mm Y offsets remain unchanged.

For the foot-link pose `(R,p)` and normalized ground normal `n`:

```text
axis = normalize(R * cylinder_axis)
center = p + R * cylinder_center
radial_down = -normalize(n - axis * dot(axis,n))
contact_reference = center + radius * radial_down
```

The full result also preserves strip endpoints, lowest finite-core endpoint,
axis tilt and nominal/edge-biased mode. The canonical continuous cross-section
reference is distinct from the lowest endpoint for a tilted finite cylinder.
C4 uses the continuous reference and requires nominal strip contact.

G1 still solves the actual foot_link origin. G2's analytic contact IK is an
explicit **+Z-ground-in-base_link** API. Contact FK additionally supports a
general normal, but that does not imply tilted-body/terrain IK support.

With the verified X/Y/Y chain and hip limits inside (-pi/2,pi/2), the exact
contact reduction uses a virtual distal Z of `foot.z + radius` plus a separate
`-radius` Z translation in the hip-rotated frame. G2 shares the G1 analytic
branch enumeration through an internal helper, then verifies its solution
using the full physical contact FK. No contact target is incorrectly passed
as a real foot-link-origin target.

The final stand is solved from the C4-A physical footprint. Initialization
anchors the solution at the final stand then solves the low compatible pose.
Every emitted ascending sample uses the previous valid solution as its seed.
This reproduces C4-C's locked contacts and linear 0.100-to-0.150 m body heights
without storing/reversing a descending numerical solve. It does not provide
a q=0-to-low-stance acquisition trajectory.

Details and formulas: `05_Firmware/MATDOG_Controller/src/motion/CONTACT_STAND.md`.

## Source provenance

| Source | SHA-256 |
|---|---|
| Canonical URDF | `3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59` |
| Foot-contact YAML | `43dc72abdc132ef2492d383efe4ed12dfe2e18cf17705e69fc21f4166c4f0966` |
| C4-A `2026-07-08_175245_C4A_offline_safe_stand_candidate.json` | `221b6551ccf1bf36cd10b2b4136da38d1d2a73af18d65e0f3486de1faa72991b` |
| C4-C `2026-07-08_190405_C4C_contact_locked_rest_to_stand_trajectory.json` | `4b667a5995e55122b9578c39042914a373aa6bb4e0be84de11381e7f2398b7b2` |

Generated stand data contain only the validated footprint, preferred semantic
seeds, height endpoints and sample count, with source hashes. They do not
contain an independent hand-authored geometry or an actuator mapping.

## Numerical comparisons

| Check | Maximum measured |
|---|---:|
| Contact FK vector difference from Python, 648 cases | 4.002966042487e-16 m/unit-vector units |
| Contact axis tilt difference | 2.220446049250e-16 rad |
| Contact IK round-trip residual, 648 cases | 1.118863022828e-16 m |
| C4-A joint difference from archived candidate | 5.498188095520e-7 rad |
| C4-C joint difference from archived 51-frame sequence | 2.110450133915e-4 rad |
| Joint difference from refined Python contact IK | 3.730215580866e-10 rad |
| Per-frame joint-delta difference from archived C4-C | 2.108700418759e-4 rad |
| Stand trajectory contact residual | 7.473417450352271e-17 m |
| Stand trajectory contact drift from first sample | 9.813077866773594e-17 m |
| Minimum joint-limit margin over the trajectory | 0.3049188401684514 rad |

All 51 generated frames retain four physical contact references at world Z=0
within 1e-9 m, with body rotation identity and nominal strip contact.
C4-A and C4-C start/middle/end were independently refined by the existing
Python contact DLS at 1e-10 m tolerance (16 leg solves).

The archived numerical C4-C solution has up to approximately 10 micrometres
of residual. Exact analytic contact targets therefore differ slightly from
its stored angles. The agreement with refined Python IK confirms the difference
is expected numerical refinement, not a branch change or contact-offset error.

### Maximum absolute per-frame joint delta

Units: **radians per geometric sample**. The body-height increment is 1 mm.
There is no sample time, implied velocity, acceleration limit or hardware
eligibility claim.

| Leg | Hip | Upper | Lower |
|---|---:|---:|---:|
| LF | 0 | 0.012288128068594917 | 0.0225291444407838 |
| RF | 2.220446049250313e-16 | 0.012288128068594917 | 0.0225291444407838 |
| RH | 2.220446049250313e-16 | 0.010920836394428535 | 0.016688362809624158 |
| LH | 2.220446049250313e-16 | 0.010920836394428535 | 0.016688362809624158 |

Global maximum: **0.0225291444407838 rad/sample**. Hip deltas are zero to
floating-point precision. Each sample exposes signed deltas, individual joint
limit margins, contact residual/drift and analytic branch labels. The first
sample has zero delta; aggregates were independently recomputed in Python.

**Selected branch changes: LF=0, RF=0, RH=0, LH=0.** All frames use hip/elbow
labels `(-1,-1)`. The 0.025-rad test threshold is only a geometric regression
bound for this fixed 51-frame reference; comparison against every archived
delta and explicit branch labels provides the independent continuity check.

### Existing C4 policy rerun

The existing Python C4 mesh-ground and knee/contact checks were rerun on the
actual newly solved C++ angles for C4-A and all 51 C4-C frames:

- 52/52 poses pass the existing policy.
- Minimum non-foot ground clearance: `0.002708862337240 m`.
- Minimum knee/contact clearance: `0.08775925732693 m`.
- The expected lower-leg fork clearance review remains; no policy was weakened.

This is the existing offline C4 policy, not a new general self-collision,
compliance, friction or dynamic-stability certification.

## Test results

**Complete controller static audit and complete host runner: PASS.**
The audit scanned 129 source files and executed the existing host runner,
including the final G1/G2 geometry, C++, Python oracle and C4 policy gates.

- G1 C++ regression: 1,228 checks, 0 failures.
- G1 Python differential regression: 2,380 vectors passed, with unchanged
  numerical maxima and identical repeated output.
- G2 C++ suite: 2,005 checks, 0 failures.
- G2 contact differential suite: 648 vectors passed, including all four legs,
  q=0, exact limits, deterministic random configurations, contact-mode
  threshold probes and tilted-normal FK.
- C4-A + all 51 C4-C frames passed contact, limit, branch and metric checks.
- 16 independently refined Python contact IK solutions passed.
- 7 unsupported contact-model/reference mutations were rejected.
- AddressSanitizer + UndefinedBehaviorSanitizer: 2,005 checks, 0 failures,
  no sanitizer diagnostics.
- Source/include review: only pure motion headers and standard math/types;
  no dynamic allocation, mutable global state, exception or hardware dependency.
- `git diff --check`: pass.

The state tests cover all **40 state/event combinations** (five states, seven
valid events and one invalid enum). They check permitted transitions, rejected
pairs leaving state unchanged, global DISABLE/FAULT, stand completion and
cancellation. A separate generator lifecycle test verifies initialization,
sequential frames, exhaustion, restart, stop and invalid/partial-frame rejection.
Completion events are supplied by the caller; the lifecycle has no hardware
side effects, clock, automatic motion or physical deceleration behavior.

Commands executed from the worktree root include:

```sh
bash 05_Firmware/MATDOG_Controller/scripts/tests/run_motion_host_tests.sh
python3 06_Software/Matdog_Core/kinematics/matdog_contact_stand_export.py --check
python3 05_Firmware/MATDOG_Controller/scripts/tests/test_contact_stand_oracle.py /tmp/matdog-g2-oracle
python3 05_Firmware/MATDOG_Controller/scripts/static_audit.py
```

The first motion run verified the G1 extraction; the complete final motion
runner, including G2, is invoked through the existing complete host runner by
`static_audit.py`. No new build/test framework was added. Standalone unit and
oracle adapters were compiled with C++17, `-Wall -Wextra -Werror -O1`,
`-fno-exceptions -fno-rtti`. The sanitizer unit executable added
`-g -fsanitize=address,undefined -fno-omit-frame-pointer`.

Numerical gates: contact FK/rotation vectors 1e-12; contact IK/stand residual
1e-9 m; archived joint/delta agreement 3e-4 rad; refined Python joint agreement
1e-7 rad. G1's 1e-12 distance/cosine/angle roundoff guards remain. Production
contact residual is verified again using the full physical model.

## Files and commits

Added under `05_Firmware/MATDOG_Controller/src/motion/`:

- `LegKinematicsInternal.h`
- `FootContact.h`, `FootContact.cpp`, `FootContactData.h`
- `StandTrajectory.h`, `StandTrajectory.cpp`, `StandReferenceData.h`
- `MotionState.h`, `MotionState.cpp`
- `CONTACT_STAND.md`

Added under `05_Firmware/MATDOG_Controller/scripts/tests/`:

- `test_contact_stand.cpp`
- `contact_stand_oracle_driver.cpp`
- `test_contact_stand_oracle.py`

Also added:

- `06_Software/Matdog_Core/kinematics/matdog_contact_stand_export.py`
- This G2 development/validation report.

Modified:

- `src/motion/LegKinematics.cpp` and `LegInverseKinematics.cpp`: internal shared
  reduction; G1 public API unchanged.
- `src/motion/README.md`: link to G2 contract.
- `scripts/tests/test_motion_oracle.py`: extend the allowed local include list.
- `scripts/tests/run_motion_host_tests.sh`: add G2 suites to the existing gate.

Local commits:

- `bb74e22` — share verified analytic chain solver; no G1 semantic change.
- `5f68945` — physical contact kinematics and verified stand definition.
- `8fa8b8a` — contact-locked stand generation and pure lifecycle.
- `d8767eb` — contact, continuity, state and C4 policy tests.
- `728185c` — contact-space and lifecycle contracts.
- A separate documentation commit records this final report.

The implementation worktree is clean after the above commits. Final status is
checked after committing this report. The calibration workstream was not
reopened. A scoped Git diff confirms no change to actuator, servo, calibration
or URDF paths. No physical hardware or serial device was accessed, no servo
command was issued, and no firmware was built/uploaded/flashed. No push, merge,
main modification or calibration-branch modification occurred.

# G3 — startup acquisition and timed stand

## Resume checkpoint and scope

- Worktree: `/home/matteo-manicardi/MATDOG/worktrees/robot-dog-gait-engine`
- Branch: `feat/gait-engine-offline-v1`
- Exact accepted G2 starting HEAD: `9c4fac0b0de5e9a4df08bc12569b24d2b8d6b78a`
- Initial status, staged/unstaged diffs, untracked files and recent history
  showed a clean G2 checkout. The interrupted attempt left no G3 implementation,
  tests or commits. Its work was read-only inspection. Nothing was discarded,
  reset, overwritten or amended to resume.
- G1 geometry/IK and G2 contact semantics are unchanged. No defect requiring a
  G1 repair was found. G3 adds a continuous accessor to the existing C4 path
  and deliberately tightens G2 lifecycle entry/completion ownership.

The [implementation contract](../../05_Firmware/MATDOG_Controller/src/motion/STARTUP_TIMING.md)
contains API semantics, mathematical equations and numerical guards.

## Physical questions: explicit answers

### What must be true before the first autonomous physical stand?

MATDOG must already be stationary in the verified C4 low support stance. In
the canonical world frame this means body translation (0,0,0.100) m, body
parallel to the flat ground, and the actual four physical foot contacts at
world Z=0 on the canonical footprint:

| Leg | Contact X (m) | Contact Y (m) |
|---|---:|---:|
| LF | 0.1301352313457785 | 0.094 |
| RF | 0.1301352313457785 | -0.094 |
| RH | -0.1437169534863073 | -0.094 |
| LH | -0.1437169534863073 | 0.094 |

The observed semantic joints must match the canonical in-limit low solution.
The physical feet must actually support the robot; touching the floor alone
does not establish that. Current pose, stationary status, flat floor, collision
clearance throughout the intended rise and support for the actual load must be
confirmed. C4-E only validates a base-origin COM proxy with a ±20 mm uncertainty
box; it does not prove actual loaded COM or dynamic stability.

G3 accepts explicit external evidence assertions and checks model consistency;
it cannot measure or establish these physical facts. Its nanometre/radian
agreement tolerances are numerical checks, not a physical observation contract.
Physical execution still requires a separately approved observation, actuator
and safety execution layer. **This offline milestone does not authorize motion.**

### Can q=0 safely be an autonomous ground-supported starting pose?

**NO / NOT PROVEN.** q=0 is a reference pose. Front/rear physical contact heights
in base_link are -0.0934/-0.1134 m, a 20 mm mismatch for a parallel body. The
existing direct joint-space q=0-to-stand path also has intermediate non-foot
ground penetration. No universal supported or suspended-release acquisition
path is justified by the current evidence. The required starting precondition
is the already verified low C4 stance above.

## Startup and acquisition architecture

| Class | Result | Support regime / output |
|---|---|---|
| A: suspended/unloaded at reference | `SUSPENDED_PATH_UNPROVEN` | External suspension; no target |
| B: manually placed feet, support unverified | `FLOOR_ACQUISITION_UNPROVEN` | Unverified floor; no target |
| C: canonical low stance with complete evidence | `READY` | Four contacts locked; low hold |
| D: unknown | `UNKNOWN_POSE` | Unknown; no target |

Missing evidence, invalid observation, body-pose mismatch, joint-limit failure,
low-stance mismatch, contact mismatch and reference failure are separate statuses.
All evidence flags default false. No acquisition from A/B/D was invented.

Segments: (1) zero-intended-motion acquisition hold at t=0 in verified low
stance; (2) contact-locked, parallel-body rise from 0.100 to 0.150 m. Both use
four nominal physical contact strips. No free-space segment is generated.
The previous valid joint solution seeds every successive IK sample.

`BodyPose` implements checked rigid world/base point transforms. Its contact
IK wrapper accepts translation and yaw, and rejects roll/pitch explicitly.
G2 remains a +Z-ground-in-base_link IK API. Full orientation-dependent cylinder
contact geometry and exact distal eccentricity are preserved. G1 continues
to target foot_link origins; physical contacts are never substituted into it.

## Timing and semantic metrics

The path is independent of the clock. External duration T and interval count N
define timestamps t=i*T/N; an external period can also construct the timing
specification. No duration or servo-rate limit is physically approved here.
Tests use T=2,5,10 seconds, each with N=100 (101 frames, including endpoints).

For u=t/T, s=10u³−15u⁴+6u⁵. Both ds/dt and d²s/dt² vanish at the endpoints.
The exact contact Jacobian and directional Hessian give q_s and q_ss, then
q_dot=q_s*s_dot and q_ddot=q_ss*s_dot²+q_s*s_ddot. Ill-conditioned/singular or
nonfinite derivatives fail explicitly. Jerk is not modeled as a limit.

At matched normalized times the tested joint positions are exactly identical
across durations. Every joint satisfies q_dot*T and q_ddot*T² invariance
(1e-12 Python oracle tolerance, 1e-10 C++ tolerance). Endpoint rates are exactly
zero in emitted frames. Reruns are byte-identical in the same binary/environment.

The following are **sampled** peak absolute rates, not certified continuous
maxima or actuator eligibility limits. Hips are numerical roundoff around zero.
All positions/derivatives use semantic URDF radians in LF/RF/RH/LH order.

| Joint | Peak rad/s, T=2 | T=5 | T=10 | Peak rad/s², T=2 | T=5 | T=10 |
|---|---:|---:|---:|---:|---:|---:|
| LF hip | 6.11639708e-17 | 2.44655883e-17 | 1.22327942e-17 | 9.96774041e-17 | 1.59483847e-17 | 3.98709616e-18 |
| LF upper | 0.415764756 | 0.166305902 | 0.0831529512 | 0.775823016 | 0.124131683 | 0.0310329206 |
| LF lower | 0.761065512 | 0.304426205 | 0.152213102 | 1.42276251 | 0.227642001 | 0.0569105002 |
| RF hip | 6.06239601e-17 | 2.42495841e-17 | 1.2124792e-17 | 7.70938541e-17 | 1.23350167e-17 | 3.08375417e-18 |
| RF upper | 0.415764756 | 0.166305902 | 0.0831529512 | 0.775823016 | 0.124131683 | 0.0310329206 |
| RF lower | 0.761065512 | 0.304426205 | 0.152213102 | 1.42276251 | 0.227642001 | 0.0569105002 |
| RH hip | 6.92854407e-17 | 2.77141763e-17 | 1.38570881e-17 | 8.75575774e-17 | 1.40092124e-17 | 3.5023031e-18 |
| RH upper | 0.464875549 | 0.18595022 | 0.0929751099 | 0.747384974 | 0.119581596 | 0.0298953989 |
| RH lower | 0.640964551 | 0.25638582 | 0.12819291 | 1.11203688 | 0.1779259 | 0.0444814751 |
| LH hip | 6.9495197e-17 | 2.77980788e-17 | 1.38990394e-17 | 1.09914309e-16 | 1.75862895e-17 | 4.39657237e-18 |
| LH upper | 0.464875549 | 0.18595022 | 0.0929751099 | 0.747384974 | 0.119581596 | 0.0298953989 |
| LH lower | 0.640964551 | 0.25638582 | 0.12819291 | 1.11203688 | 0.1779259 | 0.0444814751 |

## Contact, collision, continuity and oracle evidence

- 303 timed frames pass contact FK, limits, phases, lifecycle, timing, scaling
  and deterministic reruns.
- The 101 distinct poses pass the existing C4 mesh-ground, knee/contact and
  C4-E support-proxy policy. Matched poses at the other durations are identical,
  so those policy results apply to each corresponding frame.
- Minimum non-foot ground clearance: **0.002708862337240 m**.
- Minimum knee/contact clearance: **0.08775925732693 m**.
- Minimum support-proxy margin with ±20 mm box: **0.074 m**.
- Minimum URDF joint-limit margin: **0.3049188401684514 rad**.
- Maximum C++ contact residual: **6.938893903907228e-17 m**.
- Maximum C++ contact drift: **9.813077866773594e-17 m**.
- Independent Python contact residual/drift: **6.206335383118e-17 /
  8.673617379884e-17 m**.
- All four legs retain branch `(hip=-1, elbow=-1)`; **zero selected branch
  changes** for every duration. The sampler rejects a branch change.
- 80 independent existing Python contact IK solves around five interior times
  validate analytic joint derivatives using five-point differences. Maximum
  velocity error **2.983287304692e-9 rad/s**; acceleration error
  **5.634461444082e-7 rad/s²** (gates 1e-7 and 2e-6 respectively).
- All 51 original G2 geometric samples match the continuous path within
  1e-12 rad. The full unchanged G1/G2 Python gates also replay C4-A and all
  51 archived C4-C frames; analytic vs archived numerical differences retain
  the accepted G2 tolerances.

These are sampled offline results. They do not establish continuous swept
collision freedom, actual load stability, friction, actuator capacity or dynamic
safety. The existing C4 lower-leg fork review and COM-proxy caveats remain.

## State machine

`StandTransition` owns startup check, sampler and lifecycle. IDLE can enter
STAND_TRANSITION only through `begin()` after READY and successful initialization.
Public BEGIN_STAND/STAND_COMPLETE cannot bypass verification. STAND is reached
only after every expected valid frame, including the final s=1 frame, is emitted.
This is offline generation completion, not an assertion of physical arrival.

STOP cancels sampling and enters STOPPING. STOP_COMPLETE returns IDLE;
DISABLE/FAULT cancel and go OFF. A timed-sample failure goes OFF without a valid
fallback target. Restart requires new startup evidence. STOPPING has no physical
braking/deceleration claim.

Tests cover all 40 public state/event pairs, startup rejection, missing flags,
q=0 rejection, invalid timing, attempted premature completion, full completion,
cancellation, restart and interior numerical failure. The old G2 state matrix
was replaced by this gated coordinator matrix; its geometric tests remain.

## Files and local commits

Added in `05_Firmware/MATDOG_Controller/src/motion/`:
`BodyPose.h/.cpp`, `StartupAcquisition.h/.cpp`, `TimedStand.h/.cpp`,
`StandTransition.h/.cpp`, `STARTUP_TIMING.md`.

Modified there: `StandTrajectory.h/.cpp` (continuous path accessor),
`MotionState.h/.cpp` (private verified entry/completion), `README.md` and
`CONTACT_STAND.md` (G3 contract links and superseded lifecycle note).

Added in `05_Firmware/MATDOG_Controller/scripts/tests/`:
`test_startup_timed_stand.cpp`, `startup_timed_oracle_driver.cpp`,
`test_startup_timed_oracle.py`.

Modified there: `run_motion_host_tests.sh` (G3 gate), `test_motion_oracle.py`
(pure include allowlist), `test_contact_stand.cpp` (reject public bypass events;
full state matrix moved to G3). Also added this development/validation report.

- `01e5c1d` — verified low-stance startup, body frames and continuous C4 accessor.
- `4f2d76d` — quintic timing and analytic semantic derivatives.
- `380e064` — lifecycle entry/completion owned by verified coordinator.
- `81534de` — G3 host/oracle/C4 policy tests and existing runner integration.
- `05d9b67` — startup, timing, physical evidence and API boundary contract.
- A separate documentation commit records this final validation report.

## Validation commands and complete results

| Gate | Result |
|---|---|
| Complete existing host runner (22 original host executables plus motion gate) | PASS through static audit; exit 0 |
| G1 geometry exporter, C++ unit and Python FK/IK/C4 oracle | PASS |
| G2 contact/stand exporter, C++ unit and Python C4 contact/collision oracle | PASS; 1,925 C++ checks, 0 failures |
| G3 C++ startup/body/timing/lifecycle suite | PASS; 12,225 checks, 0 failures; 40 state/event pairs |
| G3 Python oracle | PASS; 303 frames, 101 distinct collision/support poses, 80 derivative IK solves |
| Full static audit, including existing mutation/provenance tests | PASS; 139 source files; exit 0 |
| G3 AddressSanitizer + UndefinedBehaviorSanitizer | PASS; 12,225 checks, 0 failures; no sanitizer diagnostics |
| C++17 warning/error and no-exception/no-RTTI motion builds | PASS |
| Git whitespace and scoped boundary diff | PASS |

The original host executables cover servo population/profile, DALY protocol,
Wi-Fi policy, actuator authority/write policy, calibration geometry/q0 bootstrap/
q0 capture/domain/population/manager, OTA policy, actuator runtime, calibration
execution, service readiness, LED status, both LED power profiles, HMAC, OTA
session and HTTP mailbox. All are existing offline tests with fake backends;
no production layer was modified. The static gate also runs its existing OTA
partition, DALY/safe-actuator/LED mutation, build manifest and backup gate checks.
The static audit captures successful nested output; its top-level result is
`STATIC_AUDIT = PASS`. No tests were skipped or exceptions added to make it pass.

Commands executed from the worktree root:

```sh
python3 05_Firmware/MATDOG_Controller/scripts/static_audit.py
# Invokes the complete run_host_tests.sh and run_motion_host_tests.sh gates.
python3 05_Firmware/MATDOG_Controller/scripts/tests/test_startup_timed_oracle.py /tmp/matdog-g3-oracle
g++ -std=c++17 -Wall -Wextra -Werror -fno-exceptions -fno-rtti -O1 -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I05_Firmware/MATDOG_Controller/src/motion \
  05_Firmware/MATDOG_Controller/scripts/tests/test_startup_timed_stand.cpp \
  05_Firmware/MATDOG_Controller/src/motion/*.cpp -o /tmp/matdog-g3-sanitized
/tmp/matdog-g3-sanitized
git diff --check
```

## Boundaries and repository state

Only the pure motion module, its host test support and this report changed.
Canonical URDF/contact geometry, calibration/q0/directions, servo and actuator
implementations are unchanged. Existing full host tests exercise mocked policy
code offline and the static audit reads repository sources; no hardware or
servo/actuator runtime was accessed. No serial access, servo command, raw ticks,
firmware flashing, hardware test or runtime binding was introduced or performed.
No WALK/TROT, gait scheduler, IMU control or actuator-rate enforcement was added.
No push, merge, history rewrite or modification to other worktrees occurred.
Final worktree status is checked after the report commit.

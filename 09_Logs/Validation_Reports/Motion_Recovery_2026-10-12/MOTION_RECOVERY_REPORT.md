# MATDOG Motion Recovery — verified software checkpoint, 12 October 2026

**Natural belly-supported prone → STAND 150 mm remains geometrically unverified.**
This delivery preserves the existing Motion integration and makes its geometry
failure reproducible. It is a software review candidate, not readiness for a
physical trial. No flash, OTA, torque, actuator command, calibration, EEPROM/NVS
write/erase, commit or publication was performed.

Only worktree `/home/matteo-manicardi/.codex/worktrees/motion-enablement-20261012/robot-dog`
at `fdd04345c39b50137cc7cdc50fb08cd592b8c2af` was changed. The old ASUS branch was
not used. The installed `b08322de…` firmware and generation-1 calibration are
separate facts. There is no reset, clean, destructive checkout or source deletion.

## Implemented change since the recovery checkpoint

AdaptiveStand now classifies PARTIAL_SUPPORT separately. Its acquisition route
must begin with exactly the observed foot-contact mask and belly condition;
required external support must be independently confirmed. A target foot XYZ
cannot turn one/two/three confirmed supports into four. New tests cover masks
1–14, route absence/mismatch, no output without assessment, and refusal when
feedback still reports partial support. Four foot contacts while the belly remains
supported cannot bypass acquisition. C4 rise, STAND confirmation and WALK start/
continuation also require release of ventral support. UNKNOWN/INVALID remain inhibited.
The trace now carries explicit world `base_position_m` for correct WALK renders.

RESTORE, MotionService, radian Executor, Safe Actuator Layer, ServoBus ownership,
watchdog/SAFE_OFF/revocation, G4 scheduler, Dashboard and Digital Twin are retained.
No UI feature development or Motion Core rewrite occurred in this recovery step.

## Identifying the initial pose

[Historical evidence](PRONE_EVIDENCE.md) finds the September 22 successful C4-C
closeout at `58b97392dc13efa30e166fe3802efa677dcba011`. It declares a physical stand,
but lacks a timestamped twelve-joint starting/final vector, measured base height
and contact/load capture. The July RF failure and pre-remount encoder references
are unsuitable for this installation. Consequently the actual natural rest
joint configuration is **UNKNOWN**, not q=0 and not the calibration reference.

The previous G35 REST_GROUND model uses positive lower angles and directed
upper/lower angles of 31–39 degrees: it is extended in the same direction rather
than strongly folded. The new candidates derive upper=pi/2 from a horizontal
upper axis, lower from the canonical URDF fold boundary plus disclosed research
margins .03/.08/.13 rad, and mirrored HIP roots from actual STL/plane constraints.
These are labelled geometric hypotheses, never captured readings or new limits.

The .03-rad folded hypothesis is:

| Leg | HIP [rad] | upper [rad] | lower [rad] |
| --- | ---: | ---: | ---: |
| LF | 0.066510811 | 1.570796327 | -1.575702912 |
| RF | -0.066659685 | 1.570796327 | -1.575702912 |
| RH | -0.451063455 | 1.570796327 | -1.575702912 |
| LH | 0.450903047 | 1.570796327 | -1.575702912 |

Joint order is LF/RF/RH/LH. Upper/lower directed angle is 155.269 degrees;
upper is horizontal and the lower axis is approximately 22–25 degrees below the
floor. All twelve values and the other hypotheses are in
[bounded_family.json](bounded_family/bounded_family.json).
At identity body orientation, the canonical body underside minimum is
approximately −1.09e−16 m, so `base_link Z` at belly contact is approximately 0 m.
With pitch, Z is recomputed from the rotated body's lowest vertex against world
Z=0. No IMU altitude is inferred. CAD mass is 2.48 kg; this candidate's initial
CAD COM is (−0.01442146, −0.000335765, 0.03496284) m. As-built mass distribution,
battery/electronics placement, friction and load-bearing support remain unknown.

The missing input is a coherent twelve-joint read-only capture of the actual
belly-rest pose using the current verified mapping, with timestamp, body
orientation, support plane/body height and independent contact/load information.
No hardware capture was attempted here.

## Bounded geometry comparison

The original finite family compares three fold hypotheses with rear-first and
coordinated deployment. Six candidates stop at RH outward HIP/ground constraints
at diagnostic frames 13–15. A single correction rocks the belly first, then
starts deployment: two further recipes stop at frame 24. Front overlap timing
has not begun at those failures, so the recipes cannot yet be ranked for a
successful load transfer. These results reject the tested family; they do not
prove every possible MATDOG recovery impossible.

At corrected failure frame 24, pitch=−0.150 rad, roll=0, base Z=0.019546508 m,
RH HIP=−0.785398163397, upper=1.657417188552, lower=−1.210584512476 rad:

| Canonical link vs world Z=0 | Minimum signed plane distance |
| --- | ---: |
| rh_hip_link | +13.725584 mm |
| rh_upper_leg_link | +30.942284 mm |
| rh_lower_leg_link | **−3.539071 mm** |
| rh_foot_link | **−3.395601 mm** |

[Independent witnesses](independent_folded_witnesses.json) use all 17 links and
17 canonical collision STL, 120 non-adjacent clearance pairs and the stronger
132 obstruction pairs. Initial/last-before-failure/failure snapshots have no
self-intersection; their reported worst non-adjacent clearance is a **1 mm lower
bound**, not an exact clearance measurement. The floor penetration rejects the
failure pose. All q, transforms and lowest world vertices are retained. The last
snapshot is a failed constraint evaluation, not a valid waypoint/actuator target.
The 40 ms probe sampling is diagnostic and not approved motion timing; IK branch,
operational envelopes, velocity/acceleration and swept clearance are not qualified.

The old fixture route's complete 2319-frame canonical audit finds **212
intersecting frames**, with zero unexplained plane-penetration frames. LF foot
versus LH hip begins at frame 261; LF lower versus LH hip begins at 282. The
[exact witnesses](prior_fixture_collision_witnesses.json) retain all q and mesh
transforms, V5 intersection triangle IDs and AABB overlap separately. Signed
mesh penetration depth is unknown and is not replaced by an AABB distance.
Rigid body lifting translates both links equally, so it cannot remove their
self-intersection. The extended initial model drives the anterior distal links
behind their hips across the posterior hip. The checker was not suppressed.

[Geometry reuse proof](prior_geometry_reuse.json) confirms that the new explicit
XYZ trace preserves every old joint/quaternion/height/foot sample. Self-collision
and flat-plane results are therefore reused; the complete mesh audit is not
rerun on unchanged geometry.

## Deliverable figures and traces

All figures derive from canonical URDF/STL and inertials; no generated illustration.
The 3D display uses recorded deterministic triangle subsampling. Collision checks
use full meshes, not display triangles. The 3D/projection SVG embeds the matching
PNG to avoid enormous polygon assets; time-series SVG remains vector output.

- [Folded candidates: 3D](folded_candidate_figures/canonical_3d_sequence.png),
  [XZ/YZ](folded_candidate_figures/projections_xz_yz.png),
  [base Z, pitch/roll and CAD COM](folded_candidate_figures/height_attitude_com.png),
  [feet/contact candidates](folded_candidate_figures/foot_trajectories_contacts.png).
- [Previous complete fixture sequence: 3D](prior_route_comparison/canonical_3d_sequence.png),
  [XZ/YZ](prior_route_comparison/projections_xz_yz.png),
  [base Z/pitch/roll/COM](prior_route_comparison/height_attitude_com.png),
  [feet and declared phases](prior_route_comparison/foot_trajectories_contacts.png).
- [Previous full trace with explicit XYZ](legacy_fixture_sequence_not_geometry_approved.jsonl):
  fixture acquisition → 150mm → WALK → STOP → intermediate restart, explicitly
  **not geometry-approved**. The folded family's traces stop at their failure;
  there is no fabricated continuation to 150mm.
- [Corrected candidate and frame details](rock_first_correction/bounded_family.json),
  [figure/source manifests](folded_candidate_figures/figures_manifest.json).

C0 hypotheses are available. C1/C2 probes are rejected. C3 belly-to-foot load
transfer and the bridge to C4 have not been established. C4 fixed-foot rise from
an independently supported compatible configuration remains reusable. C5
completion still requires coherent runtime feedback. Neither edge contact nor
nominal foot position establishes bearing force.

## Regressions and WALK

[Regression results](regression_results.json): fresh AdaptiveStand/WALK **11484
checks, 0 failures**, fresh Service/Coordinator/Executor/rearm **2422 checks,
0 failures**, safety boundaries **48 checks, 0 failures**, and fresh linked production Motion HTTP
**56 checks, 0 failures**. Reused unchanged-source
results: RESTORE 884; persistence 500; lifecycle 498; AP 131;
transport mode 31 semantic/2077 total; portal DOM 167; viewer 67 tests/6 files;
receipt verifier 5 tests/12 manifest mutations. Earlier full static/host/mutation
suite PASS is retained. Intentional mutation failures mean defects were detected.
The 286 historical oracles remain DEFERRED, not executed PASS.

A later aggregate stopped on missing jsdom because NODE_PATH was omitted; its
actual log is retained and is not described as a complete PASS. Existing
successful DOM/browser logs are available. Tests use explicit synthetic
observations/qualifications and fake hardware backends.

Simulation preserves 1425 feedback frames in the legacy fixture sequence,
488 WALK/coordinated STOP frames, 200 frames for 117→150mm and 200 for 153→150mm.
Nominal 150mm confirmation avoids movement. Feedback/TTL/watchdog/rearm/STOP
behavior passes software checks. WALK support/stability and loaded tracking
remain unvalidated. A different live nonzero velocity requests STOP and refuses
that command; continuous joystick direction changes remain a software gap.

## Dashboard, integration and release readiness

The six original tabs plus Motion, 48 visible actions, semantic joystick
vx/vy/yaw_rate and 17-mesh host twin remain functional. Capability admission is
Controller-owned: 7 catalog entries have software implementations; 41 are
planned/backend-disabled. The physical state disables 46 and admits STOP/status
only. [Existing browser results](../Motion_Enablement_2026-10-12/browser/browser_checks.json)
verify desktop/mobile controls, synthetic rendering and stale behavior.
No live device telemetry was accessed. Full viewer assets exceed APP/FFAT
capacity; the existing embedded portal links to host Three.js.

The path is Dashboard → Controller MotionService → Coordinator → Executor →
existing Safe Actuator Layer → existing ESP32-S3 ServoBus. Its host-wired contracts
are tested; production measured observation/segment-assessment, approved envelope,
local permit and prime/arm producers are still missing. Hardware GO alone cannot
close those software gaps. Normal RUN_STA lacks an authenticated Motion transport;
RUN recovery/no-profile AP can admit the service without opening maintenance
mutations. Authority is never granted by browser parameters.

The source snapshot, complete binary patch, build log/image/manifest and verification
receipt are external at
`/home/matteo-manicardi/MATDOG/verification-artifacts/MOTION_RC_20261012`.
The receipt is authoritative for the build result and binds all tracked/new source
bytes to ROBOT_POWERED, `MATDOG_16M_2x5M_NVS_V1`, image SHA256, MOTION_AUTHORIZED=0
and OTA_INGEST=0. The ordinary flash verifier must refuse TREE_NOT_CLEAN.
[Proposed commit/PR](../Motion_Enablement_2026-10-12/PROPOSED_PR.md) remains unpublished.
Review, commit and a clean rebuild with new manifest are prerequisites to any
later APP-ONLY authorization; no flash command is run here.

[Precise open gates](../Motion_Enablement_2026-10-12/motion_release_unsatisfied.json)
include measured physical producers, authenticated RUN_STA transport, approved
limits, identified natural starting pose and a collision-free C0→C4 route, loaded
tracking/bus timing, support/stability, live velocity transitions, real telemetry,
clean release provenance and separate operator GO with fixture/physical stop/local
assistance. Full Calibration 24/24 is not reopened.

**Resume at the RH rear-deployment constraint and missing measured initial pose,
not by rebuilding the Motion Core or exploring twelve DOF indiscriminately.**
The exact frame-24 witness and current geometry are the next review input; a
revised mechanically justified contact/body strategy must be separately qualified
before adoption. No autonomous prone acquisition is enabled in this delivery.

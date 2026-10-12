# Adaptive motion candidate — 12 October 2026

Software basis: `origin/main` at `fdd0434`. Installed closeout firmware remains
`b08322dedc5564c080f691dc98047511ef2b953f`; this implementation does not identify
that installed build with main, alter NVS/EEPROM, flash, enable torque, or move hardware.

The verified repository C4-C artifact is
`C4_rest_to_stand_trajectory/2026-07-08_190405_C4C_contact_locked_rest_to_stand_trajectory.json`,
SHA-256 `4b667a5995e55122b9578c39042914a373aa6bb4e0be84de11381e7f2398b7b2`.
It contains 51 offline contact-locked samples from **base_link** height 100 mm to 150 mm,
all marked safe under the historical C4 policy, with expected foot-fork review.
It does not contain a belly acquisition segment, hardware contact measurements,
or motion authority. Its canonical URDF SHA-256
`3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59`
and contact geometry SHA-256
`43dc72abdc132ef2492d383efe4ed12dfe2e18cf17705e69fc21f4166c4f0966`
still match the generated G1/G2 reference pins.

`AdaptiveStand::generate()` reuses `evaluateTimedStand()` for its fixed-contact
rise component, with the observed start height, requested target height, and
current footprint. The planar body transform is factored out. A higher standing
start reverses the component's clock. Thus 100 mm is a reusable C4 reference height,
not a startup requirement. Operational limits and actuator mapping are separate
runtime inputs and are never taken from the historical report.

The September 22 closeout at commit 58b97392dc13efa30e166fe3802efa677dcba011 declares a successful physical C4-C stand (CALIBRATION_BOOTSTRAP.md source lines 684–686). A timestamped twelve-joint initial/final capture and measured base height/contact trace are absent; see ../Motion_Recovery_2026-10-12/PRONE_EVIDENCE.md.
The repository artifacts searched here retain C4-A–F offline evidence and the old
C5 executor. That executor records a 9 July RF failure and is blocked; the 10 July
C5-R handoff says pre-recenter raw targets are not command eligible. These records
neither disprove the reported physical event nor establish compatibility of its
old tick mapping with the October installation. The raw encoder bridge and Station
queue executor are not reused. A reproducible physical trace of the reported
belly-to-stand event was not located among these tracked artifacts.

The preserved comparison simulation starts at the G35 `REST_GROUND` model pose, not an identified measured natural prone pose. Its extended upper/lower directed angles and 212 intersecting frames make the fixture sequence unapproved; see ../Motion_Recovery_2026-10-12/prior_geometry_reuse.json. The flat
canonical base collision mesh has minZ `-1.0999563076780766e-16m`, so belly contact
places base_link at approximately 0 m; this is geometry, not an IMU height reading.
The candidate explicitly requires a synthetic fixture witness before its first
target. The fixture lifts the rigid REST_GROUND pose to 65 mm while all feet remain
free. G2 then solves Cartesian unfolding, touchdown at rear ground anchors
(front X=-.050 m, hind X=-.2875 m, Y=±.094 m), and placement at canonical C4 anchors.
External support remains required during acquisition. Four-foot support must be
confirmed before releasing the fixture witness and using C4 to reach 150 mm.
The unfolding waypoint is generated from FK with upper angles 1.6 rad (front) and
1.8 rad (rear), retaining REST_GROUND hip/lower angles; intervening motion is
Cartesian G2 IK, never encoder interpolation.

The route is a **fixture-supported synthetic offline candidate**, never an approved
hardware path or a qualified autonomous belly-to-stand recovery. `externalSupportRequired`
is part of each relevant frame and `externalSupportConfirmed` is a separate observation.
Missing or revoked fixture support rejects before ACK or further targets. Trace
`reason` and `evidence` visibly include `EXTERNAL_FIXTURE_SUPPORT` during that phase.
Foot proximity/contact labels do not prove load, friction, COM transfer or stability.

The rejected ground-only route produced LF foot / LH hip collisions at sampled
frames 20, 25, 30 and 35; the initial full audit detected 20 colliding configurations
in its first 100 frames. Flat rear touchdown at base_link height 0 also conflicts
with mesh clearance: X=-.2847375 m gives lower-leg minimum Z=-3.088460234682011e-6 m
with hip/upper separated, whereas X=-.2846125 m gives minimum Z=2.3363488160788037e-7 m
but hip/upper intersects. These residues were not hidden with a larger tolerance.
G5-A tilted contact IK already exists; AdaptiveStand's current flat-G2 integration
does not include a verified rocking, support-transfer and loaded-feedback route.

The physical contact model uses radius 14.9 mm and center (0, 0, 14.9 mm). The canonical
collision foot STL exceeds this radial model by at most
`5.775042072383302e-6m`, measured over all collision vertices. The independent V5
trace audit records raw negative foot mesh minima and this derived numerical
bound. It does not pad body/upper/lower collisions or infer compliance, friction,
continuous collision clearance, or dynamic stability.

Run the pure simulation and independent geometry audit:

```sh
python3 05_Firmware/MATDOG_Controller/scripts/tests/run_adaptive_motion_tests.py \
  --trace 09_Logs/Validation_Reports/Motion_Enablement_2026-10-12/adaptive_motion_synthetic.jsonl
python3 05_Firmware/MATDOG_Controller/scripts/tests/audit_adaptive_motion_geometry.py \
  --trace 09_Logs/Validation_Reports/Motion_Enablement_2026-10-12/adaptive_motion_synthetic.jsonl \
  --output 09_Logs/Validation_Reports/Motion_Enablement_2026-10-12/adaptive_motion_unsatisfied.json \
  --stride 1 --workers 6
```

The JSONL trace is host data, not firmware storage. It supplies simultaneous actual
and target radians, world/base quaternion, support-plane base height, four foot
positions and states, timestamps, phase, tracking and fault fields. All frames
are explicitly tagged `SYNTHETIC_NOT_HARDWARE_QUALIFIED`. Export publishes atomically
only after the complete regression succeeds, preserving the previous file on failure.

The separate JSON report lists remaining qualification gates. No Full Calibration
is rerun. Record generation1 and the hardware-validated24/24 campaign are retained.

Physical WALK assessment additionally requires independently supplied `dynamicStabilityQualified`; synthetic or merely kinematic evidence cannot set `dynamicStabilityProven`. Same-velocity commands refresh the watch dog, and zero velocity requests coordinated stop. A changed nonzero velocity requires stop then restart; continuous velocity transitions remain an explicit software gate.

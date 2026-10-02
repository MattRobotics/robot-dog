# G4.1 foot-contact reconciliation (offline)

Baseline: G4 final `3ce85f92e2b5381b65e327dd371b2f164308f326`. This milestone changes **no accepted G1-G4 file**.
The additions are new tools under this directory, the new pure C++ unit `ContactMode.h/.cpp` (+ its test), and
new evidence under `09_Logs/Validation_Reports/G41_Contact_Reconciliation/`. Offline software only.

## Contact contract v2 (what G4.1 defines)

The G4 strict policy mixed four different things: the *reference* geometry of the tread, the *mesh* that
approximates it, the *schedule* that declares contact, and a *numerical band*. v2 separates them.

1. **Reference.** The foot tread contact is the G2 analytic cylinder (unchanged constants). The collision mesh is a lossy
   representation of it, registered 5.775 um away (root cause below). The reference is authoritative for the tread;
   the mesh is authoritative for everything else (links, non-tread foot parts, self-collision).
2. **Declared modes.** `ContactMode` = STANCE, SWING, LIFT_OFF, TOUCHDOWN, a pure function of the scheduler phase.
   LIFT_OFF and TOUCHDOWN are the exact schedule instants: foot velocity and acceleration are zero (C2 swing) and the
   foot is at z = 0. Geometric proximity never changes a mode, and a mode never excuses penetration.
3. **Tread.** Stance: analytic contact height 0 (IK exactness floor 1e-9 m). Swing: `z(u) = 64 h u^3 (1-u)^3 >= 0` in closed
   form, so interior swing contact is impossible on the analytic surface for any sampling density. Penetration of the
   analytic surface is a failure; there is no "intentional contact" reclassification.
4. **Mesh versus reference (tread).** The mesh-minus-reference height over every foot pitch is bounded in closed form by
   `R - max|v - c|` (u.w >= -|w_xz|). For the unchanged G2 reference this is -5.775 um: the declared registration
   uncertainty `U`, equal to the measured offset, not a tolerance chosen to pass. For a registered reference it is >= 0.
5. **Everything else.** Non-foot links, non-tread foot parts: mesh versus ground, penetration < -1 um or contact
   within the +/-1 um band is a failure (millimetre margins: never the limiting quantity). Non-adjacent mesh pairs:
   collision, solid containment and minimum separation, unchanged.
6. **Support.** WALK quasi-static support margin uses the hull of the G2 central support strip ends of the legs in stance
   at both neighbouring samples (one constant set per interval, so lift-off and touchdown are conservative).
   TROT static margin stays a diagnostic.
7. **Verification vocabulary.** CONTINUOUSLY_VERIFIED_ANALYTIC (closed form), CONTINUOUSLY_VERIFIED_BOUNDED (sample minus
   an interpolation-error bound with an *estimated* constant, x1.5 safety, confirmed by a 0.5 ms re-run), SAMPLED_ONLY,
   FAILED, UNRESOLVED. A sampled result is never reported as continuous.

Scope: pure fore/aft planar gaits (foot axis tilt 0), flat ground, level body.

## Root cause of the 5.775 um discrepancy

The collision STL of every foot, and independently the visual STL, contain the nominal 14.9 mm tread circle translated rigidly
by (4.534, 3.577) um in the foot_link x-z plane; the tread vertices lie on or inside that translated nominal circle (maximum radial
excess 0.0006 um, 327 vertices on it to 0.01 um). Both STLs carry the same translation to 0.004 um, so it is a property of the
CAD-to-foot_link registration, not of tessellation or numerics (float64 vs long double 3e-12 um, float32 STL 5e-7 um). The G2 YAML
audit used the visual STL and rejected a centroid offset because that depends on tessellation; the circle translation does not.
The mesh-minus-analytic height is therefore `u.d` (d the translation) plus a non-negative tessellation term.

## Alternatives (see `alternatives.json`, REPORT.md section 4)

A0 status quo, A1 registered reference (candidate G2.1), A2 analytic tread authority (policy only), A3 exact mesh support
function, A4 circumscribing cylinder, A5 widened tolerance (rejected). Selected: A2 applied in G4.1 on the unchanged G2, with
A1 evaluated as a prototype and recommended for approval. A1 would change the accepted G2 contract and its generated data, so it
is **not applied** here.

## Migration strategy if A1 is approved

1. New contract version `G2.1` beside `G2` (YAML `rigid_cylinder.center_in_foot_link_m` and radius from `root_cause.json`,
   provenance of the fit recorded). G2 stays loadable and byte-identical.
2. Regenerate `FootContactData.h`, the C4 stand export and goldens as `G2.1`; keep `G2` goldens as versioned evidence.
3. Expected size of the change (measured in `alternatives.json`): contact point shift <= 5.78 um, joint change <= 4.6e-5 rad, qdot and qddot
   relative change <= 3e-5, canonical stand joint change 4.5e-5 rad. G1 (foot_link FK) is unaffected.
4. Re-run G2/G3/G3.5/G4 regression gates; G3.5 pose and transition evidence is re-versioned, not rewritten.

## Tools

`geometry_provenance.py`, `root_cause.py`, `contact_model.py` (vectorised G2 contact for any reference), `alternatives.py`,
`lifecycle_v2.py` (+ `run_lifecycles.sh`, `run_convergence.sh`), `test_contact_audit.py`, `validate.py`,
`artifact_manifest.py`, `build_report.py`. Environment: the pinned G3.5 virtualenv. `select`-style imports follow the G4 rule:
`gait_audit/survey.py` before anything that prepends `pose_audit/`.

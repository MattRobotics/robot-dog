# G4.1 — contact contract decision (offline motion)

## Status

- Worktree: `/home/matteo-manicardi/MATDOG/worktrees/robot-dog-gait-engine`, branch `feat/gait-engine-offline-v1`.
- Accepted checkpoint: `2f38670a719024e8698cefbecf347291268df0c6` (G4.1 closed as an offline milestone).
- Evidence: [`G41_Contact_Reconciliation/REPORT.md`](../Validation_Reports/G41_Contact_Reconciliation/REPORT.md);
  G4 final evidence is [`G4_Gait_Envelope/REPORT.md`](../Validation_Reports/G4_Gait_Envelope/REPORT.md).

## Decision

| Item | Decision |
|---|---|
| **A2 / Contact Contract v2** (analytic G2 tread authority, schedule-declared contact modes, declared registration uncertainty) | **Accepted for offline development**, within its validated domain: pure fore/aft planar gaits, foot axis tilt 0, flat ground, level body |
| **A1 / G2.1** (re-register the G2 reference to the collision mesh) | **Deferred. Not implemented.** Prototype evidence stays in G4.1 for a future decision |
| **G2** contact reference, YAML, `FootContactData.h`, C4 stand | **Unchanged** |
| **G4.1** | **Closed**, with all historical failures and limitations preserved |
| **Hardware locomotion** | **Not authorized** |

The 5.775 um offset between the G2 analytic tread and the collision mesh is not a development blocker. It is negligible
against the mechanical uncertainty to be expected from the assembled robot (the URDF foot joint origin is specified to
0.1 mm; a real rubber tread, its run-out and compliance are unmeasured). Numerical precision of the model must not be
confused with mechanical accuracy of the machine.

## Governing rule for offline lifecycle assessments

- The accepted A2 contract governs offline STAND -> gait -> STAND assessments from here on.
- The historical 1 um mesh-versus-ground policy (G3.5 policy v1, G4 strict rule) **remains documented**. It still
  rejects the G4 lifecycles on the unchanged G2 (worst -1.262 um); that rejection is preserved evidence, not erased.
- No tolerance was widened and no geometric error is concealed: the mesh/analytic difference is bounded in closed
  form and declared (`U` = 5.775 um), penetration of the analytic surface remains a failure, and non-foot links keep
  the strict 1 um mesh test.
- The contact investigation is not reopened unless new evidence shows a materially relevant mechanical or
  geometric problem.

## Limits that stay in force

- TROT dynamic stability is NOT YET PROVEN; all gait parameters are study values, not hardware settings.
- Lateral and yaw motion (tilted contact axis) are outside the validated domain.
- Verified continuity statements that rest on estimated interpolation constants are labelled as such in the report.
- Calibration (q0, persistence), actuator limits and a stand authorisation remain separate dependencies.

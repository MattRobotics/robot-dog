# MATDOG G4 → G7 same-new-geometry oracle

status: **PASS**
G4 content SHA256: `4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61`
endpoints: 24
geometric contacts: 24
path obstructions: 6
obstructions preceding contact: 2

| Endpoint | G4 contact (deg) | G7 contact (deg) | abs delta (rad) | Path relation | Result |
|---|---:|---:|---:|---|---|
| lf_hip_joint:min | -46.011719 | -46.011719 | 2.720e-12 | body_vs_branch | PASS |
| lf_hip_joint:max | +45.222656 | +45.222656 | 2.657e-12 | - | PASS |
| lf_upper_leg_joint:min | -52.132813 | -52.132813 | 2.814e-12 | - | PASS |
| lf_upper_leg_joint:max | +121.875000 | +121.875000 | 6.998e-12 | - | PASS |
| lf_lower_leg_joint:min | -92.074219 | -92.074219 | 5.218e-12 | same_branch | PASS |
| lf_lower_leg_joint:max | +38.179688 | +38.179688 | 1.688e-12 | - | PASS |
| rf_hip_joint:min | -45.222656 | -45.222656 | 2.657e-12 | - | PASS |
| rf_hip_joint:max | +46.011719 | +46.011719 | 2.720e-12 | body_vs_branch | PASS |
| rf_upper_leg_joint:min | -52.132813 | -52.132813 | 2.814e-12 | - | PASS |
| rf_upper_leg_joint:max | +121.875000 | +121.875000 | 6.998e-12 | - | PASS |
| rf_lower_leg_joint:min | -92.074219 | -92.074219 | 5.218e-12 | same_branch | PASS |
| rf_lower_leg_joint:max | +38.179688 | +38.179688 | 1.688e-12 | - | PASS |
| rh_hip_joint:min | -45.156250 | -45.156250 | 2.251e-12 | - | PASS |
| rh_hip_joint:max | +46.011719 | +46.011719 | 2.720e-12 | - | PASS |
| rh_upper_leg_joint:min | -52.132813 | -52.132813 | 2.814e-12 | - | PASS |
| rh_upper_leg_joint:max | +121.875000 | +121.875000 | 6.998e-12 | - | PASS |
| rh_lower_leg_joint:min | -92.074219 | -92.074219 | 5.218e-12 | same_branch | PASS |
| rh_lower_leg_joint:max | +38.179688 | +38.179688 | 1.688e-12 | - | PASS |
| lh_hip_joint:min | -46.011719 | -46.011719 | 2.720e-12 | - | PASS |
| lh_hip_joint:max | +45.156250 | +45.156250 | 2.251e-12 | - | PASS |
| lh_upper_leg_joint:min | -52.132813 | -52.132813 | 2.814e-12 | - | PASS |
| lh_upper_leg_joint:max | +121.875000 | +121.875000 | 6.998e-12 | - | PASS |
| lh_lower_leg_joint:min | -92.074219 | -92.074219 | 5.218e-12 | same_branch | PASS |
| lh_lower_leg_joint:max | +38.179688 | +38.179688 | 1.688e-12 | - | PASS |

Expected semantic separation:

- hardware evidence and hardware verdicts are absent from V5 pure geometry
- legacy MODEL_INCOMPLETE records remain ordinary geometric contacts
- legacy PATH_COLLISION_BEFORE_ENDPOINT is represented as contact plus independent obstruction
- PATH_OBSTRUCTION relation is topology-derived and is not automatically cross-leg

No hardware evidence or safety policy was used to define V5 geometry.

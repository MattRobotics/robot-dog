# MATDOG G4 → V5 frozen-context replay oracle (non-canonical)

status: **PASS**
artifact role: **NONCANONICAL_G4_REPLAY_ORACLE**
canonical profile eligible: **false**
G4 content SHA256: `4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61`
endpoints: 24
geometric contacts: 24
no geometric contact: 0
path obstructions: 6
obstructions preceding contact: 2
tight replay max contact delta (diagnostic only): 6.998e-12 rad
tight replay max path delta (diagnostic only): 5.749e-12 rad
canonical use: **NON_CANONICAL_REPLAY_EVIDENCE_ONLY**

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

Comparison coverage:

- endpoint identity (legacy presentation ID, joint, side, replay ID)
- ordered active revolute parent-child pair
- contact status, angle, unordered contact pair, clear/contact bracket
- declared limit and declared-limit delta
- G4 reported analysis envelope and G7 q=0-to-far-bound search domain
- contact coarse step, bisection resolution, maximum and actual iterations
- exact G4 replay context at task, geometric search, and path search boundaries
- path status, angle, unordered pair, reconstructed clear/contact bracket
- path search domain, coarse step, bisection resolution, maximum and reconstructed iterations
- path-before-contact classification

Exact representation exceptions:

- G4 numerical_search.analysis_envelope_rad is a symmetric reporting envelope around the declared limit, while both algorithms actually sweep from q=0 to the directional far bound; the oracle validates the G4 envelope and compares that far bound to the complete G7 search domain.
- G4 does not serialize a path relation; G7 relation presence/absence is validated but equality is not claimed.
- G4 does not directly serialize the path clear-side bracket or path bisection count; for all six frozen interior full-coarse-step events they are deterministically reconstructed from side, path contact bound, coarse step, resolution, and maximum iterations before comparison.
- For a hypothetical G4 no-contact result, G4 has no contact result pair whereas V5 retains the active pair that was searched; the ordered active pair is compared instead.  Frozen G4 contains 24 contacts.

Expected semantic separation:

- hardware evidence and hardware verdicts are absent from V5 pure geometry
- legacy MODEL_INCOMPLETE records remain ordinary geometric contacts
- legacy PATH_COLLISION_BEFORE_ENDPOINT is represented as contact plus independent obstruction
- PATH_OBSTRUCTION relation is topology-derived and is not automatically cross-leg

No hardware evidence or safety policy was used to define V5 geometry.

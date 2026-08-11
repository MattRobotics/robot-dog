# MATDOG LF V25 — separate Hardware Reconciliation

schema: `matdog.geometry_hardware_reconciliation.v1`
geometry semantic SHA256: `e99e2b65ea8d032f94b5d1aa815432a292c1771766e556fc7115dc7a1f5de73e`
hardware evidence SHA256: `6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4`
reconciler source SHA256: `111da4045c983c4f3a5acd0061dbbe3c7b77fbda7e120693a927f5f0a40f5dfe`
agreement threshold: 2.000 deg

evidence endpoints: 6
geometry-only endpoints: 18
AGREES: 3
DISAGREES: 3
NO_GEOMETRIC_CONTACT: 0

| Evidence | Joint | Side | Geometry deg | Hardware deg | Delta deg | Result | Path context |
|---|---|---|---:|---:|---:|---|---|
| lf_hip_min | lf_hip_joint | min | -46.012 | -42.803 | -3.209 | DISAGREES | body_vs_branch @ -45.297 deg; precedes=True |
| lf_hip_max | lf_hip_joint | max | +45.223 | +39.375 | +5.848 | DISAGREES | - |
| lf_upper_leg_min | lf_upper_leg_joint | min | -52.133 | -53.525 | +1.393 | AGREES | - |
| lf_upper_leg_max | lf_upper_leg_joint | max | +121.875 | +122.607 | -0.732 | AGREES | - |
| lf_lower_leg_min | lf_lower_leg_joint | min | -92.074 | -91.846 | -0.229 | AGREES | same_branch @ -97.969 deg; precedes=False |
| lf_lower_leg_max | lf_lower_leg_joint | max | +38.180 | +34.277 | +3.902 | DISAGREES | - |

No geometry was changed. RF/RH/LH remain geometry-only predictions.
No hardware was accessed during this offline reconciliation.

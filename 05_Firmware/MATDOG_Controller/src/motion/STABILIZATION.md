# G5-A body stabilization core (offline, no hardware)

Pure C++17, fixed storage, no heap/exceptions/RTTI, no device, clock or framework dependency, no actuator authority.
It reuses G1-G4.1 unchanged and adds five units beside them. **No gain, limit or period in this milestone is an approved hardware setting.**

```text
BNO085 driver (existing, hardware)      acquisition: fills an ImuSnapshot, stamps it with the caller's clock
        |  ImuSnapshot {R_world_from_base quaternion, stampS, sequence, accuracyRad, accuracyStatus}
        v
AttitudeMonitor   (ImuAttitude.h)       validity, freshness, ordering -> Attitude {roll, pitch, up-vector}   [pure]
        |  Attitude, AttitudeStatus, isNewSample
        v
BodyStabilizer    (BodyStabilizer.h)    bounded, rate-limited, fail-safe integral law -> (roll, pitch) correction   [pure]
        |  corrected BodyPose rotation (translation and yaw unchanged)
        v
compensateStand   (TiltCompensation.h)  world-locked G2 contacts -> Cartesian targets R^T (contact - t)
        |  worldContactInverseKinematicsTilted   (TiltedContactIk.h)
        v
12 semantic URDF joint targets          -> (future, separate) actuator safety layer -> servos
```

## BNO085 data contract

- Source: the existing `Bno085Imu` driver, `SH2_ROTATION_VECTOR` only (internal rate 50 Hz, G3.1: 50.1 Hz). The driver keeps `rv_w_..rv_z_`,
  `rv_accuracy_` (rad), `rv_status_` (0..3) and `rv_count_`. **A thin adapter (not written here, no second driver) maps them to `ImuSnapshot`:**
  quaternion as is, `sequence = rv_count_`, `accuracyRad`, `accuracyStatus = rv_status_`, and `stampS` taken from the caller's clock when the
  sample is consumed. The driver and the sensor are untouched in G5-A.
- Frame: MATDOG X forward, Y left, Z up. The sensor axes coincide with `base_link` (frozen by hardware Phase D: identity mapping, no sign
  flip, see the viewer README). The quaternion is `R_world_from_base_link`. **Heading is magnetically referenced with an arbitrary zero and
  is never consumed**; only tilt from the measured up-vector is used (`g = R^T e_z`).
- Conventions (checked against the URDF geometry itself): `roll = atan2(g_y, g_z)`, +roll lifts the left side; `pitch = asin(-g_x)`, +pitch
  lowers the nose; `R = Rz(yaw) Ry(pitch) Rx(roll)`. `q` and `-q` give identical tilt.
- Validity: finite values; unit norm within `normTolerance`; `accuracyStatus >= minAccuracyStatus`; `accuracyRad <= maxAccuracyRad`; age
  `now - stamp <= maxAgeS` (equal is fresh, greater is stale); no future stamp; no caller-time or sample-time regression; no sequence
  regression; `|roll|,|pitch| <= maxPlausibleTilt` (beyond that the sample is a fault, not a correctable tilt). `AttitudePolicy` has **no
  default**: every field must be supplied.
- **Not established (hardware measurements outstanding):** the IMU mounting-level offset (what the sensor reads on a truly level robot), the
  acquisition-to-command latency, the real sample jitter, DCD-calibrated accuracy under motion/vibration.

## BodyPose correction mathematics

Desired body orientation `R = Rz(yaw_nominal) Ry(p_c) Rx(r_c)`, translation unchanged, flat ground. World contacts `c_i` stay fixed
(world-locked); the Cartesian foot targets in `base_link` are `b_i = R^T (c_i - t)` and the ground normal in `base_link` is `n = R^T e_z`.
Joints come only from contact IK against `n`; **no joint angle is ever offset directly.**

Law (plant from commanded tilt to measured tilt is an identity plus a disturbance for position-controlled legs):

```text
e[k]   = deadband(m[k] - m_level)                       m: measured tilt, m_level: calibrated level reference
c[k+1] = rate_limit( clamp( c[k] - ki * dt * e[k], +-cmax ), +-rmax * dt )      (only for a NEW valid sample)
```

The configuration is accepted only if `ki < maxStableIntegralGain(dt, N) = 2 sin(pi / (2(2N+1))) / dt`, the exact stability boundary of the
integral loop with `N` steps of delay (characteristic polynomial `z^(N+1) - z^N + ki dt`). `N` must come from a measurement; a larger delay
lowers the bound. This is a derived constraint, not a tuning value.

## Fail-safe contract

| Condition | Behaviour |
|---|---|
| valid, new, in-time sample | integrate, bounded in range and rate |
| same sample again | hold (never integrated twice) |
| stale / invalid / regressed / late step / NaN | no new correction; command ramps to zero at the bounded rate; state `RAMPING_DOWN` |
| no valid sample for `holdBeforeFaultS` | `FAULT` latched; keeps ramping to zero (never a step); `enable` refused until `resetFault` after the ramp |
| `disable()` | correction removed at the bounded rate, then `DISABLED` |
| bad configuration (gain at/above the bound, zero or non-finite fields) | `configure` returns false; nothing runs |

The stabilizer holds no actuator limit and sends nothing. The actuator safety layer is separate and later; `ActuatorEnvelope.h` only
**classifies** a requirement against a limit that carries provenance (`UNMEASURED`, `VENDOR_NOMINAL`, `BENCH_NO_LOAD`,
`HARDWARE_LOADED_VERIFIED`) and has no default limits.

## IK limitation and the extension (G2 contract unchanged)

`contactInverseKinematics` / `worldContactInverseKinematics` / `supportsFlatContactIk` are exact for a ground normal `+Z` in `base_link`
(level body) and are **not modified**. A tilted body needs a different normal, so `TiltedContactIk` is a new, explicit solver: bounded Newton
on `contactForwardKinematics` (which already takes a ground normal and holds the one G2 cylinder, strip and 2 deg nominal-tilt policy),
started from the previous solution (branch selection), no clipping, atomic over four legs. For `n = +Z` it agrees with the analytic IK
(5e-9 rad); against an independent URDF/scipy solver it agrees to 5e-14 m and 2e-11 rad.

## Regression requirements for any future change here

`test_stabilization.cpp` (C++, strict flags, ASan/UBSan), the independent oracle `independent_checks.py`, the loop simulations, the G1-G4.1 gates
and the static audit. Changing the contact reference (G2.1) would require regenerating the tilt evidence.

# MATDOG_C018_V1 — canonical ST-3215-C018 persistent profile

**Status:** FROZEN — 2026-08-27
**Applies to:** Feetech **ST-3215-C018** serial-bus servos used by MATDOG
**Machine-readable companion:** [`06_Software/Matdog_Core/config/MATDOG_ST3215_C018_V1.yaml`](../../06_Software/Matdog_Core/config/MATDOG_ST3215_C018_V1.yaml)
**Applied and verified on:** 17/17 units — see [ST3215 provisioning campaign](../../09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/README.md)

`MATDOG_C018_V1` is the persistent EEPROM operating profile MATDOG expects every ST-3215-C018
servo to hold before it is installed in the robot. It is a **state contract**, not a tuning
recipe: a unit either matches the profile exactly or it is not considered provisioned.

---

## Provenance and authorship of the numbers

These values are **MATDOG engineering choices** for this robot, selected and verified on MATDOG
hardware. They are *not* a Feetech-prescribed configuration.

> Feetech is not the author of this profile. Nothing in this document should be read as a vendor
> specification, a vendor recommendation, or a vendor-endorsed configuration. Where a value
> coincides with a factory default it is because MATDOG independently selected the same value.

Register addresses and widths are read from the C018 unit itself and from the frozen MATDOG
tooling; they are not taken from a third-party memory map. See
[Model register caveat](#model-register-caveat).

---

## ST3215 invariants that constrain this profile

| Invariant | Value |
|---|---|
| Servo variant | Feetech ST-3215-**C018** |
| Authoritative model word | register **`0x03..0x04`**, expected **`777`** |
| `GoalPosition` domain | unsigned **`0..4095`** — signed wrap forbidden |
| Physical RAW centre | **2048** |
| Final `PositionOffset` | **0** on all units |

**Forbidden operations — permanently:**

- `CalibrationOfs`
- one-key-middle / one-key centering
- factory reset
- broadcast write

### Model register caveat

The C018 exposes its model word at **`0x03`**. NormaCore's
`software/drivers/st3215/src/protocol/memory.rs` declares `ModelNumber => (0x00, 2, …)`. For the
C018 that declaration is a **source-map defect** and must not be used for identity gating. This
defect is still open upstream and is tracked as a dependency risk, not fixed here — see the
[handoff](../../09_Logs/Development_Log/2026-08-27_ST3215_CANONICAL_ARCHIVE_HANDOFF.md).

---

## Persistent target — the 20 canonical registers

Every one of these was verified identical on all 17 provisioned units after a true cold power
cycle.

| Register | Addr | Width | Target |
|---|---|---:|---:|
| MinAngle | `0x09` | 2 | 0 |
| MaxAngle | `0x0B` | 2 | 4095 |
| MaxTemperature | `0x0D` | 1 | 70 |
| MaxVoltage | `0x0E` | 1 | 140 |
| MinVoltage | `0x0F` | 1 | 40 |
| MaxTorque | `0x10` | 2 | 1000 |
| P | `0x15` | 1 | 32 |
| D | `0x16` | 1 | 32 |
| I | `0x17` | 1 | 0 |
| MinStartupForce | `0x18` | 2 | 16 |
| CWDead | `0x1A` | 1 | 1 |
| CCWDead | `0x1B` | 1 | 1 |
| ProtectionCurrent | `0x1C` | 2 | 310 |
| Mode | `0x21` | 1 | 0 |
| ProtectionTorque | `0x22` | 1 | 20 |
| ProtectionTime | `0x23` | 1 | 200 |
| OverloadTorque | `0x24` | 1 | 80 |
| SpeedClosedLoopP | `0x25` | 1 | 10 |
| OverCurrentProtectionTime | `0x26` | 1 | 200 |
| VelocityClosedLoopI | `0x27` | 1 | 200 |

Plus the two positional invariants, which are **not** part of the 20-register delta set:

| Property | Target | Notes |
|---|---:|---|
| `PositionOffset` (`0x1F`, int16 LE two's complement) | **0** | EEPROM; written by the provisioner |
| physical RAW centre | **2048** | mechanical, accepted at ±1 |

### Profile fingerprint

The 20 registers hash to a single canonical fingerprint, identical on all 17 units:

```text
ae3ae5ce6ddda1c003fcde9f8ce12fed7f1272639b183e1ba23e27eae0cc5b4d
```

A unit whose fingerprint differs is **not** running `MATDOG_C018_V1`.

---

## Baud — verification only in V1

| Register | Addr | V1 policy |
|---|---|---|
| BaudRate | `0x06` | **verification only** — expected `0` (1 Mbps) |

V1 tooling **reads and asserts** the baud register. It does **not** write it. Baud remains
outside the writable profile surface in V1.

---

## Preserve-only registers

These are read and recorded but **never written** by MATDOG tooling. The values below are what
the 17 units happened to hold — they are *observations*, not targets. A different value is not a
profile violation.

| Register | Addr | Observed on all 17 |
|---|---|---:|
| ReturnDelay | `0x07` | 0 |
| ResponseStatus | `0x08` | 1 |
| UnloadCondition | `0x12` | 12 |
| LedAlarm | `0x13` | 44 |
| AngularResolution | `0x1E` | 1 |

---

## Runtime policy — separate from the persistent profile

**`TorqueLimit` is not part of the persistent operating profile.** It is a RAM/runtime value
(`0x30`) and is deliberately excluded from the EEPROM profile contract.

### Provisioning centering constants — validated

These were exercised on real hardware during centering on all 17 units:

| Constant | Value |
|---|---:|
| `TorqueLimit` during centering | 300 |
| `GoalSpeed` | 365 |
| `Acc` | 50 |

### Post-cold observation

`TorqueLimit` read back as **1000** after the cold cycle on all 17 units. The tooling records
this as `OBSERVED_NOT_ASSERTED` — it is evidence of the power-on default, not a written target.

### Not yet determined

Gait, head and jaw **runtime** torque/speed limits are **TBD**. They must be derived after
reassembly and full recalibration, and they do not belong in this profile.

---

## Characterization constants

Selected from the NEW01 characterization campaign and compiled into the frozen provisioner:

| Constant | Value |
|---|---:|
| `PRIME_MAX_DELTA_TICKS` | 1 |
| `COLD_ABSENCE_DEBOUNCE_MS` | 1500 |
| `COLD_RETURN_STABLE_SAMPLES` | 10 |

Each provisioning session logs these as `SELECTED_CONSTANT … SOURCE=NEW01_CHARACTERIZATION`.

---

## What is *not* hardware-validated

> **The canonical-profile EEPROM write path has never been exercised on real hardware.**

Across the entire final campaign — all 22 sessions, including retries — the profile delta was:

```text
DELTA_COUNT WRITE=0 SKIP=20
```

All 17 units already held the canonical values, so the provisioner correctly skipped every one of
the 20 registers. **440 SKIP, 0 WRITE.**

This means the profile-register write branch is **implemented and statically audited, but never
physically executed**. It must not be described as hardware-validated.

The EEPROM write *mechanism* itself **was** exercised — unlock → write → readback → relock — but
only via `PositionOffset` (`0x1F`, 17 writes) and the `Lock` register (`0x37`, 62 operations).

Full detail: [provisioning campaign report](../../09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/README.md).

---

## Related documents

- [ST3215 provisioning campaign 17/17](../../09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/README.md)
- [Bench QC / Servo Quality Audit V6.1](../../09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md)
- [Frozen bench tools](../../05_Firmware/ST3215_Bench_Tools/README.md)
- [Calibration reset — 2026-08-27](../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)

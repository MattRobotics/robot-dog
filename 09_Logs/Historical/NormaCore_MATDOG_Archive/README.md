# NormaCore MATDOG archive — the Station-mediated development phase

> **This directory is HISTORICAL. Nothing here is current MATDOG truth or current runtime.**
> Current architecture: [`01_Docs/02_Architecture/ARCHITECTURE.md`](../../../01_Docs/02_Architecture/ARCHITECTURE.md)

For roughly two months MATDOG's native calibration development lived in
`MattRobotics/norma-core`, because NormaCore Station had been designated the sole owner of the
ST3215 serial bus. **That premise is superseded.** This archive preserves the engineering content
from that phase and records how the transition happened.

As of 2026-08-27, **`MattRobotics/robot-dog` is the single active MATDOG repository.**

---

## How MATDOG got here

### PHASE A — Station designated sole ST3215 bus owner

NormaCore Station was adopted as the only component permitted to own the ST3215 serial port.
Every MATDOG hardware operation had to be mediated through it. This drove the next four phases.

### PHASE B — native MATDOG calibrator developed inside `norma-core`

Because Station owned the bus, MATDOG calibration code had to live where Station lived. The native
calibrator was built as `software/drivers/st3215/src/auto_calibrate/matdog.rs` plus host tooling
under `tools/matdog/`. Merged into `norma-core` `main` via PRs #1, #2, #3.

→ [`NormaCore_Main_MATDOG_Content/`](NormaCore_Main_MATDOG_Content/README.md)

### PHASE C — LF V25 hardware validated and frozen

On 2026-08-04 the left-front leg completed a full six-contact calibration: 58/58 sequence, URDF
affine gate PASS, supervised hardware-witness gate PASS, transactional EEPROM freeze PASS. It
remains the **only** MATDOG full-leg calibration ever hardware-validated.

→ [`LF_V25_Hardware_Oracle/`](LF_V25_Hardware_Oracle/README.md)

### PHASE D — RF and generic full-leg work attempted

Two attempts to generalize LF V25 to the remaining legs:

- **RF-specific** — explored, never committed, superseded. → [`RF_Calibrator_Local_Only/`](RF_Calibrator_Local_Only/README.md)
- **Generic four-leg engine** — G0–G3C gates completed offline, never run on hardware, unfinished. → [`Generic_V25_Superseded_WIP/`](Generic_V25_Superseded_WIP/README.md)

Neither reached hardware validation.

### PHASE E — architecture changed

Direct, native ST3215 control from a dedicated **ESP32-S3** was demonstrated in practice — the
bench QC, source-signature survey and provisioner stack drove real servos with no Station in the
loop, culminating in **17/17 units provisioned** on 2026-08-27.

That removed the constraint that created Phases A–D. Station is no longer required to own the bus.

→ [Bench QC V6.1](../../Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md) ·
[Provisioning 17/17](../../Validation_Reports/ST3215_Provisioning_2026-08-27/README.md)

### PHASE F — MATDOG ownership consolidated into `robot-dog`

With Station no longer the actuation backend, splitting MATDOG across two repositories had no
remaining justification.

```text
MattRobotics/robot-dog   →  sole active MATDOG engineering repository
MattRobotics/norma-core  →  reference/upstream fork; main retained; no active MATDOG development
```

NormaCore/Station becomes historical, reference and optional tooling rather than the required
actuation backend. **This is not a ban on reusing good upstream code** — it is a statement that
MATDOG-specific ownership and development no longer live there.

---

## Archive contents

| Directory | What | Classification |
|---|---|---|
| [`LF_V25_Hardware_Oracle/`](LF_V25_Hardware_Oracle/README.md) | branch `release/matdog-lf-calibrator-v25` @ `f87dd1fb…`, 92 unique commits, 16 files | **HISTORICAL HARDWARE ORACLE** |
| [`Generic_V25_Superseded_WIP/`](Generic_V25_Superseded_WIP/README.md) | branch `matdog/generic-v25-full-leg-engine` @ `f4a87a44…`, 16 unique commits, 17 files | **SUPERSEDED WIP — never hardware validated** |
| [`RF_Calibrator_Local_Only/`](RF_Calibrator_Local_Only/README.md) | local-only uncommitted work, +1 176 lines | **SUPERSEDED WIP — never committed or pushed** |
| [`NormaCore_Main_MATDOG_Content/`](NormaCore_Main_MATDOG_Content/README.md) | MATDOG files merged into `norma-core` `main` | **HISTORICAL — main retained upstream** |
| [`bundles/`](bundles/) | git bundles carrying the unique commits of both retired branches | provenance |
| [`MIGRATION_REPORT.md`](MIGRATION_REPORT.md) | the retirement itself: gates, SHAs, verification, final repo state | record |

Integrity: each directory carries a `SHA256SUMS`; verify with `sha256sum -c SHA256SUMS`.

## Verified state at archive time

Both branch heads were checked against the **live remote** with `git ls-remote`, not taken from a
summary:

```text
release/matdog-lf-calibrator-v25    f87dd1fbc7e8100d275c74f9af448642f3429680   ✓ matched
matdog/generic-v25-full-leg-engine  f4a87a443011f15e4f105858fec62fc15ca5a77c   ✓ matched
main                                4a8ed6337261553b79c928975808d294c9ca723b
```

`MattRobotics/norma-core` had **no open pull requests**, and neither retired branch was the head of
any pull request.

---

## ⚠️ Reading rules

1. **Nothing here is current runtime.** All of it assumes Station owns the ST3215 bus.
2. **Nothing here may command hardware.** All robot calibration was reset on 2026-08-27.
3. **Do not read historical records by bus ID.** 14 of 17 servos were recoded — M11 is now ID 52
   (`NECK_PITCH`), not LF lower. Use
   [`MATDOG_SERVO_ALLOCATION.yaml`](../../../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml).
4. **Do not promote values from here.** Old q0, `PositionOffset` and span values describe a
   physical installation that no longer exists.
5. **Algorithms and decisions remain valuable.** That is why this is archived rather than deleted.

## Related

- [Canonical architecture](../../../01_Docs/02_Architecture/ARCHITECTURE.md)
- [Calibration reset](../../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)
- [ST3215 evidence index](../../ST3215_EVIDENCE_INDEX.md)
- [Historical index](../README.md)

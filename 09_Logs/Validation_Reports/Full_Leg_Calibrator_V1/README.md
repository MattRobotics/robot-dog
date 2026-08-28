# Full Leg Calibrator V1 — validation evidence

**Date:** 2026-08-28 (updated after H3–H6 implementation)
**Base commit:** `fcc1dbd1367d5d8c860a873c62a31b2620695aba`
**Tool:** [MATDOG Full Leg Calibrator V1](../../../05_Firmware/Full_Leg_Calibrator_V1/README.md)

---

## ⚠️ Hardware validation scope

> **Only stage H0 was exercised, and against the previous firmware image.**
> No servo was connected. No servo power was present. No motion was commanded.
> No EEPROM was written. **Nothing about H1 or later is validated by this work.**
> The current image, which adds H3–H6, has been compiled and offline-tested but
> **never flashed**.

| Stage | Meaning | Status |
|---|---|---|
| H0 | ESP32-S3 only, no servos attached | ✅ **PASSED 2026-08-28** |
| H1 | 12-servo read-only census | ⬜ not attempted — requires user present |
| H2 | manual-pose q0 capture, torque OFF | ⬜ not attempted |
| H3 | one joint runtime/contact characterization | ⬜ implemented, **not executed** |
| H4 | one joint full calibration | ⬜ implemented, **not executed** |
| H5 | one complete leg | ⬜ implemented, **not executed** |
| H6 | four legs sequentially | ⬜ implemented, **not executed** |
| H7 | separate freeze/promotion gate | ⬜ not part of the calibrator |

H3–H6 are now **real code paths**, offline-validated end to end. They ship
disabled: the default build is H0 and the bootstrap envelope is denied.

Motion is protected by **two independent gates** — the build stage
(`FLC_AUTHORIZED_STAGE`, default H0) and the pre-motion parameter gate
(resolved values, or an explicitly approved bootstrap envelope requiring both a
build flag and a live session confirmation). Raising the stage alone does not
unlock motion; a test asserts this at H7.

See the [hardware validation handoff](MATDOG_FULL_LEG_CALIBRATOR_V1_HARDWARE_VALIDATION_HANDOFF.md)
for the procedure and the parameter classification that resolved the old H3
deadlock.

---

## Offline validation — 206 tests, all passing

| Suite | Tests | What it proves |
|---|---|---|
| `test_matdog_full_leg_calibrator_engine.py` | 35 | **H3/H4/H5/H6 happy paths and faults through the real C++ engine** |
| `test_matdog_full_leg_calibrator_sim.py` | 42 | census, q0 capture, stage/bootstrap/characterization gates, SAFE_OFF |
| `test_matdog_full_leg_calibrator_policy.py` | 39 | provenance gating, parameter classification, bootstrap policy, joint specs |
| `test_matdog_full_leg_calibrator_detector.py` | 32 | per-sample contact decision via the real C++ detector |
| `test_matdog_full_leg_calibrator_firmware_sync.py` | 31 | static firmware audit, firmware/host constant sync, frozen-evidence integrity |
| `test_matdog_full_leg_calibrator_derive.py` | 27 | q0 derivation, affine solve, cross-check |

Full calibration suite after the change: **526 passed**, no regressions.

### Both halves are tested

The earlier suite could only show that `CALIBRATE_*` **refused**. Now that the
modes can genuinely succeed, the refusals mean something. The suite contains:

- **successful fully-gated execution** — H3 characterizes a joint (direction,
  baseline, contact, retreat, second contact, measured spread); H4 measures both
  endpoints and derives a q0 candidate; H5 calibrates three joints; H6
  calibrates twelve across four legs — all reaching a real success state;
- **refusals** — every gate refusing when its precondition is missing;
- **fail-closed faults** — with `EEPROM writes == 0`, `broadcast writes == 0`
  and `all torque OFF` asserted after every single one.

### The engine tests exercise real firmware code

`flc_contact_detector.h` and `flc_calibration_engine.h` are compiled into both
the ESP32-S3 firmware and host harnesses (`-Wall -Wextra -Werror`), driven
against a simulated ST3215 servo with a real mechanical endstop. There is no
Python re-implementation of the motion decision path.

**Three real defects were found and fixed this way.**

1. **Truncating modulo in the tick math** — caught by the detector harness on its
   first run. C++ `%` truncates toward zero, so `flcSignedTickDelta(0, 4095)`
   returned **−4095** where canonical `matdog_joint_math.signed_tick_delta`
   returns **+1**. Every wrap-boundary decision would have been wrong. Fixed with
   an explicit Euclidean `flcMod()`.

2. **Linear mean in the manual-q0 summary** — caught by self-review. A joint
   resting near the 4095/0 boundary would have reported a centre of ~2047 and a
   spread of ~4095. Moved into the shared header as `flcSummarizeTicks()`, which
   accumulates signed deltas.

3. **Joint left jammed against the endstop** — caught by the first H4 happy-path
   test. `flcMeasureEndpoint` returned immediately after the second approach,
   leaving the joint loaded against the mechanical stop and giving the next
   endpoint a start position that did not match reality; the MAX traversal then
   aborted with `WRONG_DIRECTION`. Fixed with a mandatory final retreat and an
   explicit `restTick`. This was both a correctness bug and a physical-safety
   bug, and only a test that reaches a real contact could have exposed it.

A fourth issue surfaced the same way: a fixed travel budget could not cover a
full-span traversal on a real joint. Each endpoint is now approached **from
neutral** with a budget sized from that endpoint's own geometry.

### Fault coverage (specification section 13)

| Fault | Covered by |
|---|---|
| zero responders | sim + **H0 hardware** |
| one missing expected servo | sim |
| unexpected extra servo | sim (incl. a head id appearing) |
| wrong model | sim |
| nonzero PositionOffset | sim |
| profile mismatch | sim |
| torque unexpectedly ON at census | sim |
| unstable RAW during manual q0 | sim + derive |
| q0 candidate outside plausible assembly range | derive |
| command outside 0..4095 | detector |
| approach toward wrap boundary | detector (refused, not wrapped) |
| contact never occurs | detector (travel + time budgets) |
| contact occurs too early | detector |
| one-sample false contact spike | detector (persistence) |
| repeated contacts inconsistent | detector (repeatability) |
| current spike without persistent contact | detector |
| temperature guard | detector |
| voltage guard | detector (low and high) |
| telemetry timeout | detector |
| bus read failure | detector + sim |
| retreat / torque-limit drift mid-approach | detector |
| target joint does not move | detector |
| non-target joint responds unexpectedly | sim |
| endpoint order / sign contradiction | derive |
| affine solution inconsistent with URDF | derive (scale out of range) |
| manual q0 inconsistent with derived q0 | derive (fails closed, never averages) |
| stale calibration presented | policy + sim |
| an LF V25 number entering acceptance logic | policy (provenance gating) |
| abnormal exit leaving torque enabled | sim (SAFE_OFF failure is loud) |

---

## H0 ESP32-S3 no-servo smoke test

**Session:** [`sessions/20260828T062919Z_h0_smoke/`](sessions/20260828T062919Z_h0_smoke/)
**Result: PASS — 22/22 gates.**

> ### ⚠️ This evidence predates the H3–H6 implementation
>
> The session below was run against the firmware image at commit `98a965f`, whose
> hashes are recorded in it. The current source adds the calibration engine and
> the H3–H6 command paths.
>
> **The current image has NOT been flashed and NOT been smoke-tested on
> hardware.** No reflash was performed in this work: the handoff withholds
> hardware authorization, and re-flashing would have been a hardware action taken
> without it.
>
> What the current source HAS satisfied: it compiles clean at every stage H0–H6,
> passes 206 offline tests including full H3–H6 happy paths through the real
> engine, and passes the static safety audit. What it has NOT satisfied: any
> hardware execution whatsoever.
>
> **Re-run `h0-smoke` on the new image before H1**, since the H0 gates verify the
> firmware's own reported safety state and that state now includes the bootstrap
> and characterization reporting.

### Environment

| Item | Value |
|---|---|
| Port | `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00` → `ttyACM0` |
| Arduino CLI | 1.5.1 |
| Core | `esp32:esp32` 3.3.11 |
| Library | `SCServo` 1.0.2 |
| FQBN | `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi` |
| Flash use | 343 776 bytes (10 %) |
| RAM use | 23 320 bytes (7 %) |

The historical port, CLI path and FQBN were **verified against the live system** before
use and matched.

### Hashes

| Artifact | SHA256 |
|---|---|
| firmware source `.ino` | `50220e4f73fc3a2b32052a9c5c8d5419201012e29f0ec40cb4963ad901d2b3d4` |
| `flc_contact_detector.h` | `04e4af48dd0c6e4bdd06eee05424a4abcd2458d86fdc1c0d7604a002e5156709` |
| firmware binary `.ino.bin` | `10d97a06faaf3436a00ce876bc3c9599eee279e148254fcb86d7f115384470d6` |
| host runner | `51b336dd66cd756dc6ce189529683a71ca49b6b7002dd7769f98cbcec46b942f` |
| policy module | `324e32579862953ebf95a2e8e6ded3848cdf4adca243dfddc735de8a161899c7` |
| servo allocation | `574dfd6e4cf88034655eea6b5f3505da06a4b2fc9cb198ac34cdd0398315b234` |
| geometry endpoint profile | `dd8cb42c3b916d067f97a321c5ffcdfb013dde1f2f2a6ba71f73becff360dc0f` |

Flash was written and verified by esptool (`Hash of data verified`).

### Gate results

| Gate | Result |
|---|---|
| firmware identity / version / build stamp | PASS |
| protocol labelled calibrator-local | PASS |
| Station absent from control path | PASS |
| no EEPROM write surface | PASS |
| no broadcast write | PASS |
| single GoalPosition authority | PASS |
| unsigned 0..4095 enforced | PASS |
| authorized stage is H0 | PASS |
| motion locked (8 outstanding parameters) | PASS |
| head absence expected | PASS |
| no unexpected responder | PASS |
| census correctly reports no responders | PASS |
| census refuses to pass | PASS |
| `@CALIBRATE_JOINT 13` refused | PASS |
| `@CALIBRATE_LEG LF` refused | PASS |
| `@CALIBRATE_ALL` refused | PASS |
| manual q0 refused without census | PASS |
| SAFE_OFF passes with no responders | PASS |
| SAFE_OFF idempotent | PASS |
| SAFE_OFF saw no responders | PASS |
| SAFE_OFF issued no writes | PASS |
| host policy independently blocks motion | PASS |

### Key transcript evidence

The census enumerated all 12 expected ids by joint and physical unit, found none, and
refused:

```text
CENSUS_EXPECTED_IDS=11,12,13,21,22,23,31,32,33,41,42,43
CENSUS_SERVO ID=13 JOINT=lf_hip_joint UNIT=M22 RESULT=MISSING
...
CENSUS_PRESENT=0/12
CENSUS_UNEXPECTED=0
CENSUS_HEAD_ABSENCE_EXPECTED=YES
CENSUS_FAIL_REASON=NO_RESPONDERS
CENSUS_RESULT FAIL
```

> The census FAIL is the **expected and correct** outcome with no servos attached. A
> system that refuses to proceed is passing this test.

Motion refused with three independent reasons:

```text
CALIBRATE_JOINT_REFUSED REASON=HARDWARE_STAGE_LOCKED NEEDS=H4 AUTHORIZED=H0
CALIBRATE_JOINT_REFUSED REASON=NO_FRESH_CENSUS_IN_THIS_SESSION
CALIBRATE_JOINT_REFUSED REASON=CHARACTERIZATION_REQUIRED COUNT=8
```

**Zero `WRITE` lines and zero `MOTION` lines** appear anywhere in the transcript. No byte
was written to the servo bus during the entire session.

No servo responded at any point. Had one responded, the runner would have stopped
immediately without issuing writes and left torque untouched.

---

## What is NOT validated

- **H1** — the 12-servo census against real servos;
- **H2** — manual q0 capture against real encoders;
- **H3–H6** — implemented and offline-validated, but **never executed on hardware**;
- every contact parameter measured on this build: the bootstrap envelope is a
  conservative starting point, not a measurement;
- joint encoder **direction** for all 12 joints — the engine measures it, but no
  joint has been measured yet;
- the servo-power rail, which was never energized;
- any claim that the reassembled robot matches the URDF — that is what H1–H6 will test.

Offline validation demonstrates that the software **can** perform these stages.
It is not evidence about the physical robot.

## Related

- [Leg reassembly calibration premise 2026-08-28](../../Calibration/MATDOG_LEG_REASSEMBLY_CALIBRATION_PREMISE_2026-08-28.md)
- [Calibration reset 2026-08-27](../../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)
- [ST3215 provisioning campaign](../ST3215_Provisioning_2026-08-27/README.md)

# Full Leg Calibrator V1 — validation evidence

**Date:** 2026-08-28
**Base commit:** `fcc1dbd1367d5d8c860a873c62a31b2620695aba`
**Tool:** [MATDOG Full Leg Calibrator V1](../../../05_Firmware/Full_Leg_Calibrator_V1/README.md)

---

## ⚠️ Hardware validation scope

> **Only stage H0 was exercised.**
> No servo was connected. No servo power was present. No motion was commanded.
> No EEPROM was written. **Nothing about H1 or later is validated by this work.**

| Stage | Meaning | Status |
|---|---|---|
| H0 | ESP32-S3 only, no servos attached | ✅ **PASSED 2026-08-28** |
| H1 | 12-servo read-only census | ⬜ not attempted — requires user present |
| H2 | manual-pose q0 capture, torque OFF | ⬜ not attempted |
| H3 | one joint runtime/contact characterization | 🔒 **LOCKED** |
| H4 | one joint full calibration | 🔒 LOCKED |
| H5 | one complete leg | 🔒 LOCKED |
| H6 | four legs sequentially | 🔒 LOCKED |
| H7 | final 12/12 freeze | 🔒 LOCKED |

H3+ is locked by **two independent gates**: `AUTHORIZED_STAGE = H0_ESP32_ONLY`, and eight
unresolved `CHARACTERIZATION_REQUIRED` parameters. Raising the stage alone does not
unlock motion — a test asserts this at H7.

---

## Offline validation — 140 tests, all passing

| Suite | Tests | What it proves |
|---|---|---|
| `test_matdog_full_leg_calibrator_detector.py` | 32 | contact state machine, driven through the **real C++ engine** |
| `test_matdog_full_leg_calibrator_sim.py` | 34 | census, q0 capture, mode gates, SAFE_OFF against a mock ST3215 bus |
| `test_matdog_full_leg_calibrator_derive.py` | 27 | q0 derivation, affine solve, cross-check |
| `test_matdog_full_leg_calibrator_policy.py` | 24 | provenance gating, joint specs, allocation validation |
| `test_matdog_full_leg_calibrator_firmware_sync.py` | 23 | static firmware audit + firmware/host constant sync + frozen-evidence integrity |

Full calibration suite after the change: **460 passed**, no regressions.

### The detector tests exercise real firmware code

`flc_contact_detector.h` is compiled into both the ESP32-S3 firmware and a host harness
(`tests/flc_detector_harness.cpp`, built with `-Wall -Wextra -Werror`). The fault
injection therefore runs the same translation unit that would drive servos — not a Python
re-implementation.

**Two real defects were found and fixed during this work. Both were encoder-wrap bugs.**

1. **Truncating modulo in the tick math** — caught by the harness on its first run.
   C++ `%` truncates toward zero, so `flcSignedTickDelta(0, 4095)` returned **−4095**
   where the canonical `matdog_joint_math.signed_tick_delta` returns **+1**. Every
   wrap-boundary decision in the contact state machine would have been wrong. Fixed with
   an explicit Euclidean `flcMod()`; a regression test now pins ten vectors against the
   canonical Python implementation.

2. **Linear mean in the manual-q0 summary** — caught by self-review of the diff, not by a
   test. The firmware summarized q0 samples with an arithmetic mean and a linear
   min/max. A joint resting near the 4095/0 boundary would have reported a centre of
   ~2047 and a spread of ~4095: a nonsense q0 and a false instability verdict. Fixed by
   moving the summary into the shared header as `flcSummarizeTicks()`, which accumulates
   signed deltas from the first sample; five new tests now cover it, including a direct
   comparison against `circular_tick_summary`.

The second defect is the reason the q0 summary now lives in the shared, host-compiled
header rather than in the `.ino`: logic that only exists in the sketch cannot be tested
offline.

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

- the 12-servo census against real servos (H1);
- manual q0 capture against real encoders (H2);
- every contact/endpoint parameter — all eight remain `CHARACTERIZATION_REQUIRED`;
- joint encoder **direction**, which is deliberately unmeasured for all 12 joints;
- the servo-power rail, which was never energized;
- any claim that the reassembled robot matches the URDF — that is what H1–H7 will test.

## Related

- [Leg reassembly calibration premise 2026-08-28](../../Calibration/MATDOG_LEG_REASSEMBLY_CALIBRATION_PREMISE_2026-08-28.md)
- [Calibration reset 2026-08-27](../../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)
- [ST3215 provisioning campaign](../ST3215_Provisioning_2026-08-27/README.md)

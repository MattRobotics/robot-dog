# MATDOG Firmware

This directory is reserved for low-level firmware that belongs specifically to MATDOG.

## Current status

**Bench firmware exists and is frozen.** [`ST3215_Bench_Tools/`](ST3215_Bench_Tools/) holds three
ESP32-S3 tools that have run on real hardware and are archived byte-for-byte:

| Tool | Purpose | Status |
|---|---|---|
| [Bench QC V6.1](ST3215_Bench_Tools/Bench_QC_V6_1/) | servo quality audit | frozen — 26 runs PASS |
| [Source Signature Survey V1](ST3215_Bench_Tools/Source_Signature_Survey_V1/) | read-only 71-byte state capture | frozen |
| [Provisioner V6](ST3215_Bench_Tools/Provisioner_V6/) | apply `MATDOG_C018_V1`, centre, zero offset, recode ID | frozen — 17/17 PASS |

These are **bench** tools: they run on a bench rig with a single servo attached, never on the
assembled robot under power for motion.

**MATDOG Controller V0.1 integration runtime is implemented.**
[`MATDOG_Controller/`](MATDOG_Controller/) is the first unified operational ESP32-S3 firmware:
one deployable image with modular ServoBus / BNO085 / DALY BMS / LED ring / USB diagnostics,
adapted from the hardware-proven bench and bring-up sources above rather than rewritten from
scratch. USB-only bench validation (boot, BNO085 live acquisition, existing viewer compatibility,
expected-offline classification for DALY/ST3215/LED) is recorded in
[`MATDOG_Controller/VALIDATION.md`](MATDOG_Controller/VALIDATION.md). **Motion, gait and
operational IK remain not implemented** — V0.1 is an integration and platform milestone, not a
motion controller.

The official tagged baseline and exact validated source are recorded in the
[`MATDOG_Controller` README](MATDOG_Controller/README.md#official-baseline). The root
[`README.md`](../README.md) owns the current project and hardware snapshot; the Controller
[`VALIDATION.md`](MATDOG_Controller/VALIDATION.md#present-day-baseline-and-next-gate) owns the
detailed validation status and next gate. This page is the firmware-area index.

The current hardware/control chain is:

    ASUS Ubuntu (dev) / Jetson Orin Nano Super (final)
    → native USB 2.0 Full-Speed / USB CDC
    → ESP32-S3 motion coprocessor        ← owns the ST3215 bus
    → UART GPIO17/GPIO18
    → Seeed Bus Servo Driver
    → 13 installed ST3215 servos        12 legs + NECK_ROTATION ID51

The ESP32-S3 is the operational owner of the servo serial bus.

The canonical allocation still contains 17 units. IDs 52–55 are allocated but intentionally
absent today; allocation and installation must not be conflated.

## Permanent Controller direction

The architectural direction is **one permanent `MATDOG_Controller` firmware**, extended in reviewed
stages rather than replaced by standalone operational images. It already implements IMU
acquisition, read-only battery telemetry, a USB diagnostic surface and a power-state-machine
baseline. Future integrated capabilities include:

- diagnostics, maintenance and service modes
- servo QC and provisioning
- full-leg calibration
- Wi-Fi / OTA and host transport
- deterministic servo motion / operational IK / gait execution
- closed-loop BNO085 stabilization
- watchdog / safety interlocks beyond the current health aggregation
- verified DALY write support (Discharge MOS OFF), once the K-Series protocol is identified
- Jetson onboard host integration

The branch `matdog/full-leg-calibrator-v1` is preserved as an implementation and evidence oracle.
It is not the final runtime architecture and must not be merged wholesale; selected calibration,
safety and evidence logic may be migrated into the permanent Controller in a later phase.

The frozen ST3215 tools remain immutable qualification evidence. Equivalent service capabilities
may later be integrated into the Controller without rewriting or replacing those frozen sources.

## Architecture Rule

Do not add experimental host-side gait, IK or calibration code directly to this firmware area;
host-side development belongs under `06_Software/`.

Bench firmware **is** in scope for this directory — it is genuine low-level ST3215 firmware.
Anything added here must follow the freeze policy in
[`ST3215_Bench_Tools/README.md`](ST3215_Bench_Tools/README.md): byte-for-byte source, SHA256 for
every artifact, provenance, complete command surface, toolchain/FQBN, test result, hardware
validation scope, and an explicit statement of what is **not** validated.

**Deterministic servo motion, gait execution and operational IK are decided to live on the
ESP32-S3 but are not yet written.** IMU acquisition, read-only power telemetry and a USB
diagnostic/health surface **are** written, as of `MATDOG_Controller` V0.1 — see
[`MATDOG_Controller/VALIDATION.md`](MATDOG_Controller/VALIDATION.md) for exactly what has been
exercised on real hardware versus what remains software-only. `MATDOG_Controller` is distinct
from the frozen bench tools above, which remain qualification instruments rather than robot
runtime.

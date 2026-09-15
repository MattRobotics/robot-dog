# MATDOG Firmware

This directory is reserved for low-level firmware that belongs specifically to MATDOG.

## Current Status

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

The current locomotion-development stack is:

    ASUS Ubuntu (dev) / Jetson Orin Nano Super (final)
    → native USB 2.0 Full-Speed / USB CDC
    → ESP32-S3 motion coprocessor        ← owns the ST3215 bus
    → UART GPIO17/GPIO18
    → Seeed Bus Servo Driver
    → 17 × ST3215 serial-bus servos

The ESP32-S3 is the operational owner of the servo serial bus.

## Future Scope

`MATDOG_Controller/` already implements IMU acquisition, read-only battery telemetry, a USB
diagnostic surface and a power-state-machine baseline (see above). It may later grow:

- deterministic servo motion / operational IK / gait execution
- closed-loop BNO085 stabilization
- watchdog / safety interlocks beyond the current health aggregation
- verified DALY write support (Discharge MOS OFF), once the K-Series protocol is identified
- Wi-Fi / OTA (separate future integration; not part of V0.1)
- Jetson onboard host integration

## Architecture Rule

Do not add experimental host-side gait, IK or calibration code here.

Bench firmware **is** in scope for this directory — it is genuine low-level ST3215 firmware.
Anything added here must follow the freeze policy in
[`ST3215_Bench_Tools/README.md`](ST3215_Bench_Tools/README.md): byte-for-byte source, SHA256 for
every artifact, provenance, complete command surface, toolchain/FQBN, test result, hardware
validation scope, and an explicit statement of what is **not** validated.

Those belong in:

    06_Software/

**Deterministic servo motion, gait execution and operational IK are decided to live on the
ESP32-S3 but are not yet written.** IMU acquisition, read-only power telemetry and a USB
diagnostic/health surface **are** written, as of `MATDOG_Controller` V0.1 — see
[`MATDOG_Controller/VALIDATION.md`](MATDOG_Controller/VALIDATION.md) for exactly what has been
exercised on real hardware versus what remains software-only. `MATDOG_Controller` is distinct
from the frozen bench tools above, which remain qualification instruments rather than robot
runtime.

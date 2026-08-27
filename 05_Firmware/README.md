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

**No robot flight firmware is active.** No onboard motion-controller firmware exists yet.

The current locomotion-development stack is:

    ASUS Ubuntu (dev) / Jetson Orin Nano Super (final)
    → native USB 2.0 Full-Speed / USB CDC
    → ESP32-S3 motion coprocessor        ← owns the ST3215 bus
    → UART GPIO17/GPIO18
    → Seeed Bus Servo Driver
    → 17 × ST3215 serial-bus servos

The ESP32-S3 is the operational owner of the servo serial bus.

## Future Scope

This directory may later contain:

- ESP32 motion-controller firmware
- IMU acquisition and filtering
- battery monitoring
- watchdog logic
- safety interlocks
- low-level ST3215 actuator adapter
- communication protocol between Jetson or host and motion controller

## Architecture Rule

Do not add experimental host-side gait, IK or calibration code here.

Bench firmware **is** in scope for this directory — it is genuine low-level ST3215 firmware.
Anything added here must follow the freeze policy in
[`ST3215_Bench_Tools/README.md`](ST3215_Bench_Tools/README.md): byte-for-byte source, SHA256 for
every artifact, provenance, complete command surface, toolchain/FQBN, test result, hardware
validation scope, and an explicit statement of what is **not** validated.

Those belong in:

    06_Software/

The **operational motion firmware** — deterministic servo control, gait execution, operational IK,
IMU, power telemetry and watchdog/safety — is **decided to live on the ESP32-S3 but is not yet
written**. It is distinct from the frozen bench tools above, which are qualification instruments
rather than robot runtime.

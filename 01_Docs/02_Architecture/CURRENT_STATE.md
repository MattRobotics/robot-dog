# MATDOG Current State

> **This document is superseded.** It described the Station-mediated, 12-servo architecture and a
> runtime chain (`Station → Waveshare → ST3215`) that is no longer MATDOG's architecture.
>
> Current state now lives in exactly two places, so there is no third competing description:

| For | Read |
|---|---|
| **Current architecture** — hardware, compute split, transport, validation scope | [`ARCHITECTURE.md`](ARCHITECTURE.md) |
| **Current project state** — what is validated, what is next | [root `README.md`](../../README.md) |

---

## What changed

| This document said | Reality as of 2026-08-27 |
|---|---|
| Twelve ST3215 servos | **17** — 12 leg + 5 head/jaw |
| Through the Waveshare Bus Servo Adapter | **Seeed Bus Servo Driver**; Waveshare was bench/historical |
| Runtime is `Station → Waveshare → ST3215` | **host → USB CDC → ESP32-S3 → UART → Seeed → ST3215** |
| ESP32 integration deferred until after stand/IK/walking | **ESP32-S3 is the operational bus owner today** — it provisioned all 17 servos |
| Servo mapping `LF: M13/M12/M11`, … | superseded — 14 of 17 units recoded, see [`MATDOG_SERVO_ALLOCATION.yaml`](../../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml) |
| Encoder zeros / joint directions "not yet validated" | still true, and now **reset** — see [calibration reset](../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) |

The geometry figures it recorded (225 / 95 / 90 / 110 mm, ~150 mm stand height) remain correct and
are carried in [`ARCHITECTURE.md`](ARCHITECTURE.md) and the root README.

Its safety rule also still holds, and is now machine-enforced:

> No automated multi-servo pose, gait or body-velocity command may be enabled until calibration and
> joint limits are documented and validated.

→ [Historical index](../../09_Logs/Historical/README.md)

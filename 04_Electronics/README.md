# MATDOG Electronics

Electrical architecture and hardware integration records for MATDOG.

Canonical system architecture: [`01_Docs/02_Architecture/ARCHITECTURE.md`](../01_Docs/02_Architecture/ARCHITECTURE.md)

---

## Current control chain

```text
high-level host
  ASUS Ubuntu (development)  /  Jetson Orin Nano Super (final onboard)
        ↓  native USB 2.0 Full-Speed / USB CDC
        ↓  ESP32-S3 D− = GPIO19, D+ = GPIO20
ESP32-S3 motion coprocessor          ← owns the ST3215 protocol and control
        ↓  UART
        ↓  GPIO17 TX → driver RX
        ↓  GPIO18 RX ← driver TX
        ↓  shared GND
Seeed Bus Servo Driver               ← servo-bus electrical layer
        ↓  Feetech serial bus, 1 Mbps
        ↓  custom power-distribution busbar
17 × Feetech ST-3215-C018            12 leg + 5 head/jaw
```

The GPIO17/GPIO18 UART assignment is not aspirational — it is compiled into the frozen bench
firmware (`SERVO_TX_PIN = 17`, `SERVO_RX_PIN = 18`) that provisioned all 17 servos.

### Host link

| Property | Value |
|---|---|
| Transport | native USB 2.0 Full-Speed / USB CDC — **decided, in use** |
| ESP32-S3 USB pins | D− = **GPIO19**, D+ = **GPIO20** |
| Theoretical rate | 12 Mbit/s |
| USB 3.x | neither required nor available on ESP32-S3 |
| Secondary channel | Wi-Fi command/diagnostic — planned |
| Packet/command protocol | **TBD** — transport frozen, protocol is not |

---

## Current hardware decisions

Lean by design. Each item below is a deliberate decision, not an omission.

| Decision | Status |
|---|---|
| **Seeed Bus Servo Driver** for the servo-bus electrical layer | selected |
| **No CAN transceiver** | decided — not part of the current architecture |
| **No dedicated Jetson DC/DC** in the current design | decided |
| **No separate 5 V logic DC/DC** — logic 5 V may come from the Jetson dev kit | decided |
| **One removable, externally accessible ATO main fuse** | decided — rating **TBD** |
| **Custom motor power busbar** | decided — dimensions and material **TBD** |
| **Locking 3D-printed cable housings** | decided |
| **No bulk servo capacitor** initially | decided — may be revisited after load measurement |
| **No TVS** initially | decided — may be revisited after load measurement |

> Values marked **TBD** are genuinely not frozen. They are deliberately not invented here.

### Deferred, with the reason

The bulk capacitor and TVS are omitted **initially**, not permanently. Both depend on real
transient behaviour under load, which cannot be measured until the robot is reassembled,
recalibrated and drawing current across all 17 servos. Revisit after the 17-servo power budget is
re-measured.

---

## Power

The custom distribution board and branch wiring are validated for the locomotion-development
phase. Protection is deliberately minimal in the current revision: one externally accessible ATO
main fuse, removable without disassembly.

> **The existing 3S power-load analysis was written for 12 servos.** The robot now has 17. The load
> budget must be re-evaluated before sustained multi-servo motion — see
> [`01_Docs/01_Analysis/ST3215_Quadruped_3S_Power_Load_Analysis.md`](../01_Docs/01_Analysis/ST3215_Quadruped_3S_Power_Load_Analysis.md).

---

## Current servo mapping

Authoritative machine-readable source:
[`06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml`](../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml)

```text
LF:   hip 13 (M22),   upper 12 (ELR01), lower 11 (M33)
RF:   hip 23 (NEW01), upper 22 (ELR03), lower 21 (NEW03)
RH:   hip 33 (NEW06), upper 32 (ELR02), lower 31 (NEW05)
LH:   hip 43 (M43),   upper 42 (M42),   lower 41 (M41)
HEAD: neck rotation 51 (M31), neck pitch 52 (M11),
      head rotation 53 (NEW04), head pitch 54 (NEW02), jaw 55 (ELR04)
```

> **Replaces the historical 12-servo mapping.** 14 of 17 units were recoded during provisioning, so
> historical records keyed by bus ID will associate the **wrong joint**.

## Contents

- `Servo_Mapping/` — mapping between semantic MATDOG joints and physical ST3215 IDs.

## Future contents

- wiring diagrams
- power-distribution-board documentation
- battery and BMS design
- charging and docking system
- IMU integration
- head sensors
- Jetson Orin Nano Super integration
- electrical validation measurements

---

## Historical

The **Waveshare Bus Servo Adapter** was the bench/development interface during the
Station-mediated phase and appears in evidence from that period, including the 2026-06-17 ST3215
bus validation. It is **not** the selected production interface — the current chain uses the Seeed
Bus Servo Driver.

Raw ST3215 bus diagnostics are provided by the ESP32-S3 bench tools
([`05_Firmware/ST3215_Bench_Tools/`](../05_Firmware/ST3215_Bench_Tools/README.md)). NormaCore
Station is optional/legacy tooling and is not required for any MATDOG bus operation.

→ [Historical index](../09_Logs/Historical/README.md)

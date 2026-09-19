# MATDOG Electronics

Electrical architecture and hardware integration records for MATDOG.

Canonical system architecture: [`01_Docs/02_Architecture/ARCHITECTURE.md`](../01_Docs/02_Architecture/ARCHITECTURE.md)

Canonical current project snapshot, including the physically installed population:
[`README.md`](../README.md)

---

## Current control chain

```text
high-level host
  ASUS Ubuntu (development)  /  Jetson Orin Nano Super (final onboard)
        ↓  native USB 2.0 Full-Speed / USB CDC
        ↓  ESP32-S3 D− = GPIO19, D+ = GPIO20
ESP32-S3 MATDOG Controller           ← owns the ST3215 protocol and control
        ↓  UART
        ↓  GPIO17 TX → driver RX
        ↓  GPIO18 RX ← driver TX
        ↓  shared GND
Seeed Bus Servo Driver               ← servo-bus electrical layer
        ↓  Feetech serial bus, 1 Mbps
        ↓  custom power-distribution busbar
13 × Feetech ST-3215-C018 installed  12 leg + NECK_ROTATION ID51
```

The GPIO17/GPIO18 UART assignment is not aspirational — it is compiled into the frozen bench
firmware (`SERVO_TX_PIN = 17`, `SERVO_RX_PIN = 18`) that provisioned all 17 servos.
Provisioning/allocation history is not the same as physical installation: the canonical registry
allocates 17 units, but only 13 are installed today.

### Host link

| Property | Value |
|---|---|
| Transport | native USB 2.0 Full-Speed / USB CDC — **DECIDED**; validated through the controller's native USB connection |
| ESP32-S3 USB pins | D− = **GPIO19**, D+ = **GPIO20** |
| External 4-pin connector | GPIO19, GPIO20, ESP32 GND; 5 V is **not connected** — **TO_TEST** USB-data/service predisposition |
| Theoretical rate | 12 Mbit/s |
| USB 3.x | neither required nor available on ESP32-S3 |
| Secondary channel | Wi-Fi command/diagnostic — planned |
| Packet/command protocol | **TBD** — transport frozen, protocol is not |

Historical Jetson-UART use of GPIO19/GPIO20 is **SUPERSEDED**. Their current canonical meaning is
native USB D−/D+. The existing external connector has not yet been physically validated as a USB
service link; the validated onboard/native USB path does not prove that connector. It remains a
future USB service-port candidate — not a UART — pending electrical and signal-integrity
validation.

---

## Current hardware decisions

Lean by design. Each item below is a deliberate decision, not an omission.

| Decision | Status |
|---|---|
| **Seeed Bus Servo Driver** for the servo-bus electrical layer | selected |
| **No CAN transceiver** | decided — not part of the current architecture |
| **ESP32-S3 power**: DALY-protected B+/P− domain → 5 V step-down | **DECIDED**; validated no-motion in ROBOT_POWERED (G3, 2026-09-18) |
| **Primary hardware ON/OFF/wake**: bistable pushbutton under the robot logo → DALY `KEY` directly | **DECIDED**; no ESP32 GPIO required. Function **OPEN** — in G3 the KEY switch produced no observed DALY state change; not a validated shutdown/safety barrier. KEY configuration read live 2026-09-19 via the read-only `@BMS KEY READ` probe: KEY logic **DISABLED** (`0x0055`), which explains the G3 finding; no DALY write exists |
| **One removable, externally accessible ATO main fuse** | decided — rating **TBD** |
| **Custom motor power busbar** | decided — dimensions and material **TBD** |
| **Locking 3D-printed cable housings** | decided |
| **No bulk servo capacitor** initially | decided — may be revisited after load measurement |
| **No TVS** initially | decided — may be revisited after load measurement |

> Values marked **TBD** are genuinely not frozen. They are deliberately not invented here.

### Deferred, with the reason

The bulk capacitor and TVS are omitted **initially**, not permanently. Both depend on real
transient behaviour under load, which cannot be measured until the robot is reassembled,
recalibrated and drawing current across the installed servo population. Revisit after the current
13-servo system is measured; repeat the budget if the population later expands toward all 17
allocated joints.

---

## Power and wiring — validation status

The power architecture is **DECIDED**, but the current 13-servo installed configuration has not yet
completed ROBOT_POWERED validation. Nothing here should be read as evidence for a powered,
multi-servo robot test.

### VALIDATED — proven on real hardware

- **ESP32-S3 direct ST3215 bus operation** — the ESP32-S3 exclusively owned and drove the bus.
- **Direct bus-control path**, to the extent covered by real bring-up evidence.
- **QC and provisioning** — 26 QC runs and 17/17 units provisioned.

Evidence: [Bench QC V6.1](../09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md) ·
[provisioning campaign](../09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/README.md).

> That evidence is **bench** evidence: single-unit bench rig, one servo at a time. It says nothing
> about the assembled 13-servo power system.

### TO_TEST — not yet final-hardware-validated

- the custom final **busbar / power distribution**;
- the **ATO protection implementation** (rating TBD);
- the installed **13-servo wiring**;
- the installed **13-servo power budget** and any later 17-unit expansion;
- the DALY `KEY` power/wake/shutdown function (the step-down controller supply itself ran the
  whole G3 session; the fused disconnect remains the trusted isolation method);
- the GPIO19/GPIO20 external USB-data/service connector.

Protection is deliberately minimal in the current revision: one externally accessible ATO main
fuse, removable without disassembly.

> **The existing 3S power-load analysis was written for 12 servos.** There are now 13 physically
> installed, while the canonical allocation reserves 17. The load budget must be re-evaluated
> before sustained multi-servo motion — see
> [`01_Docs/01_Analysis/ST3215_Quadruped_3S_Power_Load_Analysis.md`](../01_Docs/01_Analysis/ST3215_Quadruped_3S_Power_Load_Analysis.md).

### Historical

The earlier custom distribution board and branch wiring were validated for the **12-servo**
Station-mediated development phase. That validation belongs to a configuration the robot no longer
has, and does not carry over to today's 13-servo installation or a future 17-unit installation.

---

## Allocation versus installed population

Authoritative machine-readable source:
[`06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml`](../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml)

```text
canonical allocation: 17 units / IDs
installed today:      13 = 12 leg servos + NECK_ROTATION ID51
absent by design:      4 = 52 NECK_PITCH, 53 HEAD_ROTATION,
                           54 HEAD_PITCH, 55 JAW
```

Do not reduce the 17-unit allocation registry to match the current 13-unit installation. Fourteen
of the 17 allocated units were recoded during provisioning, so older records keyed only by bus ID
may associate the wrong joint. The root README owns the current installed-population snapshot; the
allocation YAML owns the complete unit → joint → ID registry.

## Contents

- `Servo_Mapping/` — legacy 12-leg mapping/reference; it is not the canonical 17-unit allocation
  registry or a statement of today's installed population.

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

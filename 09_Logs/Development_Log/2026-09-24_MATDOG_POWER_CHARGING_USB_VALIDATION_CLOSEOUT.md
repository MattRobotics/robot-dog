# MATDOG — Power, Charging & External USB Validation Closeout

**Date:** 2026-09-24
**Project:** MATDOG / `MattRobotics/robot-dog`
**Branch:** `feat/daly-key-readonly-probe` (PR #27, draft)
**Purpose:** Record the live hardware evidence gathered on 2026-09-24 (and the manual charging
session of 2026-09-22/23) that closes the post-rewire validation named in the
[2026-09-20 handoff](2026-09-20_MATDOG_POWER_KEY_CHARGING_NEXTGEN_HANDOFF.md) and in
[`04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md`](../../04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md)
§ 11. This is a new dated record, not a rewrite of the 2026-09-20 handoff, which stays exactly as
written and remains historically accurate for what was known on 2026-09-20.

Documentation/evidence only. No firmware change, no flashing, no DALY setting change, no command
sent to the robot as part of writing this record.

---

## 1. Repository state at session start

```text
branch:            feat/daly-key-readonly-probe
HEAD:              19fe837c3fca2fb919ab87a4a65b6861206e702f
origin/main:       dd5746c1639ebd175f4c1ff5dcd2340799b0a5dc
working tree:      clean
PR #27:            open, draft, base main @ dd5746c, head @ 19fe837
```

PR #27's title and body still said "DRAFT — hardware rewire pending" and described the `B-`/`P-`
rewire, its post-rewire validation, and the physical KEY safety barrier as BLOCKED/TO_TEST/OPEN.
This record, and the documentation updates made alongside it, close those specific items with the
evidence below. PR #27 stays **draft** — Matteo/ChatGPT still perform a final review before it is
marked ready.

Firmware identity: the commits added to this branch after the live 2026-09-19 DALY KEY sessions
(`4604e36` test-only, `efba2dc` docs-only, `19fe837` docs-only) made **no firmware source change**.
The robot exercised on 2026-09-24 was therefore still running the powered firmware already
installed from the earlier live DALY work (source commit `6322563`, the KEY write pre-transmit
mode re-check — the last commit on this branch that touched firmware source before the
documentation-only commits). No documentation-only commit was flashed to the robot.

---

## 2. Hardware correction: the `B-`/`P-` bypass is resolved

The defect recorded in the 2026-09-20 handoff and in
`04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md` § 10 (superseded below) was:

```text
TECNOIOT VIN- -> battery B-  (WRONG, historical)
```

which bridged the open discharge MOS through `ESP32 GND -> Seeed GND -> servo rail GND -> P-`.

The physical rewire has been completed:

```text
TECNOIOT VIN- -> DALY P-  (CURRENT)
```

The A–E procedure below is the live measurement that this correction actually removes the bypass.
No firmware change was needed or made for this correction.

---

## 3. Power-gate procedure A–E — live evidence, PASS

Procedure and pass criteria are unchanged from
`04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md` § 11 (written 2026-09-20, before the rewire).
This section records what was actually measured on 2026-09-24.

### A — Dead / KEY-OFF baseline

Initial state: charger disconnected, USB disconnected, KEY OFF.

| Condition | servo rail | TECNOIOT output | PAD+ → PAD- |
|---|---|---|---|
| Main fuse **removed**, KEY OFF | 0 V | 0 V | 0 V |
| Main fuse **inserted**, KEY still OFF | 0 V | 0 V | 0 V |

**Result: PASS.** Inserting the positive fuse alone does not energize the protected robot domain
while the DALY Discharge MOS is open — the historical `B-`/`P-` bypass is gone.

### B — KEY ON

Fuse inserted, USB disconnected, charger disconnected. Physical KEY switched ON.

```text
servo rail       = 12.23 V
TECNOIOT output  = 5.18 V
PAD+ -> PAD-     = 12.23 V
ESP32-S3         powered ON

DALY app: Charge MOS = ON, Discharge MOS = ON, current ≈ -0.2 A, alarms = none
No servo moved.
```

**Result: PASS.**

### C — KEY OFF (from powered state)

Fuse inserted, USB disconnected, charger disconnected. Physical KEY switched OFF.

```text
shutdown                immediate
servo rail               0 V
TECNOIOT output          0 V
PAD+ -> PAD-             0 V
ESP32-S3                 OFF
Discharge MOS            OFF
Charge MOS               remained ON
```

**Result: PASS.** This is the physical proof that the historical `B-`/`P-` bypass has been
removed: with no charger and no USB present, KEY OFF now collapses the entire protected robot
domain.

### D — KEY ON again / cold boot

Physical KEY switched ON again.

```text
TECNOIOT output  ≈ 5.3 V
servo rail       ≈ 12.2 V
ESP32-S3         ON, no reset loop, no BMS alarm
No servo moved.
```

**Result: PASS.**

### E — Powered no-motion regression

Using the established native USB CDC, no-reset diagnostic method (see § 5 below for the port
itself):

```text
@STATUS: SYSTEM health=BOOTING, power_state=RUN, mode=MAINTENANCE, profile=ROBOT_POWERED
BNO085:  init=OK detected=ONLINE expected=REQUIRED result=PASS
DALY:    init=OK detected=ONLINE expected=REQUIRED result=PASS
runtime_resets = 0
```

BMS sample during validation:

```text
pack_v=12.0 V   current_a=-0.2 A   soc=94.2 %
cells=3         cell_max_mv=4035   cell_min_mv=4019   delta_mv=16
charge_mos=ON   discharge_mos=ON   state=STATIONARY
alarms=0000 0000 0000 0000
```

BNO085 stream: ONLINE / REQUIRED / PASS, `runtime_resets=0`.

Servo census:

```text
SERVO_CENSUS=PASS lo=11 hi=55
canonical_allocated=17 expected_now=13
present_expected=13  missing_expected=0
absent_by_design=4   absent_by_design_present=0
unexpected_id=0      not_probed=0
truncated=NO
```

Servo module: `init=OK detected=ONLINE expected=REQUIRED result=PASS`.

`SAFE_OFF` executed on every installed servo (`11 12 13 21 22 23 31 32 33 41 42 43 51`): every one
returned `result=VERIFIED_OFF`. No servo moved.

**Result: PASS.**

### A–E summary

**The complete post-rewire power gate A–E is PASS.** Physical KEY OFF now removes the protected
robot domain with no charger and no USB present, and the powered no-motion regression from G3/G3.1
still holds on the currently installed firmware. This closes the "hardware `B-`/`P-` rewire
pending" item named throughout PR #27 and the roadmap.

---

## 4. Charging discovery: the common-port topology back-powers the load bus

A separate live charging session was performed on 2026-09-22 / 2026-09-23, before the 2026-09-24
power-gate session above.

```text
Battery:          Zeee LiPo 3S1P 9000 mAh
Charger:          12.6 V / 3 A CC/CV class charger
Connection:       charger + -> MATDOG PAD+ / common B+ side
                   charger - -> MATDOG PAD- / P-
```

**Critical finding.** When the external charger is physically connected between `B+` and `P-`,
KEY OFF + Discharge MOS OFF + Charge MOS ON does **not** mean the `B+`/`P-` bus is electrically
dead. The charger itself imposes voltage directly across `B+`/`P-`, and every load on that bus
(including the ESP32 and servo rail) is powered from the charger, independent of the Discharge
MOS.

Observed live:

```text
KEY OFF sensed by the DALY; Discharge MOS eventually OFF; Charge MOS ON.
Battery discharge current -> 0.
ESP32-S3 remained powered.
servo rail remained energized.
TECNOIOT remained powered.
```

When the charger was unplugged while KEY remained OFF and Discharge MOS remained OFF: servo rail,
TECNOIOT and ESP32-S3 all shut down immediately. This is expected and consistent with the current
common-port topology (charger and robot loads share the same `B+`/`P-` bus; there is no separate
load-disconnect between them).

**This corrects a previously stated, false semantic.** Any current-facing text that says
"manual charging with KEY OFF → robot domain OFF" (or equivalent) is wrong for the hardware as
built and is corrected in the canonical power document (§ 9 of this record). The correct model:

```text
MANUAL CHARGING, KEY OFF (current hardware):
  KEY OFF, Discharge MOS OFF, Charge MOS ON
  battery discharge path isolated from the pack
  BUT the B+/P- load bus is externally energized by the charger
  ESP32 ON (from the charger), servo rail electrically energized (from the charger)
  servo torque MUST remain OFF, no motion
  unplugging the charger with KEY still OFF collapses the robot domain (per § 3.C above)
```

The current DALY Discharge MOS **cannot** disconnect `B+`/`P-` loads from a charger that is
directly connected across `B+`/`P-`. A future requirement for "charger connected + battery
charging + robot load bus electrically dead" needs a separate load-disconnect or a different
topology — it is not something the existing hardware can be assumed to already do.

### Charging evidence timeline (2026-09-22 → 2026-09-24)

```text
start of charge:      pack ≈ 9.7 V,  SOC 0 %,   cell min ≈ 3.196 V
after connection:     pack ≈ 9.9 V,  ≈ 2.8 A,   cells ≈ 3.284–3.344 V
later:                pack ≈ 10.6 V, ≈ 2.9 A,   cells ≈ 3.552–3.578 V, delta ≈ 26 mV, temps ≈ 25/28 °C
near end of session:  SOC 96.5 %, pack 12.2 V, 0 A, remaining ≈ 8.6 Ah
                       Charge MOS ON, Discharge MOS OFF
                       cell max/min ≈ 4.092/4.088 V, delta 4 mV, temps ≈ 24/28 °C
next morning (~9 h,
  charger removed):   SOC 96.2 %, pack 12.2 V, 0 A
                       Charge MOS ON, Discharge MOS OFF
                       cell max/min ≈ 4.088/4.082 V, delta 6 mV, temps ≈ 21/22 °C
2026-09-24, before
  the power-gate
  session above:      SOC ≈ 95.6 %, pack ≈ 12.1 V, 0 A
                       Charge MOS ON, Discharge MOS OFF
                       cell max/min ≈ 4.075/4.060 V, delta 15 mV, temps ≈ 24/24 °C
```

This is evidence that the manual charging session and the common-port power behaviour above are
real. It is **not** evidence that the full charger/dock system is validated for autonomous
unattended charging. Remaining separate gates: autonomous dock/contact hardware, reverse-polarity
protection (not yet evidenced), complete unattended charge-acceptance behaviour, future Jetson
charging behaviour, and a long-term automated charge-termination policy. SOC alone is also not
sufficient evidence of a true FULL state — see the LED-ring FUTURE item in § 7.

---

## 5. External USB service/programming port — validated

A definitive external three-wire service connector was physically built:

```text
GPIO19  = USB D-
GPIO20  = USB D+
GND     = USB ground
(host VBUS / red +5V wire intentionally NOT connected)
```

The ESP32-S3 is self-powered from the robot's protected TECNOIOT supply through this connector,
not from host USB power.

With the onboard USB-C disconnected, using **only** the new external connector:

```text
USB enumeration:  VID:PID 303a:1001, "Espressif USB JTAG/serial debug unit"
stable path:      /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00
                   -> ../../ttyACM0
MAC:              14:c1:9f:22:75:94
```

CDC bidirectional diagnostic communication was verified through the new connector with `@STATUS`,
`@MODE STATUS`, `@BMS STATUS`, `@IMU STATUS`, `@LED STATUS`. BNO085 and DALY remained PASS,
`runtime_resets=0`.

Programming/recovery path was verified **without writing flash**, using `esptool` v5.3.1
(`esptool --chip esp32s3 ... flash-id`):

```text
Connected to ESP32-S3
Chip ESP32-S3 revision v0.2, Embedded PSRAM 8 MB, Crystal 40 MHz
USB mode: USB-Serial/JTAG
MAC 14:c1:9f:22:75:94
Stub flasher running
Detected flash size: 16 MB, Flash voltage 3.3 V
Hard resetting via RTS pin
```

After the hard reset the board re-enumerated correctly as `303a:1001 Espressif USB JTAG/serial
debug unit` at the same stable `by-id` path.

**Validated:** electrical D-/D+/GND path, native USB enumeration, CDC RX, CDC TX, USB reset / ROM
download handshake, `esptool` connectivity, flash identification/read path, post-reset
re-enumeration.

**Operational limitation, unchanged by this validation:** because host VBUS is intentionally not
connected, the external service port does **not** power the ESP32. For diagnostics/programming,
MATDOG must already be powered from its protected supply (normally KEY ON) or another explicitly
validated robot power source. With KEY OFF and the charger absent, plugging in the external
service USB alone must not power the Controller — consistent with § 3.A/C above.

The onboard USB-C is no longer required for normal service/programming access, though it remains
physically part of the dev board.

---

## 6. Final physical safe state after this session

At the end of the 2026-09-24 session:

```text
external USB unplugged
physical KEY switched OFF
servo rail shut down
ESP32-S3 shut down
main fuse removed
```

The robot was left electrically isolated for service.

---

## 7. What remains open (not closed by this record)

- **BMS KEY-configuration persistence across a true DALY power cycle** — still **TO_TEST**. Toggling
  the physical KEY or removing/reinserting the main robot fuse does not power-cycle the BMS itself,
  and no such power cycle was performed or measured. Do not infer persistence from this session.
- **Autonomous dock/contact hardware** — **FUTURE**, no hardware exists.
- **Reverse-polarity protection** — not yet evidenced either way.
- **Complete unattended/autonomous charge-acceptance validation** — **FUTURE**; only one manual,
  attended charging session exists.
- **Future Jetson charging behaviour** — **FUTURE**, architectural only.
- **Long-term automated charge-termination policy** — **FUTURE / TO_DESIGN**.
- **Charging LED-ring progress indication** — **FUTURE / TO_DESIGN**, **not implemented**. See the
  target concept recorded in `04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md` § 14. No firmware
  for this exists and none was written as part of this closeout.

---

## 8. No invented evidence

Every measurement above is exactly what was reported by Matteo from the physical session(s); no
values were interpolated, rounded beyond what was reported, or inferred beyond what the DALY app,
the Controller's own `@STATUS`/`@BMS`/`@SERVO` output, or `esptool` actually printed. Where a
figure was reported as approximate ("≈"), it is kept approximate here.

---

## 9. Documentation updated as a result

- [`04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md`](../../04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md) —
  §§ 1, 2, 5, 7, 8, 9, 10, 11 updated from BLOCKED/TO_TEST to VERIFIED where this record supports
  it; § 8 charging semantics corrected; § 10 converted to a resolved/superseded defect record; new
  external USB service-port section added.
- [`05_Firmware/MATDOG_Controller/VALIDATION.md`](../../05_Firmware/MATDOG_Controller/VALIDATION.md) —
  new dated session added; present-day summary updated.
- [`05_Firmware/MATDOG_Controller/CHANGELOG.md`](../../05_Firmware/MATDOG_Controller/CHANGELOG.md),
  [`05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md`](../../05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md),
  [`05_Firmware/MATDOG_Controller/README.md`](../../05_Firmware/MATDOG_Controller/README.md),
  [`04_Electronics/README.md`](../../04_Electronics/README.md),
  [`01_Docs/02_Architecture/ARCHITECTURE.md`](../../01_Docs/02_Architecture/ARCHITECTURE.md),
  [`01_Docs/02_Architecture/ROADMAP.md`](../../01_Docs/02_Architecture/ROADMAP.md), and the root
  [`README.md`](../../README.md) — stale BLOCKED/pending current-facing language replaced with the
  status this record supports; no firmware source change accompanies any of it.

This record itself is the evidence source for all of the above; none of those files restate
measurements that are not already here or in the 2026-09-20 handoff.

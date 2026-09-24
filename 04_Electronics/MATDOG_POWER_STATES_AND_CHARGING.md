# MATDOG — Power States, KEY Semantics and Charging

**Canonical owner** of MATDOG's power domains, power states, the physical `KEY` button, Charge/
Discharge MOS policy, daily use, storage, service isolation and charging (manual and autonomous).

Frozen: 2026-09-20 (decision), updated 2026-09-24 (post-rewire hardware validation). Input decision
record:
[`09_Logs/Development_Log/2026-09-20_MATDOG_POWER_KEY_CHARGING_NEXTGEN_HANDOFF.md`](../09_Logs/Development_Log/2026-09-20_MATDOG_POWER_KEY_CHARGING_NEXTGEN_HANDOFF.md).
Validation evidence:
[`09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md`](../09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md).

Related owners: [`04_Electronics/README.md`](README.md) (wiring, connectors, components) ·
[`01_Docs/02_Architecture/ARCHITECTURE.md`](../01_Docs/02_Architecture/ARCHITECTURE.md) (system
architecture) · [`05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md`](../05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md)
(gate criteria) · [`05_Firmware/MATDOG_Controller/VALIDATION.md`](../05_Firmware/MATDOG_Controller/VALIDATION.md)
(evidence).

Every claim below carries a status: **VERIFIED** (measured on this robot),
**IMPLEMENTED/OFFLINE-VALIDATED**, **TO_TEST**, **FUTURE** or **BLOCKED**. Architectural intent is
never written as validation evidence.

---

## 1. Power domains — the invariant

```text
Battery B+  ->  common positive distribution
Battery B-  ->  DALY B-  ONLY

DALY protected system return = P-

EVERY ordinary robot load uses  B+ / P-
```

On the protected `B+`/`P-` domain: servo rail, TECNOIOT step-down, ESP32-S3, BNO085, Seeed Bus
Servo Driver logic, XY-017/ARCELI RS485 interface, LED and auxiliary logic, the future TFT eye
electronics, and the future Jetson regulator and Jetson domain.

**No ordinary robot load may return directly to battery `B-`.** The ESP32-S3 is **not** a
raw-battery always-on load, and **no isolated DC/DC converter is required** for it. The TECNOIOT
remains the 3S → 5 V step-down provided its `VIN-` returns to `P-`.

Status: architecture **DECIDED**; physical conformance **VERIFIED** 2026-09-24 (rewire complete
and measured — see §10 and §11).

---

## 2. KEY semantics — frozen

The bistable/self-locking button under the MATDOG logo is wired directly to the DALY `KEY` input.
It is **not** connected to an ESP32 GPIO, and firmware must never drive it.

The BMS KEY logic register is configured to:

```text
0x0120 = 0x005A = DISCHARGE
```

| | KEY ON | KEY OFF |
|---|---|---|
| Discharge MOS | ON | OFF |
| Charge MOS | ON | **stays ON** |
| `P-` load domain | available | removed |
| Robot electronics | powered | off |
| Charging | possible | still architecturally possible |

**KEY controls the Discharge MOS only.** It must never be changed to `CHARGE_AND_DISCHARGE`
(`0x00AA`) and must never be mapped to the Charge MOS.

Human meaning: KEY ON = MATDOG available; KEY OFF = deliberately powered off. It is **not** a
charging command, **not** a service isolation device, and **not** a substitute for the removable
main fuse / service disconnect.

Status: the register write is **VERIFIED** (live, 2026-09-19 — see §9); the resulting physical
KEY OFF/ON rail behaviour is **VERIFIED** 2026-09-24 (§11 procedure A–E, PASS). Persistence of the
register value across a true DALY power cycle remains **TO_TEST** — see §9.

---

## 3. Charge MOS policy — frozen

```text
Charge MOS = normally ON
```

- No normal-runtime logic toggles the Charge MOS to start or stop ordinary charging.
- The physical KEY is never mapped to the Charge MOS.
- The charger/dock owns normal charge start/stop.
- The DALY Charge MOS stays a BMS protection / upper-level isolation mechanism.

Until a separately designed and validated fail-safe or service phase is explicitly opened:

```text
no write to 0x0121 (charge MOS control)
no write to 0x0122 (discharge MOS control)
no generic BMS register writer
```

Status: **IMPLEMENTED/OFFLINE-VALIDATED** — `scripts/static_audit.py` fails the build on any such
write; the mutation suite proves it.

---

## 4. Powered is not motion-enabled

Two independent facts, never collapsed into one:

```text
electrically powered   =  the P- domain is live and the Controller runs
motion enabled         =  an operating mode and authority permit actuator motion
```

A docked, charging robot is powered and awake while motion stays inhibited and servo torque is
OFF. Today the Controller has no motion capability at all, and `OperatingMode` (`MAINTENANCE` /
`RUN`) is the only boundary that exists. Status: **VERIFIED** for today's firmware; the full
`ActuatorAuthority` model is **FUTURE**.

---

## 5. Power states

Validation status is per row, for the robot **as built today** (no charger, no dock, no Jetson).

| State | KEY | Discharge MOS | Charge MOS | ESP32 | Servo torque | Motion authority | Future Jetson | Charger | Status |
|---|---|---|---|---|---|---|---|---|---|
| **OFF** | OFF | OFF | ON | OFF | OFF | none | OFF | absent | **VERIFIED** 2026-09-24 (§11 A/C) |
| **BOOT** | ON | ON | ON | ON | OFF | none | boots later | OFF | **VERIFIED** (G3/G3.1, no-motion; reconfirmed §11 B/D) |
| **READY/IDLE** | ON | ON | ON | ON | OFF | none | ON as needed | OFF | **VERIFIED** no-motion (§11 E) |
| **RUN** | ON | ON | ON | ON | enabled by authority | granted | ON | OFF | **FUTURE** — no motion capability exists |
| **DOCKING** | ON | ON | ON | ON | controlled | restricted | ON | OFF until validated | **FUTURE** |
| **CHARGING** | ON | ON | ON | ON | OFF | inhibited | OFF / low-power | ON | **FUTURE** — autonomous docking/charging hardware absent |
| **DOCKED_LOW_POWER** | ON | ON | ON | ON, low-power | OFF | inhibited | OFF | complete/maintenance | **FUTURE** |
| **MANUAL_CHARGE_KEY_OFF** | OFF | OFF | ON | **ON** (charger-backfed) | OFF (must stay OFF) | none | OFF | ON | **VERIFIED** common-port electrical behaviour 2026-09-22/23 + 2026-09-24 — see §8; full charge-cycle/dock qualification still **FUTURE** |
| **SERVICE_ISOLATED** | OFF | OFF | context-dependent | OFF | OFF | none | OFF | disconnected, fuse open | **VERIFIED** 2026-09-24 for the current hardware/procedure — see §7 |

---

## 6. Normal use

### Boot
```text
user presses KEY ON
-> DALY enables the discharge path
-> P- becomes available
-> TECNOIOT powers the ESP32
-> Controller boots; BNO085 / DALY / servo bus / peripherals initialize
-> health gates run; motion stays disallowed by the existing safety gates
```

### Daily shutdown (current ESP32-only robot)
```text
1. stop motion
2. stable rest pose
3. SAFE_OFF / servo torque OFF, verified by readback
4. KEY OFF
```
Actuator shutdown comes first; removing the power domain is the final user action. The DALY
discharge MOS is **not** a substitute for `TORQUE OFF`.

### Daily shutdown (future Jetson robot) — **FUTURE**
```text
1. stop autonomous behaviour / motion
2. stable rest pose
3. servo torque OFF
4. request graceful Jetson software shutdown
5. wait for verified shutdown / safe-to-remove-power state
6. KEY OFF
```
KEY OFF stays available as an emergency hard cut, but it is not the normal Jetson path.

### Emergency
Immediate power removal: **KEY OFF**, even if a future Jetson has not shut down cleanly.
Electrical isolation: **service disconnect / fuse removal** — the stronger action.

Distinguish clearly: logical stop · servo torque OFF · normal software shutdown · KEY power-off ·
service isolation · emergency hard power removal.

### Storage
Short/normal: KEY OFF. Long: KEY OFF **plus** the service disconnect / removable fuse opened.
Battery storage SOC and LiPo maintenance remain separate battery procedures.

---

## 7. Maintenance and service isolation

KEY OFF is now a **measured** functional power-off *through the BMS* — see §11 A/C — provided no
charger is connected (§8 explains why a connected charger changes this). It is still **not** a
substitute for the removable fuse / service disconnect as the trusted maintenance isolation point,
because a connected charger alone can re-energize the `B+`/`P-` bus regardless of KEY state.

```text
1. controlled shutdown
2. KEY OFF
3. verify the load rails are down (measure, do not assume)
4. remove/open the main service fuse / disconnect
5. disconnect the charging dock / charger
6. disconnect USB and any external grounds as required
7. verify the absence of unintended power/backfeed before work
```

Status: **VERIFIED** 2026-09-24 for the current hardware — the full sequence (KEY OFF, fuse
removed, external USB unplugged) was executed as the session closeout and measured down to 0 V on
every rail (§11, §3 of the 2026-09-24 closeout record). Step 3 (verify the load rails are down) is
now trustworthy with the charger and USB absent; it must **not** be assumed to hold with a charger
connected — see §8.

---

## 8. Charging

### Manual charging with KEY OFF — `MANUAL_CHARGE_KEY_OFF` — **VERIFIED** common-port behaviour, 2026-09-22/23 + 2026-09-24

**The old state name `MANUAL_CHARGE_OFF` and its old semantics were wrong and are retired.** A
live charging session (Zeee LiPo 3S1P 9000 mAh, 12.6 V/3 A CC/CV charger, connected charger+ →
`PAD+`/`B+`, charger- → `PAD-`/`P-`) showed that:

```text
KEY OFF -> Discharge MOS OFF (eventually) -> Charge MOS stays ON
BUT the charger is physically connected across the same B+/P- bus every ordinary load uses,
so the charger DIRECTLY ENERGIZES that bus. The robot domain is NOT off.
```

Measured live: with KEY OFF, Discharge MOS OFF and Charge MOS ON, the ESP32-S3 stayed powered, the
servo rail stayed energized, and TECNOIOT stayed powered — all backfed from the charger. Battery
discharge current went to zero (the pack itself was isolated), but the `B+`/`P-` **load bus was
not**. Unplugging the charger while KEY stayed OFF collapsed the servo rail, TECNOIOT and ESP32-S3
immediately, confirming there is no other return path once the charger is removed.

**Corrected model:**

```text
MANUAL_CHARGE_KEY_OFF (current hardware):
  KEY OFF, Discharge MOS OFF, Charge MOS ON
  battery discharge path isolated from the pack
  B+/P- load bus IS externally energized by the charger
  ESP32 ON (from the charger), servo rail electrically energized (from the charger)
  servo torque MUST stay OFF, no motion authority
  unplugging the charger with KEY still OFF collapses the robot domain
```

**The current common-port DALY topology cannot disconnect `B+`/`P-` loads from a charger that is
directly connected across `B+`/`P-`.** Do not describe KEY OFF as powering down the robot while a
charger remains connected. A future requirement for "charger connected + battery charging + robot
load bus electrically dead" needs a separate load-disconnect or a different topology — it does not
exist today and must not be assumed.

Evidence, full charging timeline and the reasoning above:
[`09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md`](../09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md)
§ 4.

### Autonomous docking and charging — **FUTURE**
When MATDOG drives itself to the dock, **the KEY stays ON**: no human presses the button. The
Discharge and Charge MOS stay ON, the ESP32 stays powered, and `P-` stays available.

```text
RUN -> DOCKING -> dock pose/contact confirmed -> stop motion -> TORQUE OFF
    -> validate charger/contact conditions -> CHARGING
```
During CHARGING: ESP32 ON, DALY telemetry ON, servo torque OFF, motion authority none, LED/TFT
reduced or off, Jetson gracefully shut down or in an approved low-power state, both MOS ON.

### Remaining charging-hardware gate — **FUTURE / OPEN**
One manual, attended, common-port charging session (2026-09-22/23) is now evidenced — see above.
That is **not** sufficient to call the charging path fully validated. Still open, as their own
gates: autonomous dock/contact topology, reverse-polarity protection (not yet evidenced),
complete unattended charge-acceptance and termination behaviour, charging current/thermal
behaviour under longer or repeated sessions, and future Jetson charging behaviour. Do not infer a
true FULL state from SOC alone — see §14 for the (future, unimplemented) LED indication concept,
which explicitly requires more than SOC.

---

## 9. What is actually verified today

| Fact | Status | Evidence |
|---|---|---|
| Controller runs on the protected domain, no motion | **VERIFIED** 2026-09-17/18 | `VALIDATION.md` § G3, § G3.1 |
| DALY KEY logic was `0x0055` DISABLED (why the KEY did nothing in G3) | **VERIFIED** 2026-09-19 | `VALIDATION.md` § DALY KEY live read-only validation |
| KEY logic set to `0x005A` DISCHARGE: FC06 acknowledged, read back `0x005A` | **VERIFIED** 2026-09-19 | `VALIDATION.md` § DALY KEY write |
| Charge/discharge MOS software controls both read `1` | **VERIFIED** 2026-09-19 | same |
| Setting survives a BMS power cycle | **TO_TEST** | not measured; only an immediate read-back exists, and no true DALY power cycle was performed on 2026-09-24 either |
| `B-`/`P-` bypass corrected (TECNOIOT `VIN-` now on `P-`) | **VERIFIED** 2026-09-24 | §10; fuse-in/KEY-OFF dead-circuit measurement |
| Physical KEY OFF removes the robot rails (no charger, no USB) | **VERIFIED** 2026-09-24 | §11 procedure A–E, PASS; 2026-09-24 closeout record |
| Powered no-motion regression on rewired hardware | **VERIFIED** 2026-09-24 | §11 E; 2026-09-24 closeout record |
| Manual charging, common-port electrical behaviour with KEY OFF | **VERIFIED** 2026-09-22/23 + 2026-09-24 | §8; 2026-09-24 closeout record § 4 |
| Complete charging/autonomous-dock qualification | **FUTURE / OPEN** | one attended session only; see §8 remaining gate |
| Autonomous docking/charging, Jetson behaviour | **FUTURE** | no hardware |
| External USB service/programming port (GPIO19/20, no host VBUS) | **VERIFIED** 2026-09-24 | new §15; 2026-09-24 closeout record § 5 |

---

## 10. Resolved hardware defect — `B-`/`P-` bypass (superseded 2026-09-24)

**Status: RESOLVED and VERIFIED 2026-09-24.** This section is preserved as the historical root
cause and correction record; it is no longer an open blocker.

Operator-reported, 2026-09-19/20 (**VERIFIED** by observation; instrument readings not archived):
during the first physical KEY test the DALY reported the **Discharge MOS OFF while the robot load
rail stayed powered**.

Root cause: the TECNOIOT step-down input return was tied to raw battery `B-`. Because the TECNOIOT
is a non-isolated buck, its input and output negatives are the same node, which created a return
path around the open discharge MOS:

```text
B- -> TECNOIOT VIN-/VOUT- -> ESP32 GND -> Seeed GND -> servo rail GND -> P-  (HISTORICAL, WRONG)
```

**Correction, completed and measured 2026-09-24:**

```text
TECNOIOT VIN-:  FROM battery B-  ->  TO DALY P-   (CURRENT)
```

The fix removed the split-ground concept, rather than adding isolation, exactly as planned. The
§11 procedure below is the live measurement that this correction actually removes the bypass:
full evidence in
[`09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md`](../09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md).

---

## 11. Post-rewire validation procedure — **VERIFIED**, PASS 2026-09-24

No motion occurred at any point. Each step was operator-executed; full observed values are
archived in
[`09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md`](../09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md)
§ 3. The criteria are unchanged from the original 2026-09-20 procedure design.

**A. Dead circuit — PASS.** Fuse removed then reinserted, KEY OFF throughout, USB and charger
disconnected: servo rail, TECNOIOT output and PAD+→PAD- all measured 0 V in both fuse states,
proving the TECNOIOT `VIN-`→`P-` rewire actually removes the `battery B- ↔ robot GND/P-` bridge.

**B. KEY ON — PASS.** Servo rail 12.23 V, TECNOIOT 5.18 V, PAD+→PAD- 12.23 V, ESP32-S3 booted, DALY
Charge/Discharge MOS both ON, current ≈ -0.2 A, no alarms, no servo motion.

**C. KEY OFF, USB disconnected — PASS.** Immediate shutdown: servo rail, TECNOIOT output and
PAD+→PAD- all 0 V, ESP32-S3 OFF, Discharge MOS OFF, Charge MOS remained ON. No alternate return
kept the robot alive — this is the direct proof the historical bypass is gone.

> USB was disconnected first for this step, consistent with the original procedure design: the
> ESP32 can otherwise stay alive from USB 5 V while the ROBOT_POWERED image drives the LED data pin
> toward an unpowered LED rail, and a ROBOT_POWERED image must never *boot* with the rail removed.

**D. KEY ON again — PASS.** TECNOIOT ≈ 5.3 V, servo rail ≈ 12.2 V, ESP32-S3 cold-booted normally,
no reset loop, no BMS alarm, no movement.

**E. Powered no-motion regression — PASS.** `@STATUS` (`health=BOOTING`, `power_state=RUN`,
`mode=MAINTENANCE`, `profile=ROBOT_POWERED`); BNO085 `ONLINE/REQUIRED/PASS`; DALY
`ONLINE/REQUIRED/PASS` (`pack_v=12.0 V`, `current_a=-0.2 A`, `soc=94.2 %`, `charge_mos=ON`,
`discharge_mos=ON`, `alarms=0`); servo census `PASS` (13 present-expected, 4 absent-by-design, 0
missing, 0 unexpected); `SAFE_OFF` → `VERIFIED_OFF` on all 13 installed servos
(`11 12 13 21 22 23 31 32 33 41 42 43 51`); `runtime_resets=0` throughout; no watchdog, brownout or
panic. No servo moved.

**The complete post-rewire power gate A–E is PASS.** The power architecture in §§1–2 is now
hardware-validated for the robot as built, with no charger and no USB present. §7 and §8 state
what is and is not true when a charger is connected.

---

## 12. Firmware policy for this architecture

```text
the physical KEY stays outside the ESP32
KEY logic stays DISCHARGE-only
no KEY -> Charge MOS behaviour
no ordinary runtime write to the Charge or Discharge MOS (docking included)
no generic BMS register writer
requestDischargeOff() / @SYSTEM SHUTDOWN stay fail-closed
servo TORQUE OFF is the normal actuator-safe action before charging or shutdown
a future Jetson shuts down gracefully before a human presses KEY OFF
dock/charger hardware is not modelled in firmware before it exists
```

Status: **IMPLEMENTED/OFFLINE-VALIDATED** — audited 2026-09-20 against the shipped Controller; no
source change was required. The single guarded commissioning write
(`@BMS KEY SET DISCHARGE CONFIRM`, `0x0120 := 0x005A`) is retained deliberately: with the register
now reading `0x005A`, its gate answers `ALREADY_CONFIGURED` and transmits nothing, so it can only
act again on a BMS that reads exactly `0x0055` — a replaced or factory-reset BMS, which is exactly
the re-commissioning case. See `05_Firmware/MATDOG_Controller/README.md`.

---

## 13. Future Jetson — architectural only

The Jetson belongs to the same protected domain: `B+ / P-` → a dedicated Jetson regulator →
Jetson. Never raw `B-`, and never a separate always-on ground domain.

Target dock behaviour: torque OFF → dock/charger validated → Jetson stops ROS/AI work →
filesystem sync → graceful shutdown or approved low-power state → ESP32 stays powered and
supervises BMS/charging/dock state. On departure: wake the Jetson through the future validated
integration → wait for readiness → run health/self-test gates → enable motion authority only when
safe → leave the dock.

The regulator voltage/current, the power-button/wake wiring and the shutdown-ready handshake are
**FUTURE** hardware items. They must not be invented in firmware or specified here before
hardware validation exists.

---

## 14. Long-term docked behaviour — **FUTURE**

With KEY ON at the dock the ESP32 may later enter a low-power operational state: reduced or
scheduled Wi-Fi, LED/TFT off, Jetson off, servo torque off, optionally reduced telemetry cadence,
ESP32 light-/modem-sleep where compatible. This is an optimization, not an electrical
requirement — and it must never justify reintroducing a raw-`B-` always-on path to save wake
latency.

---

## 15. External USB service/programming port — **VERIFIED** 2026-09-24

A dedicated external three-wire service connector exists alongside the onboard USB-C:

```text
GPIO19  = USB D-
GPIO20  = USB D+
GND     = USB ground
host VBUS (+5V) intentionally NOT connected
```

The ESP32-S3 is self-powered from the robot's protected TECNOIOT supply through this connector; it
is not powered from host USB. Validated 2026-09-24, using only the external connector with the
onboard USB-C disconnected: native USB enumeration (`303a:1001`, "Espressif USB JTAG/serial debug
unit", stable `/dev/serial/by-id/...-if00` path), bidirectional CDC (`@STATUS`, `@MODE STATUS`,
`@BMS STATUS`, `@IMU STATUS`, `@LED STATUS`), the `esptool` USB reset / ROM download handshake and
flash-identification read path (chip, PSRAM, flash size, no write), and correct re-enumeration
after the hard reset `esptool` performs. Full evidence:
[`09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md`](../09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md)
§ 5.

**Operational limitation, not removed by this validation:** because host VBUS is intentionally not
wired, this port cannot power the ESP32 on its own. Diagnostics/programming through it require the
robot already powered from its protected supply (normally KEY ON) or another explicitly validated
power source. With KEY OFF and no charger connected, plugging in this port alone must not power
the Controller — consistent with §11 A/C. The onboard USB-C is no longer required for normal
service/programming access, though it remains physically present on the dev board.

---

## 16. Remaining open items — **FUTURE / TO_TEST**

Not closed by the 2026-09-24 validation above:

- **BMS KEY-configuration persistence across a true DALY power cycle** — **TO_TEST**. No such power
  cycle (as opposed to a KEY toggle or main-fuse removal) has been performed or measured.
- **Autonomous dock/contact hardware** — **FUTURE**, no hardware exists.
- **Reverse-polarity protection** — not yet evidenced either way.
- **Complete unattended/autonomous charge-acceptance and termination validation** — **FUTURE**;
  only one manual, attended charging session exists (§8).
- **Future Jetson charging behaviour** — **FUTURE**, architectural only (§13).
- **Long-term automated charge-termination policy** — **FUTURE / TO_DESIGN**.
- **Charging LED-ring progress indication** — **FUTURE / TO_DESIGN, NOT IMPLEMENTED.** Target
  concept only, not built and not validated: source of truth is fresh DALY telemetry, never the
  KEY GPIO; during active charging, 12 LEDs each represent 1/12 of SOC, with completed segments
  fixed and the next segment slow-pulsing; FULL shows all-green slow breathing; a stale/invalid SOC
  shows indeterminate amber rather than a fabricated percentage; a charging fault shows a distinct
  red warning; FULL must eventually be derived from fresh telemetry (no charge alarm, cell/pack
  voltage near target, tapered current, and stability over time), never from SOC alone; the
  implementation must be non-blocking (no `delay()`) and must never imply any motion authority;
  unplugging the charger with KEY OFF naturally removes power and turns the ring off. No firmware
  for this exists.

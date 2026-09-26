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
| Battery discharge path (pack → `P-`) | available | removed |
| `B+`/`P-` robot domain, no external source connected | available | **removed — VERIFIED 2026-09-24** |
| `B+`/`P-` robot domain, charger connected across `B+`/`P-` | available | **NOT removed — the charger backfeeds it regardless of KEY state, VERIFIED 2026-09-22/23 + 2026-09-24** |
| Charging | possible | still possible (Charge MOS unaffected by KEY) |

**KEY controls the Discharge MOS only** — that is, it switches the battery's own discharge path,
not the `B+`/`P-` bus as a whole. It must never be changed to `CHARGE_AND_DISCHARGE` (`0x00AA`) and
must never be mapped to the Charge MOS.

**KEY OFF removes the battery discharge path through the DALY, not necessarily the `B+`/`P-` load
bus itself:**

- With **no external source connected**, removing the discharge path collapses the entire `B+`/`P-`
  robot domain: servo rail, TECNOIOT and the ESP32-S3 all go to 0 V. **VERIFIED 2026-09-24** (§11
  procedure A/C).
- If an **external charger is connected directly across `B+`/`P-`** (the current common-port
  topology — see §8), that charger energizes the load bus on its own, independent of the Discharge
  MOS. The ESP32-S3 and servo rail may stay powered from the charger even with KEY OFF and
  Discharge MOS OFF. **VERIFIED 2026-09-22/23 + 2026-09-24.**
- **KEY state alone must therefore never be interpreted as proof that the `B+`/`P-` bus is
  de-energized.** The only way to know is to check for an external source (charger/dock) and, when
  in doubt, measure the rails directly — see §7.

Human meaning: KEY ON = MATDOG available; KEY OFF = deliberately powered off **provided no external
source is connected** (see above). It is **not** a charging command, **not** a service isolation
device, and **not** a substitute for the removable main fuse / service disconnect.

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
1. controlled shutdown / SAFE_OFF as applicable
2. KEY OFF
3. disconnect the charger/dock and any other external power source
4. disconnect USB and any external grounds as required
5. verify the robot load rails are down (measure, do not assume)
6. remove/open the main battery service fuse / disconnect
7. re-verify the absence of unintended power/backfeed before physical service
```

**Why every external source is disconnected before the verification step, and the fuse comes
after it, not before:** the fuse's position relative to the charger's `PAD+`/`PAD-` tap point has
**not** been electrically traced or tested — §11's dead-circuit measurements (both fuse states,
0 V in both) were taken with **no charger connected**, and no evidence exists either way for
whether pulling the fuse alone would also interrupt a connected charger's contribution to `B+`/`P-`
(§8 already shows the charger backfeeds that bus independent of Discharge-MOS/KEY state, which is
a different node than the fuse). Until that is traced and tested, the fuse must not be assumed to
isolate a connected charger. The safe order is therefore to disconnect every known external source
(charger/dock, then USB) first, so that "verify the load rails are down" (step 5) is a clean
measurement against zero known sources rather than a reading that could be misinterpreted either
way; the fuse pull (step 6) is then the physical isolation added on top of an already-measured dead
circuit, and step 7 re-verifies nothing changed as a result of pulling it.

Status: **VERIFIED** 2026-09-24 for the current hardware, with no charger connected — the full
sequence (external USB unplugged, KEY OFF, rails measured down, fuse removed) was executed as the
session closeout (§6 of the 2026-09-24 closeout record) and every rail measured 0 V (§11, §3 of the
same record). This sequence has **not** been exercised end-to-end with a charger connected at step
3; do not assume step 5 reads 0 V if a charger or dock is still attached — see §8.

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
true FULL state from SOC alone — see §16 for LED V2 renderer status and the future charge-completion policy,
which requires evidence beyond SOC.

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
- **LED-ring presentation V2** — **IMPLEMENTED / OFFLINE-VALIDATED**, with focused
  **HARDWARE PASS** (2026-09-26) for BOOTING breathing, READY SOC mapping and the bounded
  SOC diagnostic/automatic resume on source `88062e1a1217f288ebcc161c6213dbb543ea6f8a`.
  At real 75.5% SOC, nine green LEDs ran from noon through eight o'clock; `@LED SOC TEST`
  filled/drained and returned automatically to nine segments at 75.2%. Servo preflight
  passed 12/12 with torque_enable=0 and authority remained NONE; no calibration motion
  occurred. BOOTING breathing passed, with no calibrated hue claim. The frozen SOC order is
  physical `{1,2,3,4,5,6,7,8,9,10,11,0}`: noon clockwise to eleven o'clock. READY shows
  `floor(clamp(BMS_REPORTED_SOC, 0, 100) * 12 / 100)` completed green segments at brightness 20;
  remaining pixels are off. The calculation never rounds upward.
- **Active charging detection and progress** — **IMPLEMENTED / OFFLINE-VALIDATED;
  live charging, reported-100% tail and real charging-fault animations remain TO_TEST.**
  The focused session used STATIONARY telemetry; `@LED SOC TEST` shows fixed bars and
  does not simulate CHARGING. Detection uses only
  cached DALY telemetry: valid sample, latest communication result OK, sample age at most the
  shared `kDalyTelemetryFreshnessMs=5000` bound. The bound reuses the existing telemetry contract
  (2s poll cadence, 750ms response deadline, allowance for a deferred poll). Cached
  `state_name=CHARGING` selects progress; KEY state is not an input. Completed segments stay
  green at 20; the next logical segment breathes green at 6..20 over 3 s. At reported 100%, only
  physical 0 (the final segment) breathes, so charging remains distinguishable. Fresh charging
  plus any nonzero DALY alarm word selects red breathing (6..60 over 2 s). Invalid/stale SOC
  instead shows subtle amber breathing (6..20), subject to higher-priority system states.
- **True charge-completion policy** — **FUTURE / TO_DESIGN**. A reserved
  `charge_complete_verified` input and all-green slow-breathing renderer exist, but the input
  has **no production producer** and stays false. SOC 100% is never proof of FULL. A future
  reviewed policy must establish fresh telemetry, no charge alarm, pack/cell voltage near
  target, tapered current, and stability over time; no thresholds are fabricated here.
- **Reserved battery warning/critical facts** — **IMPLEMENTED / OFFLINE-VALIDATED**
  presentation API only; both remain false with **no production producer** and their
  active presentations are **NOT HARDWARE-VALIDATED**. No battery thresholds were added.
- **Autonomous dock and unattended charging qualification** — **FUTURE**. LED V2 is a
  non-blocking presentation of existing facts and grants no motion or power-control authority.
  Unplugging the charger with KEY OFF still naturally removes power and turns the ring off.

Implementation and offline evidence:
[`2026-09-26_LED_STATUS_MANAGER_V2_FINAL.md`](../09_Logs/Development_Log/2026-09-26_LED_STATUS_MANAGER_V2_FINAL.md).

Focused hardware evidence and its exact flashed firmware provenance:
[`2026-09-26_LED_STATUS_MANAGER_V2_HW_VALIDATION.md`](../09_Logs/Development_Log/2026-09-26_LED_STATUS_MANAGER_V2_HW_VALIDATION.md).
The later docs-only closeout commit is not the installed firmware source. Active
`charge_complete_verified` likewise remains unvalidated on hardware and has no producer.

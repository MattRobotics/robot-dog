# MATDOG — Power States, KEY Semantics and Charging

**Canonical owner** of MATDOG's power domains, power states, the physical `KEY` button, Charge/
Discharge MOS policy, daily use, storage, service isolation and charging (manual and autonomous).

Frozen: 2026-09-20. Input decision record:
[`09_Logs/Development_Log/2026-09-20_MATDOG_POWER_KEY_CHARGING_NEXTGEN_HANDOFF.md`](../09_Logs/Development_Log/2026-09-20_MATDOG_POWER_KEY_CHARGING_NEXTGEN_HANDOFF.md).

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

Status: architecture **DECIDED**; physical conformance **BLOCKED** on the rewire in §10.

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
KEY OFF/ON rail behaviour is **BLOCKED** until the §10 rewire is done and §11 is executed.

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
| **OFF** | OFF | OFF | ON | OFF | OFF | none | OFF | optional | **BLOCKED** — needs §10 rewire + §11 |
| **BOOT** | ON | ON | ON | ON | OFF | none | boots later | OFF | **VERIFIED** (G3/G3.1, no-motion) |
| **READY/IDLE** | ON | ON | ON | ON | OFF | none | ON as needed | OFF | **VERIFIED** no-motion |
| **RUN** | ON | ON | ON | ON | enabled by authority | granted | ON | OFF | **FUTURE** — no motion capability exists |
| **DOCKING** | ON | ON | ON | ON | controlled | restricted | ON | OFF until validated | **FUTURE** |
| **CHARGING** | ON | ON | ON | ON | OFF | inhibited | OFF / low-power | ON | **FUTURE** — charger hardware absent |
| **DOCKED_LOW_POWER** | ON | ON | ON | ON, low-power | OFF | inhibited | OFF | complete/maintenance | **FUTURE** |
| **MANUAL_CHARGE_OFF** | OFF | OFF | ON | OFF | OFF | none | OFF | ON | **TO_TEST** — target; needs §10 + charging gate §8 |
| **SERVICE_ISOLATED** | OFF | OFF | context-dependent | OFF | OFF | none | OFF | disconnected, fuse open | **TO_TEST** — procedure §7 |

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

KEY OFF is a functional power-off *through the BMS*. It is **not** the trusted maintenance
isolation.

```text
1. controlled shutdown
2. KEY OFF
3. verify the load rails are down (measure, do not assume)
4. remove/open the main service fuse / disconnect
5. disconnect the charging dock / charger
6. disconnect USB and any external grounds as required
7. verify the absence of unintended power/backfeed before work
```

The removable fuse / service disconnect remains the trusted physical isolation point.
Status: **TO_TEST** as a written procedure; step 3 currently cannot be trusted until §10 is done.

---

## 8. Charging

### Manual charging with KEY OFF — **TO_TEST** (target)
```text
KEY OFF -> Discharge MOS OFF -> robot domain OFF
Charge MOS stays ON -> the battery can still be charged
```
This is the preferred state for a manually powered-off robot connected to a charger.

### Autonomous docking and charging — **FUTURE**
When MATDOG drives itself to the dock, **the KEY stays ON**: no human presses the button. The
Discharge and Charge MOS stay ON, the ESP32 stays powered, and `P-` stays available.

```text
RUN -> DOCKING -> dock pose/contact confirmed -> stop motion -> TORQUE OFF
    -> validate charger/contact conditions -> CHARGING
```
During CHARGING: ESP32 ON, DALY telemetry ON, servo torque OFF, motion authority none, LED/TFT
reduced or off, Jetson gracefully shut down or in an approved low-power state, both MOS ON.

### Open charging-hardware gate — **BLOCKED / OPEN**
The charging path must be validated as its own hardware gate before anything here is called
validated: charger CC/CV behaviour, dock/contact topology, negative return path, common-port
behaviour, Charge MOS behaviour, fuse interaction, reverse-polarity protection, charging current
and thermal behaviour. **No charging hardware evidence exists in this repository.**

---

## 9. What is actually verified today

| Fact | Status | Evidence |
|---|---|---|
| Controller runs on the protected domain, no motion | **VERIFIED** 2026-09-17/18 | `VALIDATION.md` § G3, § G3.1 |
| DALY KEY logic was `0x0055` DISABLED (why the KEY did nothing in G3) | **VERIFIED** 2026-09-19 | `VALIDATION.md` § DALY KEY live read-only validation |
| KEY logic set to `0x005A` DISCHARGE: FC06 acknowledged, read back `0x005A` | **VERIFIED** 2026-09-19 | `VALIDATION.md` § DALY KEY write |
| Charge/discharge MOS software controls both read `1` | **VERIFIED** 2026-09-19 | same |
| Setting survives a BMS power cycle | **TO_TEST** | not measured; only an immediate read-back exists |
| Physical KEY OFF removes the robot rails | **BLOCKED** | the §10 bypass made the first attempt inconclusive |
| Manual charging with KEY OFF | **TO_TEST** | no charger evidence |
| Autonomous docking/charging, Jetson behaviour | **FUTURE** | no hardware |

---

## 10. Known hardware defect — `B-`/`P-` bypass

Operator-reported, 2026-09-19/20 (**VERIFIED** by observation; instrument readings not archived):
during the first physical KEY test the DALY reported the **Discharge MOS OFF while the robot load
rail stayed powered**.

Root cause: the TECNOIOT step-down input return was tied to raw battery `B-`. Because the TECNOIOT
is a non-isolated buck, its input and output negatives are the same node, which created a return
path around the open discharge MOS:

```text
B- -> TECNOIOT VIN-/VOUT- -> ESP32 GND -> Seeed GND -> servo rail GND -> P-
```

**Required correction (operator action, not yet performed):**

```text
TECNOIOT VIN-:  FROM battery B-  ->  TO DALY P-
```

The fix is to remove the split-ground concept, **not** to add isolation. Until it is done and
measured, KEY OFF cannot be trusted to remove the robot rails, and the OFF / MANUAL_CHARGE_OFF /
SERVICE_ISOLATED states above stay unproven.

---

## 11. Post-rewire validation procedure — **TO_TEST**

No motion at any point. Each step is operator-executed; archive the evidence.

**A. Dead circuit** — fuse/disconnect open, USB disconnected: continuity map; confirm TECNOIOT
`VIN-` is on `P-`; prove `battery B- ↔ robot GND/P-` is **not** bridged by the TECNOIOT, ESP32,
Seeed driver, USB or any peripheral while the discharge path is open.

**B. KEY ON** — Discharge MOS ON, Charge MOS ON, servo rail correct, TECNOIOT 5 V correct, ESP32
boots normally, BMS current plausible, no alarms.

**C. KEY OFF, USB disconnected** — Discharge MOS OFF, Charge MOS ON, servo rail collapses,
TECNOIOT 5 V collapses, ESP32 off, no alternate return keeps the robot alive.

> USB must be disconnected first: the ESP32 can otherwise stay alive from USB 5 V while the
> ROBOT_POWERED image drives the LED data pin toward an unpowered LED rail, and a ROBOT_POWERED
> image must never *boot* with the rail removed.

**D. KEY ON again** — rails restore, ESP32 cold-boots normally, no reset loop, no BMS alarm, no
movement.

**E. Powered no-motion regression** — `@STATUS`, BMS read-only status, BNO085 health, servo
census, `SAFE_OFF` readback, no watchdog/brownout/panic, USB open/closed behaviour as relevant.

Only after A–E pass may the power architecture be called hardware-validated.

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

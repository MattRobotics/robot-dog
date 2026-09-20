# MATDOG — Power, KEY, Charging & NextGen Controller Architecture Handoff

**Date:** 2026-09-20  
**Project:** MATDOG / `MattRobotics/robot-dog`  
**Purpose:** Freeze the revised power-domain, DALY KEY, charging, daily-use and future Jetson power-state decisions, then guide the final documentation/firmware closeout of the current NextGen Controller phase.

---

## 1. Executive decision

The previous idea of keeping the ESP32-S3 permanently powered from the raw battery-side `B-` domain is **ABANDONED**.

MATDOG returns to a single protected robot power domain:

```text
Battery B+  -> common positive distribution
Battery B-  -> DALY B- ONLY

DALY protected system return = P-

ALL robot loads use B+ / P-
```

This includes:

```text
servo rail
TECNOIOT step-down
ESP32-S3
BNO085
Seeed Bus Servo Driver logic
XY-017 / ARCELI RS485 interface
LED / auxiliary logic as appropriate
future TFT eye electronics
future Jetson power converter / Jetson domain
```

No ordinary robot load is allowed to return directly to battery `B-`.

The TECNOIOT can remain the 3S -> 5 V step-down **provided its VIN- is connected to DALY P- rather than battery B-**.

No isolated DC/DC converter is required for the ESP32 in the definitive architecture.

---

## 2. Why the architecture changed

The temporary physical architecture had:

```text
TECNOIOT VIN+ -> battery B+
TECNOIOT VIN- -> battery B-

TECNOIOT 5 V -> ESP32
ESP32 GND -> Seeed GND -> servo rail GND -> DALY P-
```

Because the TECNOIOT is a non-isolated buck, its negative input/output belong to the same electrical domain.

This created an unintended bypass:

```text
B-
-> TECNOIOT VIN-/VOUT-
-> ESP32 GND
-> Seeed GND
-> servo rail GND
-> P-
```

Therefore the DALY discharge MOS could report OFF while the robot load rail still had a return path.

The final correction is not to add isolation. It is to remove the split ground-domain concept:

```text
TECNOIOT VIN- -> P-
```

and keep `B-` exclusively on the battery side of the DALY.

---

## 3. DALY KEY decision — frozen

The physical bistable/self-locking button under the MATDOG logo remains connected directly to the DALY `KEY` input.

It is not connected to an ESP32 GPIO.

The BMS KEY logic has been configured to:

```text
0x0120 = 0x005A = DISCHARGE
```

The live write was sent once, acknowledged and read back as `0x005A`.

Target and now accepted semantics:

```text
KEY ON
-> Discharge MOS ON
-> Charge MOS ON
-> P- protected robot domain available
-> robot can boot and operate

KEY OFF
-> Discharge MOS OFF
-> Charge MOS remains ON
-> P- load domain is removed
-> robot electronics on B+/P- turn off
-> charging path remains available
```

Important distinction:

```text
KEY controls DISCHARGE only.
KEY must NOT be changed to CHARGE_AND_DISCHARGE.
```

The separate DALY Charge MOS control is not part of the normal user ON/OFF action.

---

## 4. Charge MOS decision — frozen

Normal policy:

```text
Charge MOS = normally ON
```

Do not add normal-runtime logic that toggles the Charge MOS simply to start or stop ordinary charging.

Do not map the physical KEY to the Charge MOS.

Do not enable or introduce generic writes to the DALY MOS-control registers merely because they exist.

The charger/dock owns normal charge start/stop control.

The DALY Charge MOS remains a BMS protection / upper-level isolation mechanism.

Possible future software use of Charge MOS OFF is restricted to a separately designed and validated fail-safe or service function.

Until such a phase is explicitly opened:

```text
no new write to 0x0121
no new write to 0x0122
no generic BMS register writer
```

Existing static-audit prohibitions around those writes should remain or become stricter.

---

## 5. Human-facing power semantics

The physical KEY button is the normal user power switch.

Its meaning is:

```text
KEY ON  = MATDOG available / powered domain enabled
KEY OFF = MATDOG deliberately powered off
```

It is NOT the normal command for entering an autonomous charging session.

It is NOT a service isolation device.

It is NOT a substitute for the removable main fuse / service disconnect.

---

## 6. Current robot — normal operation

Without Jetson installed:

```text
USER presses KEY ON
-> DALY enables discharge path
-> P- becomes available
-> TECNOIOT powers ESP32
-> ESP32 boots
-> BNO085 / DALY / servo bus / peripherals initialize
-> self-test / health gates
-> motion remains disallowed until the existing Controller safety gates permit it
```

Normal shutdown:

```text
robot enters a stable rest pose
-> motion command stream stops
-> all servo torque is forced OFF / verified safe
-> user presses KEY OFF
-> discharge MOS OFF
-> P- robot domain off
-> ESP32, servo rail and robot auxiliaries off
```

Do not use DALY discharge MOS as a substitute for normal `TORQUE OFF`.
Actuator shutdown is performed first; power-domain removal is the final user power action.

---

## 7. Daily shutdown

### Current ESP32-only robot

Preferred daily shutdown:

```text
1. stop motion
2. stable rest pose
3. SAFE_OFF / torque OFF
4. KEY OFF
```

This removes normal robot loads from the battery and avoids a permanently powered ESP32 branch.

### Future Jetson robot

The physical power architecture remains the same, but shutdown order changes because Linux must be shut down cleanly:

```text
1. stop autonomous behavior / motion
2. stable rest pose
3. servo torque OFF
4. request graceful Jetson software shutdown
5. wait for verified Jetson shutdown / safe-to-remove-power state
6. KEY OFF
7. P- domain removed
```

KEY OFF remains available as an emergency hard power cut, but it is not the preferred normal Jetson shutdown path.

---

## 8. Autonomous docking and charging — key decision

When MATDOG autonomously drives to the charging dock:

```text
KEY STAYS ON
```

No human is expected to press the physical button.

Therefore:

```text
Discharge MOS stays ON
Charge MOS stays ON
ESP32 stays powered
P- stays available
```

The robot must distinguish:

```text
electrically powered
!=
motion enabled
```

The charging state is an operational low-power state, not a full hardware shutdown.

Conceptual sequence:

```text
RUN
-> DOCKING
-> dock pose/contact confirmed
-> stop motion
-> TORQUE OFF
-> validate charger/contact conditions
-> CHARGING
```

During `CHARGING`:

```text
ESP32            ON
DALY telemetry   ON
servo torque     OFF
motion authority NONE / inhibited
LED/TFT          reduced or OFF as appropriate
Jetson           future: graceful shutdown or low-power state
Charge MOS       ON
Discharge MOS    ON
KEY              remains ON
```

This allows MATDOG to remain autonomously manageable while docked and later wake/restart mission functions without any human pressing KEY.

---

## 9. Autonomous charging — future Jetson Orin Nano Super

The future Jetson belongs to the same protected robot domain:

```text
B+ / P-
-> appropriate dedicated Jetson regulator
-> Jetson
```

Do not return the Jetson supply to raw battery `B-`.

Do not create a separate raw-battery always-on ground domain.

Target dock behavior:

```text
robot reaches dock
-> torque OFF
-> dock/charger validated
-> Jetson stops ROS / AI / high-level jobs
-> filesystem sync
-> graceful Jetson shutdown or approved low-power state
-> ESP32 remains powered
-> ESP32 supervises BMS / charging / dock state
```

When a mission requires departure:

```text
ESP32 detects/request receives wake/depart condition
-> wake/power Jetson using the future validated integration
-> wait for Jetson readiness
-> run robot health/self-test gates
-> enable motion authority only when safe
-> leave dock
```

The exact Jetson regulator voltage/current, power-button/wake wiring and shutdown-ready handshake are future hardware-validation items and must not be invented in the current firmware.

---

## 10. Manual charging with KEY OFF

The architecture intentionally permits this target:

```text
KEY OFF
-> Discharge MOS OFF
-> robot domain OFF
-> Charge MOS remains ON
-> battery can still be charged through the validated charging topology
```

This is the preferred state for a manually powered-off robot that is connected to a charger.

However, the complete MATDOG charging path must still be validated as a separate hardware gate:

```text
charger CC/CV behavior
dock/contact topology
negative return path
common-port behavior
Charge MOS behavior
fuse interaction
reverse-polarity protection
charging current
thermal behavior
```

Do not mark the charging hardware `VALIDATED` until real charger tests exist.

---

## 11. Long-term docked behavior

With KEY ON at the autonomous dock, the ESP32 may enter a low-power operational state.

Future policy may include:

```text
Wi-Fi reduced / scheduled
LED/TFT OFF
Jetson OFF
servo torque OFF
telemetry at reduced cadence if justified
ESP32 light-sleep / modem-sleep if compatible
```

This is an optimization, not an electrical requirement.

Do not reintroduce a raw-`B-` always-on power path to save wake latency.

---

## 12. Storage

Short / normal storage:

```text
KEY OFF
```

Long storage:

```text
KEY OFF
+
service disconnect / removable fuse removed as appropriate
```

Battery storage SOC and LiPo maintenance remain separate battery-management procedures.

---

## 13. Maintenance / service isolation

KEY OFF is a functional power-off command through the BMS.

It is not the trusted maintenance isolation.

For electrical/mechanical service:

```text
1. controlled shutdown
2. KEY OFF
3. verify load rails down
4. remove/open main service fuse/disconnect
5. disconnect charging dock/charger
6. disconnect USB/external grounds as required
7. verify absence of unintended power/backfeed before work
```

The removable fuse/service disconnect remains the trusted physical isolation point.

---

## 14. Emergency behavior

Emergency priorities differ from normal shutdown.

If immediate power removal is necessary:

```text
KEY OFF
```

may be used even if a future Jetson has not completed graceful Linux shutdown.

If electrical isolation is required:

```text
service disconnect / fuse removal
```

remains the stronger action.

Documentation must clearly distinguish:

```text
logical stop
servo torque OFF
normal software shutdown
KEY power-off
service isolation
emergency hard power removal
```

---

## 15. Firmware implications — minimal, lean policy

Before editing source, audit the current implementation.

The new architecture DOES NOT justify adding broad new DALY write capability.

Mandatory firmware policy:

1. Keep the physical KEY external to ESP32.
2. Keep KEY logic target `DISCHARGE` only.
3. Do not add KEY -> Charge MOS behavior.
4. Do not add normal runtime writes to Charge MOS or Discharge MOS simply to support docking.
5. Keep generic raw BMS writes forbidden.
6. Keep `requestDischargeOff()` / `@SYSTEM SHUTDOWN` fail-closed unless a later explicitly designed autonomous-power-cut phase authorizes it.
7. Autonomous docking must initially leave KEY / Discharge MOS ON.
8. Servo `TORQUE OFF` is the normal actuator-safe action before charging or shutdown.
9. Future Jetson shutdown is graceful software shutdown before physical KEY OFF.
10. Do not pretend dock/charger hardware exists in firmware before the required signals and hardware have been defined.

### Commissioning write path

The one guarded `0x0120 := 0x005A` path was created to configure the real BMS.

Audit whether it should remain in the production Controller.

Preferred production direction:

```text
retain read-only KEY configuration/status capability
minimize or remove one-time commissioning write capability from normal runtime
```

But do not remove it blindly until:
- the live evidence is archived;
- persistence/recovery requirements are understood;
- there is a deliberate replacement/commissioning procedure.

Any decision here requires tests and documentation.

---

## 16. Power-state documentation to create

If no canonical document already exists, create:

```text
04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md
```

It should be the canonical owner for:

```text
power domains
B- vs P-
KEY semantics
Charge MOS semantics
daily ON/OFF
normal operation
manual shutdown
storage
maintenance
emergency isolation
manual charging
autonomous docking
autonomous charging
future Jetson behavior
future low-power dock state
known validation gaps
```

Cross-link it from at minimum:

```text
README.md
01_Docs/02_Architecture/ARCHITECTURE.md
01_Docs/02_Architecture/ROADMAP.md
04_Electronics/README.md
05_Firmware/MATDOG_Controller/README.md
05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md
05_Firmware/MATDOG_Controller/VALIDATION.md
05_Firmware/MATDOG_Controller/CHANGELOG.md
```

Historical handoffs must not be rewritten to falsify history.
Where an old document contains the abandoned ESP32-on-B- or always-on concept, mark it historical/superseded or cross-link the current canonical decision.

---

## 17. Required documentation state machine

The canonical power document should include a state table similar to:

| State | KEY | Discharge MOS | Charge MOS | ESP32 | Servo torque | Future Jetson | Charger |
|---|---|---|---|---|---|---|---|
| OFF | OFF | OFF | ON | OFF | OFF | OFF | optional/manual |
| BOOT | ON | ON | ON | ON | OFF | boot later | OFF |
| READY/IDLE | ON | ON | ON | ON | OFF | ON as needed | OFF |
| RUN | ON | ON | ON | ON | enabled by authority | ON | OFF |
| DOCKING | ON | ON | ON | ON | controlled | ON | OFF until validated |
| CHARGING | ON | ON | ON | ON | OFF | OFF/low-power future | ON |
| DOCKED_LOW_POWER | ON | ON | ON | ON low-power | OFF | OFF | complete/maintenance |
| MANUAL_CHARGE_OFF | OFF | OFF | ON | OFF | OFF | OFF | ON |
| SERVICE_ISOLATED | OFF | OFF | context-dependent | OFF | OFF | OFF | disconnected; fuse open |

Do not claim states are hardware-validated if they are only architectural targets.

---

## 18. Hardware correction that remains required

Before declaring the power architecture closed, physically correct:

```text
TECNOIOT VIN-:
FROM battery B-
TO   DALY P-
```

Then verify there is no remaining normal load return to `B-`.

Dead-circuit continuity audit should prove:

```text
battery B- -> robot ground / P-
```

is not bridged by TECNOIOT, ESP32, Seeed, USB, charger or another peripheral when the DALY discharge path is open.

USB/external-ground scenarios must be considered separately.

---

## 19. Required post-rewire live validation

No motion.

At minimum:

### A. Dead-circuit
- fuse/open disconnect as required;
- USB disconnected;
- continuity map;
- confirm TECNOIOT VIN- is P-;
- confirm no B- bypass.

### B. KEY ON
- discharge MOS ON;
- charge MOS ON;
- protected rail normal;
- TECNOIOT output correct;
- ESP32 normal boot;
- BMS current measurement plausible;
- no alarms.

### C. KEY OFF
- USB disconnected;
- discharge MOS OFF;
- charge MOS ON;
- servo rail collapses;
- TECNOIOT 5 V collapses;
- ESP32 off;
- no alternate ground keeps robot alive.

### D. KEY ON again
- protected rails restore;
- ESP32 cold boots normally;
- no reset loop;
- no BMS alarm;
- no motion.

### E. Powered no-motion regression
After correction, repeat the relevant Controller powered baseline:
- `@STATUS`;
- BMS read-only status;
- BNO085 health;
- servo population/census;
- `SAFE_OFF` readback;
- no unexpected movement;
- no watchdog/brownout/panic;
- USB open/closed behavior if relevant.

Archive exact evidence.

---

## 20. Charging validation — separate gate

Charging is not required to fake-close the Controller phase if charger hardware is not ready.

It must remain explicitly OPEN until tested.

When available, validate separately:

### Manual charge, KEY OFF
Expected:
```text
Discharge MOS OFF
Charge MOS ON
robot domain OFF
positive battery charge current
```

### Autonomous charge, KEY ON
Expected:
```text
Discharge MOS ON
Charge MOS ON
ESP32 ON
torque OFF
charger active
```

Future Jetson:
```text
graceful shutdown / low-power before long charging dwell
```

---

## 21. Repository / Git execution policy

Claude must begin by discovering reality; do not assume the historical branch/HEAD.

Run at minimum:

```bash
cd /home/matteo-manicardi/MATDOG/github/robot-dog
git status --short --branch
git branch -vv
git log --oneline --decorate -12
git worktree list
git remote -v
git fetch --prune origin
git rev-parse HEAD
git rev-parse origin/main
```

Historical hints only, to be verified:
- main had been `dd5746c...`;
- DALY KEY feature work used `feat/daly-key-readonly-probe`;
- relevant implementation commits included `2fd2b80...` and `6322563...`;
- the single live KEY config write was later executed successfully and read back as `0x005A`.

Do not reset, rebase destructively, force-push, delete branches/worktrees, rewrite tags, or overwrite evidence.

---

## 22. Required Claude execution scope

Claude is authorized to perform autonomously:

```text
repository/source audit
documentation audit
new canonical power-state documentation
cross-link/update stale current docs
minimal firmware changes genuinely required by the frozen architecture
host/offline tests
static audits
both Controller builds
manifest/provenance checks
viewer/regression checks where current project policy requires them
commits on the current feature branch
normal push of that branch
PR preparation / update
```

Hardware power-state changes remain operator actions.

No physical robot motion is authorized by this handoff.

No generic DALY writes are authorized.

No Charge MOS / Discharge MOS direct software commands are authorized.

No merge to `main` and no new release tag unless Matteo explicitly authorizes the final merge/tag after review.

---

## 23. Acceptance criteria for this closeout

The current NextGen power/KEY update may be considered ready for final review only when all are true:

```text
[ ] canonical B+/P- architecture documented
[ ] abandoned B--powered ESP32 architecture explicitly superseded
[ ] KEY = DISCHARGE-only semantics documented
[ ] Charge MOS normal policy documented
[ ] daily-use UX documented
[ ] manual OFF/storage/service behavior documented
[ ] autonomous docking/charging behavior documented
[ ] future Jetson behavior documented without inventing unvalidated hardware
[ ] firmware audited against these decisions
[ ] no unnecessary new DALY runtime writes introduced
[ ] static audits/tests PASS
[ ] Controller builds PASS
[ ] repo clean after commits
[ ] feature branch pushed
[ ] PR/diff reviewed for stale contradictory documentation
```

Hardware closure additionally requires:

```text
[ ] TECNOIOT VIN- moved from B- to P-
[ ] no B-/P- bypass in dead-circuit audit
[ ] KEY OFF actually removes robot rails
[ ] KEY ON cold-boots Controller normally
[ ] powered no-motion regression PASS
```

Charging validation remains a separately identified gate if the charging hardware is not ready.

---

## 24. Deliverable expected from Claude

Return one concise final report containing:

```text
GIT_BASELINE
FILES_ADDED
FILES_MODIFIED
FIRMWARE_CHANGES
DOCUMENTATION_CHANGES
TESTS
BUILD_IDENTITIES
LIVE_TESTS_REQUIRED
OPEN_CHARGING_GATES
COMMIT(S)
REMOTE_BRANCH
PR_STATUS
RESIDUAL_RISKS
NEXT_SINGLE_OBJECTIVE
```

Every statement must be classified as:
- VERIFIED,
- IMPLEMENTED / OFFLINE-VALIDATED,
- TO_TEST,
- FUTURE,
- or BLOCKED.

Do not convert architectural intent into fake validation evidence.

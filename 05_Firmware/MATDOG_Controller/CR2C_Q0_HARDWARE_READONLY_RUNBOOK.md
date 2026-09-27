# CR2-C — FIRST CURRENT-INSTALLATION Q0 HARDWARE CAPTURE

**Prepared:** 2026-09-27  
**Branch:** `feat/calibration-readiness-v1`  
**Status:** PREPARED / NOT AUTHORIZED / NOT EXECUTED  
**Gate type:** powered, read-only servo evidence acquisition  
**Motion:** FORBIDDEN  
**Torque ON:** FORBIDDEN  
**GoalPosition:** FORBIDDEN  
**EEPROM / PositionOffset / CalibrationOfs write:** FORBIDDEN  
**Actuator authority:** NONE  
**Output:** 12 ephemeral `Q0Evidence CANDIDATE` records only

This runbook is the first hardware consumer of CR2-A + CR2-B. It does not authorize itself.
A powered session and the application-only flash required to reach this firmware still require
explicit operator authorization under the existing G3/F0 rules.

---

## 1. Entry state

CR2-A is offline validated.

CR2-B is offline + build validated:

```text
functional firmware SHA:
  2e5cbfa43378ed8d0e76e1c2a942886ef34a89db

CR2-B host suite:
  467 / 467 PASS

all host suites:
  PASS

USB_ONLY build:
  PASS

ROBOT_POWERED build:
  PASS

audit-only follow-up SHA:
  9ff046f0302928dfd2833d848206f3d66ca75228

static audit:
  114 source files / PASS
```

Subsequent documentation/audit-state commits do not change the Controller's CR2-B behaviour.

The source defaults remain:

```text
MATDOG_ACTIVE_HARDWARE_PROFILE = USB_ONLY
MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED = 0
OTA ingest source default = 0
ActuatorRuntime backend = nullptr
```

A CR2-C run changes none of those defaults.

---

## 2. What CR2-C is intended to establish

Only:

1. one current formal 12-leg population evidence bundle;
2. twelve current raw q0 candidates measured from the present mechanical installation;
3. the measured within-session encoder spread of each candidate;
4. confirmation that every q0 observation occurred with `TorqueEnable == 0`;
5. exact binding to current joint / expected physical unit / bus mapping / Geometry V5 provenance.

It does **not** establish:

```text
q0 ACCEPTED
q0 PROMOTED
JointTransform admitted
contact endpoints
runtime motion limits
stand eligibility
hardware_motion_authorized
```

Those belong to later gates.

---

## 3. Physical safety envelope

Before any powered session:

- operator physically present;
- robot mechanically supported so no leg is load-bearing and no joint can fall under gravity;
- leg workspace clear;
- main fused disconnect physically reachable;
- charger and unrelated external sources disconnected unless the specific power topology under test
  explicitly requires them;
- polarity/fuse/protection already verified;
- controller recovery/data path available;
- no second tool connected to the ST3215 bus.

The nominal q=0 pose is established **manually**. Servo torque must not be used to hold it.

Use the current CAD/URDF calibration pose and physical square/jigs as the reference. The operator
placement has finite angular uncertainty; encoder repeatability during this capture measures
stability only, not pose accuracy.

Global stop conditions:

```text
any unexpected motion
any servo TorqueEnable != 0 after SAFE_OFF
missing/unexpected/duplicate servo identity
preflight/profile mismatch
repeated reset
power instability
wiring uncertainty
mechanical pose visibly drifting
q0 capture failure
```

On a stop condition: cease the procedure, preserve the transcript, use the physical disconnect if
needed. Do not compensate a failure by widening limits, writing PositionOffset, re-IDing a servo,
or enabling torque.

---

## 4. Flash / recovery prerequisite

CR2-C requires the current `ROBOT_POWERED` image because the source-default `USB_ONLY` image
correctly refuses the live capture.

Use the already-reviewed recovery/flash process, not a new path:

1. prove the controller/service power topology for the session;
2. confirm USB enumeration;
3. take a **fresh 16 MiB full-flash read-back** immediately before flashing and record size,
   SHA256 and recovery manifest;
4. rerun static/offline gates on the exact clean HEAD;
5. build:
   ```bash
   MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh
   ```
6. verify the build manifest says:
   ```text
   HARDWARE_PROFILE=ROBOT_POWERED
   SOURCE_STATE=CLEAN
   SOURCE_COMMIT=<exact HEAD>
   OTA_INGEST_ENABLED=0
   ```
7. only after the explicit flash authorization, flash application partition only:
   ```bash
   MATDOG_FLASH_PROFILE=ROBOT_POWERED \
   MATDOG_FLASH_BACKUP=<fresh-backup-path> \
   MATDOG_FLASH_BACKUP_MANIFEST=<fresh-backup-path>.manifest.txt \
   scripts/flash_app_only.sh
   ```

Never use the normal Arduino full upload for this gate.

The flash and the powered q0 session are separate authorization decisions. A successful flash does
not authorize q0 acquisition automatically.

---

## 5. Runtime connection

Use the already-proven no-reset native USB CDC method from
`H0_LEG_PREFLIGHT_RUNBOOK.md`:

- one persistent connection;
- do not toggle DTR/RTS;
- do not change line speed;
- flush the first dropped write with a bare newline;
- log the complete raw transcript;
- no `esptool` while the runtime session is active.

Before servo traffic:

```text
@STATUS
@MODE STATUS
@AUTHORITY STATUS
@CALIBRATION STATUS
@CALIBRATION Q0 STATUS
```

Required:

```text
MODE=MAINTENANCE
actuator authority = NONE
hardware motion = BLOCKED
q0 capture = IDLE or previous terminal state
runtime reset count = 0
```

If mode is not MAINTENANCE, stop and review before changing it.

---

## 6. Force the safe servo state before manual placement

With the robot supported and no hands in a pinch point, issue one at a time for the 12 leg joints:

```text
@SERVO SAFE_OFF 11
@SERVO SAFE_OFF 12
@SERVO SAFE_OFF 13
@SERVO SAFE_OFF 21
@SERVO SAFE_OFF 22
@SERVO SAFE_OFF 23
@SERVO SAFE_OFF 31
@SERVO SAFE_OFF 32
@SERVO SAFE_OFF 33
@SERVO SAFE_OFF 41
@SERVO SAFE_OFF 42
@SERVO SAFE_OFF 43
```

Every result must be:

```text
VERIFIED_OFF
```

Anything else is a stop condition.

No neck servo is part of q0 leg capture.

---

## 7. Manually establish nominal URDF q=0

After Torque OFF has been verified:

1. manually place each leg in the current nominal URDF q=0 calibration pose;
2. use the physical square/jigs rather than encoder 2048 as the placement reference;
3. support the mechanism so gravity cannot change the pose when released;
4. do not touch the legs once the capture starts.

Raw 2048 is not the target of this placement and is not a pass/fail criterion.

---

## 8. Initial diagnostic capture parameters

For the **first** current-installation capture, use:

```text
samples_per_joint = 9
stability_ticks   = 16
```

Rationale:

- nine samples provide a robust odd-sample circular median without making the session large;
- 16 ticks is approximately 1.4 degrees at 4096 ticks/revolution;
- this number is **only a within-session stability guard**;
- it is **not** the q0 plausibility tolerance and is **not** an acceptance window.

Do not widen the value automatically if the session fails. A failure first means the mechanical
pose, support, wiring or servo stability must be inspected.

Command:

```text
@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE
```

After this command:

- do not send `@SERVO SCAN`, `@SERVO CENSUS`, `@SERVO PREFLIGHT` or `@SERVO READ`;
- CR2-B owns the diagnostic sequence and will run its own fresh census + fresh preflight;
- `@SERVO SAFE_OFF` remains available as an independent safety de-escalation;
- do not touch the robot until the transaction reaches COMPLETE or FAILED.

---

## 9. Expected successful result

CR2-B must internally produce:

```text
formal current population evidence = PASS
observed leg slots                  = 12 / 12
q0 candidates complete             = 12 / 12
capture state                       = COMPLETE
capture failure                     = NONE
```

Then:

```text
@CALIBRATION Q0 STATUS
```

must print 12 records, one per leg joint, each carrying:

```text
bus
leg
joint
expected physical unit
raw q0 tick
measured spread
sample count = 9
state = CANDIDATE
estimator = MANUAL_ZERO_POSE
```

and the terminal statement must continue to say, in substance:

```text
accepted=NO
promoted=NO
transform_admitted=NO
motion_authorized=NO
```

If the transaction is FAILED, do not rerun blindly. Preserve the exact failure code and transcript.

---

## 10. Post-capture closeout

After a successful capture:

```text
@STATUS
@AUTHORITY STATUS
@CALIBRATION STATUS
@CALIBRATION Q0 STATUS
```

Required:

- no runtime reset;
- authority remains NONE;
- hardware motion remains BLOCKED;
- no servo motion occurred;
- q0 evidence remains CANDIDATE only.

Then return the system to the normal safe powered-off state using the already-validated physical
power procedure.

Do not merge, promote or write calibration data during the hardware session.

---

## 11. Evidence package

Create:

```text
09_Logs/Validation_Reports/Calibration_Q0_CR2C_<YYYY-MM-DD>/
  README.md
  session_transcript.txt
  preflash_backup.manifest.txt
  build_manifest.txt
  q0_status.txt
  status_open.txt
  status_close.txt
  SHA256SUMS
```

The raw transcript is authoritative and must remain unedited.

Record:

- exact source HEAD;
- application SHA256;
- build profile;
- backup SHA256;
- device identity;
- pack voltage at open/close;
- manual alignment method;
- mechanical support/jigs used;
- the 12 q0 ticks;
- all 12 measured spreads;
- final CR2-C verdict.

---

## 12. CR2-C PASS semantics

A CR2-C PASS means only:

```text
current 12-joint population: formally observed in this session
current 12 q0 values: measured as candidates
within-session stability: measured
torque during q0 samples: confirmed OFF
geometry/identity provenance: current
```

It still does not authorize motion.

The next engineering task after a clean CR2-C capture is analysis of the twelve measured q0 values
and spreads, followed by derivation of the current q0 plausibility/acceptance policy. That is the
beginning of CR3, not part of this hardware capture.

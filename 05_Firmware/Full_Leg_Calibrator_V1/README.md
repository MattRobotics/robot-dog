# MATDOG Full Leg Calibrator V1

ESP32-S3-native calibration software for the 12 MATDOG leg servos. The
ESP32-S3 owns the ST3215 bus, the motion state machine, every servo write and
every live safety decision. NormaCore Station is not in the control path.

**Current boundary:** the resumed completion was software-only. Hardware
touched in this completion: **NONE**. The current source and its final H0 image
remain unflashed. An older H0-only image passed a no-servo smoke test on
2026-08-28; that historical result does not validate the current image. The
next permitted hardware activity is a separately authorized ESP32-only H0 flash
and smoke test with servo power OFF. H1 and later are prohibited and have not
been executed.

Final offline verification: **286 calibrator-relevant tests and 606 full-suite tests pass with zero failures and zero errors**, and all 8 H0-H6 build configurations compile warning-free. See [Verification state](#verification-state).

See the [hardware validation handoff](../../09_Logs/Validation_Reports/Full_Leg_Calibrator_V1/MATDOG_FULL_LEG_CALIBRATOR_V1_HARDWARE_VALIDATION_HANDOFF.md).

## Architecture

```text
ASUS host runner / evidence manager
   06_Software/Matdog_Core/calibration/matdog_full_leg_calibrator_runner.py
        ↕ one persistent USB CDC connection
ESP32-S3 Full Leg Calibrator firmware
   05_Firmware/Full_Leg_Calibrator_V1/matdog_full_leg_calibrator_v1/
        ↕ UART1, GPIO17 TX / GPIO18 RX, 1 Mbps
Seeed Bus Servo Driver
        ↕ ST3215 serial bus
12 leg servos (ids 11,12,13,21,22,23,31,32,33,41,42,43)
```

The host can request only named semantic operations. It cannot stream
GoalPosition, compose a register write or lift a firmware gate. The firmware
does not use Station or Station Python bindings.

### One persistent physical session

H1–H6 evidence is RAM-only and is valid only inside one firmware boot, one host
lease and one uninterrupted USB connection:

```text
open link once
  → read boot/build identity
  → @SESSION_BEGIN <host nonce>
  → H1 census
  → H2 all-joint manual q0 capture
  → semantic direction witnesses
  → optional bootstrap approval
  → authorized H3/H4/H5/H6 operations
  → SAFE_OFF verification
  → @SESSION_END
  → close link
```

The runner's `session` mode keeps this connection open, including across an
operator `pause`. Separate one-shot H2–H6 commands are refused because opening
a new serial connection resets the ESP32-S3 and destroys the evidence they
would claim to consume.

The firmware reports a boot-unique session id, the active host nonce and a
session generation. Reset, reconnect, a changed identity, a new census, a new
manual-q0 capture or a fault invalidates the appropriate downstream evidence.
A reset or connection loss never resumes a physical session: start over from a
new lease and census.

## Files

| File | Role |
|---|---|
| `matdog_full_leg_calibrator_v1/matdog_full_leg_calibrator_v1.ino` | ST3215 transport, hardware adapter, parser, runtime/session safety |
| `matdog_full_leg_calibrator_v1/flc_stage_config.h` | single fail-closed build-stage/bootstrap switch and build identity |
| `matdog_full_leg_calibrator_v1/flc_contact_detector.h` | shared observation guards and contact decision, pure C++ |
| `matdog_full_leg_calibrator_v1/flc_calibration_engine.h` | shared characterization/calibration/parking/orchestration engine, pure C++ |
| `matdog_full_leg_calibrator_v1/flc_leg_plan.h` | generated 24-endpoint Geometry Compiler V5 execution plan |
| `06_Software/Matdog_Core/calibration/generate_flc_leg_plan.py` | deterministic generator and byte-exact `--check` |
| `tests/flc_detector_harness.cpp` | native detector harness |
| `tests/flc_engine_harness.cpp` | native shared-engine harness with simulated endstops/faults |
| `tools/build_stage.sh` | supported offline H0–H6 image builder; upload is a distinct hardware action |
| `tools/check_reproducible_build.sh` | proves two cold-cache builds of one commit are byte-identical |

The detector and engine headers compile into both the firmware and native host
harnesses. H6 calls the same production orchestrator exercised by the harness;
there is no separate H6 motion implementation and no Python reimplementation
of its safety decisions.

## Host connect contract

The host does **not** wait for the `FULL_LEG_CALIBRATOR_READY` banner to decide
it is connected. With the supported FQBN (`USBMode=hwcdc,CDCOnBoot=cdc`) the
ESP32-S3 does not reset when the port is opened — hardware evidence: two
consecutive opens reported the same `BOOT_SESSION_ID` and emitted nothing — and
the firmware prints its banner once in `setup()` without waiting for a host. A
connect path that required the banner could therefore only succeed inside the
~1.5 s window after a physical reset.

Connecting instead does:

```text
open port → drain and RECORD any buffered startup text
          → framed @STATUS handshake
          → verify FIRMWARE_NAME / PROTOCOL_ID / BOOT_SESSION_ID
          → @SESSION_BEGIN
```

The banner keeps its other meaning intact: observed *during* an active session
it is still proof the board reset underneath the host, and the session fails
closed. Buffered startup text is recorded in the session evidence, never
discarded — a banner that was thrown away cannot later be reasoned about.

## Protocol and evidence lifecycle

Protocol id is `FLC1`; scope is
`CALIBRATOR_LOCAL_NOT_FINAL_RUNTIME_PROTOCOL`. This is not the final MATDOG
runtime host↔ESP32 protocol.

| Command | Minimum stage | Motion/write effect |
|---|---:|---|
| `@STATUS` | H0 | read-only build, boot, lease, epoch and gate state |
| `@SESSION_BEGIN <8-hex-host-id>` | H0 | SAFE_OFF, clear volatile evidence, acquire lease |
| `@SESSION_END` | H0 | SAFE_OFF, invalidate lease/evidence |
| `@CENSUS` | H1 | read-only 12-servo identity/profile census; starts a fresh epoch |
| `@CAPTURE_Q0 <n>` | H2 | torque-OFF all-joint manual-pose sample transaction |
| `@WITNESS_DIRECTION <id> <semantic> CONFIRM` | H2 | records explicit current-build semantic witness; no motion |
| `@APPROVE_BOOTSTRAP CONFIRM` | H3 | arms the conservative envelope for the current lease/epoch only |
| `@CHARACTERIZE_JOINT <id>` | H3 | bounded encoder response/contact characterization |
| `@CALIBRATE_JOINT <id>` | H4 | generated-plan endpoint execution; RAM evidence only |
| `@CALIBRATE_LEG <LF\|RF\|RH\|LH>` | H5 | dependency-derived selected-leg orchestration |
| `@CALIBRATE_ALL` | H6 | shared 12-joint production orchestrator |
| `@SAFE_OFF` | any | unicast torque OFF plus fresh readback, idempotent |

All evidence is keyed to the active boot id, host nonce, session generation and
census epoch. Nothing is written to NVS or EEPROM.

## Direction is a semantic quantity

Raw encoder response and MATDOG kinematic direction are different facts. If a
raw target increases and PresentPosition increases, that shows only that the
servo followed the raw command. Calling it `+q` would be tautological.

For each joint, the current firmware therefore requires an explicit semantic
witness from the operator:

- `Q_PLUS_RAW_INCREASES`, or
- `Q_PLUS_RAW_DECREASES`.

The witness is accepted only after the current census and current all-joint H2
manual-q0 capture, and is invalidated with them. H3 then verifies bounded raw
encoder response without pretending that the response establishes kinematic
sign. H4 cross-checks the witness against both measured endpoints, declared
geometry and the H2 manual q0 candidate. A contradiction fails closed.

The engine also contains a geometry-plus-manual-q0 direction resolver for
evidence with an explicitly known physical-pose uncertainty. Manual sample
spread is only encoder stability; it is not physical-pose accuracy and is never
silently substituted for that uncertainty.

## Manual q0 and acceptance

H2 captures all 12 joints atomically in a manually established CAD pose while
torque is verified OFF. It produces `manual_pose_q0_candidate` evidence only.
H4 independently derives q0 from its two endpoint contacts, then reports the
manual-versus-derived residual. The two values are not averaged and H2 is not
renamed as a final calibration.

```text
MEASURED → CANDIDATE → ACCEPTED → PROMOTED
```

The calibrator can produce at most `ACCEPTED`; promotion is a separate reviewed
operation outside this firmware. If the current-hardware manual-q0 or
endpoint-vs-URDF tolerance is unknown, a valid measurement stays `CANDIDATE`
and its cross-check reports `BLOCKED_TOLERANCE_UNVALIDATED`. Unknown acceptance
tolerances do not prevent acquiring the measurement, but they cannot authorize
acceptance.

## Generated Geometry Compiler V5 plan

`flc_leg_plan.h` is generated, never hand-maintained. It contains an explicit,
fail-closed row for every canonical endpoint:

- **24 endpoint outcomes total**;
- **18** `NO_PARKING_REQUIRED` outcomes;
- **6** `PARKING_REQUIRED_1DOF` outcomes;
- a six-edge acyclic dependency graph and deterministic topological order;
- exact picoradian targets, q0 masks and source provenance.

The six parking transactions are:

| Target endpoint | Auxiliary parking move |
|---|---|
| `lf_upper_leg_joint:max` | `lh_upper_leg_joint` to +35° |
| `rf_upper_leg_joint:max` | `rh_upper_leg_joint` to +35° |
| `lf_lower_leg_joint:min` | `lf_upper_leg_joint` to +64.166666666665° |
| `rf_lower_leg_joint:min` | `rf_upper_leg_joint` to +64.166666666665° |
| `rh_lower_leg_joint:min` | `rh_upper_leg_joint` to +93.333333333331° |
| `lh_lower_leg_joint:min` | `lh_upper_leg_joint` to +93.333333333331° |

The LF↔LH and RF↔RH cases are ipsilateral cross-leg dependencies, not
diagonal parking. Production execution verifies all required q0-held joints,
parks the auxiliary joint, verifies tracking, executes the target endpoint,
returns the target to q0, reverses the parking move to its saved q0 tick and
verifies restore/torque-off. Unknown rows, inconsistent provenance, cycles,
prerequisite drift or failed restore abort closed.

Authoritative provenance:

- parking artifact file SHA256
  `e561e7fb98880843e590f4676e8559374c51d0e641102f89a803b3619722c4d7`;
- parking semantic SHA256
  `67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139`;
- final external safety-policy file SHA256
  `82f00a9414df06676babed1101a73e1a9d7bfa126592cdd7a425f66d5f01f1cc`.

These are offline sampled-geometry artifacts, not a continuous swept-volume
proof. They explicitly grant **zero motion authorizations**. Build stage, live
session state, operator approval, physical evidence and all runtime guards
remain independent requirements.

## Motion safety

### EEPROM-free and one GoalPosition authority

The write choke point is an address allowlist with a fail-closed default. The
only reachable writes are RAM TorqueEnable, RAM TorqueLimit and the single
`WritePosEx` RAM block. `PositionOffset`, ID, Lock, factory-reset and EEPROM
helpers are unreachable. TorqueEnable value 128 is refused unconditionally.
Every write is validated for a concrete leg id; broadcast id 254 cannot pass.

`flcWritePosEx()` is the only GoalPosition authority. It accepts only unsigned
ticks in `[0,4095]`; crossing a wrap boundary is refused, never normalized into
a motion command.

### Safe transaction ordering

Before enabling torque, the engine reads fresh telemetry with torque expected
OFF, validates the start/q0 context, writes TorqueLimit, preloads GoalPosition
to the **current measured position while torque remains OFF**, verifies that
hold goal, and only then enables and verifies torque. This prevents retained
stale GoalPosition from causing an unguarded jump.

The hardware adapter keeps its motion-stage capability alive for the entire
transaction, including each guarded position command. Every exit requests
torque OFF and requires fresh readback proving OFF. Missing readback is a loud
fault with `CUT_SERVO_POWER_NOW`, never assumed safe.

### One guard path for every moving segment

Direction probes, baseline motion, contact approaches, retreats, q0 returns,
parking and reverse restore all share the same per-sample guard path. It checks:

- live boot/session/census identity and fresh telemetry;
- driver error and servo status byte;
- torque state, TorqueLimit and exact GoalPosition readback;
- unsigned domain and wrap-boundary refusal;
- hard current, temperature and voltage limits;
- time and travel budgets;
- minimum movement, wrong-direction and tracking failures;
- contact-too-early and persistent multi-channel contact evidence;
- q0-held/non-target drift, parking prerequisite drift and restore accuracy.

Any segment fault aborts the shared orchestration, verifies target and auxiliary
torque OFF, invalidates the session and requires a new physical-session lease.

## Progressive hardware stages

```text
H0  ESP32 only; servo power OFF             ← default
H1  12-servo read-only census
H2  all-joint manual-pose q0 capture
H3  one-joint characterization
H4  one-joint full calibration
H5  one complete leg
H6  four legs through shared orchestrator
H7  separate freeze/promotion gate; not part of this calibrator
```

The stage comes from `flc_stage_config.h`, defaults to H0, and is stamped into
the build identity. Raising it only widens which named operation may be
considered; it does not bypass the host policy, physical session, census,
manual q0, semantic direction, bootstrap or runtime guards.

The bootstrap envelope is a conservative first-motion candidate, not current
hardware evidence. It requires both a build flag and a live approval bound to
the current session/epoch. It is never canonical or persistent.

## Offline build and test

The following are software-only operations. They do not authorize flashing or
hardware access:

```bash
cd 05_Firmware/Full_Leg_Calibrator_V1
./tools/build_stage.sh 0
./tools/build_stage.sh 1
./tools/build_stage.sh 2
./tools/build_stage.sh 3 --bootstrap
./tools/build_stage.sh 4 --bootstrap
./tools/build_stage.sh 5 --bootstrap
./tools/build_stage.sh 6 --bootstrap

cd ../../../06_Software/Matdog_Core/calibration
python3 -m pytest tests/ -k full_leg_calibrator -q
python3 -m pytest tests/ -q
```

Do not append `--upload` under the current authorization. Do not open the USB
port or run the hardware runner until Matteo explicitly authorizes H0 in the
current session. After that authorization, the permitted scope is only the
final ESP32-only H0 flash and no-servo smoke test described in the handoff;
servo power remains OFF and H1+ remains prohibited.

### Verification state

Measured on this branch with no hardware attached.

| Check | Result |
| --- | --- |
| Calibrator-relevant tests (`-k full_leg_calibrator`) | **286 passed, 0 failed, 0 errors** |
| Full calibration suite (`tests/`) | **606 passed, 0 failed, 0 errors** |
| Native harnesses (`-Wall -Wextra -Werror`) | detector + shared engine build clean |
| Generated geometry header (`--check`) | `FLC_GEOMETRY_PLAN_CHECK=PASS` |

H0–H6 compile matrix, every configuration clean:

| Stage | Bootstrap | Build | Compiler warnings |
| --- | --- | --- | --- |
| `H0` | n/a | PASS | 0 |
| `H1` | n/a | PASS | 0 |
| `H2` | n/a | PASS | 0 |
| `H3` | APPROVED | PASS | 0 |
| `H4` | APPROVED | PASS | 0 |
| `H5` | APPROVED | PASS | 0 |
| `H6` | APPROVED | PASS | 0 |
| `H3` | DENIED | PASS | 0 |

### Build reproducibility

Two cold-cache builds of the same commit now produce a **byte-identical**
application binary, so the SHA256 is usable as a pre-flash integrity check:

```bash
./tools/check_reproducible_build.sh 0
```

This was not true before 2026-08-29. The firmware printed `BUILD_DATE=__DATE__`
and `BUILD_TIME=__TIME__`, and the Arduino ESP32 core embeds its own
`Compile Date` string, so the compile instant leaked into the image and every
rebuild of identical source produced a different hash. Both halves are fixed:
the firmware now reports `BUILD_SOURCE_EPOCH` derived from the commit, and
`build_stage.sh` exports `SOURCE_DATE_EPOCH` so the toolchain expands those
macros identically everywhere, including inside the core.

Runtime provenance stays `BUILD_GIT_SHA` + `BUILD_WORKTREE_DIRTY`, read back
from the running firmware. A hand build with no stamp reports
`BUILD_PROVENANCE=UNSTAMPED` rather than claiming provenance it does not have.

Status: **IMPLEMENTATION COMPLETE — OFFLINE TESTED — HARDWARE H1+ NOT EXECUTED.**
No USB port was opened, no image was flashed and no servo was energised to
produce any number above.

Evidence: [Full Leg Calibrator V1 validation reports](../../09_Logs/Validation_Reports/Full_Leg_Calibrator_V1/README.md).

# MATDOG Safe Actuator Layer — S0 write-surface audit and design

**Audit date:** 2026-09-22 · **Branch:** `feat/controller-safe-actuator-layer-v1`
**Parent:** `feat/controller-calibration-manager-v1` @ `f243b6f`

This phase does **not** enable motion. It designs and implements the software boundary
every future actuator write must pass through, and proves the boundary's decisions
offline before any transport exists to act on them.

```text
Calibration Execution Engine        [NOT IMPLEMENTED]
        |
        v
Safe Actuator Layer                 policy core: THIS PHASE
        |
        v
ActuatorAuthority verification      core/ActuatorAuthority.h (already shipped)
        |
        v
write policy / limits / transaction semantics
        |
        v
ServoBus                            [NOT CONNECTED — see §7]
```

---

## 1. S0 — the exact write surface, verified

Every SCServo transaction in the entire Controller firmware. This is the complete list,
obtained by enumerating every `st_.<method>(` call site in `src/`; there is exactly one
`SMS_STS` instance in the tree and it is private to `ServoBus`.

| # | Call site | Transaction | Direction | Class |
|---|---|---|---|---|
| 1 | `ServoBus.cpp:55` | `Ping(id)` | read | `READ_ONLY` |
| 2 | `ServoBus.cpp:63` | `readWord(id, MODEL_L)` | read | `READ_ONLY` |
| 3 | `ServoBus.cpp:108` | `Ping(id)` (scan tick) | read | `READ_ONLY` |
| 4 | **`ServoBus.cpp:145`** | **`EnableTorque(id, 0)`** | **write** | **`SAFETY_DEESCALATION`** |
| 5 | `ServoBus.cpp:147` | `readByte(id, TORQUE_ENABLE)` | read | `READ_ONLY` |
| 6 | `ServoBus.cpp:178` | `readWord(id, PRESENT_POSITION_L)` | read | `READ_ONLY` |
| 7 | `ServoBus.cpp:179` | `readWord(id, PRESENT_SPEED_L)` | read | `READ_ONLY` |
| 8 | `ServoBus.cpp:180` | `readWord(id, PRESENT_LOAD_L)` | read | `READ_ONLY` |
| 9 | `ServoBus.cpp:181` | `readByte(id, PRESENT_VOLTAGE)` | read | `READ_ONLY` |
| 10 | `ServoBus.cpp:182` | `readByte(id, PRESENT_TEMPERATURE)` | read | `READ_ONLY` |
| 11 | `ServoBus.cpp:183` | `readByte(id, TORQUE_ENABLE)` | read | `READ_ONLY` |

**The handoff's expected fact is VERIFIED, not assumed:**

```text
the only actuator write currently present is torque OFF inside ServoBus::safeOff()
```

One write. It removes torque, it cannot add it, and `scripts/static_audit.py`
(`check_torque_enable`) fails the build if `EnableTorque` is ever called with a non-zero
second argument anywhere in the tree.

### 1.1 Classification of every token named in the handoff

| Token | State | Class | Evidence |
|---|---|---|---|
| `EnableTorque` | EXISTS, 1 call site, argument pinned to `0` | `SAFETY_DEESCALATION` | `ServoBus.cpp:145`; audit `check_torque_enable` |
| torque ON | ABSENT | `FUTURE_WRITE_PATH` | no call site; audit forbids non-zero argument |
| `WritePos` / `WritePosEx` | ABSENT | `FORBIDDEN` today, `FUTURE_WRITE_PATH` behind this layer | audit `check_forbidden_literals` |
| `RegWritePosEx` / `SyncWritePosEx` | ABSENT | `FORBIDDEN` | audit `check_forbidden_literals` |
| `SyncWrite` | ABSENT | `TO_DESIGN` | no multi-joint evidence exists yet — see §6 |
| `GoalPosition` | ABSENT | `FUTURE_WRITE_PATH` | `SMS_STS_GOAL_POSITION` is a forbidden literal |
| `TorqueEnable` (register) | EXISTS, read only | `READ_ONLY` | `ServoBus.cpp:147,183` — the `safeOff` proof readback |
| `CalibrationOfs` | ABSENT | `FORBIDDEN` | forbidden literal **and** `MATDOG_JOINT_CALIBRATION.yaml` `forbidden:` list |
| `PositionOffset` | ABSENT from code | `PROVISIONING` — outside this layer | YAML forbids rewriting it to compensate mounting error; all 17 units hold `0` |
| EEPROM (`unLockEprom`/`LockEprom`) | ABSENT | `FORBIDDEN` in this layer; boundary `TO_DESIGN` | forbidden literals; see §6 |
| ID write | ABSENT | `FORBIDDEN` | audit `check_servo_id_write` bans raw `writeByte`/`writeWord` in `ServoBus.cpp` |
| vendor re-initialisation | ABSENT | `FORBIDDEN` | forbidden literal |
| `safeOff` / `SAFE_OFF` | EXISTS, ungated in every mode and authority state | `SAFETY_DEESCALATION`, **outside this layer** | §5 |
| `ServoBus` | EXISTS, single instance, UART1 | transport | `ServoBus.h:202-203` |
| `ActuatorAuthority` | EXISTS — owner + generation + inhibit | arbiter | §3 |
| `CalibrationManager` | EXISTS, owns no transport | session/evidence | audit `check_calibration_boundaries` |

### 1.2 Non-servo writes, for completeness

Not actuator writes, and deliberately **not** in this layer's scope:

- **DALY KEY** — one reviewed frame (`0x0120 := 0x005A`), MAINTENANCE-gated, pinned
  byte-for-byte by `check_daly_write`. A BMS configuration write, not an actuator.
- **LedRing GPIO47** — status output, gated by `kLedRailPowered`.
- **OTA flash writes** — an actuator *inhibitor*, never an owner (`OtaAuthorityGate`).

---

## 2. The decisive limits fact

`06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml` declares:

```text
calibration_reset:
  state: CALIBRATION_RESET_PENDING_FULL_RECALIBRATION
  effective: "2026-08-27"
  all_joint_data_below_is_stale: true
  hardware_motion_authorized: false
```

and for **all 12 leg joints**, without exception:

```text
first_stand_limit_rad: {min: null, max: null}
measured_contact_rad:  {min: null, max: null}
safe_limit_rad:        {min: null, max: null}
```

**There is not one accepted joint bound in the current repository.** This is not a gap to
fill with a default — it is the verified state of the machine after the 2026-08-27
reassembly. Per the handoff's §14 rule, the only safe result for a position-class command
today is:

```text
WRITE NOT AUTHORIZED
```

The policy therefore ships with an empty limit table that **cannot** be populated from
historical evidence, and every `POSITION_COMMAND` resolves to
`REJECT_NO_ACCEPTED_LIMITS`. That is the correct answer, not a placeholder.

---

## 3. Authority and lease binding

`core::ActuatorAuthorityArbiter` already supplies everything needed; this layer adds no
second lock and no second authority model.

| Question (handoff §8) | Answer |
|---|---|
| 5. How is a lease bound to a transaction? | `plan()` captures `AuthorityLease{owner, generation}` **and** the `OperatingMode` **and** a policy epoch into the transaction. |
| 6. How is stale authority prevented from writing? | `commit()` re-reads the live arbiter and compares owner, generation, inhibit and mode. A captured lease is evidence of what was true at plan time, never a permit. |
| 7. What if authority is lost between planning and the bus write? | `commit()` returns `REJECT_STALE_GENERATION` / `REJECT_WRONG_OWNER` / `REJECT_NO_AUTHORITY` and moves the transaction to `REJECTED`. It cannot be resumed. |

There is no "check once then write later" path. `plan()` returning `ACCEPT` grants
nothing; only `commit()`, re-verifying against live state in the same call that would
hand the command to a transport, can authorise.

Additional fail-closed triggers, all tested: exclusivity inhibit taken (OTA),
`OperatingMode` changed to one incompatible with the owner, arbiter `forceClear()`,
`reset()` of the policy, and transaction replay.

---

## 4. Operation classes — only what evidence justifies

```text
TORQUE_ENABLE              runtime torque application
POSITION_COMMAND           a single-joint goal position
CALIBRATION_CONTACT_PROBE  a bounded approach expecting a contact witness
```

`CALIBRATION_CONTACT_PROBE` is justified by recovered LF V25 evidence: contact detection
is a distinct operation with its own witness band (`ContactWitness`,
`LF_CONTACT_WITNESS_TOLERANCE_TICKS = 24`) and its own failure taxonomy
(`ContactState`), not a position command that happens to stop early.

**`EEPROM_WRITE` and `ID_WRITE` are deliberately absent from the enum.** Repository
evidence does not settle whether persistent provisioning belongs to
`ActuatorAuthority::PROVISIONING` or to a separate post-acceptance transaction
(`CALIBRATION_SOURCE_PRECEDENCE.md` §7 records this as `TO_DESIGN`). Rather than guess,
the type makes a persistent write **inexpressible** through this layer — the same
discipline that keeps `bus_id` out of `JointIdentity`. `check_safe_actuator_boundaries`
fails the build if either class is added.

Owner→operation eligibility:

| Owner | `TORQUE_ENABLE` | `POSITION_COMMAND` | `CALIBRATION_CONTACT_PROBE` |
|---|---|---|---|
| `CALIBRATION` | yes | yes | yes |
| `MOTION` | yes | yes | **no** |
| `DIAGNOSTICS`, `QC`, `PROVISIONING` | no | no | no |
| `NONE` | no | no | no |

`MOTION` cannot issue a contact probe: a probe is a calibration measurement whose whole
purpose is to drive a joint into a mechanical endstop. `DIAGNOSTICS`/`QC`/`PROVISIONING`
have no runtime actuator command today, and inventing one for them is out of scope.

---

## 5. SAFE_OFF stays outside, structurally

`SAFE_OFF` is **not** routed through this layer and must never be. The independence is
structural rather than promised:

- `ServoBus` has no reference to the arbiter or to this layer — `check_actuator_authority`
  fails the build if `ServoBus.{h,cpp}` so much as contains the string `authority`;
- the `@SERVO SAFE_OFF` command branch may not reference authority or operating mode;
- `check_safe_actuator_boundaries` additionally forbids any `SafeActuator`/policy symbol
  from appearing in `ServoBus.{h,cpp}` or in the `SAFE_OFF` branch.

So `SAFE_OFF` remains reachable with no authority, with stale authority, with a failed
calibration manager, with a failed transaction, and when the policy rejects everything.
The policy core cannot express a torque-off operation at all — there is no enum value for
it, so it cannot be made to gate one.

---

## 6. EEPROM / provisioning boundary

```text
runtime actuator command  !=  persistent provisioning/configuration write
```

Kept separate, unchanged from `CALIBRATION_SOURCE_PRECEDENCE.md` §7:

| Behaviour | Classification |
|---|---|
| `PositionOffset` write | `PROVISIONING RESPONSIBILITY` — all 17 units hold `0`; the YAML forbids rewriting it to compensate mounting error |
| `CalibrationOfs`, one-key-middle | `FORBIDDEN` — named in the YAML's `forbidden:` list |
| Persisting an accepted calibration | `TO_DESIGN` — evidence does not settle the owner |
| Multi-joint synchronised write | `TO_DESIGN` — no current evidence; the LF V25 oracle calibrated one joint at a time |

This phase adds nothing to any of them.

---

## 7. Why the runtime adapter is `TO_IMPLEMENT`, not write-disabled code

The handoff permits a write adapter that exists but is source-gated off. It is **not**
built here, for a concrete reason rather than caution:

A runtime adapter capable of torque-on or a goal position would have to contain a call to
`EnableTorque(id, 1)` or a goal-position primitive. `scripts/static_audit.py` fails the
build on **both** — `check_torque_enable` on any non-zero `EnableTorque` argument, and
`check_forbidden_literals` on `SMS_STS_GOAL_POSITION` / `WritePosEx` / `RegWritePosEx` /
`SyncWritePosEx`. Those are existing, hardware-era safety invariants. Writing the adapter
now would require weakening them **before** anything could validate the replacement, and
an `#if 0` around a call site does not restore an invariant the audit no longer enforces.

Nothing is lost by waiting: with no accepted joint limits (§2) and
`hardware_motion_authorized: false`, the policy cannot return `ACCEPT` for a position
command in any case. The adapter is the last thing to build, behind a real recalibration
and an explicit hardware authorization — not the first.

```text
SAFE ACTUATOR POLICY           IMPLEMENTED + COMPILED + OFFLINE TESTED
SAFE ACTUATOR RUNTIME ADAPTER  TO_IMPLEMENT
DEFAULT BUILD WRITE REACHABILITY   torque OFF only (SAFE_OFF), unchanged
CURRENT HARDWARE MOTION AUTHORIZATION   BLOCKED
```

---

## 8. Minimum API for a future calibration execution engine

Answering handoff §8.1 — the engine needs exactly this and nothing more:

```cpp
policy.begin(&arbiter);
policy.plan(command, lease, mode, &txn);   // -> WriteDecision, binds lease+mode+epoch
policy.commit(&txn);                       // -> WriteDecision, re-verifies live state
policy.abort(&txn);
policy.reset();                            // invalidates every outstanding transaction
```

The engine supplies a `JointIdentity` (leg, joint kind, physical unit label — never a bus
id), an operation class, and a target. It never receives a transport handle: `commit()`
returning `ACCEPT` is a decision, and the adapter that would act on it does not exist yet
(§7).

---

## 9. Related

- [`CALIBRATION_SOURCE_PRECEDENCE.md`](CALIBRATION_SOURCE_PRECEDENCE.md) — source precedence, §7 EEPROM boundary
- [`DEVELOPMENT_GATES.md`](DEVELOPMENT_GATES.md) — the calibration gate
- [`../../06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml`](../../06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml) — `calibration_reset:` is the current authority

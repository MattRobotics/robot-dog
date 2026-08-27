# ST-3215-C018 — native reboot / software-reset capability audit

**Date:** 2026-08-27
**Question:** does the Feetech ST-3215-C018 offer a native, supported way to
restart / reboot / re-initialise its own MCU **without removing power**, so the
Characterizer and Provisioner cold-acceptance phases can be automated?

**Verdict: NOT PROVEN — no such primitive exists.**
**Additional finding: the one candidate that does exist is a FACTORY RESET, and
it is already reachable from NormaCore. See §4.**

---

## 1. Answer

The Feetech serial bus protocol used by this servo family has **seven**
instructions and **none of them is a reboot, restart or MCU reset**.

The complete instruction set, quoted from the vendor's own protocol manual
(§2.1):

| Instruction | Function | Value | Param len |
|---|---|---:|---:|
| PING (查询) | Query the working status | `0x01` | 0 |
| READ DATA (读) | Query the characters in the control table | `0x02` | 2 |
| WRITE DATA (写) | Write characters into the control table | `0x03` | ≥1 |
| REG WRITE DATA (异步写) | Deferred write, acts on ACTION | `0x04` | ≥2 |
| ACTION (执行异步写) | Triggers REG WRITE writes | `0x05` | 0 |
| **RESET (复位)** | **Reset control table to factory value** | **`0x06`** | **0** |
| SYNC WRITE DATA (同步写) | Simultaneous control of multiple servos | `0x83` | ≥2 |

There is **no `0x08`**. The only occurrences of the byte pattern `0X08` in the
entire manual are position payloads (`0X0800` = 2048). The assumption that
`0x08` might be a reboot comes from the Dynamixel protocol, not from Feetech,
and it does not transfer.

There is likewise **no register** anywhere in the `0x00..0x46` control table
that triggers a restart. The only "special value" write documented by the
vendor for this exact model is register 40 (`0x28`) = 128, which is the
one-key mid-position calibration MATDOG already forbids.

---

## 2. Evidence, in the mandated source order

### 2.1 Vendor protocol manual — PRIMARY, DECISIVE

```
Serial Bus Smart Control servo — Communication Protocol Manual
SHENZHEN FEETECH RC MODEL CO.,LTD.
Revision V1.01, 2019-02-19, "Universal SCS and SMS Series servo"
§1.3 Instruction type
```

Retrieved 2026-08-27 from the Feetech documentation mirrored by Seeed Studio:
`https://files.seeedstudio.com/wiki/robotics/Actuator/feetech/Communication_Protocol_Manual.pdf`

§1.3 is an exhaustive table ("The following instructions are available for
Feetech Serial Bus Intelligent servo Communication Protocol"). It is
reproduced in full in §1 above. `0x06` is documented verbatim as:

> RESET（复位） — Reset control table to factory value — 0x06 — 0

**Confidence: HIGH.** This is the vendor's own protocol specification and the
table is explicitly exhaustive.

**Residual gap, stated honestly:** the manual scopes itself to "SCS and SMS
series". The C018 is an ST-3215 (STS series). No STS-specific protocol manual
was found locally or online. The inference that the instruction set is
family-wide is supported by §2.2 and §2.4 and is rated HIGH, not CERTAIN.

### 2.2 Installed SCServo library — CORROBORATING

`/home/matteo-manicardi/Arduino/libraries/SCServo/` v1.0.2
(fork of `waveshareteam/ugv_base_general`, FEETECH-authored `SCServo`).

`src/INST.h` defines exactly:

```
INST_PING 0x01  INST_READ 0x02  INST_WRITE 0x03  INST_REG_WRITE 0x04
INST_REG_ACTION 0x05  INST_SYNC_READ 0x82  INST_SYNC_WRITE 0x83
```

`0x06` is **deliberately absent**. This library ships the STS/SMS application
layer (`SMS_STS.h/cpp`) over a shared communication layer (`SCS.cpp`), i.e.
FEETECH itself treats SCS / SMS / STS as one protocol with three memory maps.

The full example set — `Ping`, `WritePos`, `RegWritePos`, `SyncWritePos`,
`FeedBack`, `ProgramEprom`, `CalibrationOfs`, `Broadcast` — contains **no**
reboot or reset example.

`SCS::writeBuf(ID, MemAddr, nDat, nLen, Fun)` is `protected` (`SCS.h:43-44`),
so the arbitrary-instruction byte `Fun` is **not reachable** from our firmware
without deliberately subclassing. This is a safety property worth preserving.

**Confidence: HIGH** as corroboration.

### 2.3 Vendor product specification for the exact model — CORROBORATING

`MATDOG/github/robot-dog/01_Docs/03_Datasheets/Feetech-ST3215-C018.pdf`
(FEETECH, ST-3215-C018, edition A/0, 2023-07-20, 8 pages)

Documents protocol type, ID range, baud, PID, modes 0–3, multi-loop, feedback
and electronic protections. It documents **no reset, reboot or restart
capability of any kind**. The only "special" operation it names is §7-14
"置中功能 / Step Middle Position — 40号地址输入128" (write 128 to address 40),
the forbidden one-key-middle calibration.

Two incidental facts worth recording:

* §7-13 multi-loop: `掉电圈数不保存` — **the turn counter is not preserved
  across power loss.** An MCU reset and a power loss are therefore *not*
  equivalent for multi-turn state (relevant to §3).
* §7-6 neutral position is stated as `180° (2047)`. MATDOG canonical centre is
  physical RAW **2048** with ±1 tick acceptance. Noted as an observation only;
  no change proposed here.

**Confidence: HIGH** as negative evidence for the exact model.

### 2.4 Independent third-party SCS/STS SDK — CORROBORATING

`scsservo_constants.mjs` (LeRobot-Arena `feetech.js`, targets Feetech SCS/STS
servos including STS3215) defines instructions `1, 2, 3, 4, 5, 0x55, 0x82,
0x83`. **No 6, no 8.**

**Confidence: MEDIUM** (third-party), consistent with all of the above.

---

## 3. Required per-primitive report — RESET `0x06`

The one candidate primitive that exists, reported as mandated:

| Field | Finding |
|---|---|
| name | RESET (复位) |
| instruction / opcode | `0x06`, instruction byte; not a register |
| packet format | `FF FF <ID> 02 06 <checksum>`, zero parameters |
| ACK semantics | returns a normal status packet; **not** documented in detail |
| TorqueEnable prerequisite | none documented |
| **what is actually reset** | **"Reset control table to factory value"** — the EEPROM control table is restored to factory defaults. This is a **factory reset**, not an MCU restart. |
| reboot time | not documented by the vendor. NormaCore empirically waits 100 ms settle + up to 200 ms poll (`port.rs`), implying return within ~300 ms |
| RAM state after | undefined by the vendor; presumed re-initialised from the reset EEPROM |
| EEPROM persistence | **DESTROYED** — that is the instruction's stated purpose |
| ID behaviour | **presumed reset to the factory ID** (factory default is 1). Not separately documented. |
| PositionOffset behaviour | **presumed reset to factory 0**, destroying digital-zero calibration |
| risks | catastrophic for a provisioned MATDOG unit: loses target ID, `PositionOffset = 0` calibration, and the entire `MATDOG_C018_V1` persistent profile. Recovery requires full re-provisioning. On a bus scan the unit would reappear at a different ID. |
| source / evidence | Feetech Communication Protocol Manual V1.01 §1.3 (§2.1 above) |
| confidence | **HIGH** that it is a factory reset; MEDIUM on the exact ID/offset consequences, which the vendor does not itemise |

**MATDOG disposition: PERMANENTLY FORBIDDEN.** It is already covered by the
existing "never factory-reset a servo" constraint. It must never be added to
the Characterizer or Provisioner write surface, and the fact that the
installed library cannot emit it (§2.2) should be treated as a feature.

---

## 4. CRITICAL SIDE FINDING — NormaCore ships a reachable factory reset

`norma-core/software/drivers/st3215/src/protocol/packet.rs:33`

```rust
Self::Reset { .. } => 0x06,
```

`packet_test.rs:92` asserts the exact wire bytes:

```
FF FF 01 02 06 F6      // "Reset" motor 1
```

This is emitted for a **single motor ID**, not broadcast, and is exposed all
the way up to the API surface:

* `protobufs/drivers/st3215/st3215.proto:216` — `ST3215ResetCommand { port_name, motor_id }`
* `software/drivers/st3215/src/port.rs:1125-1175` — handler, which logs
  *"Processing ST3215 Reset command"*, then *"Block the worker until the servo
  finishes rebooting"*, then polls `RESET_REBOOT_TIMEOUT_MS = 200`.

The code and its comments describe this as a **reboot**. Per the vendor manual
it is a **factory reset of the control table**. Both behave identically from
the outside — the servo goes away and comes back in ~100–300 ms — which is
exactly why the misreading is easy to make and dangerous to keep.

Corroborating reason for caution: NormaCore's C018 register map is *already
known to be defective* — `protocol/memory.rs:46` declares
`ModelNumber => (0x00, 2, ...)` where the C018 model word is at `0x03`
(`MATDOG_ST3215_PROVISIONER_V1_DESIGN.md` §5). NormaCore is therefore not an
authoritative source-map for this servo, and its `Reset` naming must not be
taken as evidence of reboot semantics.

**Recommended follow-up, outside the scope of this audit and not performed
here:** treat `ST3215ResetCommand` as a destructive operation in NormaCore —
rename, gate or remove it — before any MATDOG servo is provisioned, since a
single accidental invocation on a provisioned unit silently un-provisions it.

---

## 5. Equivalence analysis (conditional — the primitive does not exist)

Recorded for completeness, because it informs the hardware option.

Had a genuine, non-destructive MCU reboot existed, it **would** have been
technically sufficient for most of what the Provisioner's cold phase proves,
since a real MCU restart re-reads EEPROM into RAM:

| Property to prove | MCU reset sufficient? |
|---|---|
| persistent profile survives and re-loads | YES |
| ID persisted | YES |
| `PositionOffset = 0` persisted | YES |
| `Lock = 1` persisted | YES |
| `TorqueEnable == 0` at boot | YES |
| RAM `TorqueLimit` reverts from 300 | YES |
| servo becomes reachable again | YES |
| **brown-out / rail-collapse behaviour** | **NO** |
| **multi-turn counter loss on power loss** (datasheet §7-13) | **NO** |

So a reboot would have been *sufficient for the EEPROM-persistence question*
but *not equivalent* to a power cycle for rail behaviour and multi-turn state.
This distinction is now moot: no such instruction exists.

---

## 6. Conclusion

```
A. C018 native reset/reboot support : NOT PROVEN
   - no reboot/restart instruction exists in the Feetech instruction set
   - no control-table register triggers a restart
   - the only reset primitive (0x06) is a FACTORY RESET and is forbidden
```

The physical servo-rail power cycle therefore **remains technically necessary**
for cold acceptance. It is not being retained merely because a previous handoff
said so — it is retained because the vendor protocol provides no alternative.

No reset is invented. No Characterizer V2 or Provisioner V2 is implemented on
the strength of a primitive that does not exist.

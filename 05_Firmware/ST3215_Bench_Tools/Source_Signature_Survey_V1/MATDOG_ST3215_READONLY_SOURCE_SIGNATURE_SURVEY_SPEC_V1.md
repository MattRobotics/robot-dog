# MATDOG ST3215 — Read-Only Source Signature Survey V1
Date: 2026-08-26
Status: DESIGN ONLY — execution requires explicit current-session hardware authorization.

## Objective
Capture the real current as-found state of each allocated ST-3215-C018 before any provisioning code is allowed to recognize that source profile.

## Safety scope
Allowed only when explicitly authorized:
- one servo connected at a time;
- serial discovery/read operations;
- Torque OFF command + readback as a safety action.

Forbidden:
- any position/goal command;
- Torque ON;
- EEPROM unlock;
- EEPROM write;
- ID change;
- CalibrationOfs;
- factory reset;
- broadcast write.

## Per-unit procedure
1. Operator states physical unit label (Mxx / ELRxx / NEWxx).
2. Verify only one servo is electrically connected.
3. Open the existing known commissioning transport.
4. Force Torque OFF and verify `TorqueEnable == 0`.
5. Scan IDs 0..253; require exactly one responder.
6. Record discovered source ID.
7. Read model from 0x03 and require 777.
8. Dump the complete 71-byte state block.
9. Persist raw bytes before decoding.
10. Decode all mandatory fields.
11. Compute SHA256 over raw dump.
12. Repeat full dump once and require all EEPROM bytes to match; runtime telemetry may vary only in explicitly transient fields.
13. Close without any motion or EEPROM operation.

## Mandatory decoded EEPROM fields
- Model: 0x03..0x04
- ID: 0x05
- Baud: 0x06
- ReturnDelay: 0x07
- ResponseStatus: 0x08
- MinAngle: 0x09..0x0A
- MaxAngle: 0x0B..0x0C
- MaxTemperature: 0x0D
- MaxVoltage: 0x0E
- MinVoltage: 0x0F
- MaxTorque: 0x10..0x11
- UnloadCondition: 0x12
- LedAlarm: 0x13
- P: 0x15
- D: 0x16
- I: 0x17
- MinStartupForce: 0x18..0x19
- CWDead: 0x1A
- CCWDead: 0x1B
- ProtectionCurrent: 0x1C..0x1D
- AngularResolution: 0x1E
- PositionOffset: 0x1F..0x20, decode signed little-endian int16
- Mode: 0x21
- ProtectionTorque: 0x22
- ProtectionTime: 0x23
- OverloadTorque: 0x24
- SpeedClosedLoopP: 0x25
- OverCurrentProtectionTime: 0x26
- VelocityClosedLoopI: 0x27

## Mandatory RAM / telemetry fields
- TorqueEnable 0x28
- Acc 0x29
- GoalPosition 0x2A..0x2B
- GoalTime 0x2C..0x2D
- GoalSpeed 0x2E..0x2F
- TorqueLimit 0x30..0x31
- Lock 0x37
- PresentPosition 0x38..0x39
- PresentSpeed 0x3A..0x3B
- PresentLoad 0x3C..0x3D
- PresentVoltage 0x3E
- PresentTemperature 0x3F
- Status 0x40
- Moving 0x42
- PresentCurrent 0x45..0x46

## Output record
One immutable JSON/JSONL record per physical unit containing:
- physical_unit;
- timestamp UTC;
- discovered_source_id;
- exact responder list from scan;
- raw_state_hex;
- raw_state_sha256;
- second_dump_sha256;
- decoded EEPROM fields;
- decoded RAM/telemetry fields;
- PositionOffset raw_u16 and signed_i16;
- `physical_raw = floor_mod(displayed + signed_offset, 4096)`;
- torque-off verification;
- result / reason.

## Classification policy
Survey does NOT provision.

It may label a snapshot only as:
- CANONICAL_TARGET_EXACT
- KNOWN_LEGACY_CANDIDATE (evidence only; not write-authorized yet)
- UNKNOWN_OR_INVALID

Accepted write-source signatures are frozen only after offline comparison of all collected records and a second review.

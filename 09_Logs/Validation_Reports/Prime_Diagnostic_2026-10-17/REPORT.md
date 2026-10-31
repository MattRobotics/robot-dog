# MATDOG — supervised PRIME diagnostics, 17 October 2026

**GO for the subsequent review of a diagnostic APP-only flash. Not a corrective
release, flash authorization or hardware qualification.** Physical cause still
unknown. No hardware access, serial acquisition, servo command, reset or flash
was performed while preparing this patch. No general audit / 8.8M campaign.

## Provenance and scope

Branch `diag/servo-prime-20261017`, isolated worktree
`~/MATDOG/worktrees/robot-dog-prime-diagnostics-20261017`, created directly from
`9512816ea4cc083b09a2d7d4f823fbaf610a10dd`. The functional changes of `29664dd`
are not inherited. Only its in-memory UART test fixture was reused.

Operator evidence: two read-only scans on installed `9512816` found 12/12 valid,
torque OFF, goal/speed/ACC 0 after boot, fresh feedback and no hardware faults.
This is not the failed PRIME transaction. No further idle scans are proposed.

The production write remains exactly `SMS_STS::WritePosEx` followed by ONE
validated READ of SRAM40..49, width10, for PRIME 20/5. Its comparison and
three-state verdict remain those of `9512816`. No per-register read, new retry,
normalization, tolerance, speed/ACC or mechanical-limit change is introduced.
The installed SCServo 1.0.2 library is not edited. SAFE_OFF remains unchanged.

## Frozen evidence

`ValidatedServoRead::ReadCaptured` optionally copies the ORIGINAL status packet
to RAM without issuing another transaction. Ordinary background reads use the
same parser without requesting a capture. A rejected PRIME immediately copies
that goal transaction through the backend/runtime boundary into MotionSession's
fixed rejection log before the next poll, retry, SAFE_OFF or goal write.

Every record includes slot, canonical ID, attempt 1..3, target position/speed/ACC,
write ACK result, write start/end, read start/end and completion timestamps
(uint32 milliseconds), requested register address/width, received byte count,
all bytes actually consumed (including partial/invalid frame), drained RX count,
validation failure and decoded field values. Mismatch bits: position 1, speed 2,
ACC 4. A valid position alone never suppresses the other mismatch bits.

Invalid/absent responses retain the original UNCERTAIN verdict. Trusted decoded
values remain -1 on invalid packets. If the complete raw control payload exists,
USB export also decodes it as `raw_position/raw_speed/raw_acc`, explicitly marked
`decoded_valid=0`; these untrusted numbers never influence the write verdict.
There is no substituted later telemetry snapshot. Policy/commit refusals carry
`capture=0/NOT_ATTEMPTED`, not an invented servo response.

Capacity 36 = 12 joints x 3 attempts, no allocation or overwrite. SAFE_OFF and
subsequent reads/writes cannot clear retained records. A trial is one-shot:
another START requires explicit CLEAR while idle. A full buffer refuses another
write. Reboot/power loss DOES lose RAM evidence: export before resetting.

## Diagnostic release interlocks

Controller inhibits torque-ON and streaming goal frames BEFORE peripheral or
command processing at every boot of this branch. There is no USB software
unlatch. Normal Dashboard Motion admission is refused; Dashboard/network code
itself is unchanged. Boot still performs no automatic PRIME or motion.

The only admitted diagnostic trial is an explicit USB request with current
nonce and fixture/operator/cutoff/torque OFF confirmation. Existing runtime,
gen1 calibration/limits, exclusive authority, power/BMS, IMU, fresh feedback,
stationary torque OFF, session and policy gates all still apply. START supplies
the existing finite 3s presence window, not indefinite authorization. Legacy
supervised-validation capability remains 0 in the produced build.

After the twelfth verified PRIME, MotionSession requests SAFE_OFF with
`PRIME_DIAGNOSTIC_COMPLETE_NO_ENERGIZE` BEFORE changing to ENERGIZING. A rejection
keeps the existing bounded 3 attempts and then SAFE_OFF: no skipping failed joints
to reach later ones. STOP, deadman, faults, freshness and batch timeout still
revoke the trial. The independent ServoBus inhibit also refuses torque-ON and
streaming frames even if another caller attempts them. The latch survives
SAFE_OFF and CLEAR and is re-established on reboot of this diagnostic release.

## Future supervised acquisition — NOT executed

After a separately reviewed/authorized APP-only flash and supervised boot,
use the single exclusive USB owner without DTR/RTS resets. Fixture/free legs,
KEY ON, operator and physical cutoff as the real installation permits.
Verify correct source/profile, READY, session/executorOFF, torque OFF and no fault.
Do not issue STAND/WALK/FAST. No Dashboard is needed to start this trial.

```text
@MOTION VALIDATION PRIME_DIAG STATUS
@MOTION VALIDATION PRIME_DIAG START <nonce> CONFIRM_FIXTURE_PRESENT_POWER_CUT_TORQUE_OFF_NO_ENERGIZE
```

Use the nonce from STATUS; it must not be guessed or replayed automatically.
Wait for sessionOFF and SAFE_OFF verification. Then:

```text
@MOTION VALIDATION PRIME_DIAG STATUS
@MOTION VALIDATION PRIME_DIAG READ 0
```

Read each index 0..records-1 individually while draining USB, saving the full
line. STATUS exposes count, overflow, inhibit and retained last_revoke. READ
is refused while session/executor/service is active and performs RAM formatting
only, no servo traffic. Export truncation is refused explicitly. Slot order:
`13,12,11,23,22,21,33,32,31,43,42,41`. No per-ID exception.

If another supervised trial is later approved, first save all evidence; only
then may `@MOTION VALIDATION PRIME_DIAG CLEAR <nonce> CONFIRM_CLEAR_RAM_EVIDENCE`
clear RAM while idle. It does NOT release the torque inhibit. Existing
`@MOTION VALIDATION STOP` remains available without diagnostic nonce/capability.

## Timing / STOP limits

No UART transaction is added to PRIME: original write/ACK plus one block READ.
The 8ms per-read commissioning timeout, 1500ms PRIME phase deadline, 400ms feedback
age and 3-attempt limit are unchanged. Capture/log insertion are fixed RAM copies
of at most 38 raw bytes plus metadata. No formatting/USB print occurs in the
rejection path; export runs only at idle. The 1500ms phase timer is not extended.

The original ACK uses inter-byte timeout and a drain without a hard RX-flood
deadline. Therefore even 16ms (ACK+READ) is only a simplified budget, NOT a WCET.
Watchdogs are not an immediate Controller STOP deadline. STOP is processed at
the existing cooperative boundaries, after an in-flight legacy call returns;
no new hard stop-latency guarantee is claimed. The diagnostic interlock keeps
torque OFF irrespective of successful PRIME and never authorizes ENERGIZE.
No timeout, watchdog, freshness or STOP gate was weakened to obtain a PASS.

## Targeted offline verification

Evidence: `~/MATDOG/verification-artifacts/PRIME_DIAGNOSTIC_20261017/`.

| Target | Result |
|---|---|
| Real MotionSession→policy/runtime→ServoBus→installed SCServo, ASan/UBSan | PASS 12,791 checks total, including 1,773 transport checks |
| Each slot0..11, position/speed/ACC mismatch, missing response | Three original attempts retained with exact slot/ID/target/raw packet; SAFE_OFF preserves records |
| Six parser fault types on all 12 IDs, ACK loss | Original verdict preserved; complete/partial/absent bytes and validity retained |
| Transient rejection on every joint then success | All 12 earlier records survive later writes; final transition is SAFE_OFF, never ENERGIZE |
| STOP at every slot and between rejected attempts | No later goal/torque write; retained record unchanged after OFF |
| Batch timeout / stale feedback / commit refusal | Refuse without new UART write; no stale capture attributed to a refusal |
| Production USB diagnostic/STOP branches, controller-state fakes | PASS 43 checks: nonce/token/idle/freshness/torque/speed/busy gates, RAM-only export, CLEAR preserves inhibit |
| Existing MotionService/Executor | PASS 2,459 checks |
| Existing validated-read parser | PASS 135 checks |
| Existing finite presence/clock checks | PASS |
| Five source-only safety checks | PASS: write surface, torque-enable surface, scoped timeouts/categories, SAFE_OFF readback |

Run the focused integration via
`python3 05_Firmware/MATDOG_Controller/scripts/tests/run_prime_diagnostic_tests.py`
and USB branch tests via `test_prime_diagnostic_usb.py`. These tests simulate
feedback approvals and UART devices; they are not physical bus timing or boot
qualification. Original normal-motion unit behavior is checked by the existing
executor suite, but this candidate intentionally inhibits normal Motion on robot.

## Build / handoff

Commit and build receipts reside in the external evidence directory. The build
uses official `scripts/build.sh`, CLEAN source, ROBOT_POWERED, OTA 0,
SUPERVISED_VALIDATION 0 and commit-time SOURCE_DATE_EPOCH, with external root
`PRIME_DIAGNOSTIC_20261017/build-powered/`. The manifest binds app SHA, ELF,
SDK and partition table. No installed firmware or prior candidate is overwritten.

**GO is scoped to reviewing this diagnostic candidate for a subsequent
APP-only flash.** It is not GO for corrective behavior, ENERGIZE, STAND or WALK.
Do not reuse `9512816`/`29664dd` designations for this different binary. A fresh
diagnostic designation and separate ROM/write/boot authorizations remain part
of that later workflow. The actual physical PRIME cause remains unknown until
the frozen failed transaction is acquired under supervision.

Calibration records, mechanical limits, EEPROM, BMS, network/AP credentials,
Dashboard, partition layout and SAFE_OFF implementation were not changed.

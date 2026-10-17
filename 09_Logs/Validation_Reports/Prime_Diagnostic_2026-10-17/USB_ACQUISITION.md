# PRIME_DIAG host acquisition qualification

Compiled source and powered application stay 8b9ee70a60bfa777ed50484c5e9546ceffd72f3b / 45843799db102804627674d7ed51ae9a6def9ce5d6a850cc842cc95773fa2601. Only host tools and qualification documentation are added. The V3 gate and flasher are unmodified.

The logger scripts/release/prime_diag_logger.py is for the separately authorized diagnostic firmware only. Run it AFTER the supervised flash, independent verify-flash, preservation checks in ROM, one separately supervised boot and readiness verification. It performs no reset, flash, servo register transaction, torque command, STAND or locomotion.

Before any physical operation, the operator must actually verify fixture support, free legs, presence, accessible physical cutoff and KEY ON powering MCU and servo rail together; USB is data-only. Previous flash authorization persists. Software cannot observe fixture/KEY/disconnector state. An absent current physical check stops before opening a device. Never invent that evidence.

An actual operator setup observation is recorded with UTC in physical-check.json (fixture, operator_present, key_on, cutoff_accessible all true; utc ISO8601 with timezone). The logger refuses an incomplete or more than 300-second-old observation before opening USB. Do not generate a true record from an assumption. No hardware access was used to qualify the logger.

Future command, only after all V3 and hardware gates PASS:

```sh
python3 05_Firmware/MATDOG_Controller/scripts/release/prime_diag_logger.py --physical-check /absolute/path/to/actual-physical-check.json --output /absolute/path/to/new-acquisition-directory
```

One os.open for the full test, TIOCEXCL, raw terminal, HUPCL disabled; no baud change, DTR/RTS ioctl, reopening, retry of START or automatic reset. The kernel/driver's behavior on opening remains a hardware qualification limit; no intentional modem toggles exist. One exclusive owner only; no concurrent bridge, serial monitor, Dashboard actuation or network changes.

Commands are allowlisted. First disable telemetry/IMU/BMS USB streams; query exact source/profile/layout/app0, cached startup census, a bounded telemetry snapshot, fresh sensors, and PRIME_DIAG STATUS. Confirm RESTORED gen1/fingerprint be8d5902bb4e37da, all canonical12 mapped/fresh, torque/speed0, boot SAFE_OFF completed, READY, no prior trial or evidence, mode1 and torque_inhibit1. Only then issue once:

```text
@MOTION VALIDATION PRIME_DIAG START <current_nonce> CONFIRM_FIXTURE_PRESENT_POWER_CUT_TORQUE_OFF_NO_ENERGIZE
```

No automatic heartbeat. The existing firmware's finite presence, PRIME timeout and bounded retries are unchanged. Poll RAM status; wait for OFF with reason SAFE_OFF_VERIFIED. Query each PRIME_DIAG READ index0..count-1 once, persist every record immediately, compare count/nonce/overflow and unique slot/attempt sequence, independently validate original raw packet, checksum and decoded mismatches. Capture0 policy/commit refusals are distinguished from attempted transactions and transport failures. A post-test snapshot confirms OFF/torque0. Disable streams and close the one descriptor. Never CLEAR or reset before complete export.

All successfully written TX bytes and all received RX chunks are journaled as base64 with UTC and monotonic timestamps; incomplete/invalid data are retained before parser rejection. TX_INTENT is separate from TX. receipt.json, records.json, before/after snapshots, terminal-status.json and log-digest.json bind the acquisition. No reconnect or automatic trial repetition.

PASS_ACQUISITION means safely completed data acquisition, not corrective or motion qualification. COMPLETE_NO_ENERGIZE means no persistent refusal reproduced; REJECTION_CAPTURED reports the original register/transport refusal. Other gate, integrity, reset, overflow, timeout or cleanup failures remain FAIL. On failure after START, send STOP once and attempt SAFE_OFF verification and one RAM export if export has not already started. A partial/corrupt export is retained and never automatically repeated.

Ctrl-C/SIGTERM requests STOP and bounded recovery. At unexpected physical movement, use the disconnector immediately; physical safety overrides RAM evidence. A blocked legacy UART call cannot be preempted by host STOP. If communication prevents verified SAFE_OFF, export may remain firmware-gated and RAM can be lost by the necessary power cut. No maximum STOP latency or physical torque guarantee is inferred from unit tests.

Offline tests use a fragmented USB simulator and syscall doubles, including all12 slots, ACC/position/speed mismatches,36 records, invalid packets, actual production C++ formatter, boot/source/nonce/restore/readiness gates, timeout, incomplete response, STOP, no-reset/no-reopen, durable bytes and record integrity. Run scripts/tests/test_prime_diag_logger.py with Python3. No device opens occur in these tests.

Flash remains exclusively through flash_app_only.sh and an accepted committed V3 designation. The runner derives the active slot; independently require app0=0x010000 before invocation, while in ROM with no competing owner. Candidate write-end0x3c9c20/erase-end0x3ca000. Compare bootloader/table/otadata/default NVS/matdog NVS in ROM before/after. Rollback is APP-only9512816; never restore old fullflash NVS. No retries or alternative flashing path.

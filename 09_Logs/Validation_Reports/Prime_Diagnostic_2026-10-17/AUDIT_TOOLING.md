# Audit tooling qualification for unchanged diagnostic source8b9ee70

The first complete fresh audit exited1 with7 findings. Its authentic complete log and metadata are retained externally as release/static-audit-source.log/json. No result is relabelled PASS. The mandatory operational suite itself passed8,814,792 checks.

Five findings were obsolete lexical expectations: ServoReadValidation now returns detailed ReadValidity, the guards use compact whitespace, and ValidatedServoRead calls the detailed parser before copying data. The safety predicates have not changed in compiled firmware. The audit now recognises the WHOLE reviewed detailed parser body with whitespace ignored only, including exact null/size/header/ID/length/status/checksum behavior and bool adapter. It additionally requires the guarded copy/return sequence and rejects any early success or unvalidated byte/word return. The old bool-parser path remains checked as before.

The Safe Actuator mutation suite also failed its unmutated baseline solely due to those five findings. Dedicated new audit tests prove all17 parser/adapter/drain/deadline/early-return mutations are refused and harmless whitespace is accepted. They are added to the official host runner, with all existing suites retained.

The remaining host-runner failure was a stale network mutation fixture: it searched for the former direct authorize-then-heartbeat branch. Actual production now uses const bool beat followed by if(beat). The mutation targets that actual guard, replacing authorization+CSRF with cookie-only proof. The unchanged network audit must still detect the unsafe mutant. No production network or heartbeat code is changed.

Only scripts/static_audit.py, its host runner and mutation tests are repaired. release_candidate_gate.py, verify_designated_candidate.py, flash_app_only.sh, build.sh and every compiled input remain byte-identical to source8b9ee70. The ROBOT_POWERED binary remains SHA45843799db102804627674d7ed51ae9a6def9ce5d6a850cc842cc95773fa2601. A second full official audit is required and logged as static-audit-final; no waiver or generic reuse mode is used.

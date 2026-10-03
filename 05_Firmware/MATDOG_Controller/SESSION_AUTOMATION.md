# Release session automation — offline preparation, 2026-10-03

The executable distribution lives in
`/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_POST_ABORT_THERMAL_20261003`.
`matdog_01_flash.sh`, `matdog_02_full_calibration.sh` and
`matdog_03_verify_and_finalize.sh` are packaging entry points, using snapshots of
this repository's native runner and the release-stage helper. There is one
calibrator and one serial connection at a time. No-argument invocation and
`--offline-check` perform file/Git checks only. Hardware stages require
`--hardware-session --session-dir /absolute/new/session/path`.

## Current admission: INITIAL_POSE_BLOCKED

The package's `initial-pose-plan.json` deliberately has `qualified_path=false`.
This stops calibration before hardware I/O. Application-only flashing is
independent of this mechanical gate. Do not flip that flag
as an operator override. Releasing the gate requires an actual qualified
mechanical procedure with evidence and a new reviewed package.

The CR2-C runbook establishes nominal URDF Q0 manually with torque OFF and
physical square/jigs, supported against gravity. It does not qualify a path
from the observed RF/RH residual pose after shutdown while forbidding forced
reducer back-driving, cover removal and RESET. The same-boot recovery witness
of firmware `125d981b02a3` cannot authorize an earlier boot. Historical positions
are diagnostic context only. The physical placement/qualification required to
remove this block is documented in the package's `INITIAL_POSE_PLAN.md`.

## Stage contracts once the gate has been qualified

1. The first stage pins CLEAN firmware commit `125d981b02a38673c8c54f2096632c99fd1f29ff`
   and its canonical build directory in a detached flash worktree. It checks
   hashes and the historical backup locally, then calls only the unchanged
   `scripts/flash_app_only.sh`. That script verifies ESP32-S3 MAC, installed
   partition table/active slot and effective erase range, performs one
   application write and independent flash verification. An independent native
   connection checks signature, MAINTENANCE, authority NONE, SAFE_OFF 13/13,
   and twelve valid torque-off encoder readings before `FLASH_OK.json`.
   Those readings may be far from Q0; no pose attestation or repositioning is
   performed in this stage. FLASH_OK explicitly records nominal_pose_verified=false.
   USB failure stops; no repeated writes, BOOT/EN request or full-image fallback.
2. The second stage retains the qualified pose gate, consumes FLASH_OK, checks
   the same boot and actual torque-off encoders, then requires the operator's
   physical GO checklist. FLASH_OK supplies no motion authorization. The native
   runner executes `--phase all --no-flash`, fresh Q0 12/12, promotion, recovery
   and LF/RF/RH/LH export 24/24, followed by `--phase persist --no-flash`.
   SAVE/ACK exact generation and the application identity are recorded.
3. The third stage requires a real operator power cycle, observes USB absent and
   returned, and invokes native `--phase verify-persistence` in read-only mode.
   It checks a new uptime, exact ACK generation, record intact and motion
   unauthorized. Only then are relevant regressions run and actual evidence
   appended to VALIDATION, CHANGELOG, runbook and architecture in a separate
   documentation commit. Integration uses an isolated worktree, refuses dirty
   state, conflicts, divergent main, gait ancestry or different firmware,
   reruns regressions on the merged tree and pushes main without force.
   An unavailable remote/conflict retains the local result and reports BLOCKED.

Any consumed or failed session refuses automatic retry. Native errors and
operator SIGINT/SIGTERM trigger ABORT/SAFE_OFF while the link is available;
link loss leaves firmware protections and the accessible physical disconnect
indispensable. Every stage exports its logs, manifest and evidence hashes.
Tests use synthetic controllers or temporary local Git repositories; these
receipts never qualify the packaged initial-pose gate or real hardware.

## DALY guard on the existing native link

`--require-daly` starts the firmware's existing `@BMS STREAM`, not a second
transport. A strict complete telemetry block must report ONLINE/REQUIRED/PASS,
comm OK, three cells, pack >=10.8 V, minimum cell >=3600 mV, consistent extrema,
discharge MOS ON and zero alarms. SOC is recorded but does not authorize motion.
Malformed, incomplete, missing, stale or failed telemetry is fail-closed.
A summary-only DALY line cannot refresh a measurement. Firmware sample age is
included in the host freshness calculation; the existing maximum is 5 seconds.
The wait/monitor loops check it at most every 50 ms, and a partial body has a
1-second deadline. The stream reports existing 2-second polls (750 ms UART
poll timeout); it cannot react faster than those polls. Missing data is stopped
by 5-second freshness plus scheduling/USB latency. This is a host supervision
layer, not a replacement for firmware current, thermal, position and timeout
protections during mechanical contact. Real bus/load timing remains unvalidated.

## Provenance

Firmware/application build ID and SHA remain pinned to `125d981b02a3`; host
runner/DALY/tooling and later documentation have separate commits. No firmware,
Geometry V5, EEPROM, partition or NVS erase change is part of this tooling step.
The frozen flash worktree remains CLEAN at the firmware commit. The original
checkout and gait worktree are preserved. Main has not been integrated offline.

## Startup qualification follow-up

The RF LOWER direction is +1 in the bound profile; the corrected observed
path uses +351 ticks, with no intended-stop pair excluded. The old observed
negative-sign CAD case is superseded. The targeted startup tool covers support
bands independently and reports geometry evidence only. It does not grant
authority or restore Q0 after a reboot. Read-only boot LOAD discards decoded
record values; fresh promoted Q0 are RAM-only. A coherent twelve-joint reference
for the RF LOWER MAX interruption must be established before a startup motion
can be admitted. An earlier LF acquisition (RF UPPER 2107) must not be completed
with the later RF interruption's reference (2106). The package's final report
records the admission decision; this host change does not implement a powered
startup executor.

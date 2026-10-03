# Release session automation — offline preparation, 2026-10-03

The executable distribution lives in
`/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_POST_ABORT_THERMAL_20261003`.
`matdog_01_flash.sh`, `matdog_02_full_calibration.sh` and
`matdog_03_verify_and_finalize.sh` are packaging entry points, using snapshots of
this repository's native runner and the release-stage helper. There is one
calibrator and one serial connection at a time. No-argument invocation and
`--offline-check` perform file/Git checks only. Hardware stages require
`--hardware-session --session-dir /absolute/new/session/path`.

## Current admission: explicit qualified startup program

The distributed package pins the CLEAN firmware build and verified complete
reference from `full_cal_nvs_20261003_134848`. `qualified_path=true` admits only
the tested fixed RF LOWER MAX return, with fresh population/readbacks and the
absolute support bands in STARTUP_RECOVERY_OFFLINE_ASSESSMENT.md. It is not an
operator override or a hardware PASS. Installation continuity, body/passive
joint support against gravity, clear area, disconnect and charger-off are
mandatory physical GO conditions. If those cannot be established without
moving the robot, stop with SAFE_OFF.

1. Flash: all canonical identity/binary/backup/layout/OTA/NVS gates, one
   application-only write, then signature/MAINTENANCE/NONE/SAFE_OFF13 verification.
   Mechanical Q0 does not gate flashing; this stage issues no repositioning.
2. Calibration: consume FLASH_OK and same-boot proof, check fresh encoder
   compatibility, then one operator GO for the entire procedure. The existing
   native runner qualifies the startup path read-only; if needed it executes
   only 21→22→32 and verifies nominal/SAFE_OFF. It then acquires/promotes new
   Fresh Q0, INITIAL RECOVERY, LF/RF/RH/LH, export24/24, SAVE CHECK/SAVE/ACK.
   Historical reference values never enter the normal transform table; old LF
   contacts are not combined with the new capture.
3. Finalization: real operator power cycle; read-only ACK generation/record
   verification and motion unauthorized. Only after hardware PASS do relevant
   regressions, actual-evidence documentation and controlled isolated merge run.
   Dirty trees, different firmware, conflicts, gait ancestry and unavailable
   remote stop integration; no force push. None of this is executed offline.

Any consumed/failed session refuses automatic retry. The scoped startup grant
is explicit and can only prime at the current position and return the three
fixed joints to the immutable references. Unknown poses or support-band drift
refuse before torque or trigger SAFE_OFF. Boot does not start qualification or
restore authority. See the single technical report in the release package for
hashes, test results and residual physical assumptions.

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

Firmware and tools are bound to the new clean release commit, recorded in the
package manifests. The previous candidate125d981 and installed be0c12979e5b
images are preserved. Geometry V5, EEPROM, partition table and NVS layout are
unchanged. Boot LOAD remains diagnostic/read-only; the new immutable reference
is accessible solely by the explicit startup command pair. Original checkout,
gait worktree and historical package snapshots are preserved. No main merge
has been performed offline.

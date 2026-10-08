# 2026-10-06 — thermal verdict from direct reads (v0.2.0-dev.3)

dev.2 stopped LF at UPPER MAX with every servo at 32..35 C: three block-read
temperature spikes on the moving bus 12 (95, 78, 77), each refuted by three
direct reads, tripped the 3-in-30 s latch. The same spikes are in the
2026-10-01 24/24 log (23 of them) and in the 2026-10-03 log.

dev.3, by operator decision:

- block-read PresentTemperature is diagnostic: over 70 C it only opens a
  confirmation;
- three DIRECT samples over the unchanged 70 C limit confirm
  over-temperature; three at or under it refute the block read
  (`BULK_TEMP_ARTIFACT_SUSPECT`, counted, never latched);
- a failed, invalid, incoherent or expired confirmation is
  `THERMAL_TELEMETRY_FAULT`, fail closed;
- the runner starts the firmware's own POST_ABORT RECOVERY once after an
  allow-listed, proven-safe leg failure (same boot only).

Nothing else in the motion or safety envelope changed. A recovery across a
reboot does not exist and was not improvised: details in
`09_Logs/Validation_Reports/MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md`.

## Outcome on the robot (candidate b3fd945)

- Committed-tree qualification PASS: static audit, motion 20/20, 113/113
  mutants, both sandboxed builds. ROBOT_POWERED 1,178,272 bytes `5e2cbd17...`.
- App-only flash over ROM no-stub PASS with exact read-back; boot census
  13/13, READY.
- The two displaced joints had been put back at q=0 by hand while dev.2 was
  still running (torque off, actuator counters unchanged). Verified 12/12
  within 4 ticks of the preserved q0. No firmware recovery ran for that.
- Fresh Q0 12/12, then LF, RF, RH, LH: 24/24 `HARDWARE_CONTACT_CALIBRATED`,
  `all_contact_calibrated=1`, in 8.5 minutes.
- 38 block-read temperature values over 70 C, all refuted by direct reads,
  none latched; one direct read was spurious too (86) and was outvoted.
- SAVE CHECK, SAVE generation 1, wrong ACK refused, exact ACK; after a reset
  the record loads `VALID_ACKNOWLEDGED`, intact, with MOTION_AUTHORIZED=0 and
  no transform admitted. Only `matdog_nvs` changed on flash.
- Not done: real power cycle, cross-reboot recovery route, fixture gates.

Full results and residuals: section "Results" of the dev.3 delta audit.

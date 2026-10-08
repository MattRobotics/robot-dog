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

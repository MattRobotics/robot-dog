# MATDOG CR3 — Persisted CR2-C q0 evidence package

Date: 2026-09-27

The CR2-C RAM coordinator ends at CANDIDATE by design. CR3 needs the already captured current
installation evidence to remain usable after an application reboot without pretending the raw
servo centre is q0 and without silently rebinding the measurements to a future geometry model.

The firmware therefore carries a compact, read-only snapshot of the **CANDIDATE summary** whose
source is:

`09_Logs/Validation_Reports/Calibration_Q0_CR2C_2026-09-27/q0_status.txt`

SHA-256:

`37290f77a5a44db5cacbe145dc1014e83d7d79296bc83e5cd181c74e18548b7f`

The snapshot freezes:
- the twelve current physical-unit identities and bus addresses;
- the twelve measured q0 ticks;
- capture session 1;
- 9 samples/joint and zero spread;
- the six Geometry V5 provenance hashes in force during the capture.

It does **not** freeze an operational transform. On every use the snapshot is reconstructed as a
CANDIDATE and is passed through CR3 `acceptQ0Candidate()` and `promoteAcceptedQ0()`.
Promotion additionally requires an explicit current-installation confirmation. A geometry hash
change makes the package unusable rather than silently current.

This is project-side persistence only. It performs no EEPROM write and changes no servo state.

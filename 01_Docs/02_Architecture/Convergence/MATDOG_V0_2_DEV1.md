# MATDOG Controller 0.2.0-dev.1 software convergence

The local integration branch starts exactly at
`aee49bf306f88d56562634aeba08b767f92b247c`, the clean Wi-Fi V3 head.
Persistence `7b258b36c257bd455f135aee2667d035c4929544` and post-abort/startup
`1a5e0085098eeb907319f6f1a00693869444d9cf` are ancestors and are not replayed.

The authoritative owners are that baseline's orchestration, command routing,
calibration/persistence/recovery, actuator policy, ServoBus, BNO085 driver,
Wi-Fi/portal/TLS, OTA writer, layout and build safety infrastructure. The motion
source is `e1704719979789cd9c9f18741e4546725558797e`; only the audited pure runtime
library, tests/tooling and frozen evidence are selectively copied. Current
installation-specific servo allocation and encoder direction remain authoritative.

This development candidate compiles pure G1 through G5-A motion modules without
connecting them to physical command paths. General motion remains unauthorized,
RESTORE is not implemented, operational JointLimits remain unapproved and OTA
ingest remains disabled. The live BNO085-to-ImuSnapshot adapter is deferred.

The pre-convergence archive is external and immutable:
`/home/matteo-manicardi/MATDOG/MATDOG_PRE_CONVERGENCE_20261005`.
The audit report SHA256 is
`79789642fd25335eee9aa82184724dee2cba16d22d03def04f4731806ff688fb`.
The preservation receipt SHA256 is
`9da94451d135191077b4021276c6809f1f493b5279fa467bdedc3f31f8d23354`.
Fresh validation, source provenance, selective-port and conflict manifests are
written outside historical worktrees under
`/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_V0_2_DEV1_CONVERGENCE_20261005`.
Historical PASS evidence is retained unchanged and is not fresh qualification.

No hardware, serial ports, flash, NVS, EEPROM or OTA is accessed in convergence.
No fetch, pull, push, release tag or diagnostic PHY erase is permitted. Hardware
validation requires separate authorization after independent source review.

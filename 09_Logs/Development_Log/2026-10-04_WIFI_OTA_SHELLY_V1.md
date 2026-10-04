# MATDOG Wi-Fi / provisioning / OTA V1 — development log

Mandate: V3 handoff and operational TXT attached 2026-10-04; offline only.
Branch `feat/controller-wifi-ota-shelly-v1`, independent worktree
`/home/matteo-manicardi/MATDOG/worktrees/robot-dog-wifi-ota-shelly`.
Immutable provisional base `1a5e0085098eeb907319f6f1a00693869444d9cf`.
The original checkout is dirty and untouched. Other worktrees (post-abort,
gait and diagnostic a06314e) remain independent. The latest local persistence
thread and post-abort reports describe offline qualification, not completed
hardware persistence acceptance. **CAL_PERSIST_BASE remains BLOCKED** pending
hardware evidence; do not promote this branch to a final release.

| Classification | Surface | Work / complexity / gate |
|---|---|---|
| EXISTS | WifiPolicy, snapshot, retry/backoff | retain host-tested lifecycle |
| MODIFY | WifiManager | single worker owns radio, asynchronous scan, profiles, AP; high |
| ADD | Config/storage/policies | version/CRC standard NVS, test-before-commit, recovery; high |
| ADD | Offline portal | AP-only authenticated writes, session/CSRF, bounded forms; high |
| MODIFY | HttpTransport | preserve mailbox and sole OTA writer; TLS gate and lifecycle; high |
| RISK | HWCDC | 3.3.11 transport activity is not proof of service session; sleep OFF fallback |
| RISK | TLS | library available; identity provisioning and heap/handshake need hardware qualification |
| DEFER | Jetson handover | contract only; no automatic radio disable |
| DEFER | HW E2E | all flash, OTA, reboot, servo and NVS device operations prohibited |

Diagnostic a06314e is reference only: no PHY erase/marker imported.
Source OTA ingest stays 0; partition layout stays MATDOG_16M_2x5M_NVS_V1.
No shared binaries, manifests, credentials or runtime are modified.

## Implementation and offline closure

Worker radio owner, dual profiles/roam/AP, standard network NVS, transactional test,
AP-local portal security, optional TLS/pinned client and atomic mailbox correlation
implemented. No calibration data algorithm or partition layout changes.
Physical admin/AP provisioning remains for future hardware validation; no credentials
were provisioned on a device. HWCDC adapter retains NO_SLEEP until session proof exists.
Remote reboot and hardware TLS/OTA acceptance remain BLOCKED/TO_TEST; ingest 0.

After the workstation power loss the worktree survived; temporary logs/tools did not.
Verification was rerun into gitignored `build/verification` on durable disk. Added a real
WifiManager/fake-radio test and real loopback TLS rejection tests. Inherited host fixture
and audit/linkage drift was corrected without changing calibration implementation.
Detailed evidence and final artifact receipt are linked from the
[V3 report](../Validation_Reports/2026-10-04_WIFI_OTA_SHELLY_V3_OFFLINE.md).

Implementation commit: `96656917813408eb0fda208913ac9b83f1b7b7c5`. Full host suite and global static audit PASS;
network 5279/5279, real adapter 108/108, client 13 tests, V3 mutation 4 tests, DOM smoke PASS.
Final documentation commit is the CLEAN build source recorded in the generated receipt.

Final client review disabled Python legacy Common Name fallback: SAN is mandatory.
Added missing-SAN and insecure-context negative cases before final source freeze.

# MATDOG — recovered commissioning implementation, 14 October 2026

The original workspace and Claude worktrees were accessible. Base PR #49 was verified open/draft at `73f86a6508daf1c69548489b5bac2f2f1a90c0bd`. Runtime dirty changes (17 tracked plus 3 untracked files) were preserved before editing in a private patch/tar snapshot with SHA256. The original checkout and Dashboard worktree remain intact. Host bridge `102c0ba` and viewer `6ad36b0` were integrated by ordinary cherry-pick; no reset, clean, rebase, force-push or merge was performed.

## Delivered code

- Recovered Claude Controller/USB adapter, commissioning admission, telemetry, RAM limits, Session, two-tick inward PRIME and staged entry. Experimental parameters remain in `CommissioningParameters` RAM; hardware limits, exclusive authority, readback freshness, power/temperature/BMS, deadman, STOP and watchdog remain enforced. No SafeActuator policy relaxation or persistent calibration/EEPROM/NVS change.
- Corrected DIAGNOSTIC scope: the reviewed VALIDATION path requires a compatible explicitly selected RAM mode. Mode changes clear both reviewed/commissioning bindings and limits, while retaining RESTORE transforms; neither mode selection nor RESTORE grants a permit.
- Corrected complete `@TEL` transmission: bounded single record, zero USB TX timeout, whole-record free-space check, 12288-byte ring, monotonically advancing sequence even when a record is skipped. The larger ring also fits ordinary diagnostic bursts. Hardware throughput remains to be measured.
- Entry duration includes the asynchronous write box: for HIP/UPPER, start deviation + tracking + half sample step + 1 microradian must fit the loaded tube. Quintic stages slow automatically within the RAM segment timeout; an impossible budget/duration is refused. The four independently advanced LOWER ranges are covered by the separate Cartesian-box proof.
- Existing private TLS can be referenced by `MATDOG_TLS_IDENTITY_FILE` outside all worktrees; no header is copied into repository or exported artifacts. Its digest is checked before/after compilation and recorded. The real recovered certificate/key pair matched and certificate validity was checked offline. Listener behavior remains a hardware check.
- Dashboard separates freshness, RESTORE, permit, fault, capability and transport errors; measures RTT, retries GET at most once and never replays POST; the browser accepts a LAN host Twin URL. Host UI keeps real LIVE, FAKE_DEVICE, TRACE and synthetic data distinct.

## Geometry and RAM policy

See `GEOMETRY_ENTRY_REPORT.md` and `entry_geometry_bundle_20261014.json`. The 10-mrad symmetric entry tube is BLOCKED by an LF UPPER/LOWER collision; it is not an approved hardware entry. The 4-mrad computational bundle covers 120 non-adjacent pairs, all 12 adjacent joints without excluded triangles, independent LOWER progress, and the downward-only 3-mm ventral envelope including adjacent HIP links. Physical cover containment, fixture/cables and encoder/installation/mapping accuracy are external checks, not claims supplied by this computation.

For the measured historical start, use the 4-mrad bundle and RAM `tracking_tolerance_rad=0.0035`. A `segment_timeout_ms=30000` can accommodate a small start deviation; it does not relax the geometric budget or actuator bounds. The default 6-mrad tracking value is intentionally generic and is NOT admitted against a 4-mrad entry tube. Loading a different suitable policy/certificate needs no firmware rebuild. No manual precision repositioning or synthetic approval is prescribed.

## Offline evidence and freeze

The final clean source commit will be compiled after this document is committed. Fresh final campaign receipts, independent read-only review, exact application/manifest/hash, toolchain identity and `FLASH_GO_NO_GO.md` are kept outside the repository under `/home/matteo-manicardi/MATDOG/verification-artifacts/RUNTIME_COMMISSIONING_20261014/`. Their results will be recorded in PR #49 after checks complete; this document does not predeclare them PASS.

The full static audit also runs the host owner suites. The additional final release runner executes Motion, RESTORE, lifecycle, supervised/paired/quantized pipeline, commissioning composition, network/DOM and build identity suites. The retained large historical route/stride certificates are checked by digest/status rather than falsely regenerated. Viewer/bridge/browser tests use simulation and cannot certify Android LIVE or servo movement.

## Hardware / APP-ONLY boundary

No serial port was opened, flash command run, torque applied or motion executed. The robot USB by-id node was absent during offline preparation. The host LAN address observed was `192.168.1.149`; Android reachability is untested. The original A/B 16-MiB files and independent recovered copy match `9a2bf4bd25b581abc50eabadb70c5c4f54c802c789fdbd908723040939e76c83`. They predate the 13-October APP-only write; the installed application readback separately matches `5577bd3c8da5f6c44d8fb9ca9b3d049399e73ccf37d218e716b773e3a87b5914`. No new full dumps or archive changes are required or performed.

The canonical flasher is unchanged. Final external build products can be staged into its ignored `build/esp32.esp32.esp32s3/` directory after manifest verification; the worktree stays on the compiled frozen HEAD. This avoids changing a canonical gate or confusing a later documentation HEAD with compiled source. Flash remains NO-GO pending fresh hardware preflight and explicit authorization for the exact SHA256; Motion additionally requires its own phase authorization and physical checks. PR #49 stays draft, with no merge or stable release promotion.

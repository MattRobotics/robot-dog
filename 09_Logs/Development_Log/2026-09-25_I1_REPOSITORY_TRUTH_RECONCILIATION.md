# I1 — Repository truth reconciliation

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1` (base `feat/h0-current-leg-preflight-v1` @ `b95ea31641609fbc29c5d67dd1deb776d59c2504`)

This closes gate **I1** of the MATDOG NextGen software-only integration workflow (V3 handoff,
2026-09-25). Purpose: audit `ROADMAP.md`, `ARCHITECTURE.md`, `DEVELOPMENT_GATES.md`, the root
`README.md`, `CHANGELOG.md`, `REPOSITORY_VERIFICATION_INDEX.md`, the actual code/test inventory,
build manifests, calibration current-truth documents, geometry provenance and flash/recovery
evidence against each other, resolve stale CURRENT documentation, and explicitly retain the six
legacy open items from the historical Geometry Compiler V5 snapshot. This document does not
rewrite dated historical facts; it records what was found and what was fixed.

## I0 audit summary (preceded this gate)

- `origin/main` = `2017277d8fb74a430f1f75593ace035dfa717d12` — confirmed.
- `origin/feat/h0-current-leg-preflight-v1` = `b95ea31641609fbc29c5d67dd1deb776d59c2504` — confirmed.
- H0 vs `main`: 53 ahead / 0 behind — confirmed.
- `feat/controller-nextgen-integration-v1` did not exist locally or on `origin` before this
  session; worktree directory did not exist. Created from exact H0 HEAD, no content commit, pushed
  with upstream set, remote HEAD verified equal to local.
- All five other worktrees clean; no stashes; no local-only commits on any branch; no open PRs.
- Historical full-flash backup `~/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-10.bin`
  verified byte-for-byte: 16,777,216 bytes, SHA256
  `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32` — matches the value recorded
  in `VALIDATION.md` and in the V3 handoff exactly.
- Passive USB check (no hardware modification): `lsusb`, `/dev/ttyACM*`, `/dev/serial/by-id` all
  show no ESP32 present. **`USB_STATE=ESP32_NOT_ENUMERATED`.** Consequently
  `POWER_ISOLATION_GATE=UNPROVEN` and `FLASH_RECOVERY_GATE=DEFERRED_UNPOWERED` for today's session.
  This does not block software work.

## Findings

### F1 — README.md snapshot banner was stale (fixed)

The root `README.md` opened with `**Current snapshot — 2026-09-15**` and listed "the immediate
milestone is the no-motion `ROBOT_POWERED` validation sequence" as current, while the same file's
own "Where we are" section (updated through 2026-09-24) already showed G3/G3.1 PASS, the DALY KEY
gate closed, and the external USB service port validated. The banner predated five days of
material progress recorded lower in the same document.

**Fix applied:** banner updated to 2026-09-25, now states that `ROBOT_POWERED` no-motion is
VALIDATED, names the firmware actually installed on the robot (`6322563`, not flashed since), and
states the active workstream is software-only NextGen integration on
`feat/controller-nextgen-integration-v1`, none of it hardware-tested.

### F2 — H0 leg preflight was undocumented in the two canonical gate-status owners (fixed)

`src/servo/ServoPreflight.*`, the `@SERVO PREFLIGHT` command, and
`H0_LEG_PREFLIGHT_RUNBOOK.md` (444 offline checks, IMPLEMENTED / COMPILED / OFFLINE TESTED, not
flashed) existed in the repository but were absent from both `ROADMAP.md` (Phase 2, stage 4 row)
and `DEVELOPMENT_GATES.md` (G4 section) — the repository's own stated canonical owners for "what
exists" and "what must be true before a stage may begin." A reader consulting only those two files
would not know H0 preflight exists.

**Fix applied:** added `@SERVO PREFLIGHT` / H0 to the G4 "already implemented" list in both files,
with its offline-test count and hardware status, cross-referenced to the runbook.

### F3 — `REPOSITORY_VERIFICATION_INDEX.md` is self-aware historical, not stale (no fix needed)

This root-level file's name suggests a live repository-wide verification index, but its own
header correctly labels it `HISTORICAL / REFERENCE — 2026-08-11 snapshot` and defers current
status to `README.md` / `ARCHITECTURE.md`. The root README's own documentation table already links
it as `(HISTORICAL/REFERENCE, not project status)`. No inconsistency found; left unchanged.

### F4 — Six legacy open items existed only in the historical snapshot (fixed)

The seven items listed in `REPOSITORY_VERIFICATION_INDEX.md` under "Issues recorded for the
planned next phase" (2026-08-11) were, for six of them (the live-FK status mismatch plus the five
items carried in the V3 handoff §7), not restated in any *current-truth* document. `#2` (8
unresolved conservative clearance lower bounds) was already correctly retained in
`CALIBRATION_BOOTSTRAP.md` §7. The other five/six had no current home.

**Fix applied:** added §9 to `05_Firmware/MATDOG_Controller/CALIBRATION_SOURCE_PRECEDENCE.md`
(the current-truth calibration audit document), restating all six items with an explicit status
and owner each. None was resolved — per the V3 handoff's explicit instruction not to fix the
live-FK enum mismatch "by blindly changing either string." See that section for the full table.

### F5 — Code/test inventory matches documented claims (no fix needed)

`05_Firmware/MATDOG_Controller/src/` contains exactly the modules `ROADMAP.md` and
`DEVELOPMENT_GATES.md` describe (`actuator/`, `calibration/`, `core/`, `imu/`, `network/`,
`power/`, `servo/`, `status/`, `update/`). `scripts/tests/` contains one host suite per documented
area (`test_servo_population.cpp`, `test_daly_protocol.cpp`, `test_wifi_policy.cpp`,
`test_actuator_authority.cpp`, `test_actuator_write_policy.cpp`, `test_calibration_geometry.cpp`,
`test_ota_policy.cpp`, `test_calibration_domain.cpp`, `test_calibration_manager.cpp`,
`test_servo_profile.cpp`) plus the OTA-partition and build-manifest Python suites and the two
static-audit suites — matching the "10 host suites, 5379 checks" baseline. No orphaned or
undocumented module found.

## Reconciliation matrix

`IMPLEMENTED != VALIDATED`, per the repository's own status vocabulary (root README).

| AREA | IMPLEMENTED | OFFLINE_TESTED | HARDWARE_TESTED | TO_IMPLEMENT | TO_TEST | TO_DESIGN | FROZEN/HISTORICAL | SOURCE |
|---|---|---|---|---|---|---|---|---|
| Controller V0.1 baseline | yes | yes | yes (`USB_ONLY`) | — | — | — | FROZEN tag `matdog-controller-v0.1.0` | README, ROADMAP #1 |
| `ROBOT_POWERED` no-motion (G2/G3/G3.1) | yes | yes | yes (no-motion only) | — | — | — | — | DEVELOPMENT_GATES G2/G3/G3.1, VALIDATION.md |
| LED ring low-level driver | yes | n/a (hw-only) | yes (as part of G3 conservative LED test) | — | — | — | — | `src/status/LedRing.*`, DEVELOPMENT_GATES G3 |
| DALY KEY / power baseline (PR #27) | yes | n/a | yes (live, 2026-09-19/24) | — | BMS KEY-config persistence across a true power cycle | — | — | `04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md` |
| External USB service port (GPIO19/20) | yes | n/a | yes (2026-09-24) | — | — | — | — | same |
| Wi-Fi station runtime (W1) | yes | yes (1141 checks) | no | — | association, DHCP, RSSI/IP, reconnect, heap stability | — | — | ROADMAP #10, `src/network/` |
| OTA-A update core | yes | yes (561+40+51 checks) | no | transport, authentication | everything hardware-side | — | — | ROADMAP #13, `src/update/` |
| ActuatorAuthority + OTA-B inhibit | yes | yes (751 checks) | no | — | host disconnect / armed-transaction case (no transaction exists yet) | — | — | ROADMAP #5 |
| Safe Actuator Layer policy core | yes (decision core only) | yes (335 checks) | no | runtime adapter to `ServoBus` | — | — | — | ROADMAP #14, `src/actuator/` |
| Calibration domain + Manager (LF V25 oracle) | yes (foundation only) | yes (702+322 checks) | no | execution engine (18 phases), direction measurement | — | persistence/promotion boundary | LF V25 replay is HISTORICAL_REPLAY | ROADMAP #7, CALIBRATION_SOURCE_PRECEDENCE.md |
| H0 leg preflight (`ServoPreflight`) | yes | yes (444 checks) | no | — | full 12/12 live run | — | — | DEVELOPMENT_GATES G4 (this gate), H0_LEG_PREFLIGHT_RUNBOOK.md |
| Geometry Compiler V5 canonical bundle | yes (compiler + artifacts) | yes (299/299 at merge) | LF only | — | RF/RH/LH per-leg hardware campaigns | — | evidence record FROZEN/HISTORICAL (2026-08-11 snapshot); artifacts REUSED as current | CALIBRATION_BOOTSTRAP.md, REPOSITORY_VERIFICATION_INDEX.md |
| Formal recalibration (leg population / H1) | no | no | no (blocked) | full engine | 12/12 leg servo H1 | — | last formal H1 = 6/12, HISTORICAL, not superseded | ROADMAP #8 |
| Diagnostics/Maintenance beyond H0 (G4) | partial | partial | no | `SYSTEM_SELF_TEST`, `SOURCE_SIGNATURE`, `PROFILE_AUDIT`, consolidated health | — | — | — | DEVELOPMENT_GATES G4 |
| Service / Provisioning | no | no | no | full design + implementation | — | yes | Provisioner V6 FROZEN oracle | DEVELOPMENT_GATES |
| QC | no | no | no | full design + implementation | — | yes | Bench QC V6.1 FROZEN oracle | DEVELOPMENT_GATES |
| HostLink semantic layer | partial (`CommandRouter` = USB-only adapter) | partial | no | transport-neutral schema | — | yes | — | ROADMAP #9 |
| Embedded Web UI (UI-0..UI-9) | no | no | no | everything | — | UI-0 architecture contract only | — | DEVELOPMENT_GATES, ARCHITECTURE.md |
| Motion / IK / gait / stabilization | no | no | no | everything | — | yes | historical C4-C stand trajectory is an oracle/reusable asset, not authorization | ROADMAP #15-24, V3 §15.3 |
| Flash/recovery evidence | n/a | n/a | historical backup byte-verified today | fresh read-back of current device | requires proven power isolation first | reconstruction of installed-app source (`6322563`) binary | 2026-09-10 backup is the only full-flash evidence; predates installed app | V3 §3, VALIDATION.md |

## Legacy items — explicit retention

All six items are now carried in a current-truth document, not only in the historical snapshot.
See [`CALIBRATION_SOURCE_PRECEDENCE.md` §9](../../05_Firmware/MATDOG_Controller/CALIBRATION_SOURCE_PRECEDENCE.md#9-legacy-open-items-carried-forward-i1-2026-09-25)
for the full table. Summary: none resolved, none silently patched, each has an explicit owner
(`I5` Calibration Execution Architecture, a future per-leg hardware campaign, or "no violation
found, rule stands").

## Outcome

`I1 = PASS`. No architectural decision, contradiction, or safety-relevant ambiguity was found —
only stale/missing documentation, corrected above. Proceeding to `I2` — LED Status Manager.

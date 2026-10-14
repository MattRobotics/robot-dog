# MATDOG 2026-10-14 — Claude Code implementation work order

**Base**: PR #47 head `13404187b1a2abf1ac1aafb4bf379b58cac36916`. This branch adds *only* a tested **recognition-only** `FixturePoseClassifier.h` + nine-case standalone unit test. It **does not** grant motion authority or certify collision-free transitions.

## Critical existing behavior to preserve

- Already-installed source `b147f3884b227ec819e98a6f00c214a0aac2278e`; APP SHA `5577bd3c8da5f6c44d8fb9ca9b3d049399e73ccf37d218e716b773e3a87b5914`. Any rebuilt app MUST have a new verified digest.
- Gen-1 persistence validated, manual RAM-only RESTORE works. No Full Calibration, q0/EEPROM/NVS writes, bootloader/table/otadata changes.
- `pairedProneCursor` uses the certified 6-mrad route tube; `AdaptiveStand::CLASSIFY` uses it for *pose recognition*, which incorrectly rejects the actual prone fixture position. **Do not enlarge the certified tracking tube**.
- The new classifier is pure, conservative and **recognition only**. It consumes radians in order LF(H/U/L), RF, RH, LH, and accepts the rounded measured prone pose with a max delta 7.65 degrees; it never permits hardware motion.
- Measured [degrees]: LF (-2.29,80.16,-91.76), RF (2.37,79.54,-90.62), RH (-2.55,82.53,-91.41), LH (2.29,82.35,-91.41). All LOWER at physical mechanical contact. Physical contact is **not** an admissible holding target.
- New ventral IMU cover may extend 3 mm below certified STL: inspect/measure; do not silently alter q0/body height/COM.
- Keep stop, freshness, bus singleton, power cut, hard limits and signed position safeguards. No counterfeit 12/12 approved limits or synthetic evidence passed as hardware PASS.

## A. Motion workstream — priority for tonight

1. Inspect `AdaptiveStand.cpp`, `PairedProneRoute.cpp`, `MotionFeedback.cpp`, `LocalValidation.cpp`, `MotionService.cpp`, `MotionExecutor.cpp`, `MotionCoordinator.cpp` and their current host tests. Confirm deployed source vs branch delta. Add the pure classifier to the runtime diagnostic path **without using it as motion authority**.
2. Implement a separate `EntryPlanner` with first PRIME target == **fresh raw/actual servo feedback**, bounded speed/acceleration/step, no discontinuity. Model a staged departure *away* from all LOWER endstops. Independently validate **every transition** against feasible limits, complete joint configuration/collision, fixture and 3-mm ventral uncertainty. Do not rely on quintic interpolation alone for clearance.
3. Create a run-time `FIXTURE_COMMISSIONING` admission profile distinct from loaded/autonomous certifications, explicit local scope, finite TTL, operator presence, fresh IMU and all 12 encoder samples, safe STOP and acknowledgment. Runtime policy may be selected without a future reflash; safety invariants must remain always enforced.
4. Integrate classifier only into the **recognition** stage; preserve the strict existing `pairedProneCursor` for certified route membership and follow-up tracking. A transition from recognized prone to nominal route must require its **own reviewed, bounded evidence**. If the existing 12 joint limits cannot admit the contact start, do not fake approval: add explicit legal pre-entry recovery envelope with measured clearance review.
5. Write negative tests: no fixture/stale/reboot, wrong signs/NaN, physical endstop held under command, any overshoot, bus stale, missing servo, no command from unknown pose, STOP during each transition, no irreversible storage changes, and regression on existing certified profile.
6. Run native C++ host tests including this added test:
```bash
cd 05_Firmware/MATDOG_Controller
c++ -std=c++17 -Wall -Wextra -Werror -pedantic -O2 -Isrc scripts/tests/test_fixture_pose_classifier.cpp -o /tmp/matdog_fixture_pose_test
/tmp/matdog_fixture_pose_test
python3 scripts/tests/run_adaptive_motion_tests.py
```
7. Rebuild ROBOT_POWERED only after integration and all relevant tests. Produce source SHA, binary SHA256, size, part offset/layout checks, manifest, test receipts, no hardware flash or movement. File an exact blocked/ready verdict.

## B. Dashboard/Twin workstream — separate worktree, no conflicting writes

1. Reuse `06_Software/Matdog_Core/viewer/matdog_bno085_fullbody_viewer`: 17 collision STL, canonical URDF, actual/target ghost and validated BNO085 frame/recenter. Do not bundle 6.61 MB into 5-MiB app slot.
2. `MotionDashboard.h` presently uses ~750 ms abort, ~1-Hz polling, 1.5-s stale threshold. Diagnose true response timings/reason codes rather than just lengthening timeouts.
3. Serve a same-origin **host URL reachable by Android** over the actual LAN/AP topology. Never hardcode `localhost` as the phone endpoint. Real feed must include BNO085 quaternion and 12 actual+target values, age/boot nonce, mode and distinguish LIVE from SYNTHETIC/TRACE/STALE. Keep the ESP local AP dashboard usable even with host down.
4. One owner for console/USB; no second serial opener or WebSerial reader. Read-only bridge/fanout with authenticated command path. Measure bandwidth and 10–20 Hz target feasibility before increasing any servo polling.
5. Browser E2E with REAL telemetry is required before claiming Android LIVE. Keep 48 actions marked implemented vs planned.

## Independent adversarial review

Freeze an immutable candidate commit. Use **Codex or another independent reviewer** read-only on that commit, not the implementer's working tree. Specifically challenge: first raw goal/torque transient; 12-joint encoder freshness, negative LOWER limits, collision under full 12D configuration, infeasible fixture clearance, serial-reset effects, stale Web command after reconnect, NVS/OTA integrity, command scopes and fail-closed handling. Record review findings + fixes + final re-test hashes.

## Acceptance evidence

No claim of STAND or WALK until on-fixture operator-run hardware log shows goals, actuals, currents/speeds, 12 mapped transforms, live BNO085, STOP recovery, timestamps and qualitative clearance. WALK suspended only after supervised STAND and a separately authorized bounded segment. PR #47 remains draft, Issue #46 release and Issue #48 RESTORE remain OPEN. No merge or flash without operator approval of precise digest.

**Operator-provided handoff V2 with photographs is the controlling specification**. Treat this work order as a thin source-pinned task list, not a replacement for the full handoff.

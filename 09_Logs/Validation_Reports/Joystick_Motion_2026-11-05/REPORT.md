# MATDOG 1.0.1-dev.15 — joystick Motion, PRONE rest and STOP recovery, runtime cost

**IMPLEMENTED NOT VALIDATED HW.** Offline candidate built from a clean commit; no flash, OTA, serial port, robot API, NVS/EEPROM write, push or merge. dev.14 (`4ff2aad`, APP `71028f95…`) remains the installed reference; its worktree and artifacts are unchanged.

Branch `matdog/dev15-joystick-motion`, worktree `~/MATDOG/worktrees/matdog-dev15-joystick-motion-20261105`, base `4ff2aadf4ed907d39f2b3ad4f9e6af54db5decee`.

## Candidate (round 2, after the independent review of `5ccaa0a`)

| Item | Value |
|---|---|
| Version | `1.0.1-dev.15`, `ROBOT_POWERED`, OTA ingest 0, motion stack `MATDOG_SIMPLE_V1` |
| Source commit | `928b8b73277eb6925372764e59740556fedb5781`, CLEAN |
| APP SHA256 | `660660303a2093d639b963aff9cfa878b560c9d83b56d7f1bd53e8f437fa56c9` (1,690,160 B, 32.2 % of the 5 MiB slot; dev.14 1,669,584 B) |
| Partition table | `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`, layout `MATDOG_16M_2x5M_NVS_V1` (identical to dev.14) |
| Merged image | `c6c7f3575dc82aa4afa31fe62beb62a873a706f22da37e46d7ccfa6eef9998b8` |
| Manifest | `matdog_build_manifest.txt` `3d66982dc25ffae12ab6e9ac01da96cfe0f3ca4f427ecf404277a05bc2b22f38`, BUILD_UTC 2026-11-06T10:07:06Z |
| Artifacts | `~/MATDOG/build-output/dev15-joystick-motion-20261105/final-build-928b8b7/artifacts/ROBOT_POWERED-ota0/` (manifest, ELF, map, SHA256SUMS) |
| Rollback | dev.14 APP `71028f95…` unchanged on the robot and in its build output (not exercised) |

The first candidate (source `478b0f0`, APP `743c0e0c…`) and the intermediate round-2 builds (`cefa8f2`, `93d344c`, `f9302c7`) are superseded.

## Round 2: the three review items

| Item | Root cause | Change | Evidence (offline) |
|---|---|---|---|
| Steering support | Binding support edge RF–LH while RH swings (cycle 2.077, real fillet contacts, URDF COM 2.928 kg): 12 mm lateral sway left +0.349 mm steady (cw), −0.904 mm random_1 | Steered lateral sway 12 → 14 mm (one constant); WALK STANDARD, `kSteer` box and its FCL certificate unchanged | Sweep below; envelope PASS with random support gated > 0 |
| PRONE terminal | The torque-ON terminal had no rest; the operator saw MATDOG stay down after power removal with a slight settle | After the verified PRONE_LAND arrival the ordinary SAFE_OFF releases the torque (not an operator latch): READY / `PRONE_REST`; STAND re-acquires the settled pose through ACQUIRE/REFERENCE/ENABLE | `test_simple_motion` PRONE REST, STOP suite, dashboard contract/DOM, PRONE hardware replay 69/69 to rest |
| Diagnostics | Edge IK returned only the second solve's Newton/FK work; a temperature confirmation left no trace of bulk/direct/outcome | IK result carries `solves` and the summed work; the port reports each direct confirmation (servo, time, bulk, direct or −1, bus outcome); status `thermal.checks/last` | Re-solve replay test; thermal NO_RESPONSE/OK test and status JSON |

### Steering sway sweep (`sway_sweep.txt`, full scenario set, real fillet contacts)

| Steered sway | Min support steady | Min support random | Max compensated stance tilt | Min non-foot clearance | Gates |
|---|---|---|---|---|---|
| Y 12 mm (first candidate) | +0.35 mm (cw) | −0.90 mm (random_1) | 11.09° | 2.23 mm | PASS (random not gated) |
| Y 13 mm | +1.10 mm | −0.18 mm | 11.48° | 2.21 mm | PASS |
| **Y 14 mm (candidate)** | **+1.85 mm (cw)** | **+0.55 mm (random_1)** | **11.87°** | **2.19 mm** | **PASS** |
| Y 15 mm | +2.60 mm | +1.28 mm | 11.95° | 2.17 mm | FAIL: IMU envelope rays beyond the 12° fillet limit |
| X 14 mm (Y 12) | +2.40 mm | +1.22 mm | 11.10° | 2.21 mm | FAIL: backward `WALK_IK_OR_WORKSPACE` |

The lateral sway moves the COM away from the binding diagonal by ~0.75 mm per mm but also rolls the stance feet; 14 mm is the largest value that keeps the whole IMU envelope inside the fillet limit. Nominal and compensated targets stay ≥ 35.9 / 26.2 mrad inside `kSteer`, whose certificate covers every joint combination in the box. A static margin of +0.55 mm is a model result, not a dynamic stability claim.

### Corrections to the first report

- Compensated stance-foot tilt max is **11.09°** (12 mm sway), not ≤ 9.3° (that was the nominal stance tilt); min non-foot clearance **2.23 mm**, not ≥ 2.26 mm.
- `release_in_start` never reaches WALK: support/clearance are **NOT_APPLICABLE** (the script printed 1e9); the steady and random JSON overwrote each other. Both fixed in `run_joystick_envelope.sh`.
- Stack: the call-chain analyzer followed decoding artefacts (calls into the middle of a symbol). Corrected worst direct chains of `SimpleMotion::update`: dev.14 **8096 B**, first candidate 8208 B (reported 8832 → 9104 B).
- "Operational host tests PASS" was vacuous: `run_operational_host_tests.sh` without arguments runs nothing. Round 2 runs them by name (below).
- IK work counters under-counted the fillet re-solve: JOYSTICK worst release really does 8 leg solves, 12 Newton steps, 23 FK (reported 4/8/15).

### PRONE rest

- No OFF before the verified PRONE_LAND arrival; STOP in any descent phase keeps the held-RAW recovery (12/12).
- Lost OFF readback: re-broadcast, then 13/13; no position frame and no enable for 500 releases at rest; a neutral joystick does not take over the `LIE_DOWN` command (completes `PRONE_REST` once OFF is verified).
- STOP/SAFE_OFF during or after the rest: the usual operator latch (`SAFE_OFF_STAND_REFUSED`).
- STAND from a settled pose (±6 ticks on every joint): REFERENCE = settled RAW with torque OFF, 12 enables, STANDING. Outside every certified region: `INITIAL_POSE_OUTSIDE_SAFE_ENTRY` before any enable.
- Settling margin of the scripted terminal to the entry box (mrad, LF RF RH LH hip/upper/lower): 25.3 209.6 84.1 · 25.3 209.6 65.7 · 28.3 211.5 56.5 · 28.3 211.5 55.0. A hip settling beyond ~1.4–1.6° takes the pose out of the entry region.
- Removed the unreachable torque-ON terminal (`State::PRONE`, phase `PRONE_CONTACT_UNVERIFIED`, its STAND/STOP branches).

## Measurements (offline)

Release cost proxy (`run_motion_cost_probe.sh`, libm calls per Motion release, tilted IMU), first candidate → round 2: HOLD100, WALK45 and WALK60 identical (WALK45 sin/cos 242/270 mean/max, inverse trig 64/67, sqrt 94.5/148). JOYSTICK mean sin/cos 329 → 334.5 (14 mm sway, +1.6 %), worst release unchanged 542 / 127 / 202, i.e. ~2× WALK45; work counters of that release now 8 leg solves, 12 Newton, 23 FK. JOYSTICK cost is the same with a level IMU (structural: fillet contact and re-solve in gait and compensation, retargets), not a removable redundancy. **ESP32-S3 release time is not measured offline.**

Stack, worst direct-call chain (Xtensa ELF, `callx` unresolved, corrected analyzer): `SimpleMotion::update` dev.14 8096 B → first candidate 8208 B → round 2 **8208 B**; `generateWalk` 6256 → 6368 → **6368 B**. An intermediate round-2 build reached 8512 B: a field added to `WorldTiltedIkResult` made the compiler hold a 300 B temporary in `worldEdgeContactInverseKinematicsTilted` (80 → 384 B); the solve count now sits in `TiltedIkResult` (frame 80 B with the production flags).

Thermal diagnostics cost in an anomaly: one 24-byte record copy per confirmation already performed by the existing read (no extra bus traffic, logging or allocation); status JSON 4448 B in the test case (budget 4608 B).

## Regression (round 2 candidate commit)

All PASS on `928b8b7` (17/17): build (CLEAN), `run_simple_motion_tests.sh` (incl. JOYSTICK, PRONE REST, THERMAL, PRONE STOP recovery 12/12, IMU slew, fillet re-solve accounting), `run_stop_support_tests.sh`, `run_simple_dashboard_tests.sh` (fixtures 27, DOM 48 commands / 14 envelopes, contract 240, Twin 52, portal budget 28), `run_joystick_envelope.sh` (support > 0 steady and random, clearance ≥ 2 mm), `run_motion_cost_probe.sh`, `test_stand_profile_timing`, `test_http_fence_contention.py` (723), `test_motion_http_transport.py` (97), `test_slack_exempt_session.py`, `test_static_audit_network_v3.py`, `test_stop_adapter.py`, `test_twin_assets.py`, `test_dev_identity_build.py`, `test_release_candidate_gate.py`, `run_motion_convergence_tests.py` (364/364), `static_audit.py` (STATIC_AUDIT = PASS). Twin viewer sources unchanged since the first candidate (vitest 99/99 there).

Orphan operational host tests (no runner) on dev.14 and dev.15: 22 of 23 already fail on dev.14 with the same exit (legacy expectations); `test_stand_profile_timing` passes on both. `test_prone_hardware_replay` (adapted to the rest) still stops at its dev.14 assertion (replayed box margin ≤ 4 ticks on slots 2/5); with that assertion reported instead, 69/69 recorded descents reach the rest with OFF verified.

## Not closed

- **Motion budget on the ESP32-S3**: not measurable offline. JOYSTICK worst release ~2× WALK45 in libm calls (fillet re-solves in gait and compensation, retargets). Gate `MOTION_BUDGET_UNAVAILABLE` kept.
- **PRONE passive support**: offline only; settle, loads and the margin to the entry region must be observed under supervision.
- Random joystick transients: +0.55 mm static minimum; dynamic effects unmodelled.
- Native DISENGAGE/ALIGN still end a STOP in SAFE_OFF; RH_HIP temperature spikes: cause unknown, now recorded with bulk/direct/outcome.

## First supervised hardware sequence (not executed)

1. Flash only after an explicit decision; verify APP SHA256, boot identity 1.0.1-dev.15, gen-1, SAFE_OFF 13/13.
2. STAND150 HOLD 60 s: `timing.max_work`/`first_miss`, `missed_releases`, `max_gap_us`, stack high-water, IMU age, `imu_correction`.
3. WALK STANDARD 45 one cycle + WALK STOP: same records, COMPENSATE section, solves/IK/Newton/FK.
4. PRONE with a hand under the body: OFF 13/13, settle per joint (`pose_region` must stay `PRONE_ENTRY`), loads before OFF (exit record); then STAND from rest.
5. STOP in PRONE_DEPLOY, STAND (`STAND_FROM_STOPPED_PRONE`).
6. Joystick on the fixture, then on the floor at 25 %: each axis, release, page close (1 s timeout), STOP in each; `thermal.last` after any hot sample.

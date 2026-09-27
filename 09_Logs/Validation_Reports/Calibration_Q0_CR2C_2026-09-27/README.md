# MATDOG CR2-C - Current-installation q0 hardware evidence

Date: 2026-09-27
Source commit flashed and measured: 315d4ade6ff0de59f6f3032f9864accb1680c669
Build ID: 315d4ade6ff0
Hardware profile: ROBOT_POWERED
Final verdict: CR2-C PASS - READ-ONLY HARDWARE CAPTURE COMPLETE

## Scope

This package records the first current-installation q0 acquisition after the 2026-08-27
reprovisioning/current mechanical installation. It does not accept or promote calibration and
does not authorize motion.

Permanent state at closeout:

    accepted=NO
    promoted=NO
    transform_admitted=NO
    motion_authorized=NO
    hardware_motion=BLOCKED
    authority=NONE

## Acquisition contract

- operator manually aligned all four legs to nominal URDF q=0;
- physical square/jigs/support used as the reference, not raw 2048;
- robot mechanically supported so gravity could not alter the pose;
- Torque OFF verified on all 12 measured leg servos immediately before capture;
- 9 round-robin samples per joint;
- stability budget 16 ticks;
- fresh census + fresh preflight in the same capture session;
- no Torque ON, GoalPosition, EEPROM, PositionOffset, CalibrationOfs or servo-ID write.

## Result

    capture state       = COMPLETE
    failure             = NONE
    population          = PASS 12/12
    q0 candidates       = 12/12
    samples per joint   = 9
    max measured spread = 0 ticks

| Bus | Leg | Joint | Unit | q0 tick | Delta vs 2048 | Spread |
|---:|---|---|---|---:|---:|---:|
| 11 | LF | LOWER | M33 | 2087 | +39 | 0 |
| 12 | LF | UPPER | ELR01 | 2100 | +52 | 0 |
| 13 | LF | HIP | M22 | 1996 | -52 | 0 |
| 21 | RF | LOWER | NEW03 | 1985 | -63 | 0 |
| 22 | RF | UPPER | ELR03 | 2092 | +44 | 0 |
| 23 | RF | HIP | NEW01 | 2030 | -18 | 0 |
| 31 | RH | LOWER | NEW05 | 2034 | -14 | 0 |
| 32 | RH | UPPER | ELR02 | 2042 | -6 | 0 |
| 33 | RH | HIP | NEW06 | 2081 | +33 | 0 |
| 41 | LH | LOWER | M41 | 2073 | +25 | 0 |
| 42 | LH | UPPER | M42 | 2089 | +41 | 0 |
| 43 | LH | HIP | M43 | 2035 | -13 | 0 |

Distance from raw centre is diagnostic only; q0 was never required to equal 2048. The widest
observed absolute offset is 63 ticks on RF LOWER. Within-session spread is zero on all joints.

## Flash/recovery provenance

Fresh pre-flash full backup:

    path   = /home/matteo-manicardi/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-27_160127_nostub.bin
    size   = 16777216
    sha256 = 339c01f7805b9f44c113074bf9a82b460e14b8fe55e6796ca25b0f24323cb7ce
    method = NO_STUB_FULL

Application flashed to verified active app0 at 0x010000:

    application_size   = 1036560
    application_sha256 = 753936ac1dc12d4af11f6ffaed51a8aaeda5b76f260e35e188a3d065f9f52b59
    verify-flash       = PASS

## Closeout

    pack_v         = 11.3 V
    current_a      = -0.3 A
    soc            = 66.3 %
    cell_max_mv    = 3804
    cell_min_mv    = 3795
    delta_mv       = 9
    alarms         = 0000 0000 0000 0000
    runtime_resets = 0

All 13 installed servos (12 legs + neck rotation ID 51) returned VERIFIED_OFF again at closeout.

The runbook requested pack voltage at both open and close. A dedicated opening BMS snapshot was not
captured in the preserved CR2-C transcripts, so opening pack voltage is recorded as NOT_CAPTURED
rather than reconstructed or guessed. This is an evidence-package completeness note; CR2-C PASS
semantics are the read-only population/q0/torque/stability/provenance facts.

## Next gate

CR3 begins with q0 acceptance/promotion policy. No candidate in this package is operational
calibration until that later gate explicitly accepts, persists and promotes it with current
provenance.

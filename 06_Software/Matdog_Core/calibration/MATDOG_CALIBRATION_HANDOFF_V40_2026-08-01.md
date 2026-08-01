# MATDOG Calibration Handoff — V38 / V39 / V40

Date: 2026-08-01  
Status: native program complete offline; supervised hardware checkpoints pending  
No merge into `main`

## Confirmed LF hardware checkpoint before V38

The individual LF mechanical contacts are complete and repeatable:

```text
M12 UPPER MIN: 1443 / 1443
M12 UPPER MAX: 3443 / 3442
M11 LOWER MIN: 3094 / 3092
M11 LOWER MAX: 1664 / 1666
M13 HIP MIN:   2530 / 2530
M13 HIP MAX:   1595 / 1595
```

The combined HIP V37 cycle completed `Done 20/20`, verified global torque OFF and released the serial port.

The historical endpoint comparison proves that M12 is already URDF-consistent, while M11 and M13 must be reacquired with the firmer motion envelope before replacing HOME.

## V38 — LF complete sequence

NormaCore:

```text
branch: matdog/full-calibration-v38
head: c5eca2774ec3ba4c37319ed2e16ab008236a2cc6
PR: #7 draft
workflow: 100/100 ST3215 tests PASS
```

Token:

```text
LF_LEG_FULL_V38
```

Runner distributed to the operator:

```text
MATDOG_LF_FULL_V38_MODEL_ZERO_PREPARE_AND_LAUNCH.sh
sha256: 27a4ef701d77ed48996923ff0f25045df6db751e0363c9639deba6f8b80a3111
```

## V39 — LH complete rear-leg sequence

NormaCore:

```text
branch: matdog/lh-full-calibration-v39
head: c62a540a52811e2c2e30fda5b76d171bb3761115
PR: #8 draft
workflow run: 30695940942
104/104 ST3215 tests: PASS
```

Token:

```text
LH_LEG_FULL_V39
```

Rear-leg geometry decision:

```text
front-leg parking required: false
M42 upper horizontal: 3072
M41 lower folded/parallel: 3038
```

Runner:

```text
MATDOG_LH_FULL_V39_MODEL_ZERO_PREPARE_AND_LAUNCH.sh
sha256: c744759fcfc245166ef0b469766a5b7204011a695913ebe9a789229b86a2778e
```

The runner hard-blocks until an LF V38 hardware PASS with three q=0 values exists in the evidence archive.

## V40 — complete 24-contact / 12-joint program

NormaCore:

```text
branch: matdog/all-legs-full-calibration-v40
head: 070fc72b1bc8f8e84aae5a51bc16410967dbb435
PR: #9 draft
workflow run: 30696523861
108/108 ST3215 tests: PASS
Station viewer build: PASS
Station release build: PASS
```

Token:

```text
MATDOG_ALL_LEGS_FULL_V40
```

Sequence:

```text
LF six contacts → LF model-zero gate
RF six contacts → RF model-zero gate
RH six contacts → RH model-zero gate
LH six contacts → LH model-zero gate
all 12 accepted q=0 targets
all HIP → all UPPER → all LOWER
verify 12 holds
global torque OFF
```

Final artifact:

```text
id: 8817474539
digest: sha256:46f85da7c61f463593b97e813bcb9bb7f9f6f2e76586e38a369869fc9f41c953
matdog.rs: a955f7de9a1c3405cf4d4e705d545e499162ba3cb378261bb9ca7afcf53999b7
matdog_test.rs: 23d60c377ee40b8f71c8969989b70e8098b0ae7126392d81af4e631f347ee696
Station: 757c015b4cbecbe700668948be75ee142b6d561789b275830a0d0a8ad4dff3c9
```

Runner:

```text
MATDOG_ALL_LEGS_FULL_V40_PREPARE_AND_LAUNCH.sh
sha256: d9baf4c85d22f56b6aabdbe27a45cc557e6c9b6da9d66a136485d6197ff6a617
```

The runner hard-blocks until both LF V38 and LH V39 hardware PASS records exist.

## Operator order

Never run the scripts in parallel.

```text
1. bash ~/Downloads/MATDOG_LF_FULL_V38_MODEL_ZERO_PREPARE_AND_LAUNCH.sh --prepare-and-launch
2. bash ~/Downloads/MATDOG_LH_FULL_V39_MODEL_ZERO_PREPARE_AND_LAUNCH.sh --prepare-and-launch
3. bash ~/Downloads/MATDOG_ALL_LEGS_FULL_V40_PREPARE_AND_LAUNCH.sh --prepare-and-launch
```

Each launcher:

- pins the reviewed remote head;
- materializes the exact Rust sources;
- verifies source hashes;
- reruns all tests and builds Station;
- performs a read-only hardware preflight;
- requires explicit physical confirmations;
- starts Station but never clicks Auto Calibrate;
- archives logs, result and accepted q=0 values;
- verifies the serial port is free after controlled shutdown.

## Permanent safety contract

- Station is the sole serial owner.
- GoalPosition remains standard unsigned ST3215.
- Motion writes are RAM-only.
- No EEPROM, Position Offset, LOCK, reset, ResetCalibration, RegWrite, Action, Save or Freeze.
- Contact corridors and mechanical guards remain URDF-derived.
- Hard-current abort remains active.
- Any failed stage blocks the next stage.
- Verified global torque OFF is required on success and failure.

## Next evidence update

After each supervised run, record:

- all repeated contact ticks;
- endpoint-derived zero candidates;
- endpoint disagreement;
- accepted software q=0 targets;
- final hold positions;
- global torque-OFF result;
- serial-port release;
- archive path and SHA-256 manifest.

Only after V40 hardware PASS should the 12 accepted software q=0 values be written into the canonical MATDOG YAML and used to regenerate HOME, LOW_STAND and NOMINAL_STAND trajectories.

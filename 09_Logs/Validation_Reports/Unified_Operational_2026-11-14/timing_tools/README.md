# Timing diagnostic analysis — QEMU instruction reference (14/11/2026)

`analyze_diag.py` analyses the `@MOTION TIMING` dumps (`MOTION_PMU` lines) of the dev.8/dev.8u P0 diagnostic builds and compares the hardware instructions per release with a QEMU reference. Read-only, no device I/O. Usage: `python3 -I analyze_diag.py <run-dir>`.

## QEMU instructions per CCOUNT (`QEMU_K`)

The QEMU P0 tracer times the Motion sections in emulated CCOUNT. Instructions = CCOUNT × `QEMU_K`. The factor depends on the emulator configuration, not on the ESP32-S3 clock.

| Item | Value |
|---|---|
| Emulator | Espressif QEMU 9.2.2, build `esp_develop_9.2.2_20260417` |
| Options | `-machine esp32s3 -icount shift=3` (8 ns of virtual time per instruction) |
| CCOUNT model | about 40 MHz of virtual time, whatever CPU frequency the firmware configures (QEMU reports 80 MHz to it) |
| Measured `QEMU_K` | **3.077–3.101** instructions per CCOUNT, used as **3.088** |
| Previous constant | 1.5625 (an assumed 80 MHz CCOUNT model): the reference was under-counted 1.98× |

**Measurement:** `qemu_k_calibration/qcal2.ino`, built with the bench FQBN `esp32:esp32:esp32s3:CDCOnBoot=default,PSRAM=disabled,FlashMode=dio,FlashSize=4M`, Arduino ESP32 3.3.11.
- With interrupts disabled, three loops of 200 000 instructions: zero-overhead `loop` of `nop`, ALU + taken branch, loads and stores.
- `shift=3`: 64 498–64 995 CCOUNT for every mix.
- CCOUNT scales with the icount step: `shift=2` gives K ≈ 6.2, `shift=4` gives K ≈ 1.52.
- Outputs: `qemu_k_calibration/qemu_shift{2,3,4}.txt`.

**Cross-check against the hardware PMU of 14/11 (build `0526cf6b9d79`):**
- WALK NOMINAL is pure computation with no servo-port call. With K = 3.088 it is 0.92–0.99× the tracer.
- The remaining hardware excess sits in the sections that call the real `SimpleServoPort::resolve/decodeRaw` (calibration lookups and checks), which the QEMU bench replaces with a one-line stub.

**Re-measure `QEMU_K`** if the emulator build or the `-icount shift` changes.

## Effect on the 14/11 session

- **Acceptance band:** unchanged (0.7–1.3).
- **Reference:** the same tracer means, rescaled from K = 1.5625 to K = 3.088:

  | Class | Reference |
  |---|---|
  | WALK | 794–866 k |
  | RECOVERY | 1136–1314 k |

- **Verdicts:**

  | Class | Hardware / reference | Verdict |
  |---|---:|---|
  | WALK | 1.21× | DOMINANT `istall_mem` (0.46 of the cycles) |
  | RECOVERY | 1.41× | INCONCLUSIVE (still outside the band) |
  | OTHER | — | DOMINANT `istall_mem`, unchanged |

Evidence: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_PATCHA_TIMINGBASE_20261114/qemu_k/`.

## Fixture validation suite (15/11/2026)

`fixture_suite.py` drives the whole fixture sequence (blocks A–G) from a single start, with the firmware's existing USB commands only: `@STOP`, `@MOTION STAND/PRONE/WALK STANDARD/WALK STOP/WALK MODE`, and `@MOTION JSON` with owner `local` for the joystick and the 32 mm/s preset. It checks every command three ways: its ACK, the real Controller state over HTTP, and its completion.

**Safety**
- Before every movement it checks: BMS OK, fresh, no alarm; health not FAULT; IMU valid; no STOP or command in progress; power READY; expected state; 12 joints fresh, status 0, < 65 °C, ≥ 10.0 V.
- A FAULT, a BMS loss, an unexpected state or a timeout ends the suite with one `@STOP`, unless a STOP is already verified.
- It never resumes automatically, never resets, never retries.
- Joystick values are checked against the published range of `SimpleMotion::joystickMode()`: `walk_mode.joystick`, else `walk_mode.selected`.

**Usage**
1. Run `python3 -I fixture_suite.py <run-dir> [--plan]`. Start it first: it is the only USB owner (no-reset opener `cr5_stand50_session.open_serial`, path in `OPENER`) and it creates the run directory.
2. Then start `http_watch.py <run-dir>` from the dev.8u session tools. It records `http.jsonl` and queues `@MOTION TIMING` only while Motion is not engaged.
3. To continue after an abort from the held STOP it left: `--start <step> --resume-stand <mm> [--skip <steps>]`.
4. Analyse with `analyze_diag.py`. To analyse several recordings as one session, concatenate them in time order: the PMU sums are since boot and never reset.

**Summary:** `suite_summary.py <run-dir>...` prints, read-only, every step, D1, joint temperatures, the IMU correction and the last state.

Hardware record of the first run: `../HW_VALIDATION_8530d3a_20261115.md`.

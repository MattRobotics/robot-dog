# MATDOG — LED ring RMT DMA + dirty-frame: hardware validation (2026-10-10)

**Scope:** LED WS2812B transport only, no motion authorization and no general firmware/release acceptance.
**Provenance:** validated local firmware source commit `fee9383156fad914b03af27788c5519954e65fca` (parent `b3fd945bdaf37d192d97b605ac0f59b67f1dba45`). These historical commits are **not** imported as ancestors of public `main`; only the two LED source file blobs are selectively integrated.

## Firmware and installed image

- Installed and read-back verified: `ROBOT_POWERED`, `0.2.0-dev.3`, `build_id=fee9383156fa`, `GIT_DIRTY=NO`.
- Application SHA-256: `b3e2f01659fc4b03745616cfaf7185c747b3e98fc837f9d9b543e4af08e3bfc0`, 1,179,920 bytes.
- Active RMT transport at boot: **`LED_TRANSPORT=RMT_DMA`** on ESP32-S3 GPIO47, 12 WS2812B GRB pixels. The built candidate also has an explicit DMA-to-legacy fallback, not exercised in this successful run.
- The deployed image **is not** an image built from `main`. A rebuild from `main` produces a different Git provenance and build ID. **DO NOT FLASH MAIN**.

## Checks performed

| Gate | Observed result |
|---|---|
| Preflight | Prior dev.3 image `b3fd945bdaf3` identified; `READY / RUN / MAINTENANCE / NONE`; no actuation authorization |
| Full flash backup | 16 MiB read-only backup verified on the ASUS; **kept private, not committed or uploaded** |
| Flash | Official application-only flash and read-back verification **PASS**; no changes to bootloader, partition table, NVS, calibration, Wi-Fi or BMS |
| Boot | `LED_TRANSPORT=RMT_DMA`, IMU PASS, expected servo census **13/13**, calibration persistence `VALID_ACKNOWLEDGED`, BMS communications OK |
| LED observation | **10 minutes**, SoftAP active with **zero connected clients**, 6 steady green segments: **no observed spurious blue/red pixels, flashing or brightness jumps**; no reset or fallback to legacy |
| Three-servo scan | Median 17.5 ms vs historical dev.3 median 18 ms |
| Servo census 11..55 | 13 found, 920–927 ms vs 921–932 ms on dev.3 |
| Startup servo census | 4,007 ms vs prior 4,002–4,023 ms |
| Single-servo ping | Max 400 µs; gate 5,000 µs |
| Available heap | 162,636 B vs 166,092 B on dev.3 (DMA buffer overhead approximately 3.4 kB) |
| Offline gates | Static audit, full host suite, dirty-frame host and transport DMA host tests PASS; target compilation PASS |

## Limits of acceptance

- The LED-only behavior is **hardware-validated under the tested conditions**; the historical event rate before the update was not measured, so absence of a rare event is not statistically proven.
- The DMA and dirty-frame changes were deployed together; their individual effectiveness cannot be distinguished from this observation.
- No oscilloscope waveform was captured; real DMA-to-legacy fault fallback was not provoked.
- No movement or calibration-with-motion was performed or authorized. `MOTION_AUTHORIZED=0`, `RESTORE=NOT_IMPLEMENTED`.
- The AP portal HTTP 403 fix is **not present** in this installed LED candidate and remains a separate integration task.
- Private backup images and credential-bearing NVS dumps must not be published.

## Source integration policy

Public `main` receives only the firmware source contents of `src/status/LedRing.cpp` (Git blob `e3c7cfe8b1ade6976b8634127f510b615284ff84`) and `src/status/LedRing.h` (Git blob `0dd26b9402f16969f5057117f61b2af4c5d5a806`), plus documentation. No dev.3 history, third-party reverse-engineering extracts, image binaries or private backup artifacts are imported. Source equivalence is not equivalent to installed binary identity or to release acceptance.

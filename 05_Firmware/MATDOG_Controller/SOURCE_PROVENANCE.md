# MATDOG Controller V0.1 — Source Provenance

This audit was performed before any line of the unified controller was written, per
`MATDOG_CONTROLLER_V01_INTEGRATION_HANDOFF_REV3_USB_ONLY_2026-09-15.md` section 29.
Original standalone files were not modified; nothing here is a rewrite from first
principles — every module below is an adaptation of a specific, hashed, hardware-tested
source.

## Repository state at audit time (2026-09-15)

```text
branch (before this work): main
HEAD: 03992a473bba9220f32172ac57ac5d860f3d1462
remote: https://github.com/MattRobotics/robot-dog.git
latest merged: PR #23 "feat(viewer): add validated BNO085 full-body viewer"
working tree: clean
```

Feature branch for this work: `feat/matdog-controller-v01`.

## Candidate sources found under `~/MATDOG/runtime/esp32`

| Source | Path | Size | mtime | SHA256 | Selected |
|---|---|---:|---|---|---|
| ST3215 commissioning (frozen, committed) | `05_Firmware/ST3215_Bench_Tools/Bench_QC_V6_1/matdog_servo_commissioning.ino` | 131386 | 2026-08-25 | `74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd` | YES (transport only) |
| ST3215 commissioning (runtime copy) | `~/MATDOG/runtime/esp32/matdog_servo_commissioning/matdog_servo_commissioning.ino` | 131386 | 2026-08-25 | `74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd` | identical to frozen — confirmed byte-for-byte, not used directly |
| ESP32-S3 bring-up | `~/MATDOG/runtime/esp32/matdog_s3_bringup/matdog_s3_bringup.ino` | 1512 | 2026-08-22 | `cb72471041609a72fd9bc770e53dbf046175ebb3d7a5832bd24808e032de95b3` | reference only (chip/flash/PSRAM identification pattern) |
| Servo UART bring-up | `~/MATDOG/runtime/esp32/matdog_servo_uart_bringup/matdog_servo_uart_bringup.ino` | 1118 | 2026-08-22 | `63efdb6a7118a9530cf7a6051646711f58dc4427366fb3df8d80c3c0cbeca336` | reference only — confirms GPIO17/18 @ 1 Mbps SCServo init pattern |
| BNO085 Phase C3 DCD control | `~/MATDOG/runtime/esp32/matdog_bno085_dcd_phase_c3/matdog_bno085_dcd_phase_c3.ino` | 9260 | 2026-09-05 | `51bb3016ac20226b812e1e892328768495348278cb19c320de34930cccd103e6` | **YES** — canonical, viewer-validated |
| DALY RS485 probe V1 | `~/MATDOG/runtime/esp32/matdog_daly_rs485_probe_v1/matdog_daly_rs485_probe_v1.ino` | 2223 | 2026-09-10 20:09 | `00d32f1a80c2438b6ace57ac93a58a5e71dab2386ba9fb20f2833ff6a572862e` | reference only — raw hex dump, no decoder |
| DALY RS485 probe V2 | `~/MATDOG/runtime/esp32/matdog_daly_rs485_probe_v2/matdog_daly_rs485_probe_v2.ino` | 4302 | 2026-09-10 20:14 | `4fd7fd3982ab57376ad9a5ade7ed497fb2515234465612a59aac22f756acb81c` | **YES** — canonical decoder |
| LED / ring / WS2812 sketch | — | — | — | — | **NONE FOUND** — new module |

Full runtime inventory command used:

```bash
find "$HOME/MATDOG/runtime/esp32" -maxdepth 4 -type f \
  \( -name '*.ino' -o -name '*.h' -o -name '*.hpp' -o -name '*.c' -o -name '*.cpp' \) \
  -printf '%TY-%Tm-%Td %TH:%TM:%TS %s %p\n' | sort
```

No LED/ring/WS2812/NeoPixel source of any kind exists under `~/MATDOG/runtime/esp32` —
confirmed by a targeted grep (`-iname '*led*' -o -iname '*ring*' -o -iname '*ws2812*' -o
-iname '*neopixel*'`), which only matched unrelated bring-up files and a QC audit
document. The handoff's own prediction — "this is the one genuinely new low-level
module allowed in V0.1" — holds.

## Resolved ambiguities

### DALY probe V1 vs V2

The handoff manifest only names `matdog_daly_rs485_probe_v1.ino`, but a V2 exists,
created 5 minutes later on the same bench session (2026-09-10 20:09 → 20:14). V1
transmits the same style of read request and dumps raw hex only — it has no decoder.
V2 adds CRC16/Modbus validation and a full register decoder whose output fields
(cell voltages, pack V, current, SOC, temps, MOS state, alarms) match **exactly** the
"known real telemetry" quoted in the handoff (cell ≈ 3.78 V, pack ≈ 11.3 V, SOC ≈
49.8 %, MOS = ON). V2 is therefore the source that actually produced the cited
evidence, and is the one adapted into `src/power/DalyBms.cpp`. V1 is superseded
reference material only; neither file was modified.

### `matdog_s3_bringup.ino` and `matdog_servo_uart_bringup.ino`

The handoff asked Claude Code to locate `matdog_s3_bringup.ino` if present. Both it
and a second, previously-unlisted file (`matdog_servo_uart_bringup.ino`) were found.
Both are minimal, non-QC bring-up sketches that only print chip/UART identification
banners with all servo commands explicitly disabled. They contributed no code beyond
confirming the GPIO17/18 @ 1 Mbps SCServo init pattern already present in the frozen
commissioning source; neither was a hard-stop ambiguity.

### UART peripheral collision (found during audit, not in the handoff)

`matdog_servo_commissioning.ino` (frozen) and **both** DALY probes independently use
`HardwareSerial(1)` for their respective bus, since each was a standalone sketch that
never had to coexist with the other. Running ServoBus and DalyBms simultaneously on
the same physical UART controller index would be a real conflict. `src/power/DalyBms.h`
binds to `HardwareSerial(2)` instead — same GPIO15/16 pins, same 9600 8N1 framing,
same Modbus request bytes; only the internal ESP32 UART controller index changes.
`scripts/static_audit.py` asserts the two indices stay different so this cannot
silently regress.

## What was preserved vs. adapted, per module

### ServoBus (`src/servo/ServoBus.{h,cpp}`)

Preserved from the frozen commissioning source: GPIO17/18, 1 Mbps, `SCServo`/`SMS_STS`
transport object, `st.Ping()`, `st.readWord()`/`st.readByte()` register access,
`SMS_STS_MODEL_L` (0x03) as the model-word register, no automatic ping/scan on boot.

Not imported: `@NORMALIZE_MATDOG` / `runNormalizeMatdog()` (real EEPROM unlock + 5
register writes), the bench QC motion/thermal/load campaign logic, `@DUMP_RAW`,
`@QC_FAST`, and every EEPROM lock/unlock/ID/CalibrationOfs primitive. The only write
V0.1 exposes is `EnableTorque(id, 0)` behind `@SERVO SAFE_OFF <id>` — it can only
remove torque, never enable it, and never touches EEPROM.

### Bno085Imu (`src/imu/Bno085Imu.{h,cpp}`)

Preserved byte-for-byte in spirit from Phase C3: pin map, `SPI.begin()` /
`begin_SPI()` sequence, `SH2_CAL_ACCEL | SH2_CAL_MAG` calibration config,
`sh2_setDcdAutoSave(false)`, the five enabled SH2 reports, and — critically — the
exact printed line formats (`RV`, `MAG`, `GYR`, `COUNTS`, `SAVE_GATE`,
`UNEXPECTED_RUNTIME_RESET`) on the same ~500 ms cadence, verified against
`lineParser.ts` and the 59/59 passing viewer test suite.

Deliberately **not** imported: the `SAVE` serial command and its `sh2_saveDcdNow()`
call. Handoff sections 12/24/28 require any DCD-save capability to be separated from
normal runtime and never triggered automatically; V0.1 removes the write path
entirely — no code path to `sh2_saveDcdNow()` exists anywhere in this firmware.

Deliberately **different**: the standalone sketch calls a `fatal()` that halts
forever on any SPI/session-config failure. That is correct isolated bench behaviour
but wrong in a unified controller, where a missing/failed IMU must degrade only the
IMU module while ServoBus/DalyBms/LedRing/USB keep running (handoff section 33).
`begin()` returns `false` and sets `ModuleHealth::FAULT` instead of blocking forever.

### DalyBms (`src/power/DalyBms.{h,cpp}`)

Preserved from probe V2: GPIO15/16, 9600 8N1, the exact 8-byte Modbus read query
(`D2 03 00 00 00 3E D7 B9`), CRC16/Modbus validation, the 129-byte response framing,
and the full register decode map (cells, temps, pack V/I/SOC, MOS state, alarms).

Deliberately **different**: both probes busy-wait up to 750 ms per poll inside
`loop()`. Doing that here would stall BNO085 acquisition and the USB command router
for the same window, which handoff section 20 explicitly forbids ("no module may
monopolize loop()"). `DalyBms::update()` instead runs a small non-blocking
send/accumulate/timeout state machine bounded by the same 750 ms deadline, called
every controller loop iteration instead of blocking one call.

`requestDischargeOff()` is present only as a fail-closed placeholder that transmits
nothing and always returns `false`, per handoff 8A.8 — the K-Series write protocol has
not been identified or bench-verified. `scripts/static_audit.py` asserts this method's
body has not been changed to something that transmits.

DALY KEY probe (2026-09-19): the query bytes, CRC and the probe-V2 register decode moved
unchanged into the Arduino-free `src/power/DalyProtocol.{h,cpp}` so the offline host suite
links them; `DalyBms` keeps the UART. The second read frame there,
`81 03 01 00 00 78 5B D4`, and the `0x0115`/`0x0120`/`0x0121`/`0x0122` decode come from static
inspection of DALY's official BMSTool V1.14.79 (downloaded from dalybms.com, never run), not
from a MATDOG bench source; the read was live-verified on MATDOG's unit on 2026-09-19 (see
`VALIDATION.md`).

### LedRing (`src/status/LedRing.{h,cpp}`)

No prior MATDOG source exists. Built on `Adafruit NeoPixel` (installed for this task,
pinned at 1.15.5 — see `scripts/build.sh` / installed-library record below), the
mature option already suggested by the handoff over hand-rolled RMT/timing code.
Boots OFF, caps brightness well under hardware maximum, and every effect is driven by
a non-blocking `update()` state machine.

## Toolchain baseline recorded at audit time

```text
arduino-cli 1.5.1 (commit 01f3d4f2b)
esp32:esp32 core 3.3.11 (only installed core; unchanged by this task)
```

Installed libraries before this task:

```text
Adafruit BNO08x         1.2.5
Adafruit BusIO          1.17.4
Adafruit Unified Sensor  1.1.15
SCServo                  1.0.2
```

Added for this task:

```text
Adafruit NeoPixel        1.15.5   (new dependency — LedRing module, no prior
                                    WS2812 library was installed or vendored)
```

No existing library or core version was upgraded.

## Flash backup gate

```text
file    : ~/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-10.bin
size    : 16777216 bytes (0x1000000) — matches expected 16 MiB exactly
sha256  : 5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32
```

Only one full-flash backup candidate exists on this machine; the second filename
mentioned in the handoff (`matdog-esp32-fullflash-20260910-2308.bin`) was not found
and is not required since the one candidate present passes verification.

## USB device

```text
/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00
-> ../../ttyACM0   (confirmed present during this session)
```

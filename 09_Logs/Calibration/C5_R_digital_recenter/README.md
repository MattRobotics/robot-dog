# MATDOG C5-R — Digital-Zero Calibration Record

## Result

```text
STATUS: PASS
SERVOS VERIFIED: 12 / 12
EEPROM LOCK: 1 on all servos
TORQUE: 0 on all servos
CANONICAL DISPLAYED q0: 2048 ticks
MAXIMUM RAW DEVIATION FROM CAPTURE: 3 ticks
```

All 12 MATDOG ST3215 Position Offset values were verified by a complete EEPROM
read-back after restarting NormaCore Station.

## Final offsets

| Servo | Joint | Signed Position Offset |
|---:|---|---:|
| M11 | LF lower | +101 |
| M12 | LF upper | +859 |
| M13 | LF hip | -505 |
| M21 | RF lower | -1986 |
| M22 | RF upper | -891 |
| M23 | RF hip | -1687 |
| M31 | RH lower | -2021 |
| M32 | RH upper | -953 |
| M33 | RH hip | -470 |
| M41 | LH lower | -1824 |
| M42 | LH upper | +979 |
| M43 | LH hip | -740 |

## Canonical artifacts

- pre-calibration ST3215 backup;
- mechanical `q = 0` encoder capture;
- digital-zero calculation record;
- final 12-servo EEPROM read-back;
- SHA-256 integrity sidecars.

Final audit:

```text
2026-07-10_145457Z_final_12_offset_readback.json
SHA-256: 15619d23ddcb17651ba729a0d69309b5b56befeb3377f123ff7f131582fcf8ec
```

Canonical reusable software:

```text
06_Software/Matdog_Core/calibration/matdog_digital_zero_calibration.py
```

Procedure:

```text
06_Software/Matdog_Core/calibration/MATDOG_DIGITAL_ZERO_CALIBRATION.md
```

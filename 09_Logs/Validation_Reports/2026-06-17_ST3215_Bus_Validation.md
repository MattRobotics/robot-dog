# ST3215 Bus Validation

> ## ⚠️ SUPERSEDED DOCUMENTATION — STATION-MEDIATED PHASE
>
> This document describes the **Station-mediated, 12-servo** architecture and the pre-2026-08-27
> physical installation. It is **historical evidence**, not current operational truth.
>
> - The **ESP32-S3 coprocessor** now owns the ST3215 bus, not NormaCore Station.
> - The robot has **17 servos** (12 leg + 5 head/jaw), not 12.
> - **All calibration was RESET** on 2026-08-27; 14 of 17 servos were recoded to new bus IDs.
>
> Current: [ARCHITECTURE.md](../../01_Docs/02_Architecture/ARCHITECTURE.md) ·
> [calibration reset](../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) ·
> [historical archive](../../09_Logs/Historical/NormaCore_MATDOG_Archive/README.md)


Date: 2026-06-17

## Hardware Tested

- Asus Ubuntu
- NormaCore Station
- Waveshare Bus Servo Adapter
- Custom ST3215 distribution board
- 12x ST3215 servos

## Objective

Validate bus communication and custom distribution board before starting IK and gait development.

## Results

- All 12 servos detected correctly.
- No communication errors.
- Immediate servo response observed.
- Temperatures within normal range.
- NormFS queues empty.
- CPU usage low.

## Station Lag Analysis

Observed:

- Average lag: 200-230 ms
- Max lag: 500-600 ms
- Occasional STALE status

Investigation confirmed that:

lag = telemetry age

and NOT:

real servo response delay

Station displays:

latency = now - monotonicStampNs

STALE appears when latency exceeds 500 ms.

## Conclusions

Validated:

- Bus topology
- Wiring section
- Custom distribution board
- Waveshare adapter

No evidence of communication limitations caused by hardware.

Servo response is immediate.

The observed lag is a telemetry visualization artifact.

## Status

APPROVED FOR IK AND GAIT DEVELOPMENT

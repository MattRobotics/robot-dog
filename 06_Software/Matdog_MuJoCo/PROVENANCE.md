# Provenance of the MATDOG MuJoCo twin

Simulation only. Nothing here is a hardware result or a gate for using a build.

## Firmware sources

The twin compiles and reads, unchanged, the firmware sources of this tree. The
branch `sim/mujoco-twin-20261110` starts at `ae70e0301237ea75b2e694e62578fd636c50477a`
(1.0.2-dev.6 ACC 220 candidate, Motion feature `5617f0e`).

Final candidate `450d2c793dcb76d08ea341eb5c4b1e50215cee40` (dashboard fix; not installed, not
validated on hardware): `git diff --quiet ae70e03 450d2c793dcb -- <paths below>` → **IDENTICAL**.
The twin is not rebased on it.

| Used by | Path |
|---|---|
| `fw_bridge/fk_reference` | `05_Firmware/MATDOG_Controller/src/motion` (LegKinematics, LegInverseKinematics, FootContact) |
| `fw_bridge/export_sequences`, `fw_bridge/cosim` | `05_Firmware/MATDOG_Controller/src/simple`, `05_Firmware/MATDOG_Controller/src/motion`, `05_Firmware/MATDOG_Controller/src/core/MotionService.*`, `05_Firmware/MATDOG_Controller/src/core/MotionJson.*`, `05_Firmware/MATDOG_Controller/src/servo/ServoPopulation.*`, `05_Firmware/MATDOG_Controller/scripts/tests/test_simple_motion.cpp`, `05_Firmware/MATDOG_Controller/scripts/tests/simple_host_stub` |
| `servo.py` (reference algorithm) | `05_Firmware/MATDOG_Controller/scripts/offline_motion/servo_plant.inc` |
| `m2_stand_hold.py` (gen-1 map) | `05_Firmware/MATDOG_Controller/scripts/tests/test_preenable_bus.cpp` (`q0[]`), `05_Firmware/MATDOG_Controller/src/actuator/CalibrationGeometryProfileData.h` (`encoder_direction`) |
| `build_model.py` | `03_CAD/URDF/matt_robodog_rev00`, `06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml`, `05_Firmware/MATDOG_Controller/src/motion/FootContactData.h` |

## Canonical inputs (verified by `build_model.py`, see `generated/PROVENANCE.json`)

- URDF REV01-FIX `matt_robodog_rev00.urdf`: `ce63034e727e82ee82267fb97cea01cd6e171e850cb2d42cb42cf763ee3706f7` (19/19 files of `SHA256SUMS.txt`).
- `MATDOG_FOOT_CONTACT_GEOMETRY.yaml`: `bcf3372d951dc62baaaf774bd2eceb05a034039f7897cf82e1576d0deb41e7aa`.

## Handoff and replay data (outside Git, read only)

`~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_ACC220_OPERATIONAL_20261110/`
- handoff as delivered: `594edaa7774996d80d59412968f8ee28244a4495a6665f5c27b79ecbfb2cd78e`;
  revised by the firmware workstream on 2026-11-10 13:38 (TorqueLimit 1000, build 450d2c7,
  `imufollows_*` warning): `134978ab025cb6b9be0398d36ac9c02b4ceba1f0fa03d291decb75b83d98f1c9`;
- `mujoco_replay/*.csv` (fixed IMU, kept as diagnostic reference, never modified): regenerated
  bit for bit from `ae70e03` with `build_and_export.sh`;
  `1a890c9f057ae01c joystick_full_slow_fast_slow.csv; 9de8319e7c4d53d2 preset_fast_stop_midswing.csv; 82c7fb4c950681ee preset_slow_standard_fast_slow.csv; 97abe11bf6722473 preset_slow_to_fast_stop_during_change.csv; `

## Hardware data used (read only, outside Git)

- Bench QC V6.1 RAW, 26 captures: `~/MATDOG/runtime/esp32/qc_campaign/`, checked against
  `09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/RAW_BIN_SHA256SUMS.txt`.
- rc.11 STAND150 HOLD: `~/MATDOG/verification-artifacts/MATDOG_RC11_HW_20261030T060305Z/02-stand150/joints_timeseries.csv`,
  `ee531faa941d130825ed767cf3808f48efa0e67e4699c06ae938e0f1d7918459`.
- dev.4 ground walk: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV4_GROUND_20261108/MATDOG_DEV4_GROUND_VALIDATION_2026-11-08/evidence/02_ground_walk100/matdog-motion-trace (1).json`,
  `3a810799586215beb7e2dcb175a7804aec581b504df2c10557b2c8f2622f8cd0`.

## Environment

Python 3.12.3, packages pinned in `requirements.txt` (MuJoCo 4.0.0, official PyPI wheels),
venv `~/MATDOG/tools/mujoco-venv` (`setup_venv.sh`); g++ (Ubuntu 13.3.0-6ubuntu2~24.04.1) 13.3.0. The ESP32 toolchain
(`~/.arduino15`, `arduino-cli`, system `python3`) is not used.

## Results

Report, results, sequences and `SHA256SUMS`: `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_TWIN_20261110/` (outside Git).

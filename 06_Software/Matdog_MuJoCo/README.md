# MATDOG MuJoCo twin

Offline simulation of MATDOG in [MuJoCo](https://mujoco.org) (official PyPI wheels, pinned in
`requirements.txt`). It derives its model from the canonical REV01-FIX URDF and replays, or runs in
closed loop, the production `SimpleMotion` of this tree. **Simulation only: no result here is a
hardware result, and nothing here is required before using a hardware build.** No firmware,
calibration, NVS, servo ID, STOP or mechanical limit is modified; no robot access.

## Setup

```bash
06_Software/Matdog_MuJoCo/setup_venv.sh            # venv in ~/MATDOG/tools/mujoco-venv
06_Software/Matdog_MuJoCo/fw_bridge/build.sh <build dir>
```

`build.sh` compiles three host tools against the firmware sources of this tree, unchanged:
`fk_reference` (LegKinematics + FootContact), `export_sequences` (SimpleMotion on the scripted rig of
`test_simple_motion.cpp`, as the handoff's `export_walk_replay.cpp`) and `cosim` (the same
SimpleMotion fed by the twin).

## Files

| File | Role |
|---|---|
| `build_model.py` | URDF + foot contact YAML → `generated/matdog.xml` (derived MJCF) and `generated/PROVENANCE.json`; refuses inputs whose SHA-256 differ from `SHA256SUMS.txt` |
| `twin.py` | slot map (firmware order = `qpos[7:19]`), CSV readers, contact reference of `FootContact.cpp` |
| `servo.py` | ST3215-C018 model: register profile (`servo_plant.inc` algorithm) + P loop, speed feedforward, TorqueLimit clamp, DC motor; every parameter tagged MEASURED / FIRMWARE / VENDOR / IDENTIFIED / HYPOTHESIS |
| `identify_bench.py` | fits the unloaded servo dynamics on the bench QC V6.1 RAW captures (500 Hz) |
| `dynamics.py` | physics replay engine and metrics (tracking, attitude, slip, support margin, duty, STOP) |
| `m1_kinematics.py` | milestone 1: model vs URDF, MuJoCo FK vs firmware FK, law feet vs sent joints, STAND poses |
| `m2_stand_hold.py` | servo model under load vs the rc.11 STAND150 HOLD (hardware) |
| `m3_replays.py` | open-loop replays of SLOW / STANDARD / FAST, mode changes, joystick, STOP, with sensitivity |
| `m3_hw_compare.py` | twin vs the dev.4 ground walk (SLOW law, hardware trace) |
| `cosim.py` | closed loop with `fw_bridge/cosim` |
| `view.py` | interactive viewer or PNG snapshots of any sequence CSV |
| `report_tables.py` | markdown tables of all results |
| `fw_bridge/` | `build.sh`, `fk_reference.cpp`, `export_sequences.cpp`, `cosim.cpp`, `rig_record.h` (shared record columns) |
| `requirements.txt`, `setup_venv.sh` | pinned environment |
| `PROVENANCE.md` | firmware sources used (identical in `ae70e03` and `450d2c793dcb`), input and hardware-data hashes |

## Runs

```bash
PY=~/MATDOG/tools/mujoco-venv/bin/python; B=<build dir>; S=<sequences dir>; O=<out dir>
$PY build_model.py
$B/export_sequences $S
$PY m1_kinematics.py $B/fk_reference $S <delivered mujoco_replay dir> $O
$PY identify_bench.py ~/MATDOG/runtime/esp32/qc_campaign ../../09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/RAW_BIN_SHA256SUMS.txt $O
$PY m2_stand_hold.py <rc.11 02-stand150/joints_timeseries.csv> $O
$PY m3_replays.py $S $O [walks stops hip sensitivity lateswing]
$PY m3_hw_compare.py <dev.4 matdog-motion-trace (1).json> $S $O
$PY cosim.py $B/cosim $O [scenario ...]
$PY view.py $S/imufollows_walk_slow.csv          # MUJOCO_GL=egl for --png
```

## Model boundaries

- Kinematics, masses, CoM and inertias are the URDF's (checked to 1e-12). The foot contact is the
  analytic cylinder of `MATDOG_FOOT_CONTACT_GEOMETRY.yaml` with a sharp rim (the 2 mm fillet is not
  modelled); other links collide with the floor only through their convex hulls; no self-collision.
- Floor rigid and level, friction 0.8 (hypothesis), MuJoCo soft contact.
- The servo model is identified unloaded at ≤ 600 step/s and ACC ≤ 50; loaded braking, ACC 180/220
  dynamics, load-dependent gear friction and the internal speed loop are not measured.
- Open-loop replays carry the scripted rig's references (with its level-IMU artifact: use the
  `imufollows_*` sequences or the closed loop for representative runs). In the closed loop
  SimpleMotion polls two servos per release (each every 120 ms) as on the robot; reads see the
  state at the release boundary without bus latency and the IMU is the exact base orientation.

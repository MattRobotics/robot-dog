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

`build.sh` compiles three host tools against the firmware sources of this tree (or of `MATDOG_FW_TREE`,
e.g. the installed build's checkout; commit and dirty files in `<build dir>/SOURCE.txt`), unchanged:
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
| `m4_hip_stop.py` | STOP_MEASURED_OUTSIDE_BOX: hip evidence from hardware traces vs the twin, closed-loop attribution variants |
| `rca_ground_stop.py` | ground STOP RCA: per STOP, the joint out of the STOP box and since when, reference, error, duty, foot forces, COM, attitude, settle criteria; the gait's own stop |
| `m5_hw.py` | M5: hardware session logs (dashboard trace, recorder `http.jsonl`, `serial.log`, preflight, flash log): provenance, real per-joint sampling rate, commands, Motion states, replayable episodes |
| `m5_compare.py` | M5: a hardware run replayed in closed loop with its own commands (walk commands at the same `walk_cycles`), each snapshot observed on the twin as the controller observed the robot (per-joint reading age, IMU time); paired HW/SIM tables; `--fixture` for the suspended robot, `--variant` for one correction |
| `m5_imu_tilt.py` | M5: IMU tilt against the four-feet plane from the encoders, per ground HOLD, with the heading (IMU mounting offset fixed in the body vs floor slope) |
| `view.py` | desktop viewer (play/pause, step, speed, fault, replay selection) and MP4/PNG offscreen, with tagged overlays and the hip-vs-kHeight plot |
| `demo.sh` | renders the visual demo (5 MP4 + key frames) offscreen |
| `report_tables.py` | markdown tables of all results |
| `m6_dev7.py` | M6: the 1.0.2-dev.7 session (b5610620d0ce) replayed in closed loop: 16 ground windows (7 STOPs) and 2 fixture windows with the joystick on three axes, and the M1 static HOLDs (`hold`); `--variant dev7_ground` is the corrected ground twin of `REPORT_M6.md` |
| `m6_summary.py` | M6: per variant, the 7 STOPs against the hardware interval, twin/HW ratios of error, duty and speed, hip margins, IMU, yaw rate |
| `m6_legs.py` | M6: swing-foot load proxy, validated on the twin (known foot force) before it is read on the hardware |
| `m6_gait.py`, `m6_gait_summary.py` | M6 gait study: one Motion build (dev.7 or a `gait_study/*.patch`) over a fixed scenario set, metrics from the twin's physics only |
| `gait_study/` | `build_variant.sh` (a patch on an exported copy of a commit, never a checkout) and the candidate patches of `REPORT_M6.md` |
| `REPORT_M6.md` | M6 report: dev.7 correlation and gait study |
| `fw_bridge/` | `build.sh`, `fk_reference.cpp`, `export_sequences.cpp`, `cosim.cpp`, `rig_record.h` (shared record columns) |
| `requirements.txt`, `setup_venv.sh` | pinned environment |
| `PROVENANCE.md` | firmware sources used (identical in `ae70e03` and `450d2c793dcb`), input and hardware-data hashes |
| `REPORT_M5.md` | M5 final report: the 450d2c7 hardware session (provenance, reconciliation with its `REPORT_HW_SESSION.md`), HW/SIM comparison on the fixture and on the ground, residual discrepancies |

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
$PY m4_hip_stop.py $B/cosim <cl dir> $O <hardware traces ...>
$PY rca_ground_stop.py <cl dir> $O [scenario ...]
$PY m5_hw.py <session dir | trace.json | http.jsonl> [--out report.json]          # provenance, rates, episodes
$PY m5_compare.py $B/cosim $O <name> <hw files> --from S --to S [--fixture] [--mode SLOW] [--variant imu_mount]
$PY m5_imu_tilt.py <ground trace.json | session dir ...> [--from S]                  # S: controller time the feet are on the floor
```

## Visual supervision

```bash
cd 06_Software/Matdog_MuJoCo; PY=~/MATDOG/tools/mujoco-venv/bin/python
$PY view.py --list                                   # catalog (data: $MATDOG_TWIN_DATA)
$PY view.py stop_fuori_box modi stand150             # desktop viewer on the ASUS (X11)
$PY view.py m5_terra_fast_stop m5_fixture_fast_stop # M5: hardware samples [HW] on the twin's replay
MUJOCO_GL=egl $PY view.py stop_fuori_box --mp4 out.mp4 --start 9 --end 16.5 --fps 25
./demo.sh <out dir>                                  # the whole demo, no display needed
```

Viewer keys: SPACE play/pause, LEFT/RIGHT one 20 ms release (paused), UP/DOWN speed x2 / /2,
F first fault, R restart, N/P next/previous replay, TAB MuJoCo panel; mouse rotates, pans and
zooms the tracking camera. Every replay is recomputed with the nominal servo model; a closed-loop
replay is checked release by release against the feedback SimpleMotion read. Overlay tags: [SIM]
MuJoCo, [FW] production SimpleMotion output, [HW] measured on the robot, [IPOTESI] assumed.

## Model boundaries

- Kinematics, masses, CoM and inertias are the URDF's (checked to 1e-12). The foot contact is the
  analytic cylinder of `MATDOG_FOOT_CONTACT_GEOMETRY.yaml` with a sharp rim (the 2 mm fillet is not
  modelled); other links collide with the floor only through their convex hulls; no self-collision.
- Floor rigid and level, friction 0.8 (hypothesis), MuJoCo soft contact (it creeps under a sustained
  tangential force; `dynamics.Twin(noslip=5)` removes the creep, part of `dev7_ground`).
- `servo.Params.gear_efficiency` < 1: load-dependent gear friction (default 1 = the model below, unchanged);
  `dev7_ground` uses 0.5 with the smooth form (`gear_friction_v0_rad_s`), identified on static HOLDs only.
- The servo model is identified unloaded at ≤ 600 step/s and ACC ≤ 50; loaded braking, ACC 180/220
  dynamics, load-dependent gear friction and the internal speed loop are not measured. Compared, not
  refitted, with the 450d2c7 fixture runs (unloaded, 1000–1950 step/s, ACC 50–220): paired error RMSE
  3.8–10.4 ticks; under load on the ground the twin carries ~1.9x the hardware duty (`REPORT_M5.md`).
- Open-loop replays carry the scripted rig's references (with its level-IMU artifact: use the
  `imufollows_*` sequences or the closed loop for representative runs). In the closed loop
  SimpleMotion polls two servos per release (each every 120 ms) as on the robot; reads see the
  state at the release boundary without bus latency and the IMU is the exact base orientation
  (`m5_compare.py --variant imu_mount` rotates it by the pitch offset measured on the ground; not default).

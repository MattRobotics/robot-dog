# Motion vNext — fixes after the Codex audit of `1160dce` (2026-11-11)

Offline only: no device, no flash. Firmware status: **IMPLEMENTED NOT HARDWARE
VALIDATED**. Commits on `1160dce`: `f6c899e` (motion), `fecced4` (dashboard),
`3221764` and `acd849c` (offline tools), then this documentation commit.
Model: `servo_plant.inc` (ST3215 response, per-seed load, ACC/brake capability,
latency, deadband, static flex 0.5-1.5x the dev.15 HIP pattern, 11.1 and 9.6 V).
A plant result is software against a modelled servo, never a hardware result.

## 1. Codex findings reproduced (VERIFIED OFFLINE)

| Finding | Reproduction | Cause |
|---|---|---|
| STANDARD backward (enabled in a copy): seeds 3, 6 fail at both voltages | identical (2 of 6 per voltage, `STOP_MEASURED_OUTSIDE_BOX` at c1.40) | HIP reference -7 ticks, static flex -10 (flex 1.43x): measured -17 against -16.3 of kHeight; WALK never checks the measured pose, STOP does |
| FAST backward seed 25 at 9.6 V (36 seeds) | identical (c0.304) | STOP in the start-ramp sway transfer: the arrest carries the HIP -4 -> -8 ticks, flex 1.30x |
| SLOW backward 1/6 per voltage, also on `450d2c7` | identical (seed 4) | same mechanism, flex 1.42x |
| STANDARD steered -> fore/aft handover, 3/12 STOPs fail | 6/72 per direction on 36 seeds (two directions), SLOW 1/72, FAST 1/72 (`1160dce`) | handover into kHeight at a sway transfer; the reading the arrest is built on holds the servo trail, not the flex |
| `stop_box_out=0` misleading | confirmed: plant internal position, successful STOPs only | test defect |
| Scenario 7: 6/6 STOPs per voltage instead of 12 | confirmed (now `planned=12 missed=6` with the 1160dce scaling) | the plant scaled to the selection, not to the walking law |
| Gate passed scenarios without WALK and ignored refusals | confirmed | test defect |
| Page scaled the held stick to the selection at once (FAST backward + STANDARD selected -> neutral, WALK STOP) | confirmed by code reading and DOM test | firmware/page defect |
| `WALK_MODE_AFTER_STEERING` promised on a steered backward walk; FAST selected during steering with ACC 180 accepted, never applied | confirmed | firmware defect |
| vy 1 mm/s alone in STANDARD starts a walk on the spot | confirmed | firmware defect |
| FAST `frontal_changes` -0.44 mm (FAST straight deceleration, not steering), `random_8` +0.34 mm | confirmed (frames identical to `450d2c7`) | law ramp |
| FCL 11.08 / 10.99 mm are cell-centre distances | confirmed: certified lower bounds SLOW 3.035, STANDARD 3.022, FAST 3.003 mm (required 3 mm) | report defect (README corrected) |

Same mechanism in a wider form, found here: a STOP at **every release of the
four-contact (sway transfer) windows** (`run_stop_matrix.sh window`, 36 seeds x
2 V x 14 releases = 1008 STOPs per path) also fails forward, on `450d2c7` too.

## 2. Fixes (firmware)

- Backward base sway `JoystickSpec::backSwayYM`: SLOW 1.125, STANDARD 0.75, FAST 0.5 mm.
- STANDARD backward enabled: 50 mm per cycle, 16.7 mm/s.
- STANDARD forward sway 1.5 -> 1.25 mm (dev.6 selection rule kept: static support >= 1.7, capture >= 1.41 mm).
- `regimeHandover`: no handover in a four-contact window, in the two releases before or the six after it; the arrest STOP must fit the destination box narrowed by each joint's measured residual plus the reference's travel since the reading and two releases. Refused: the walk stays in its regime and retries.
- FAST forward joystick decrease over 3 cycles (`decelCycles`, SLOW and STANDARD keep 2).
- `joystickMode()`: limits of the walking law until it takes the selection; status `walk_mode.joystick` (only when it differs from `selected`, bounded status budget); dashboard scales to it and keeps it after a deferred ACK; `WALK_MODE_AFTER_STEERING` for forward steered walks only; ACC ceiling checked there; `JOYSTICK_BELOW_BAND`.

Unchanged: STOP, `StopPlan`, boxes, `pointSafe`, `compensationSafe`, tolerances,
deadlines, settling, NVS, geometry, SLOW forward law, FAST forward law.

## 3. Results on the final source

**servo_plant, 36 seeds, both voltages** (`run_servo_plant.sh`, every planned STOP counted):
0 faults, 0 missed in all 24 mode x scenario cells. Planned and verified STOPs per
voltage: STOP8_RESUME 288, MODE_CHANGES 72, JOY_STEER_STOP8 288, JOY_BACK_STOP8 288,
JOY_STEER_IMU 288, JOY_BACK_MODES 72, per mode; measured box exit 0.0 ticks.

**STOP in the sway-transfer windows** (1008 STOPs per path; flex = plant factor of the failing seed):

| Path | `450d2c7` | `1160dce` | final |
|---|---|---|---|
| SLOW forward | 44 | 44 | 44 (flex >= 1.28x) |
| STANDARD forward | 36 | 36 | 2 (flex 1.16x, one seed) |
| FAST forward | 10 | 10 | 10 (flex >= 1.13x) |
| SLOW backward | - | (1.5 mm) | 0 |
| STANDARD backward | - | disabled | 0 |
| FAST backward | - | - | 0 |

**Handover replay** (`run_stop_matrix.sh handover`, 36 seeds x 2 V, a STOP the
release the regime changes): every mode and direction 0 failed of the STOPs
executed (STANDARD 287/287, FAST 288/288, SLOW 241/241). Trials without a
handover (the walk keeps its steered regime, the safe side): SLOW 25 + 22
(IMU attitude outside the fore/aft envelope, same as `1160dce`), STANDARD 1.
`1160dce`: 14 failed of 864.

**Joystick gate** (strict, `acd849c` tool): SLOW PASS (min static support 0.12 mm,
`random_4` in a backward segment: 0.39 mm on `450d2c7`/`1160dce`, the price of the
backward HIP room; 35 of 45 scenarios frame-identical to `450d2c7`, the 10 others
carry backward segments or a steered -> fore/aft handover), STANDARD PASS (min
0.38 mm), FAST PASS (min 0.51 mm; `frontal_changes` -0.44 -> +0.84 mm, `random_8`
+0.34 -> +0.51 mm). Non-foot clearance >= 2.28 mm. Static geometric support only:
no dynamic stability is claimed (MuJoCo V2 cannot resolve margins of this size).
The strict gate fails on a refused command, a scenario without WALK, a missing
planned STOP (servo_plant `missed`) and keeps faults in the denominator: checked
with negative runs (refused -> rc 1; no WALK -> rc 1; SLOW backward with the old
sway -> seed 4 listed, 43/44 of 48 planned; scenario 7 with the 1160dce scaling
-> `planned=12 missed=6`). Only exception: `release_in_start` (WALK optional).
`MOTION_BUSY` is accepted only in `WALK_SETTLE` after a WALK STOP.

**Steady laws** (`walk_support_check` / `walk_tilt_envelope_check`, static model):

| Law | Static support | STOP capture | HIP margin kHeight | Tilt envelope |
|---|---|---|---|---|
| STANDARD forward 1.25 mm | 1.99 mm | 1.51 mm | 10.0 ticks | PASS (strides 0-2) |
| SLOW backward 1.125 mm | 1.49 mm | 1.72 mm | 11.0 ticks | PASS |
| STANDARD backward 0.75 mm | 1.66 mm | 1.84 mm | 12.5 ticks | PASS |
| FAST backward 0.5 mm | 1.52 mm | 1.38 mm | 13.7 ticks | PASS |

## 4. Not closed

Forward STOP in the sway-transfer windows (SLOW 44, STANDARD 2, FAST 10 of 1008,
at a plant flex >= 1.28x, 1.16x, 1.13x the HW pattern). 0 needs SLOW 1.0 mm
(capture 1.04 mm), STANDARD 1.0 or FAST 0.875 mm (capture 1.32 mm): below the
dev.6 selection rule, so not applied. HW measured SLOW flex: 5-6 ticks at the
extremes (about 0.8x the pattern); STANDARD and FAST flex unknown. Decision with
HW flex data (a fixture cannot load the HIPs).

## 5. Other checks

Host `test_simple_motion` PASS (multidirectional joystick 172 runs, 173 STOPs;
flex-deferred handover; ACC ceiling at selection; below-band; saturation).
Dashboard suite PASS (DOM, joystick mode incl. a deferred selection with the stick
held, contract 6112 checks, Chrome layout/STOP). Joystick refresh after HTTP 503:
existing DOM coverage (outcome unknown shown, the held vector re-sent at the next
tick); the Controller ends the walk by WALK STOP after 1 s without an accepted
refresh. `run_motion_frame_digest.sh`: 16 lines identical to `1160dce` and
`450d2c7`. QEMU and static audit: final report.

## 6. MuJoCo V2 (integrative evidence, limits explicit)

From `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_V2_VNEXT_1160dce_20261111/`:
the twin reproduced `frontal_changes` and `random_8` with identical nominal
frames; `frontal_changes` is FAST straight deceleration. The twin cannot resolve
static margins of +0.34 / -0.44 mm (simulated foot lift 0-0.4 mm against 5-6.4
commanded, HIP errors 7-15 ticks); its ground STOP faults recur in forward
walking too and do not single out firmware defects; no run certified a STOP on
three contacts (the swing foot stays loaded). No threshold, box, STOP or
settling was changed for the twin's faults.

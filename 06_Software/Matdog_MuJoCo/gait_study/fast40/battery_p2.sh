#!/usr/bin/env bash
# FAST 40 phase 2 (2026-11-13): twin battery of the candidate F40T_JC against dev.7 (corrected twin
# dev7_ground). dev.7 and F40 nominal runs are reused from twin/final (same twin, same scenarios);
# F40T_JC runs every fast_* scenario with its own JoystickSpec (--fast-joy 16:0.10:50). Perturbations:
# body mass +-10 %, COM x +-5 mm, foot friction mu 0.6 / 1.0, on both builds.
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"; A=~/MATDOG/verification-artifacts/MATDOG_MUJOCO_FAST40_20261112; P=$A/p2/twin_final
JC="F40T_JC 2.75:110:105 $A/builds/F40T_JC"; D7="dev7 2.5:80:105 dev7"; J="--fast-joy 16:0.10:50"
ALL="fast_fwd fast_fwd_32 fast_fwd_stop fast_back fast_back_stop fast_left fast_right fast_yaw fast_yaw_stop fast_diag fast_diag_yaw fast_diag_stop fast_sweep fast_modes_cycle $(for k in 0 1 2 3 4 5 6 7; do printf 'fast_stopcyc_%s ' $k; done)$(for k in $(seq -w 0 15); do printf 'fast_stopdense_%s ' $k; done)"
SENS="fast_fwd fast_left fast_right fast_yaw $(for k in 0 1 2 3 4 5 6 7; do printf 'fast_stopcyc_%s ' $k; done)"
MU="fast_fwd fast_back fast_left fast_right fast_yaw fast_stopcyc_0 fast_stopcyc_2 fast_stopcyc_4 fast_stopcyc_6"
SC="$ALL" OUT=$P/final JOBS=7 EXTRA="$J" "$here/run_twin.sh" "$JC"
for s in "m+10:--set base_mass_kg=1.878" "m-10:--set base_mass_kg=1.536" "cx+5:--set com_x=0.005" "cx-5:--set com_x=-0.005"; do n=${s%%:*}; e=${s#*:}
  SC="$SENS" OUT=$P/sens_$n JOBS=7 EXTRA="$e $J" "$here/run_twin.sh" "$JC"
  SC="fast_left fast_right fast_yaw" OUT=$P/sens_$n JOBS=7 EXTRA="$e" "$here/run_twin.sh" "$D7"; done
for s in "mu06:--set mu=0.6" "mu10:--set mu=1.0"; do n=${s%%:*}; e=${s#*:}
  SC="$MU" OUT=$P/$n JOBS=7 EXTRA="$e $J" "$here/run_twin.sh" "$JC"
  SC="$MU" OUT=$P/$n JOBS=7 EXTRA="$e" "$here/run_twin.sh" "$D7"; done
echo BATTERY_P2_DONE

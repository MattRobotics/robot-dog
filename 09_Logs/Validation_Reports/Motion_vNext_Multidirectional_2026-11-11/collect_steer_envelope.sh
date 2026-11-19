#!/usr/bin/env bash
# Scratch: per-joint envelope of the steered STANDARD/FAST laws (gate scenarios + direction grid).
set -uo pipefail
C=/tmp/claude-recover/mv/collect; mode=$1; out=$C/m$mode; rm -rf $out; mkdir -p $out
limits=$(JOY_MODE=$mode $C/joy_collect --limits)
gate=$(sed -n "/^mapfile -t lines < <(JOY_LIMITS/,/^PY$/p" /home/matteo-manicardi/MATDOG/worktrees/matdog-motion-multidir-vnext-20261111/05_Firmware/MATDOG_Controller/scripts/offline_motion/run_joystick_envelope.sh | sed '1d;$d')
mapfile -t lines < <(JOY_LIMITS="$limits" python3 -c "$gate"; JOY_LIMITS="$limits" python3 - <<'PY'
import itertools,math,os
f=[float(x) for x in os.environ['JOY_LIMITS'].split()];lim=f[:3]
v=lambda a:','.join(repr(x) for x in a)
for s in itertools.product((-1,0,1),repeat=3):
    if s==(0,0,0):continue
    n=math.sqrt(sum(x*x for x in s))
    for amp in (.5,1):print('g'+''.join('m' if x<0 else str(x) for x in s)+f'_{amp}','26:'+v([amp*s[j]/n*lim[j] for j in range(3)]))
PY
)
echo "${#lines[@]} scenarios"
for line in "${lines[@]}"; do name=${line%% *};args=${line#* }
  res=$(JOY_MODE=$mode JOY_ENVELOPE=$out/envelope.txt $C/joy_collect $out/$name.txt $args 1:0,0,0 2>/dev/null || true); echo "$name $res" >> $out/collect.log
done
echo "done $(wc -l < $out/envelope.txt) envelope rows"

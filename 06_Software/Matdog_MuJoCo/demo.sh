#!/usr/bin/env bash
# Visual demo of the MATDOG twin (simulation, not hardware): MP4 replays and key frames,
# rendered offscreen (MUJOCO_GL=egl, no display needed), H.264 MP4 playable anywhere.
# Usage: demo.sh <out dir>    data: $MATDOG_TWIN_DATA (default ~/MATDOG/verification-artifacts/MATDOG_MUJOCO_TWIN_20261110)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"; py="${PY:-$HOME/MATDOG/tools/mujoco-venv/bin/python}"; out="$1"; mkdir -p "$out"
export MUJOCO_GL=egl
v() { "$py" "$here/view.py" "$@"; }
v stand150 --mp4 "$out/01_stand150.mp4"
v modi --mp4 "$out/02_modi_slow_standard_fast_slow.mp4"
v stop_fuori_box --mp4 "$out/03_stop_fuori_box_standard.mp4" --start 9 --end 16.5 --fps 25           # half speed
v stop_fuori_box_anche_x2 --mp4 "$out/04_stop_fuori_box_anche_x2.mp4" --start 9 --end 16.5 --fps 25  # half speed
v stop_non_assestato --mp4 "$out/05_stop_non_assestato_fast.mp4" --start 10 --end 18
v stand150 --png "$out/01_stand150.png" --at 4.9
v stop_fuori_box --png "$out/03_stop_fuori_box.png" --at 13.06 13.12
v stop_fuori_box_anche_x2 --png "$out/04_stop_fuori_box_anche_x2.png" --at 13.06 13.12
v stop_non_assestato --png "$out/05_stop_non_assestato.png" --at 13.14 16.26
(cd "$out" && for f in *.mp4; do ffprobe -v error -select_streams v:0 -show_entries stream=codec_name,width,height,r_frame_rate,nb_frames -show_entries format=duration -of csv=p=0 "$f" | tr '\n' ' '; echo " $f"; done)

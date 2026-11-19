#!/usr/bin/env bash
# OFFLINE test of the 16/11 MEMSTOP tools against tests/sim/sim_device.py (never the robot).
# run_case.sh <case> <http-port> [stray]   env: SIM_HEAP_MIN0 SIM_DROP_AT SIM_DROP_TO SIM_MODE
set -uo pipefail
cd "$(dirname "$0")"
CASE=$1; HP=$2; STRAY=${3:-}
D=cases/$CASE; rm -rf "$D"; mkdir -p "$D"; cp -p ../hw_next/tools/* "$D"/
python3 -I sim/sim_device.py "$HP" "$D/pty.txt" > "$D/sim.log" 2>&1 &
SP=$!
for i in $(seq 100); do [ -s "$D/pty.txt" ] && break; sleep 0.05; done
export MATDOG_SIM_PTY=$(cat "$D/pty.txt") MATDOG_SIM_BASE=http://127.0.0.1:$HP MATDOG_BASE=http://127.0.0.1:$HP
cd "$D"
if [ -n "$STRAY" ]; then          # an unregistered watcher, as on 16/11 (real PID kept only for the cleanup)
  mkdir -p stray_run/ctl.d
  setsid nohup bash -c 'echo $$ > stray.pid; exec python3 -I http_watch.py stray_run' > stray.console 2>&1 < /dev/null &
  sleep 1.5
fi
python3 -I run_acquisition.py h_case fixture_block_h.py h_case > acq.console 2>&1; CODE=$?
python3 -I session_proc.py check --expect-none > check_after.txt 2>&1; CHK=$?
[ -n "$STRAY" ] && { kill "$(cat stray.pid)"; sleep 0.5; kill -0 "$(cat stray.pid)" 2>/dev/null && echo "stray alive" || echo "stray closed by its real PID"; }
kill $SP; wait $SP 2>/dev/null
echo "CASE=$CASE acquisition exit=$CODE check_after=$CHK"
tail -3 acq.console; grep -h "MEMORY_GATE\|M0\|ABORT\|COMPLETE" h_case.console 2>/dev/null | head -4
echo "USB commands received by the sim: $(grep -c ' CMD ' sim.log)  Motion: $(grep ' CMD ' sim.log | grep -v 'MOTION TIMING' | grep -c '@MOTION\|@STOP')"

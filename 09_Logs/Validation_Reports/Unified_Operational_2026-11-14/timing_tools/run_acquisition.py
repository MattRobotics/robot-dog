#!/usr/bin/env python3
"""One fixture acquisition, from start to verified closure (replaces run_with_watch.sh after the 16/11 MEMSTOP).

1. Refuses unless the session is clean: no live session process at all (session_proc.check --expect-none), so the
   USB port and the robot's HTTP have no other client; refuses if /status already reports heap_min_free < 24 KB.
2. Starts the fixture tool (its USB owner and HTTP poller; M0 memory gate is its first step), then, once the tool's
   ctl.d exists, the run's single http_watch.py, then heap_guard.py on the tool's real PID. Exactly two HTTP clients.
3. Waits for the tool to end, stops the watcher through its registered PID, waits for the guard, and verifies with
   session_proc.check --expect-none that nothing is left. Prints ACQUISITION_CLOSED or ACQUISITION_NOT_CLEAN.
Exit code: the tool's, or 4 if the closure is not clean. Nothing here sends anything to the robot except one GET /status.

Usage: run_acquisition.py <run-dir> <tool.py> <tool args, including the same run-dir>"""
import importlib.util, os, sys, time, urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('session_proc', ROOT / 'session_proc.py')
sp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sp)
BASE = os.environ.get('MATDOG_BASE', 'http://192.168.4.1')
LIMIT = 24 * 1024


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    run, tool = sys.argv[1], sys.argv[2:]
    if (ROOT / run).exists():
        print('REFUSE: %s exists' % run)
        return 1
    if not sp.check(expect_none=True):
        print('REFUSE: session not clean (see PROBLEM lines)')
        return 1
    st = dict(l.split('=', 1) for l in urllib.request.urlopen(BASE + '/status', timeout=3).read().decode().splitlines() if '=' in l)
    if int(st.get('heap_min_free', 0)) < LIMIT:
        print('REFUSE: heap_min_free %s < %d before the run' % (st.get('heap_min_free'), LIMIT))
        return 1
    (ROOT / (run + '_start_utc.txt')).write_text(sp.utc() + '\n')
    p, _ = sp.start(run + '_tool', 'tool', run, run + '.console', [sys.executable, '-I'] + tool)
    for _ in range(200):
        if (ROOT / run / 'ctl.d').is_dir() or p.poll() is not None:
            break
        time.sleep(0.05)
    if not (ROOT / run / 'ctl.d').is_dir():
        print('RUN_FAILED: no %s/ctl.d' % run)
        p.wait()
        sp.check(expect_none=True)
        return 1
    sp.start(run + '_watch', 'watch', run, run + '.watch.console', [sys.executable, '-I', 'http_watch.py', run])
    sp.start(run + '_guard', 'guard', run, run + '.guard.console',
             [sys.executable, '-I', 'heap_guard.py', run, str(sp.REG / (run + '_tool.pid'))])
    code = p.wait()
    sp.stop(run + '_tool')                 # already ended: records the verified closure
    sp.stop(run + '_watch')
    for _ in range(100):                   # the guard ends with the tool (2 s poll)
        if not sp.alive_as(*[sp.json.loads((sp.REG / (run + '_guard.json')).read_text())[k] for k in ('pid', 'argv')]):
            break
        time.sleep(0.1)
    sp.stop(run + '_guard')
    clean = sp.check(expect_none=True)
    print('ACQUISITION_CLOSED' if clean else 'ACQUISITION_NOT_CLEAN', 'tool exit', code, flush=True)
    return code if clean else 4


if __name__ == '__main__':
    sys.exit(main())

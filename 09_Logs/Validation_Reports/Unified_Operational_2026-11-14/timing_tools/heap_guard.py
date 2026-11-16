#!/usr/bin/env python3
"""Live memory guard for one fixture run (16/11 evening mandate): every 2 s re-reads <run-dir>/http.jsonl and
<run-dir>/serial.log with heap_check.scan; when a heap_min_free sample is below 24 KB or an allocation-failure
line appears, sends SIGTERM to the PID in <pid-file> (the run's own fixture tool). That is the tool's existing
external stop: Abort -> safe_end (one STOP if engaged and not faulted), no retry, no resume. Nothing is sent
to the robot from here. Exits when the PID is gone.

Usage: heap_guard.py <run-dir> <pid-file>"""
import importlib.util, os, signal, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('heap_check_mod', ROOT / 'heap_check.py')
hc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hc)

run, pid = sys.argv[1], int(Path(sys.argv[2]).read_text().split()[0])
log = open(str(Path(run)) + '_heap_guard.txt', 'a')


def alive():
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


print('%s guard start pid %d' % (time.strftime('%FT%TZ', time.gmtime()), pid), file=log, flush=True)
while alive():
    mins, largest, free, stamps, alloc = hc.scan(run)
    low = min(stamps) if stamps else None
    if (low and low[0] < hc.LIMIT) or alloc:
        print('%s MEMORY_STOP heap_min_free %s at %s, allocation lines %d -> SIGTERM %d'
              % (time.strftime('%FT%TZ', time.gmtime()), low[0] if low else '-', low[1] if low else '-', len(alloc), pid),
              file=log, flush=True)
        os.kill(pid, signal.SIGTERM)
        break
    time.sleep(2)
mins, largest, free, stamps, alloc = hc.scan(run)
print('%s guard end: samples %d, heap_min_free min %s, heap_largest min %s, allocation lines %d'
      % (time.strftime('%FT%TZ', time.gmtime()), len(mins), min(mins) if mins else '-', min(largest) if largest else '-', len(alloc)),
      file=log, flush=True)

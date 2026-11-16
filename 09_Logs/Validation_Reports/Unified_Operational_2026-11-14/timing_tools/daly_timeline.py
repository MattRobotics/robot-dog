#!/usr/bin/env python3
"""DALY episode of 16/11/2026 (11:51Z): every @BMS STATUS / stream block of the passive USB logs, read-only.
Usage: daly_timeline.py <serial.log> [...]  -> TSV on stdout: host_utc, detected, comm, age_ms, poll_done_utc
(= host_utc - age_ms: when the last completed poll closed), uptime of the last OK reply, D1 and D2 fields, SOC, current."""
import re, sys, datetime as dt
rows = []
for path in sys.argv[1:]:
    cur = None
    for line in open(path, errors='replace'):
        t = line[:29]
        if ' DALY   init=' in line:
            m = re.search(r'detected=(\S+) .*result=(\S+)', line)
            cur = {'src': path.split('/')[-2], 'utc': t, 'detected': m.group(1), 'result': m.group(2)}
            rows.append(cur)
        elif cur is not None and '  comm=' in line:
            m = re.search(r'comm=(\S+) age_ms=(\d+)', line); cur.update(comm=m.group(1), age=int(m.group(2)))
        elif cur is not None and 'soc=' in line and 'pack_v=' in line:
            m = re.search(r'current_a=(\S+) soc=([\d.]+)%', line); cur.update(cur_a=m.group(1), soc=m.group(2))
        elif cur is not None and 'rx_bytes=' in line:
            for k, v in re.findall(r'(\w+)=(\S+)', line): cur[k] = v
        elif cur is not None and ' d2 samples=' in line:
            for k, v in re.findall(r'(\w+)=(\S+)', line): cur['d2_' + k] = v
def done(r):
    t = dt.datetime.fromisoformat(r['utc'].replace('+00:00', '').strip())
    return (t - dt.timedelta(milliseconds=r.get('age', 0))).strftime('%H:%M:%S.%f')[:-3]
cols = ['src', 'utc', 'detected', 'comm', 'age', 'poll_done', 'rx_bytes', 'first_byte_ms', 'stale_bytes', 'fail_streak', 'last_ok_uptime_ms',
        'd2_samples', 'd2_rx_low', 'd2_rx_edges', 'd2_tx_low_idle', 'd2_fifo_nonzero', 'd2_fifo_max', 'soc', 'cur_a']
print('\t'.join(cols))
for r in rows:
    if 'age' not in r: continue
    r['poll_done'] = done(r); r['utc'] = r['utc'][11:23]
    print('\t'.join(str(r.get(c, '')) for c in cols))

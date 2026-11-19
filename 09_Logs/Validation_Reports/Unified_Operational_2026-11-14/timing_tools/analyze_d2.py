#!/usr/bin/env python3
"""D1 + D2 per DALY poll from a passive USB log (serial.log of usb_session.py / fixture_suite.py), read-only.

Each '@BMS STATUS' block (also printed every 2 s by @BMS STREAM ON) ends with the D1 line
('  rx_bytes=... fail_streak=...') and, on builds with D2 (46b3175), the D2 line
('  d2 samples=... rx_low=... rx_edges=... tx_low_idle=... fifo_nonzero=... fifo_max=...').
D1 describes the poll behind 'comm='; D2 the last completed poll period (request to request), so the
two can be one poll apart when the block is printed between a reply and the next request. A silence
lasts many polls, so its classification does not depend on that offset.

Classes (fixed before the session, MATDOG_1_0_2_DIAG_46B3175_HW_20261116/README.md §D2):
  OK       comm=OK, rx_bytes 129
  PARTIAL  rx_bytes 1..128
  SILENT   rx_bytes 0
For SILENT periods: RX_IDLE_HIGH (rx_low 0, fifo_nonzero 0), RX_HELD_LOW (rx_low >= half the samples),
RX_ACTIVITY (edges or some low samples), FIFO_UNREAD (fifo_nonzero > 0 while rx_bytes 0), TX_LOW_IDLE (tx_low_idle > 0).
Usage: analyze_d2.py <serial.log> [<out.tsv>]"""
import re, statistics as st, sys

recs = []
cur = None
for line in open(sys.argv[1], errors='replace'):
    t, _, s = line.rstrip('\n').partition(' ')
    if s.startswith('  comm='):
        cur = {'t': t, 'comm': s.split()[0][5:]}
        recs.append(cur)
    elif cur is not None and s.startswith('  rx_bytes='):
        for k, v in re.findall(r'(\w+)=(\w+)', s):
            cur[k] = int(v) if v.isdigit() else v
    elif cur is not None and s.startswith('  d2 '):
        for k, v in re.findall(r'(\w+)=(\d+)', s):
            cur['d2_' + k] = int(v)
        cur = None


def cls(r):
    rb = r.get('rx_bytes')
    if rb is None:
        return 'NO_D1'
    return 'OK' if (r['comm'] == 'OK' and rb == 129) else ('SILENT' if rb == 0 else 'PARTIAL')


def silent_kind(r):
    if 'd2_samples' not in r:
        return 'NO_D2'
    n = max(r['d2_samples'], 1)
    k = []
    if r['d2_fifo_nonzero']:
        k.append('FIFO_UNREAD')
    if r['d2_rx_low'] >= n / 2:
        k.append('RX_HELD_LOW')
    elif r['d2_rx_low'] or r['d2_rx_edges']:
        k.append('RX_ACTIVITY')
    else:
        k.append('RX_IDLE_HIGH')
    if r['d2_tx_low_idle']:
        k.append('TX_LOW_IDLE')
    return '+'.join(k)


for r in recs:
    r['class'] = cls(r)
    r['kind'] = silent_kind(r) if r['class'] != 'OK' else ''
d2 = [r for r in recs if 'd2_samples' in r]
print('records %d, with D2 %d' % (len(recs), len(d2)))
by = {}
for r in recs:
    by.setdefault(r['class'], []).append(r)
for c in sorted(by):
    rs = [r for r in by[c] if 'd2_samples' in r]
    line = '%-8s %5d' % (c, len(by[c]))
    if rs:
        f = lambda k: [x[k] for x in rs]
        frac = [x['d2_rx_low'] / max(x['d2_samples'], 1) for x in rs]
        line += ('  samples med %d [%d..%d]  rx_low frac med %.3f [%.3f..%.3f]  edges med %d [%d..%d]  fifo_nonzero max %d  fifo_max max %d  tx_low_idle max %d'
                 % (st.median(f('d2_samples')), min(f('d2_samples')), max(f('d2_samples')), st.median(frac), min(frac), max(frac),
                    st.median(f('d2_rx_edges')), min(f('d2_rx_edges')), max(f('d2_rx_edges')), max(f('d2_fifo_nonzero')),
                    max(f('d2_fifo_max')), max(f('d2_tx_low_idle'))))
    print(line)
kinds = {}
for r in recs:
    if r['class'] in ('SILENT', 'PARTIAL'):
        kinds[(r['class'], r['kind'])] = kinds.get((r['class'], r['kind']), 0) + 1
for k in sorted(kinds):
    print('  %s %s: %d' % (k[0], k[1], kinds[k]))
# contiguous non-OK episodes
ep, start = [], None
for i, r in enumerate(recs):
    bad = r['class'] != 'OK'
    if bad and start is None:
        start = i
    if not bad and start is not None:
        ep.append((start, i - 1))
        start = None
if start is not None:
    ep.append((start, len(recs) - 1))
print('non-OK episodes: %d' % len(ep))
for a, b in ep:
    seg = recs[a:b + 1]
    print('  %s .. %s  polls %d  first %s rx=%s  kinds %s' % (seg[0]['t'][11:23], seg[-1]['t'][11:23], len(seg), seg[0]['class'],
                                                            seg[0].get('rx_bytes'), sorted({x['kind'] for x in seg})))
if len(sys.argv) > 2:
    keys = ['t', 'comm', 'class', 'kind', 'rx_bytes', 'first_byte_ms', 'stale_bytes', 'fail_streak', 'last_ok_uptime_ms',
            'd2_samples', 'd2_rx_low', 'd2_rx_edges', 'd2_tx_low_idle', 'd2_fifo_nonzero', 'd2_fifo_max']
    with open(sys.argv[2], 'w') as o:
        o.write('\t'.join(keys) + '\n')
        for r in recs:
            o.write('\t'.join(str(r.get(k, '')) for k in keys) + '\n')

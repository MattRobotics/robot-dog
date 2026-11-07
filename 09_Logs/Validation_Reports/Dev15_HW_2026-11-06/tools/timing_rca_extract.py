#!/usr/bin/env python3
"""Reproduce the numbers of TIMING_RCA_2026-11-07.md from the 15 dev.15 Motion traces.

Read-only. Usage (run with -I, outside the extracted folder):
  python3 -I timing_rca_extract.py <dir containing traces/matdog-motion-trace (N).json>

Wire-format note (dev.15 7997ad4, MotionService.cpp:139 vs :180): the JSON key
"timing.max_work" carries the FIRST-MISS record and "timing.first_miss" carries the
MAXIMUM record. This script relabels them and checks the swap on every record.
Each array: [READ,NOMINAL,COMPENSATE,CONVERT,FRAME,REST] of the release, the same six
of the release before it, then [solves, ik, newton, fk] of the release.
"""
import collections, glob, json, re, statistics as st, sys

PARTS = ['READ', 'NOMINAL', 'COMPENSATE', 'CONVERT', 'FRAME', 'REST']

def traces(root):
    files = glob.glob(root + '/traces/*.json')
    return sorted(files, key=lambda p: int(re.search(r'\((\d+)\)', p).group(1)))

def main(root):
    recs, swap_ok, swap_n, seen = {}, 0, 0, set()
    rate = collections.defaultdict(lambda: [0, 0, 0]); bounds = set(); commands = {}
    for f in traces(root):
        n = int(re.search(r'\((\d+)\)', f).group(1)); d = json.load(open(f)); S = [s['status'] for s in d['samples']]
        for c in d['commands']: commands[(d['started_utc'], c['id'])] = c
        for a, b in zip(S, S[1:]):
            key = (d['started_utc'], a['now_ms'])
            if a['phase'] != b['phase'] or a['state'] != b['state'] or key in seen: continue
            seen.add(key); dt = b['now_ms'] - a['now_ms']
            if 0 < dt <= 2000:
                r = rate[a['state'] + '/' + a['phase']]; r[0] += dt; r[1] += b['releases'] - a['releases']; r[2] += b['missed_releases'] - a['missed_releases']
        for s in S:
            ct, t = s['command_timing'], s['timing'] or {}
            mx, fm = t.get('first_miss'), t.get('max_work')   # swapped on the wire
            if mx and ct['max_work_us'] and (n, tuple(mx)) not in recs:
                swap_n += 1; swap_ok += sum(mx[:6]) == ct['max_work_us']
            for kind, arr, ph in (('MAX', mx, ct['max_work_phase']), ('FIRST_MISS', fm, ct['first_miss_phase'])):
                if arr and (kind, tuple(arr)) not in recs:
                    recs[(kind, tuple(arr))] = dict(trace=n, now=s['now_ms'], kind=kind, phase=ph, cur=arr[:6], prev=arr[6:12], work=tuple(arr[12:]))
            if fm and ct['first_miss_lateness_us']:
                W = sum(fm[6:12]); bounds.add((ct['first_miss_phase'], ct['first_miss_lateness_us'], W, ct['first_miss_lateness_us'] + 20000 - W))
    print(f'# key swap check: "first_miss" array sums to command_timing.max_work_us in {swap_ok}/{swap_n} distinct records')
    R = list(recs.values()); print(f'# distinct timing records: {len(R)}')
    print('\n## per phase / identical work signature (current release, us): min/median/max')
    g = collections.defaultdict(list)
    for r in R: g[(r['phase'], r['work'])].append(r)
    for (ph, w), rs in sorted(g.items(), key=lambda x: -len(x[1])):
        if len(rs) < 3: continue
        col = lambda i: [r['cur'][i] for r in rs]; tot = [sum(r['cur']) for r in rs]
        m = lambda v: f'{min(v)}/{st.median(v):.0f}/{max(v)}'
        print(f'{ph:<14} w={w} n={len(rs):>2} READ {m(col(0))} NOMINAL {m(col(1))} COMPENSATE {m(col(2))} CONVERT {m(col(3))} FRAME {m(col(4))} TOTAL {m(tot)}')
    print('\n## releases preceding a max/first-miss record (prev six), per phase: median us')
    gp = collections.defaultdict(list)
    for r in R:
        if sum(r['prev']): gp[r['phase']].append(r['prev'])
    for ph, ps in gp.items():
        if len(ps) < 5: continue
        print(f'{ph:<14} n={len(ps):>2} ' + ' '.join(f'{PARTS[i]} {st.median([p[i] for p in ps]):.0f}' for i in range(6)) + f' TOTAL {st.median([sum(p) for p in ps]):.0f}  >20ms {sum(sum(p) > 20000 for p in ps)}/{len(ps)}')
    print('\n## effective release rate per state/phase (consecutive samples, overlapping windows de-duplicated)')
    for k, (dt, rel, mis) in sorted(rate.items(), key=lambda x: -x[1][0]):
        if dt >= 3000: print(f'{k:<26} {dt/1000:7.1f} s releases {rel:6} missed {mis:5} -> {rel/(dt/1000):5.1f} Hz, {mis/(dt/1000):4.2f} missed/s')
    print('\n## first lost period: (phase, lateness_us, previous release work W_N, l_N+O_N = lateness+20000-W_N)')
    for b in sorted(bounds): print(' ', b)
    print('\n## non-200 commands (unique by session+id)')
    for c in commands.values():
        if c.get('http') != 200: print(' ', c.get('label'), c.get('type'), c.get('http'), c.get('ack'), 'client headers latency ms', round(c['headers_ms'] - c['sent_ms'], 1))

if __name__ == '__main__':
    main(sys.argv[1])

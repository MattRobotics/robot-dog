#!/usr/bin/env python3
"""Motion FP32 (18/11/2026): QEMU (ESP32-S3, -icount shift=3, CCOUNT) per scenario and variant: heaviest release cost,
NOMINAL mean/max over WALK releases, releases over 160k/280k/320k, and the heaviest release of each P0 class with its
parts. Base and P1 B3/D8/H/E come from the tolerance review run (same rig), S from this folder. CCOUNT is QEMU's
deterministic count, not a hardware time."""
import re, sys, collections
T = '/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_1_0_2_P0_TOLERANCE_20261118/qemu/out'
Q = 'qemu/out'
src = {'base': [T + '/walk_base/target.txt', Q + '/walk_base_m16/target.txt'], 'p1': [T + '/walk_p1min/target.txt', Q + '/walk_p1_m16/target.txt'],
       'fp32': [Q + '/walk_fp32/target.txt'], 'p1fp32': [Q + '/walk_p1fp32/target.txt']}
rel = re.compile(r'REL (\S+)\s+(NOM|COST)\s+#(\d+)\s+(\S+)\s+cost=(\d+) READ=(\d+) NOMINAL=(\d+) COMPENSATE=(\d+) CONVERT=(\d+) FRAME=(\d+) REST=(\d+) \|(.*)')
def load(files):
    sc = {}; rels = collections.defaultdict(list); stack = None
    for f in files:
        for l in open(f, errors='replace'):
            l = l.replace('\r', '')
            m = re.match(r'SCENARIO (\S+)\s+releases=(\d+) walk_releases=(\d+) nominal_mean=(\d+) nominal_max=(\d+)', l)
            if m: sc[m.group(1)] = dict(rel=int(m.group(2)), walk=int(m.group(3)), nmean=int(m.group(4)), nmax=int(m.group(5)))
            m = re.match(r'BUDGET (\S+)\s+over280k=(\d+) over320k=(\d+) over360k=(\d+) over400k=(\d+)', l)
            if m: sc[m.group(1)].update(o280=int(m.group(2)), o320=int(m.group(3)))
            m = rel.match(l)
            if m:
                probes = dict((k, (int(v), int(n))) for k, v, n in re.findall(r' (\w+)=(\d+)/(\d+)', m.group(12)))
                rels[m.group(1)].append(dict(idx=int(m.group(3)), phase=m.group(4), cost=int(m.group(5)), nom=int(m.group(7)), comp=int(m.group(8)), probes=probes))
            m = re.match(r'STACK motion_task_min_free_bytes=(\d+)', l)
            if m: stack = min(stack or 1 << 30, int(m.group(1)))
    return sc, rels, stack
data = {v: load(f) for v, f in src.items()}
print('Scenario | variant | heaviest release | NOMINAL mean / max (WALK) | releases >280k / >320k | stack min free')
for s in ('B3', 'D8', 'H', 'E', 'S'):
    for v in ('base', 'p1', 'fp32', 'p1fp32'):
        sc, rels, st = data[v]
        if s not in sc: continue
        x = sc[s]; mx = max((r['cost'] for r in rels[s]), default=0)
        print(f"{s:3s} {v:7s} max={mx:7d}  nominal={x['nmean']:6d}/{x['nmax']:6d}  >280k={x.get('o280','?'):>4} >320k={x.get('o320','?'):>4}  stack={st}")
# P0 classes on the heaviest recorded releases: (1) two or more compensation solves, (2) nominal solve + compensation
# solve in the same release, (3) WALK with more than 4 contact IK (steered edge re-solves).
def cls(r):
    p = r['probes']; ct = p.get('compensationTarget', (0, 0))[1]; sg = p.get('solveGaitInto', (0, 0))[1]; cik = p.get('contactInverseKinematics', (0, 0))[1]
    return [c for c, ok in (('1_double_comp', ct >= 2), ('2_nominal+comp', sg >= 1 and ct >= 1), ('3_walk_double_ik', cik > 4 * max(sg, 1))) if ok]
print('\nHeaviest recorded release per P0 class (all scenarios):')
for c in ('1_double_comp', '2_nominal+comp', '3_walk_double_ik'):
    row = []
    for v in ('base', 'p1', 'fp32', 'p1fp32'):
        best = max(((r['cost'], s, r) for s, rs in data[v][1].items() for r in rs if c in cls(r)), default=None, key=lambda t: t[0])
        row.append(f"{v}={best[0]}({best[1]} {best[2]['phase']})" if best else f"{v}=-")
    print(f'  {c:18s} ' + '  '.join(row))

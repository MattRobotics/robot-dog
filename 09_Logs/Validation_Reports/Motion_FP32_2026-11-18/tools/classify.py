#!/usr/bin/env python3
"""Decomposition of the RAW differences between two bench runs (frames2_*.bin: clock, RAW[12], nominal valid, nominal
target[12] in rad before the IMU compensation), phase-aligned as compare_phases.py. For every joint of every aligned
frame: dRAW = dNOM + dCOMP exactly, where NOM is the nominal target in ticks and COMP = RAW - 2048 - NOM the compensation
contribution (rounding included). A frame whose nominal matches the other run's nominal at a shift of 1-2 releases
(and not at 0) is a TIME SHIFT. Continuity: per phase, max RAW step and second difference, and the nominal second
difference (ticks) of the WALK phases. Usage: classify.py <out_a> <out_b> [<out_a> <out_b> ...]"""
import sys, struct, os, re, math
from collections import Counter, defaultdict
K = 2 * math.pi / 4096
def phases(f):
    d = defaultdict(list)
    for l in open(f):
        m = re.match(r'PHASE (\S+) (\d+) (\S+)', l)
        if m: d[m.group(1)].append((int(m.group(2)), m.group(3)))
    return d
def frames(f):
    b = open(f, 'rb').read(); n = len(b) // 145; out = []
    for i in range(n):
        o = 145 * i; c = struct.unpack_from('<q', b, o)[0]; raw = struct.unpack_from('<12H', b, o + 8); v = b[o + 32]
        nom = struct.unpack_from('<12d', b, o + 33); ia = struct.unpack_from('<2d', b, o + 129); out.append((c, raw, v, nom, ia))
    return out
corrd = defaultdict(list); cls = Counter(); worst = defaultdict(float); big = Counter(); cont = defaultdict(lambda: [0, 0, 0, 0]); nomj = defaultdict(lambda: [0.0, 0.0])
pairs = list(zip(sys.argv[1::2], sys.argv[2::2]))
for A, B in pairs:
    PA, PB = phases(os.path.join(A, 'tol.txt')), phases(os.path.join(B, 'tol.txt'))
    for sc in PA:
        pa, pb = PA[sc], PB[sc]
        if [x[1] for x in pa] != [x[1] for x in pb]: cls['PHASE_SEQUENCE_DIFFERS'] += 1; continue
        fa, fb = frames(os.path.join(A, 'frames2_%s.bin' % sc)), frames(os.path.join(B, 'frames2_%s.bin' % sc))
        for run, fr in ((0, fa), (1, fb)):  # continuity per phase (RAW steps, second differences; nominal 2nd diff in WALK)
            ph = pa if run == 0 else pb; times = [t for t, _ in ph]
            import bisect
            for i in range(2, len(fr)):
                k = bisect.bisect_right(times, fr[i][0]) - 1; p = ph[k][1].split('/')[1] if k >= 0 else '?'
                if bisect.bisect_right(times, fr[i-2][0]) - 1 != k: continue
                step = max(abs(fr[i][1][j] - fr[i-1][1][j]) for j in range(12)); sd = max(abs(fr[i][1][j] - 2*fr[i-1][1][j] + fr[i-2][1][j]) for j in range(12))
                c = cont[(p, run)]; c[0] = max(c[0], step); c[1] = max(c[1], sd); c[2] += sd >= 3; c[3] += 1
                if p.startswith('WALK') and fr[i][2] and fr[i-1][2] and fr[i-2][2]:
                    nsd = max(abs(fr[i][3][j] - 2*fr[i-1][3][j] + fr[i-2][3][j]) for j in range(12)) / K
                    q = nomj[run]; q[0] = max(q[0], nsd); q[1] += 1
        for i in range(len(pa)):
            ea, eb = pa[i][0], pb[i][0]; na = pa[i+1][0] if i + 1 < len(pa) else 1 << 62; nb = pb[i+1][0] if i + 1 < len(pb) else 1 << 62
            ga = {t - ea: r for r in fa for t in [r[0]] if ea <= t < na}; gb = {t - eb: r for r in fb for t in [r[0]] if eb <= t < nb}
            for off in sorted(set(ga) & set(gb)):
                a, b = ga[off], gb[off]
                d = max(abs(x - y) for x, y in zip(a[1], b[1]))
                if d == 0: cls['identical'] += 1; continue
                if not (a[2] and b[2]): cls['different_no_nominal(%s)' % pa[i][1]] += 1; continue
                dn = [(b[3][j] - a[3][j]) / K for j in range(12)]; dc = [(b[1][j] - b[3][j] / K) - (a[1][j] - a[3][j] / K) for j in range(12)]
                mn, mc = max(abs(x) for x in dn), max(abs(x) for x in dc)
                shift = None
                if mn >= .5:
                    for s in (-1, 1, -2, 2):
                        o2 = ga.get(off + s * 20000)
                        if o2 and o2[2] and max(abs(b[3][j] - o2[3][j]) / K for j in range(12)) < .5: shift = s; break
                key = ('TIME_SHIFT' if shift is not None else 'NOMINAL' if mn >= mc else 'COMPENSATION')
                cls[key] += 1; worst[key] = max(worst[key], d); worst[key + '_nom'] = max(worst[key + '_nom'], mn); worst[key + '_comp'] = max(worst[key + '_comp'], mc)
                if d >= 3: big[(key, pa[i][1].split('/')[1])] += 1
                dcorr = max(abs(b[4][0] - a[4][0]), abs(b[4][1] - a[4][1])) * 1000; corrd[key].append((dcorr, d))
print('FRAMES', dict(cls))
for k in ('NOMINAL', 'COMPENSATION', 'TIME_SHIFT'):
    print('%-12s max dRAW=%d  max |dNOM|=%.3f ticks  max |dCOMP|=%.3f ticks' % (k, worst[k], worst[k + '_nom'], worst[k + '_comp']))
print('frames with dRAW>=3 by class and phase:', dict(big))
for k, v in corrd.items():
    big3 = [c for c, d in v if d >= 3]
    print('%-12s applied IMU correction difference: max %.3f mrad overall, max %.3f mrad where dRAW>=3 (%d frames); frames with dRAW>=3 and correction difference <0.05 mrad: %d' % (k, max(c for c, _ in v), max(big3) if big3 else 0, len(big3), sum(1 for c in big3 if c < .05)))
print('CONTINUITY (run a/b): phase  max_step  max_2nd_diff  frames_2nd_diff>=3 / frames')
for p in sorted({k[0] for k in cont}):
    a, b = cont[(p, 0)], cont[(p, 1)]
    print('  %-16s %3d/%3d  %3d/%3d  %4d/%4d of %d/%d' % (p, a[0], b[0], a[1], b[1], a[2], b[2], a[3], b[3]))
print('NOMINAL 2nd difference in WALK (ticks): max a=%.3f b=%.3f over %d/%d frames' % (nomj[0][0], nomj[1][0], nomj[0][1], nomj[1][1]))

#!/usr/bin/env python3
"""IRAM sets after the 16/11 MEMSTOP (read-only, same replay as iram2/tools/eval2.py and eval_sets.py).
Greedy sets by misses per byte over placeable Motion functions only: local or global text symbols (nm t/T); the weak
COMDAT inline functions (W/w, e.g. StopPlan::build, jointSlot) are excluded because they cannot be placed in IRAM
(l32r 'literal placed after use' at link, 16/11). Also evaluates the named sets G2c (installed in cb9ed85) and G2s.
Usage: eval3.py <iram2 dir> <budget> [<budget> ...]"""
import importlib.util, re, subprocess, sys
from pathlib import Path
D = Path(sys.argv[1]); budgets = [int(b) for b in sys.argv[2:]]
T = D / 'tools'
src = (T / 'eval_sets.py').read_text(); ns = {'__file__': str(T / 'eval_sets.py')}
exec(src.split('def main')[0], ns)  # definitions only (SETS, NM); eval_sets.py runs main() at import
NM = ns['NM']; elf = str(D / 'out/base/build/qreal.ino.elf'); dump = str(D / 'out/base/dump.bin')
EXCL = re.compile(r'ServoPort::referenceFault|SimpleMotion::update\(|stepWalkMode|::sample\(matdog::motion::GaitParameters|settledUnderLoad')
rows = []
for l in open(D / 'out/base/rank_motion.txt'):
    m = re.match(r'(\d+)\s+([\d.]+)\s+(\d+)\s+([\d.]+)\s+(?:[\d.]+\s+){5}\d+\s+[\d.]+\s+(\w)\s+(.*)$', l.rstrip())
    if m: rows.append((float(m.group(4)), int(m.group(3)), m.group(5), m.group(6)))
syms = []
for line in subprocess.run([NM, '-C', '-S', '--defined-only', elf], capture_output=True, text=True, check=True).stdout.splitlines():
    p = line.split(None, 3)
    if len(p) == 4 and p[2] in 'tTwW' and 0x42000000 <= int(p[0], 16) < 0x44000000: syms.append((int(p[0], 16), int(p[1], 16), p[2], p[3]))
def hits(names): return [s for s in syms if any(s[3].startswith(x) for x in names)]
def replay(names):
    rng = ['%x-%x' % (a, a + s) for a, s, _, _ in hits(names)]
    out = subprocess.run([str(T / 'icache_replay'), dump] + rng, capture_output=True, text=True).stdout
    return {m.group(1): (int(m.group(2)), int(m.group(3))) for m in re.finditer(r'SUMMARY (K\d) mean_miss=(\d+) max_miss=(\d+)', out)}
base = replay([])
def show(label, names):
    h = hits(names); res = replay(names)
    print('%s bytes=%d symbols=%d weak=%d ' % (label, sum(s[1] for s in h), len(h), sum(s[2] in 'wW' for s in h)) +
          ' '.join('%s %d/%d (%+.0f%%/%+.0f%%)' % (k, v[0], v[1], 100 * (v[0] - base[k][0]) / base[k][0], 100 * (v[1] - base[k][1]) / base[k][1]) for k, v in sorted(res.items())), flush=True)
print('BASE ' + ' '.join('%s %d/%d' % (k, *v) for k, v in sorted(base.items())), flush=True)
G2C = ns['SETS']['G2c_no_inline']
G2S = [x for x in G2C if not x.startswith(('matdog::motion::contactInverseKinematicsGroundNormal(', 'matdog::motion::validGaitParameters('))]
show('G2c', G2C); show('G2s', G2S)
for x in G2C:
    h = hits([x]); print('   member %6d %s %s' % (sum(s[1] for s in h), ''.join(sorted({s[2] for s in h})), x.split('(')[0]))
cand = [r for r in rows if not EXCL.search(r[3]) and r[2] in 'tT']; cand.sort(key=lambda r: -r[0])
for b in budgets:
    sel = []; used = 0
    for d, s, t, n in cand:
        if used + s <= b: sel.append(n); used += s
    show('GREEDY_NOWEAK budget=%d' % b, sel)
    print('   functions: ' + '; '.join(n.split('(')[0] for n in sel), flush=True)

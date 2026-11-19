"""Read-only: section split [READ, NOMINAL, COMPENSATE, CONVERT, FRAME, REST] of each paired window's heaviest
release, from section 5 of two ab_compare.py outputs (AB_TABLES.md). Groups by phase class of that release."""
import json, statistics, sys
NAMES = ('READ', 'NOMINAL', 'COMPENSATE', 'CONVERT', 'FRAME', 'REST')
def rows(path):
    out, on = [], False
    for line in open(path):
        if line.startswith('## 5'): on = True; continue
        if on and line.startswith('## '): break
        if not on or not line.startswith('| ') or line.startswith('| Strumento') or line.startswith('|---'): continue
        c = [x.strip() for x in line.strip().strip('|').split('|')]
        for mx, ph, sec in ((c[3], c[4], c[5]), (c[8], c[9], c[10])):
            out.append((int(mx), ph, json.loads(sec)))
    return out
def cls(ph):
    return 'WALK' if ph.startswith('WALK') else 'PRONE' if ph.startswith('PRONE') else 'STAND/HEIGHT'
for tag, path in zip(sys.argv[1::2], sys.argv[2::2]):
    r = rows(path)
    print('== %s: %d window maxima (A+B)' % (tag, len(r)))
    for k in ('WALK', 'PRONE', 'STAND/HEIGHT', 'ALL'):
        sel = [x for x in r if k == 'ALL' or cls(x[1]) == k]
        if not sel: continue
        means = [statistics.mean(s[2][i] for s in sel) for i in range(6)]
        maxs = [max(s[2][i] for s in sel) for i in range(6)]
        print('  %-12s n=%2d  max release %5d us | mean of sections %s | max of sections %s' % (
            k, len(sel), max(s[0] for s in sel), ' '.join('%s=%d' % (n, m) for n, m in zip(NAMES, means)),
            ' '.join('%s=%d' % (n, m) for n, m in zip(NAMES, maxs))))

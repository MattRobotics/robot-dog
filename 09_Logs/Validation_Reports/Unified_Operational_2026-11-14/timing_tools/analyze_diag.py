"""dev.8/dev.8u P0 diagnostic session analysis (builds be9198d, 0526cf6 and successors). Read-only; no device I/O.
Usage: python3 -I analyze_diag.py <run-dir>   (run-dir holds serial.log, http.jsonl)

1. The @MOTION TIMING dumps in serial.log (MOTION_PMU lines: since-boot sums per
   class WALK/RECOVERY/OTHER and per counter group, MotionTiming.h kPmuGroups).
   Each trial is the DELTA between two consecutive dumps; the boot baseline is the
   first dump. Every armed release of the class is in it (the sums are never reset).
2. Self-checks per trial: group-0 cycles at every level / CCOUNT (must be ~1, else
   the counter configuration is not counting what it claims), real clock of the
   NOMINAL+COMPENSATE sections (CCOUNT / us, expected 240 MHz), instructions per
   release against the QEMU references of the same sequence.
3. Decomposition of the NOMINAL+COMPENSATE cycles that issue no instruction, and
   the dominant cause (>= half of that excess), or INCONCLUSIVE.
4. Per armed command of each trial window (HTTP 1 Hz command_timing): missed,
   overruns, max release; since-boot missed/overrun deltas of the window, taken
   from the last sample before the dump (the dump itself delays idle releases)."""
import collections, json, re, sys
from pathlib import Path

CLASSES = ('WALK', 'RECOVERY', 'OTHER')
# QEMU P0 tracer on b561062 (dev.8 REPORT.md §3): mean NOMINAL+COMPENSATE
# instructions per release, STANDARD yaw (A) and steered sweep (B). The tracer
# times sections in emulated CCOUNT; instructions = CCOUNT x QEMU_K.
# QEMU_K depends on the emulator configuration, NOT on the ESP32-S3 clock:
#   Espressif QEMU 9.2.2 (esp_develop_9.2.2_20260417), -machine esp32s3,
#   -icount shift=3 (8 ns of virtual time per instruction); its CCOUNT model
#   advances at ~40 MHz of virtual time whatever CPU frequency the firmware
#   configures (QEMU reports 80 MHz to it). Measured 14/11/2026: 200 000
#   instructions = 64 498-64 995 CCOUNT for a zero-overhead nop loop, an ALU +
#   branch loop and a load/store loop -> 3.077-3.101 instructions per CCOUNT;
#   CCOUNT scales with 2^shift (shift 2: ~6.2, shift 4: ~1.52). Cross-check on
#   the hardware PMU of 14/11: WALK NOMINAL (pure computation, no servo-port
#   call) is 0.92-0.99x the tracer with this factor. Re-measure QEMU_K if the
#   emulator build or the -icount shift changes.
# The dev.8 constant 1.5625 (an assumed 80 MHz CCOUNT model) under-counted the
# reference 1.98x; the values below are the same tracer means rescaled to the
# measured factor. The acceptance band (0.7-1.3) is unchanged.
QEMU_K, QEMU_K_DEV8 = 3.088, 1.5625
QEMU_REF_DEV8 = {'WALK': (402e3, 438e3), 'RECOVERY': (575e3, 665e3)}  # as published with K = 1.5625
QEMU_REF = {c: tuple(v * QEMU_K / QEMU_K_DEV8 for v in r) for c, r in QEMU_REF_DEV8.items()}
NUM = re.compile(r'(\w+)=([^ ]+)')


def parse_serial(path):
    dumps, cur = [], None
    for line in open(path, errors='replace'):
        stamp, _, text = line.rstrip('\n').partition(' ')
        if text.startswith('>>> @MOTION TIMING'):
            cur = {'utc': stamp, 'pmu': {}, 'heaviest': {}, 'tasks': [], 'refused': False}
            dumps.append(cur)
        elif cur is None:
            continue
        elif text.startswith('MOTION_TIMING=REFUSED'):
            cur['refused'] = True
        elif text.startswith('MOTION_PMU '):
            f = dict(NUM.findall(text))
            cur['pmu'][(f['class'], int(f['group']))] = {k: int(v) for k, v in f.items() if v.isdigit()}
        elif text.startswith('MOTION_PMU_HEAVIEST '):
            f = dict(NUM.findall(text))
            cur['heaviest'][f['class']] = f
        elif text.startswith('MOTION_PMU_TASK '):
            m = re.match(r'MOTION_PMU_TASK name=(.*) core=(\S+) priority=(\S+) runtime_us=(\S+)$', text)  # task names may hold spaces
            if m:
                cur['tasks'].append({'name': m[1], 'core': m[2], 'priority': m[3], 'runtime_us': m[4]})
    return [d for d in dumps if not d['refused']]


def delta(a, b):
    out = {}
    for key, row in b['pmu'].items():
        base = a['pmu'].get(key, {}) if a else {}
        out[key] = {k: v - base.get(k, 0) for k, v in row.items()}
    return out


def nc(row, field):  # NOMINAL + COMPENSATE
    return row.get('nominal_' + field, 0) + row.get('compensate_' + field, 0)


def analyse_class(d, cls, ref):
    g = {k[1]: v for k, v in d.items() if k[0] == cls and v.get('releases', 0) > 0}
    if 0 not in g:
        return None
    r = {'releases': sum(v['releases'] for v in g.values())}
    us = sum(nc(v, 'us') for v in g.values())
    cc = sum(nc(v, 'cc') for v in g.values())
    r['clock_mhz'] = cc / us if us else 0
    g0 = g[0]
    r['cycles_over_ccount'] = g0['c1'] / g0['cc'] if g0['cc'] else 0  # whole release, every level
    r['insn_per_release'] = nc(g0, 'c0') / g0['releases']
    r['ipc'] = nc(g0, 'c0') / nc(g0, 'c1') if nc(g0, 'c1') else 0
    r['qemu_ratio'] = r['insn_per_release'] / (sum(ref) / 2) if ref else None
    share = lambda grp, c: (nc(g[grp], c) / nc(g[grp], 'cc')) if grp in g and nc(g[grp], 'cc') else None
    r['istall'], r['dstall'] = share(1, 'c0'), share(1, 'c1')
    r['istall_mem'], r['istall_muldiv'] = share(2, 'c0'), share(2, 'c1')
    r['dstall_mem'], r['bubbles'] = share(3, 'c0'), share(3, 'c1')
    r['irq_cycles'] = share(5, 'c0')
    r['irq_insn_per_release'] = nc(g[5], 'c1') / g[5]['releases'] if 5 in g else None
    r['istall_irq'], r['dstall_irq'] = share(6, 'c0'), share(6, 'c1')
    r['interrupts_per_release'] = g[4]['c0'] / g[4]['releases'] if 4 in g else None
    r['window_exc_per_release'] = g[4]['c1'] / g[4]['releases'] if 4 in g else None
    wall = sum(v['wall_us'] for v in g.values())
    r['other_tasks'] = sum(v['other_us'] for v in g.values()) / wall if wall else 0
    r['other_pinned'] = sum(v['other_pinned_us'] for v in g.values()) / wall if wall else 0
    r['wall_ms_per_release'] = wall / r['releases'] / 1e3
    r['nc_ms_per_release'] = us / r['releases'] / 1e3
    # Cycles that issue no instruction, as a share of the NOMINAL+COMPENSATE cycles.
    r['excess'] = 1 - r['ipc'] if r['ipc'] else None
    cand = {k: r[k] for k in ('istall_mem', 'dstall_mem', 'istall_muldiv', 'bubbles', 'irq_cycles') if r.get(k) is not None}
    cand['other_tasks'] = r['other_tasks']
    dom = max(cand, key=cand.get)
    checks = []
    if not 0.98 <= r['cycles_over_ccount'] <= 1.02:
        checks.append('PMU cycles/CCOUNT %.3f: counter configuration not validated' % r['cycles_over_ccount'])
    if r['ipc'] <= 0:
        checks.append('instructions not counted')
    if r['qemu_ratio'] is not None and not 0.7 <= r['qemu_ratio'] <= 1.3:
        checks.append('instructions per release %.2fx the QEMU reference' % r['qemu_ratio'])
    if not 238 <= r['clock_mhz'] <= 242:
        r['verdict'] = 'CLOCK: %.1f MHz' % r['clock_mhz']
    elif checks:
        r['verdict'] = 'INCONCLUSIVE (' + '; '.join(checks) + ')'
    elif r['excess'] and cand[dom] >= .5 * r['excess']:
        r['verdict'] = 'DOMINANT %s = %.2f of the cycles (excess %.2f)' % (dom, cand[dom], r['excess'])
    else:
        r['verdict'] = 'NO SINGLE DOMINANT CAUSE (largest %s = %.2f, excess %.2f)' % (dom, cand[dom], r['excess'] or 0)
    return r


def http_windows(path, edges):
    """Samples of each window [edge_i + 3 s, edge_i+1): the 3 s guard skips the
    idle releases a dump printout may have delayed."""
    import datetime
    ts = lambda t: datetime.datetime.fromisoformat(t).timestamp() if t else 0.0
    samples = []
    for line in open(path):
        rec = json.loads(line)
        if rec.get('path') == '/api/motion/status' and rec.get('s'):
            samples.append((rec['host_utc'], ts(rec['host_utc']), rec['s']))
    out = []
    for lo, hi in zip(edges, edges[1:]):
        lo_t, hi_t = ts(lo) + (3 if lo else 0), ts(hi)
        win = [(t, s) for t, x, s in samples if lo_t <= x < hi_t]
        if not win:
            out.append(None)
            continue
        cmds = collections.OrderedDict()
        for t, s in win:
            ac = s.get('active_command', {})
            key = (ac.get('id'), ac.get('sequence'), ac.get('accepted_ms'))
            ct = s.get('command_timing', {})
            c = cmds.setdefault(key, {'first': t, 'reasons': []})
            c.update(last=t, missed=ct.get('missed'), overruns=ct.get('overruns'), max_work_us=ct.get('max_work_us'),
                     max_work_phase=ct.get('max_work_phase'), work_hist=s.get('work_hist'))
            if not c['reasons'] or c['reasons'][-1] != s['reason']:
                c['reasons'].append(s['reason'])
        first, last = win[0][1], win[-1][1]
        out.append({'from': win[0][0], 'to': win[-1][0], 'commands': list(cmds.values()),
                    'missed_delta': last['missed_releases'] - first['missed_releases'],
                    'overrun_delta': last['overrun_releases'] - first['overrun_releases'],
                    'releases_delta': last['releases'] - first['releases']})
    return out


def main():
    run = Path(sys.argv[1])
    dumps = parse_serial(run / 'serial.log')
    print('QEMU reference (instructions per release, K=%.3f per CCOUNT): %s' % (QEMU_K, ', '.join('%s %.0f-%.0f k' % (c, r[0] / 1e3, r[1] / 1e3) for c, r in QEMU_REF.items())))
    print('dumps accepted: %d (%s)' % (len(dumps), ', '.join(d['utc'][11:19] for d in dumps)))
    if not dumps:
        return 1
    snaps = dumps
    if dumps[0]['pmu']:
        print('NOTE: the first dump already holds PMU sums (no boot baseline): trial 1 starts at boot')
        snaps = [{'utc': '', 'pmu': {}, 'heaviest': {}, 'tasks': []}] + dumps
    windows = http_windows(run / 'http.jsonl', [s['utc'] for s in snaps]) if (run / 'http.jsonl').exists() else []
    fmt = lambda x: '-' if x is None else '%.2f' % x
    for i in range(1, len(snaps)):
        d = delta(snaps[i - 1], snaps[i])
        print('\n== trial %d: %s -> %s' % (i, snaps[i - 1]['utc'][11:19] or 'boot', snaps[i]['utc'][11:19]))
        for cls in CLASSES:
            r = analyse_class(d, cls, QEMU_REF.get(cls))
            if not r:
                continue
            print('  %-8s releases %d, wall %.2f ms/release (N+C %.2f ms), clock %.1f MHz, PMU cycles/CCOUNT %.3f' % (
                cls, r['releases'], r['wall_ms_per_release'], r['nc_ms_per_release'], r['clock_mhz'], r['cycles_over_ccount']))
            print('           N+C: %.0f k instr/release (QEMU x%s), IPC %.2f, excess %s | istall %s (mem %s, mul/div %s) dstall %s (mem %s) bubbles %s' % (
                r['insn_per_release'] / 1e3, fmt(r['qemu_ratio']), r['ipc'], fmt(r['excess']), fmt(r['istall']), fmt(r['istall_mem']),
                fmt(r['istall_muldiv']), fmt(r['dstall']), fmt(r['dstall_mem']), fmt(r['bubbles'])))
            print('           IRQ/exception level: cycles %s, %s instr/release, istall %s dstall %s | interrupts %s, window exc %s per release | other tasks %.3f (pinned %.3f)' % (
                fmt(r['irq_cycles']), fmt(r['irq_insn_per_release']), fmt(r['istall_irq']), fmt(r['dstall_irq']),
                fmt(r['interrupts_per_release']), fmt(r['window_exc_per_release']), r['other_tasks'], r['other_pinned']))
            print('           VERDICT', r['verdict'])
        for cls, h in snaps[i]['heaviest'].items():
            print('  heaviest %-8s since boot: release %s work %s us, group %s (%s), nominal %s us compensate %s us, other %s us' % (
                cls, h.get('release'), h.get('work_us'), h.get('group'), h.get('events'), h.get('nominal_us'), h.get('compensate_us'), h.get('other_us')))
        w = windows[i - 1] if i - 1 < len(windows) else None
        if w:
            print('  HTTP window %s..%s: since-boot missed +%d, overruns +%d, releases +%d' % (
                w['from'][11:19], w['to'][11:19], w['missed_delta'], w['overrun_delta'], w['releases_delta']))
            for c in w['commands']:
                print('    command %s..%s %-60s missed %s overruns %s max %s us (%s) hist %s' % (
                    c['first'][11:19], c['last'][11:19], '>'.join(c['reasons'])[:60], c['missed'], c['overruns'],
                    c['max_work_us'], c['max_work_phase'], c['work_hist']))
    if snaps[-1]['tasks']:
        print('\n== tasks at the last dump (run time since boot, U32 us: wraps after 71.6 min)')
        for t in snaps[-1]['tasks']:
            print('  %-16s core %3s prio %3s runtime %s us' % (t.get('name'), t.get('core'), t.get('priority'), t.get('runtime_us')))
    return 0


if __name__ == '__main__':
    sys.exit(main())

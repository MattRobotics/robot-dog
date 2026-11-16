#!/usr/bin/env python3
"""WALK_SETTLE peak analysis (P0 closure, 16/11/2026). Read-only, no device I/O. Usage:
    python3 -I analyze_settle.py <run-dir> [<run-dir> ...]
For each run directory (targeted acquisition or suite: serial.log, results.json, http.jsonl):
1. per command (HTTP command_timing at the end of each step): max release, its phase, COMPENSATE,
   solves, overruns, missed;
2. the heaviest OTHER release since boot (MOTION_PMU_HEAVIEST of the last @MOTION TIMING dump):
   its PMU group is the one the 7-release rotation armed for it (release % 7). The COMPENSATE
   section is read against the OTHER class sums of the same dump (same section, every OTHER release):
     group 0 instructions / cycles: instructions measured directly;
     group 1 I-side / D-side GlobalStall cycles: stall fractions; instructions estimated from
             cycles = instructions + stalls + bubbles, with the class bubbles/instructions ratio
             (groups 3 and 0 of the same dump); band with that ratio x0.5..x1.5;
     group 2 instruction memory stalls / mul-div stalls; group 3 data memory stalls / bubbles;
     group 4 interrupts / window exceptions; groups 5-6 work above level 0 (ISRs, critical sections);
3. HTTP 1 Hz samples taken in WALK_SETTLE and in the HOLD right after: IMU reason, refusal
   (refused_by), wanted and applied correction.
QEMU reference: the rig's WALK_SETTLE releases with the same work (1 solve, 4 IK, 4 Newton, 11 FK)
cost 0.62-0.69 M instructions in COMPENSATE (MATDOG_1_0_2_P0_PEAKS_20261116/out/settle_p3b)."""
import json, re, sys
from pathlib import Path

RIG_SETTLE_INSN = (0.62e6, 0.69e6)
KV = re.compile(r'(\w+)=(\S+)')


def dumps(serial):
    """Group MOTION_PMU / MOTION_PMU_HEAVIEST lines by dump (a new dump starts at each MOTION_PMU group=0 of WALK
    after a non-PMU line). Returns a list of {'sums': {(class, group): kv}, 'heaviest': {class: kv}, 't': utc}."""
    out, cur = [], None
    for line in serial.read_text(errors='replace').splitlines():
        if ' MOTION_PMU ' in line or ' MOTION_PMU_HEAVIEST ' in line:
            kv = dict(KV.findall(line))
            if cur is None:
                cur = {'sums': {}, 'heaviest': {}, 't': line[:23]}
                out.append(cur)
            if ' MOTION_PMU_HEAVIEST ' in line:
                cur['heaviest'][kv['class']] = kv
            else:
                cur['sums'][(kv['class'], int(kv['group']))] = kv
        elif cur is not None and 'MOTION_PMU' not in line and line.strip():
            if cur['heaviest']:
                cur = None
    return [d for d in out if d['sums'] or d['heaviest']]


def ratio(kv, a, b):
    return int(kv[a]) / int(kv[b]) if int(kv[b]) else float('nan')


def peak(d):
    h = d['heaviest'].get('OTHER')
    if not h:
        return ['  no heaviest OTHER release in this dump']
    g = int(h['group'])
    cc, c0, c1 = (int(h['compensate_' + k]) for k in ('cc', 'c0', 'c1'))
    o = ['  heaviest OTHER since boot: phase %s release %s work %s us (lateness %s us, other tasks %s us), group %d (%s)' % (
        h['phase'], h['release'], h['work_us'], h['lateness_us'], h['other_us'], g, h['events']),
         '    sections us: read %s nominal %s compensate %s convert %s frame %s rest %s; work solves %s ik %s newton %s fk %s' % (
        h['read_us'], h['nominal_us'], h['compensate_us'], h['convert_us'], h['frame_us'], h['rest_us'],
        h['solves'], h['ik'], h['newton'], h['fk']),
         '    COMPENSATE: cycles %d, %s %d (%.1f %%), %s %d (%.1f %%), %.1f MHz' % (
        cc, h['events'].split('/')[0], c0, 100 * c0 / cc, h['events'].split('/')[1], c1, 100 * c1 / cc, cc / int(h['compensate_us']))]
    s = d['sums'].get(('OTHER', g))
    if s:
        o.append('    OTHER class, same group, COMPENSATE of every OTHER release: %.1f %% / %.1f %%' % (
            100 * ratio(s, 'compensate_c0', 'compensate_cc'), 100 * ratio(s, 'compensate_c1', 'compensate_cc')))
    s0, s3 = d['sums'].get(('OTHER', 0)), d['sums'].get(('OTHER', 3))
    if g == 0:
        o.append('    instructions MEASURED: %.2f M (IPC %.3f); rig with the same work %.2f-%.2f M' % (c0 / 1e6, c0 / cc, *(x / 1e6 for x in RIG_SETTLE_INSN)))
    elif g == 1 and s0 and s3:
        b = ratio(s3, 'compensate_c1', 'compensate_cc') / ratio(s0, 'compensate_c0', 'compensate_cc')  # class bubbles per instruction
        est = lambda k: (cc - c0 - c1) / (1 + k * b)
        o.append('    instructions ESTIMATED (cycles - stalls = instructions + bubbles, class bubbles/instruction %.3f): %.2f M '
                 '(band %.2f-%.2f M for that ratio x1.5..x0.5); rig with the same work %.2f-%.2f M' % (
                     b, est(1) / 1e6, est(1.5) / 1e6, est(.5) / 1e6, *(x / 1e6 for x in RIG_SETTLE_INSN)))
    return o


def http_settle(run):
    p = run / 'http.jsonl'
    if not p.exists():
        return ['  no http.jsonl']
    o, prev = [], None
    for line in p.read_text().splitlines():
        try:
            d = json.loads(line)
        except ValueError:
            continue
        s = d.get('s') or {}
        ph = s.get('phase')
        if ph == 'WALK_SETTLE' or (ph == 'HOLD' and prev == 'WALK_SETTLE'):
            ic = s.get('imu_correction') or {}
            o.append('  %s %-11s %-10s refused_by=%-10s wanted %s applied %s' % (
                d['host_utc'][11:23], ph, ic.get('reason'), ic.get('refused_by', '-'), ic.get('wanted_mrad'), ic.get('applied_mrad')))
        prev = ph
    return o or ['  no sample in WALK_SETTLE']


def commands(run):
    p = run / 'results.json'
    if not p.exists():
        return ['  no results.json']
    o = []
    for r in json.loads(p.read_text()):
        ct = r.get('command_timing') or {}
        if ct.get('max_work_phase') == 'WALK_SETTLE' or r['step'].split()[0] in ('S4', 'S6', 'D5', 'D7', 'D9', 'D11'):
            o.append('  %-44s %s max %5s us %-12s compensate %5s us solves %s overruns %s missed %s' % (
                r['step'][:44], r['result'], ct.get('max_work_us'), ct.get('max_work_phase'), ct.get('max_compensate_us'),
                ct.get('max_work_solves'), ct.get('overruns'), ct.get('missed')))
    return o or ['  no release command']


def main():
    for a in sys.argv[1:]:
        run = Path(a)
        print('== %s' % run)
        print(' commands ending in or containing a WALK_SETTLE peak (HTTP command_timing):')
        print('\n'.join(commands(run)))
        ds = dumps(run / 'serial.log') if (run / 'serial.log').exists() else []
        print(' @MOTION TIMING dumps with PMU lines: %d' % len(ds))
        if ds:
            print(' last dump %s:' % ds[-1]['t'])
            print('\n'.join(peak(ds[-1])))
        print(' HTTP samples in WALK_SETTLE and the HOLD after it:')
        print('\n'.join(http_settle(run)))


if __name__ == '__main__':
    main()

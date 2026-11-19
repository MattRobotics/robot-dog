#!/usr/bin/env python3
"""Read-only summary of a fixture_suite.py run (no device I/O).
Usage: python3 -I suite_summary.py <run-dir> [<run-dir> ...]

Per run: every step of results.json (PASS/FAIL, duration, missed/overrun deltas,
command_timing at the end of the step, STOP/resume fields), the abort record of
events.jsonl, and from http.jsonl (http_watch.py, /status every 5 s and
/api/motion/status at 1 Hz):
 - DALY D1 over the run: samples, OK count, rx_bytes / first_byte / stale_bytes /
   fail_streak values, alarms, pack voltage and SoC range;
 - Motion: states and faults seen, maximum joint temperature and minimum joint
   voltage per joint, IMU correction extremes (applied, mrad) and reasons;
 - the last motion status (state, reason, SAFE_OFF mask, torque)."""
import collections, json, sys
from pathlib import Path


def load_jsonl(p):
    out = []
    if not p.exists():
        return out
    for line in p.read_text(errors='replace').splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            pass
    return out


def summary(run):
    print('=' * 100)
    print('RUN', run.name)
    res = json.loads((run / 'results.json').read_text()) if (run / 'results.json').exists() else []
    for r in res:
        extra = {k: r[k] for k in ('stop_verified_s', 'complete_s', 'standing_after_s', 'walking_after_s', 'resume_allowed', 'resume_reason',
                                   'reason', 'ack_reason', 'height_estimate_mm', 'state', 'torque_active', 'safe_off_mask') if k in r}
        ct = r.get('command_timing') or {}
        print('%-4s %-62s %5.1f s  missed %+d overruns %+d | cmd missed %s first %s/%s us, max work %s us (%s) comp %s | %s %s' % (
            r['result'], r['step'][:62], r['seconds'], r.get('missed_delta', 0), r.get('overrun_delta', 0),
            ct.get('missed'), ct.get('first_miss_phase') or '-', ct.get('first_miss_lateness_us'), ct.get('max_work_us'), ct.get('max_work_phase'),
            ct.get('max_compensate_us'), json.dumps(extra), r.get('why', '')))
    ev = load_jsonl(run / 'events.jsonl')
    for e in ev:
        if e['kind'] in ('abort', 'abort_stop_sent', 'abort_stop_state', 'abort_no_stop', 'ctl_refused'):
            print('EVENT', json.dumps(e)[:400])
    print('steps: %d PASS, %d FAIL' % (sum(r['result'] == 'PASS' for r in res), sum(r['result'] == 'FAIL' for r in res)))

    h = load_jsonl(run / 'http.jsonl')
    st = [x for x in h if x.get('path') == '/status' and isinstance(x.get('s'), dict)]
    ms = [x for x in h if x.get('path') == '/api/motion/status' and isinstance(x.get('s'), dict)]
    if st:
        c = collections.Counter
        comm = c(x['s'].get('bms_comm') for x in st)
        print('DALY D1: %d /status samples %s..%s; comm %s' % (len(st), st[0]['host_utc'][11:19], st[-1]['host_utc'][11:19], dict(comm)))
        print('  rx_bytes %s | first_byte_ms %s | stale %s | fail_streak max %s | fresh %s | alarms %s' % (
            dict(c(x['s'].get('bms_rx_bytes') for x in st)), dict(sorted(c(x['s'].get('bms_first_byte_ms') for x in st).items())),
            dict(c(x['s'].get('bms_stale_bytes') for x in st)), max(int(x['s'].get('bms_fail_streak', 0) or 0) for x in st),
            dict(c(x['s'].get('bms_fresh') for x in st)), dict(c(x['s'].get('bms_alarms') for x in st))))
        v = [float(x['s']['bms_pack_v']) for x in st if x['s'].get('bms_pack_v')]
        soc = [float(x['s']['bms_soc_percent']) for x in st if x['s'].get('bms_soc_percent')]
        cur = [float(x['s']['bms_current_a']) for x in st if x['s'].get('bms_current_a')]
        cmin = [int(x['s']['bms_cell_min_mv']) for x in st if x['s'].get('bms_cell_min_mv')]
        if v:
            print('  pack %.1f..%.1f V, SoC %.1f..%.1f %%, current %.1f..%.1f A, cell min %d mV; health %s; uptime %s..%s ms' % (
                min(v), max(v), min(soc), max(soc), min(cur), max(cur), min(cmin),
                dict(c(x['s'].get('system_health') for x in st)), st[0]['s'].get('uptime_ms'), st[-1]['s'].get('uptime_ms')))
    if ms:
        states = collections.Counter((x['s']['state'], x['s'].get('fault')) for x in ms)
        print('Motion: %d status samples; (state, fault) %s' % (len(ms), dict(states)))
        faults = [x for x in ms if x['s'].get('fault') or x['s']['state'] == 'FAULT']
        if faults:
            print('  FAULT samples: %d, first %s %s' % (len(faults), faults[0]['host_utc'], faults[0]['s'].get('reason')))
        tmax, vmin = {}, {}
        for x in ms:
            for j in x['s'].get('joints', []):
                n = j['joint']
                tmax[n] = max(tmax.get(n, -1), j.get('temperature', -1))
                vmin[n] = min(vmin.get(n, 999), j.get('voltage_dv', 999))
        print('  joint temperature max (C): %s' % ' '.join('%s=%s' % kv for kv in tmax.items()))
        print('  joint voltage min (dV): %s' % ' '.join('%s=%s' % kv for kv in vmin.items()))
        ic = [x['s'].get('imu_correction') or {} for x in ms]
        ap = [a for d in ic for a in (d.get('applied_mrad') or [])]
        wa = [a for d in ic for a in (d.get('wanted_mrad') or [])]
        if ap:
            print('  IMU correction applied %.1f..%.1f mrad, wanted %.1f..%.1f mrad; reasons %s; imu_valid %s' % (
                min(ap), max(ap), min(wa), max(wa), dict(collections.Counter(d.get('reason') for d in ic)),
                dict(collections.Counter(x['s'].get('imu_valid') for x in ms))))
        last = ms[-1]
        s = last['s']
        print('  last %s: state %s phase %s reason %s fault %s safe_off %s mask %s torque %s missed %s overruns %s max_work %s us' % (
            last['host_utc'][11:23], s['state'], s['phase'], s['reason'], s.get('fault'), s['safe_off_verified'], s['safe_off_mask'],
            s['torque_active'], s['missed_releases'], s['overrun_releases'], s['max_release_work_us']))


for a in sys.argv[1:]:
    summary(Path(a))

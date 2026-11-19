"""M0 memory gate, read only, the first step of every fixture run (after the 16/11 MEMSTOP).

It runs before any Motion command, once the run's two HTTP clients are both active: the fixture tool's own poller
(started by Suite.open) and the run's single http_watch.py (started by run_acquisition.py once ctl.d exists). It
waits at least SETTLE_S with both active, so the start burst (capabilities, /status, first polls of both clients)
is inside the measured window, then requires:
  - heap_min_free >= 24 KB (24 576 B, the mandate's threshold) on every sample: the tool's latest /api/motion/status
    (transport) and /status, and every watcher sample of the run;
  - no allocation-failure line in the run's serial.log (heap_check.ALLOC).
Below the threshold the step raises Abort before any Motion command: Suite.safe_end sends no STOP to a robot that is
not engaged, so the robot is left untouched (READY, torque OFF). Nothing is sent to the robot from here. The live
guard (heap_guard.py) and every existing check and STOP stay as they are for the rest of the run."""
import importlib.util, json, time
from pathlib import Path

LIMIT = 24 * 1024
SETTLE_S = 20.0
MIN_WATCH = 12
ROOT = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('heap_check_mod', ROOT / 'heap_check.py')
hc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(hc)


def watcher_samples(run, since):
    out = []
    p = Path(run) / 'http.jsonl'
    if not p.exists():
        return out
    for line in open(p, errors='replace'):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        s = r.get('s')
        if not isinstance(s, dict):
            continue
        src = s if r.get('path') == '/status' else (s.get('transport') or {})
        if 'heap_min_free' in src:
            out.append((r['host_utc'], int(src['heap_min_free']), int(src.get('heap_free', -1))))
    return [x for x in out if x[0] >= since]


def memory_gate(S, Abort):
    t0 = time.time()
    since = time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(t0))
    while True:
        if S.term:
            raise Abort('M0: SIGTERM (external stop request)')
        m, mt, st, stt = S.snap()
        why = S.health(m, st, mt, stt)
        if why:
            raise Abort('M0: %s' % why)
        if m['torque_active'] or m['state'] not in ('READY', 'STOPPED'):
            raise Abort('M0: robot not at rest before the gate (%s/%s, torque %s)' % (m['state'], m['reason'], m['torque_active']))
        ws = watcher_samples(S.run, since)
        low_now = min([int(m['transport']['heap_min_free']), int(st['heap_min_free'])] + [v for _, v, _ in ws])
        if low_now < LIMIT:                # below the threshold already during the wait: stop at once
            break
        if time.time() - t0 >= SETTLE_S and len(ws) >= MIN_WATCH:
            break
        if time.time() - t0 > 60:
            raise Abort('M0: the run watcher is not active (%d samples in 60 s)' % len(ws))
        time.sleep(0.5)
    m, mt, st, stt = S.snap()
    vals = [('tool /api/motion/status', int(m['transport']['heap_min_free'])), ('tool /status', int(st['heap_min_free']))]
    vals += [('watcher %s' % u[11:23], v) for u, v, _ in ws]
    low = min(vals, key=lambda x: x[1])
    S.log.flush()
    alloc = hc.scan(S.run)[4]          # the run's serial.log, same pattern as heap_check.py
    info = {'heap_min_free': low[1], 'source': low[0], 'margin_b': low[1] - LIMIT, 'watch_samples': len(ws),
            'heap_free_now': int(m['transport']['heap_free']), 'alloc_lines': len(alloc), 'waited_s': round(time.time() - t0, 1)}
    S.event('memory_gate', **info)
    if low[1] < LIMIT or alloc:
        raise Abort('M0 MEMORY_GATE: heap_min_free %d B (%s) < %d B or allocation-failure lines %d; no Motion command sent'
                    % (low[1], low[0], LIMIT, len(alloc)))
    return info


def with_gate(seq_fn, Abort):
    """Wrap a sequence builder: M0 first, then the unchanged steps."""
    def seq(S):
        return [('M0 memory gate (read only, both HTTP clients active, before any Motion command)',
                 lambda: memory_gate(S, Abort))] + seq_fn(S)
    return seq

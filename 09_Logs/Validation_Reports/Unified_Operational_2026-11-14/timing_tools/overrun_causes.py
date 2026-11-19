#!/usr/bin/env python3
"""Read-only: overruns per step grouped by the cause class of the step's heaviest release (command_timing):
attenuated STAND/PRONE/HEIGHT (max_work_phase in the STAND/PRONE/HEIGHT phases), WALK_SETTLE, steered walk
(joystick steps), WALK STOP / frontal walk, other. Usage: overrun_causes.py <suite-dir>[,<H-dir>] ..."""
import json, sys, collections
def cls(step, ct):
    ph = ct.get('max_work_phase') or ''
    name = step['step']
    if ph == 'WALK_SETTLE':
        return 'WALK_SETTLE'
    if ph in ('ALIGN', 'STAND_DEPLOY', 'STAND_LIFT', 'HEIGHT', 'PRONE_HEIGHT', 'PRONE_LIFT', 'PRONE_DEPLOY', 'PRONE_LAND', 'ACQUIRE', 'ENABLE', 'DISENGAGE'):
        return 'ATTENUATED STAND/PRONE/HEIGHT'
    if 'joystick' in name or name.split()[0] in ('D3', 'D5', 'D7', 'D9', 'D11'):
        return 'STEERED WALK (+release)'
    if 'resume' in name or name.split()[0] in ('H7', 'H9', 'H14', 'H16'):
        return 'RECOVERY / RESUME'
    return 'FRONTAL WALK / WALK STOP / OTHER'
for arg in sys.argv[1:]:
    agg = collections.Counter(); tot = 0
    for d in arg.split(','):
        for x in json.load(open(d + '/results.json')):
            o = x.get('overrun_delta', 0)
            if o:
                agg[cls(x, x.get('command_timing') or {})] += o; tot += o
    print(' + '.join(p.rstrip('/').split('/')[-1] for p in arg.split(',')), 'total', tot)
    for k, v in agg.most_common():
        print('   %-36s %3d  (%4.1f %%)' % (k, v, 100.0 * v / tot))

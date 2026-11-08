#!/usr/bin/env python3
"""MATDOG dev.4 ground validation (2026-11-08): per-window metrics from the
original dashboard traces. Read-only. Samples are de-duplicated by Controller
uptime (now_ms) across the overlapping exports (2)/(3); commands by sequence.
Counter deltas are taken between the first and last sample of each window.
Optional: --fk <fk_feet binary> adds the lateral/fore-aft excursion of the
mean of the four contact references (firmware FK of the 12 measured joints).
Usage: python3 -I ground_metrics.py <evidence dir> [--fk path]"""
import json, sys, os, subprocess, statistics
ev=sys.argv[1]; fk=sys.argv[sys.argv.index('--fk')+1] if '--fk' in sys.argv else None
files=['02_ground_walk100/matdog-motion-trace (1).json','03_ground_joystick_and_recovery/matdog-motion-trace (2).json','03_ground_joystick_and_recovery/matdog-motion-trace (3).json']
rows={}; cmds={}
for f in files:
    d=json.load(open(os.path.join(ev,f)))
    for s in d['samples']: rows.setdefault(s['status']['now_ms'],s['status'])
    for c in d['commands']:
        old=cmds.get(c['sequence'])
        assert old is None or (old['label'],old['issued_at_ms'])==(c['label'],c['issued_at_ms'])
        cmds[c['sequence']]=c
T=sorted(rows)
# Windows in Controller uptime (ms), boot after KEY OFF/ON. Bounds from the state timeline.
W=[('STAND150 from PRONE (boot 2)',32052,45714),('HOLD 150',45714,50259),('HEIGHT 150->120',50259,52377),
   ('WALK STANDARD 100 #1',52377,92255),('WALK STOP + settle #1',92255,99606),('HOLD 120',99606,105207),
   ('JOYSTICK lateral (vy 3.0, vx 1-1.6 mm/s)',292454,300517),('joystick release + settle',300517,308907),
   ('JOYSTICK forward (vx 6.9, vy 0.1-0.4 mm/s)',313504,321174),('joystick release + settle',321174,329208),
   ('JOYSTICK yaw CCW 1.76 deg/s',338319,344616),('joystick release + settle',344616,351956),
   ('WALK STANDARD 100 #2',388720,439125),('STOP during WALK',439125,441213),
   ('STAND150 from stopped WALK',446457,457307),('STAND 85/135/80',460794,472009),('PRONE',473753,485302)]
def win(a,b): return [t for t in T if a<=t<=b]
def feet(ts):
    if not fk: return None
    inp=''.join(f"{t} "+' '.join(repr(x) for x in rows[t]['telemetry']['actual_rad'])+"\n" for t in ts)
    out=subprocess.run([fk],input=inp,capture_output=True,text=True,check=True).stdout.split('\n')
    my=[];mx=[]
    for line in out:
        if not line: continue
        v=line.split(); st=[int(v[1+4*l]) for l in range(4)]
        if any(st): continue
        my.append(1e3*sum(float(v[3+4*l]) for l in range(4))/4); mx.append(1e3*sum(float(v[2+4*l]) for l in range(4))/4)
    return (max(my)-min(my), max(mx)-min(mx), len(my)) if my else None
print('window,t0_s,t1_s,seconds,samples,releases,overruns,overrun_pct,missed,missed_per_s,max_release_work_us_end,loop_stack_min_free_end,mean_foot_y_pp_mm,mean_foot_x_pp_mm')
for name,a,b in W:
    ts=win(a,b); s0,s1=rows[ts[0]],rows[ts[-1]]
    dt=(s1['now_ms']-s0['now_ms'])/1e3; rel=s1['releases']-s0['releases']; ov=s1['overrun_releases']-s0['overrun_releases']; mi=s1['missed_releases']-s0['missed_releases']
    f=feet(ts); fy,fx=(f'{f[0]:.1f}',f'{f[1]:.1f}') if f else ('','')
    print(f'"{name}",{a/1e3:.1f},{b/1e3:.1f},{dt:.1f},{len(ts)},{rel},{ov},{100*ov/max(rel,1):.1f},{mi},{mi/max(dt,1e-9):.2f},{s1["max_release_work_us"]},{s1["transport"]["loop_stack_min_free"]},{fy},{fx}')
print()
print('totals: distinct samples',len(T),'commands',len(cmds),'faults',sum(rows[t]['fault'] for t in T),
      'non-200',sum(1 for c in cmds.values() if c.get('http')!=200),
      'min loop stack',min(rows[t]['transport']['loop_stack_min_free'] for t in T),
      'min heap',min(rows[t]['transport']['heap_min_free'] for t in T),
      'max release work',max(rows[t]['max_release_work_us'] for t in T),'max gap',max(rows[t]['max_gap_us'] for t in T),
      'late',max(rows[t]['transport']['late'] for t in T),'busy',max(rows[t]['transport']['busy'] for t in T))

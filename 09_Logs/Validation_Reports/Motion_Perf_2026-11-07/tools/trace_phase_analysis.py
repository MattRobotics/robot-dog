#!/usr/bin/env python3
"""Per-phase release/overrun/missed deltas from MATDOG dashboard traces.

Read-only. Each sample carries cumulative counters (releases, missed_releases,
overrun_releases) and the Motion phase at sampling time; consecutive sample
deltas are attributed to the phase of the later sample. Samples are
de-duplicated by (now_ms) across overlapping files so windows are not summed
twice. Usage: python3 -I trace_phase_analysis.py <trace.json>...
"""
import json, sys, collections

def load(path):
    with open(path) as f:
        return json.load(f)

def main(paths):
    seen=set(); rows=[]
    for p in paths:
        d=load(p)
        for s in d['samples']:
            st=s['status']; key=st.get('now_ms')
            if key in seen: continue
            seen.add(key)
            rows.append((key,p,st))
    rows.sort(key=lambda r:r[0])
    print(f"distinct samples: {len(rows)} from {len(paths)} files")
    per=collections.defaultdict(lambda:[0,0,0,0.0,0])  # releases, missed, overruns, seconds, samples
    prev=None
    maxwork=collections.defaultdict(int)
    records={}
    for now,p,st in rows:
        ph=st['phase']
        maxwork[ph]=max(maxwork[ph],st.get('max_release_work_us',0))
        ct=st.get('command_timing',{})
        tm=st.get('timing',{})
        mw=tm.get('max_work') if isinstance(tm,dict) else None
        if mw:
            records[tuple(mw)]=(ph,ct.get('max_work_phase'),tm.get('max_work_meta'))
        if prev is not None:
            pn,pst=prev
            dr=st['releases']-pst['releases']; dm=st['missed_releases']-pst['missed_releases']
            do=st.get('overrun_releases',0)-pst.get('overrun_releases',0); dt=(now-pn)/1000.0
            if dr>=0 and dm>=0 and do>=0 and 0<dt<5:
                e=per[ph]; e[0]+=dr; e[1]+=dm; e[2]+=do; e[3]+=dt; e[4]+=1
        prev=(now,st)
    print(f"{'phase':16s} {'rel':>7s} {'missed':>7s} {'overrun':>7s} {'sec':>7s} {'ovr%':>6s} {'miss/s':>7s} {'maxwork_us':>10s}")
    for ph,e in sorted(per.items(),key=lambda kv:-kv[1][0]):
        rel,mi,ov,sec,n=e
        print(f"{ph:16s} {rel:7d} {mi:7d} {ov:7d} {sec:7.1f} {100*ov/max(rel,1):6.1f} {mi/max(sec,1e-9):7.2f} {maxwork[ph]:10d}")
    print("\nDistinct max_work records (current parts | previous parts | solves,ik,newton,fk):")
    print(" sample_phase      cmd_phase        read  nomin  compen  conv  frame rest | prev: read nomin compen conv frame rest | s ik nw fk | total")
    for rec,(ph,cph,meta) in sorted(records.items(),key=lambda kv:-sum(kv[0][:6])):
        cur=rec[:6]; pv=rec[6:12]; w=rec[12:16]
        print(f" {ph:16s} {str(cph):16s} "+" ".join(f"{v:5d}" for v in cur)+" | "+" ".join(f"{v:5d}" for v in pv)+" | "+" ".join(f"{v:2d}" for v in w)+f" | {sum(cur):6d} meta={meta}")

if __name__=='__main__':
    main(sys.argv[1:])

from pathlib import Path
import importlib.util,json,tempfile,subprocess,sys,csv
p=Path(__file__).resolve().parents[1]/'validation/motion_observer.py';spec=importlib.util.spec_from_file_location('obs',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
s={'phase':'PRONE_CONTACT_UNVERIFIED','state':'MOVING','now_ms':1000,'stale':False,'active_command':{'id':'prone-1','outcome':'ACCEPTED'},'telemetry':{'actual_rad':[.17,1.4615,-1.5246,-.17,1.4615,-1.5246,-.17,1.5708,-1.5533,.17,1.5708,-1.5533]},'joints':[{'slot':i,'valid':True,'age_ms':20,'raw':2048,'commanded_raw':2050} for i in range(12)]}
r=list(m.margin_rows(s,100,5));assert len(r)==12 and all(x['gen1_margin_ticks']>0 and x['box_margin_ticks']>0 and x['error_raw_ticks']==2 for x in r)
assert all(x['q_rad'] is None for x in m.margin_rows(s,100,300))
s['stale']=True;assert all(x['gen1_margin_ticks'] is None for x in m.margin_rows(s,100,0));s['stale']=False
with tempfile.TemporaryDirectory() as d:
 d=Path(d);(d/'trace.json').write_text(json.dumps({'samples':[{'status':s,'received_ms':100,'roundtrip_ms':5}]}));subprocess.run([sys.executable,str(p),'--replay-json',str(d/'trace.json'),'--out',str(d/'out')],check=True);assert len(list(csv.DictReader((d/'out/joints.csv').open())))==12
assert '/api/motion/command' not in p.read_text() and 'serial' not in p.read_text()
print('MOTION_OBSERVER PASS: offline replay, 12 margins, stale excluded, GET-only, no timed STOP')

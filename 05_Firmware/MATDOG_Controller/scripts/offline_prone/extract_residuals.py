#!/usr/bin/env python3
"""Extract 23 historical HOLD snapshots, never open a device.
CSV `load` is signed formatted telemetry, NOT the servo sign-bit word.
"""
import csv,json,hashlib,sys
from pathlib import Path
source=Path(sys.argv[1]);here=Path(__file__).resolve().parent
rows=[r for r in csv.DictReader(source.open()) if r['phase']=='HOLD'];groups={}
for r in rows:groups.setdefault(r['dev_ms'],[]).append(r)
assert len(groups)==23 and all(len(g)==12 for g in groups.values())
dirs=[-1,1,-1,-1,-1,1,1,-1,1,1,1,-1]
data={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'directions':dirs,'cases':[]}
for t,g in groups.items():
 g.sort(key=lambda r:int(r['slot']));c={'dev_ms':int(t),'err':[int(r['err']) for r in g],'load':[int(r['load']) for r in g],'load_mag':[int(r['load_mag']) for r in g]}
 assert all(abs(v)==m for v,m in zip(c['load'],c['load_mag']));data['cases'].append(c)
(here/'stand100_residuals.json').write_text(json.dumps(data,indent=2)+'\n')
h='// Recorded HOLD residuals, transformed to the positive-sign host encoder.\n// CSV load is signed; servo load uses magnitude plus direction bit 1024.\n// Source and hash: ../offline_prone/stand100_residuals.json\nstruct ResidualCase { int error[12],load[12]; };\nstatic const ResidualCase residualCases[]={\n'
for c in data['cases']:
 es=[d*e for d,e in zip(dirs,c['err'])];ls=[abs(v)|(1024 if d*v<0 else 0) for d,v in zip(dirs,c['load'])]
 for e,l in zip(es,ls):
  if abs(e)>12:assert (e>0)==bool(l&1024) and (l&1023)>=4*abs(e) and (l&1023)<=400
 h+='{{'+','.join(map(str,es))+'},{'+','.join(map(str,ls))+'}},\n'
h+='};\n';(here.parent/'tests/prone_residual_fixture.h').write_text(h)

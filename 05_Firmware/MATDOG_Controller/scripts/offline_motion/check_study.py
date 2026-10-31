#!/usr/bin/env python3
"""Regression assertions on generated OFFLINE study; never a contact certificate."""
import csv,json,math,sys
from pathlib import Path
out=Path(sys.argv[1]);rows=list(csv.DictReader((out/'route_analysis.csv').open()));s=json.loads((out/'rest_summary.json').read_text())
assert rows
for r in rows:
 assert all(math.isfinite(float(r[f'q{i}'])) for i in range(12))
 assert float(r['box_margin'])>=.03999999
 assert float(r['gen1_margin_rad'])>0
for group in ('0','1'):
 forward=[r for r in rows if r['route']=='current' and r['group']==group]
 reverse=[r for r in rows if r['route']=='reverse' and r['group']==group]
 for a,b in [(forward[0],reverse[-1]),(forward[-1],reverse[0])]:
  assert max(abs(float(a[f'q{i}'])-float(b[f'q{i}'])) for i in range(12))<1e-8
 assert abs(float(forward[-1]['t'])-float(reverse[-1]['t']))<1e-8
 # 10 ms nominal samples must obey 0.55 rad/s (within CSV quantization).
 for group_rows in [forward,reverse]:
  for a,b in zip(group_rows,group_rows[1:]):
   dt=float(b['t'])-float(a['t']);assert dt>0
   assert max(abs(float(b[f'q{i}'])-float(a[f'q{i}'])) for i in range(12))<=.55*dt+1e-6
# These are rejection conditions: reversing all vias cannot assert belly landing.
assert s['reverse_0']['belly_clearance_start_end_mm'][-1]>5
assert s['reverse_1']['belly_clearance_start_end_mm'][-1]>4
assert s['current_0']['contacts']==['lf_foot_link,rf_foot_link,rh_foot_link,lh_foot_link']
benefit=s['current_0']['front_load_N_range'][1]-s['front_early_0']['front_load_N_range'][1]
assert abs(benefit)<.1
print('STUDY_REGRESSION PASS: limits/box/rate samples, reverse endpoints, belly landing NOT established, front benefit <0.1 N; NOT HARDWARE')

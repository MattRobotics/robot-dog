#!/usr/bin/env python3
"""Intersect the entry proof with measured-limit + physical-error bounds.
No raw limit is changed. Firmware checks the same folded-side bounds before
binding the planner. Existing PASS rows on larger domains remain valid;
FAIL/UNRESOLVED and expanded adjacent domains are proved afresh.
"""
import argparse,json,re
from pathlib import Path
from certify_measured_entry_region import Region,PairModel,sha,URDF,ROUTE,ROOT
from certify_adjacent_axis import certify
from paired_prone_model import LEGS
HEADER=ROOT/'05_Firmware/MATDOG_Controller/src/motion/OperationalTolerance.h'

def main():
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if a.output.exists():raise SystemExit('preserve certificates')
 d=json.loads(a.input.read_text());assert d['identity']['urdf_sha256']==sha(URDF) and d['identity']['route_sha256']==sha(ROUTE)
 h=HEADER.read_text();mins=[float(x) for x in re.search(r'kEntryLowerMeasuredMinRad\[4\]=\{([^}]*)\}',h)[1].split(',')]
 get=lambda n:float(re.search(r'\b'+n+r'=([.0-9]+);',h)[1])
 model=get('kModelUncertaintyRad');local=get('kEntryAdjacentModelRad');assert 0<local<=model<d['tracking_tube_rad']
 scene=PairModel().scene
 for n,m in d['identity']['meshes'].items():assert m['sha256']==sha(scene.model.collision_mesh_path(n))
 parent_box={j:tuple(v) for j,v in d['box_rad'].items()};box=dict(parent_box)
 for i,l in enumerate(LEGS):
  j=l+'_lower_leg_joint';lo,hi=box[j];box[j]=(max(lo,mins[i]-model),hi)
 assert all(parent_box[j][0]<=lo<=hi<=parent_box[j][1] for j,(lo,hi) in box.items())
 r=Region(scene,box);inherited=[];rerun=[]
 for i,row in enumerate(d['nonadjacent']):
  if row['status']=='PASS':inherited.append(row['pair']);continue
  new=r.certify_pair(*row['pair'],row['required_m'],180);d['nonadjacent'][i]=new;rerun.append(new);print('NONADJACENT',new,flush=True)
 adj_rerun=[]
 for i,row in enumerate(d['adjacent']):
  j=row['joint'];lo,hi=box[j]
  if j.endswith('lower_leg_joint'):lo=max(lo,mins[LEGS.index(j[:2])]-local)
  # HIP/UPPER region is already strictly inside URDF: full physical tube.
  oldlo,oldhi=row['domain_rad'];d['adjacent_domain_rad'][j]=[lo,hi]
  if row['status']=='PASS_CONTINUOUS' and oldlo<=lo<=hi<=oldhi:continue
  v=certify(scene,j,lo,hi,240);new=dict(joint=j,domain_rad=[lo,hi],**v);d['adjacent'][i]=new;adj_rerun.append(new);print('ADJACENT',j,v,flush=True)
 d['box_rad']={j:list(v) for j,v in box.items()};d['lower_measured_min_rad']=mins;d['adjacent_model_allowance_rad']=local;d['nonadjacent_model_allowance_rad']=model
 d['nonadjacent_pass']=sum(x['status']=='PASS' for x in d['nonadjacent']);d['adjacent_pass']=sum(x['status']=='PASS_CONTINUOUS' for x in d['adjacent']);d['all_pass']=d['nonadjacent_pass']==120 and d['adjacent_pass']==12
 d['domain_restriction']=dict(parent_path=str(a.input.resolve()),parent_sha256=sha(a.input),parent_box_rad={j:list(v) for j,v in parent_box.items()},inherited_pass_pairs=inherited,rerun_pairs=rerun,adjacent_rerun=adj_rerun,method='same CAD and thresholds; intersect with enforced actual LOWER limits + explicit physical error; never inherit a non-PASS',tool_sha256=sha(__file__),contract_sha256=sha(HEADER))
 d['scope']='HIP/UPPER +/-10deg, same two entry stages,40mrad CAD envelope intersected with enforced LOWER measured bounds +15mrad physical-model allowance; adjacent LOWER uses explicit8mrad local angular qualification bound'
 a.output.write_text(json.dumps(d,indent=1)+'\n');print('BOUNDED_ENTRY_ALL_PASS',d['all_pass'],flush=True)
if __name__=='__main__':main()

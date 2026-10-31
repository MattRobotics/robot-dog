#!/usr/bin/env python3
"""Bind the candidate's actual constants and immutable CAD inputs to its proofs.
Offline only. Never treats a sample or UNRESOLVED as continuous clearance.
"""
import argparse,hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
FW=ROOT/'05_Firmware/MATDOG_Controller'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--artifacts',type=Path,required=True);a=p.parse_args()
 h=(FW/'src/motion/OperationalTolerance.h').read_text()
 def literal(n):return float(re.search(r'\b'+n+r'=([.0-9]+);',h)[1])
 tube=literal('kGeometryTubeRad');tracking=literal('kTrackingRad');step=literal('kMaxStepRad');model=literal('kModelUncertaintyRad')
 assert tracking+step+3.141592653589793/4096+2e-6+model<tube
 names={'entry':'entry_region_40mrad_bounded.json','route':'route_boxes_40mrad.json','walk':'walk_region_40mrad.json','height':'height_region_40mrad.json'}
 rows={};urdf=ROOT/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
 for scope,name in names.items():
  path=a.artifacts/name;d=json.loads(path.read_text());assert d['all_pass'] is True,(scope,'not continuously proved')
  assert d.get('tracking_tube_rad',d.get('tube_rad'))==tube
  assert d['identity']['urdf_sha256']==sha(urdf)
  for link,digest in d['identity']['meshes'].items():
   assert (digest['sha256'] if isinstance(digest,dict) else digest)==sha(urdf.parent/'meshes/collision'/(link+'.stl'))
  if scope=='entry':
   assert d['adjacent_model_allowance_rad']==.008 and d['nonadjacent_model_allowance_rad']==model
   assert d['domain_restriction']['contract_sha256']==sha(FW/'src/motion/OperationalTolerance.h')
   parent=Path(d['domain_restriction']['parent_path']);assert sha(parent)==d['domain_restriction']['parent_sha256']
  if scope=='route':
   assert len(d['segments'])==251
   for s in d['segments']:assert s['pass_'] and len(s['pairs'])==120 and all(x['status']=='PASS' and x['required_m']==.003 for x in s['pairs'])
  else:
   assert d['nonadjacent_pass']==120 and d['adjacent_pass']==12
   assert len(d['nonadjacent'])==120 and all(x['status']=='PASS' for x in d['nonadjacent'])
   assert d['clearance_m']==.003 and d['ventral_extra_m']==.003
  assert len(d['adjacent'])==12 and all(x['status']=='PASS_CONTINUOUS' for x in d['adjacent'])
  if scope in ('walk','height'):
   header=FW/'src/motion'/('OperationalWalkProfile.cpp' if scope=='walk' else 'StandHeightRegion.h')
   assert d['identity']['profile_sha256']==sha(header)
  rows[scope]={'path':str(path.resolve()),'sha256':sha(path),'tube_rad':tube,'all_pass':True}
 print(json.dumps(dict(result='PASS',tracking_rad=tracking,max_step_rad=step,model_uncertainty_candidate_rad=model,geometry_tube_rad=tube,certificates=rows,hardware_validated=False),indent=2))
if __name__=='__main__':main()

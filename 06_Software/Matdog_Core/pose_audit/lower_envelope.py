"""Resolve the lower four-foot envelope against the explicit ground tolerance."""
import json
import numpy as np
from model import Model,LEGS,TOL,transform,sha
from survey import OUT,save,general_ik
from diagnostics import diagnostics

def main():
 m=Model();modes=json.loads((OUT/'contact_modes.json').read_text());families=[]
 for i,lift in enumerate(modes['lift_off_attempts']):
  seed=np.array(lift['endpoint']['q']);targets=np.array(lift['endpoint']['contacts']);targets[:,2]=0;rows=[]
  # Resolve all legs separately, including small L/R tessellation differences.
  for height in [0.,1e-8,1e-7,5e-7,1e-6,1.001e-6,1.01e-6,2e-6,5e-6,1e-5]:
   body=transform(xyz=(0,0,m.body_height+height));q=general_ik(m,targets,body,seed)
   if q is None:rows.append({'height_above_body_ground_m':height,'valid':False,'errors':['IK_FAILURE']});continue
   regime='BODY_AND_FOOT_SUPPORT' if height==0 else 'FOOT_SUPPORT';v=m.evaluate(q,body,regime,LEGS);v['height_above_body_ground_m']=height;v['diagnostics']=diagnostics(m,q,body);rows.append(v);seed=q
  families.append({'family':i,'samples':rows})
  print('lower envelope',i,[(r['height_above_body_ground_m'],r['valid']) for r in rows],flush=True)
 save('lower_envelope',{'provenance':{'canonical_sources':m.sources,'source_sha256':sha(OUT/'contact_modes.json'),'generator_sha256':sha(__file__)},'families':families,'limiting_constraint':'BODY_GROUND_CONTACT','geometric_infimum_height_above_body_ground_m':0.0,'numerical_undeclared_contact_boundary_m':TOL,'minimum_observed_pure_foot_height_above_body_ground_m':min(r['height_above_body_ground_m'] for f in families for r in f['samples'] if r['valid'] and r['regime']=='FOOT_SUPPORT'),'physical_clearance_approval':False})
if __name__=='__main__':main()

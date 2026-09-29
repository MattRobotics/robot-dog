"""Broader body/foot support families and explicit contact-transfer route search."""
import json,itertools
import numpy as np
from scipy.optimize import brentq
from model import Model,ROOT,LEGS,transform,TOL,sha
from survey import OUT,save,leg_ground,leg_clear,general_ik
from ik import ContactIK

def main():
 m=Model();ik=ContactIK();rest=json.loads((OUT/'rest_search.json').read_text());base=rest['selected'];b=transform(xyz=(0,0,m.body_height));families={};rows=[]
 # Resolve exact mesh foot contact with variable hip; retain actual edge support.
 for i in (0,2):
  found=[]
  for hip,u in itertools.product(np.linspace(*m.limits[3*i],9),np.linspace(*m.limits[3*i+1],31)):
   def floor(lower):
    q=np.zeros(12);q[3*i:3*i+3]=(hip,u,lower);tf=m.fk(q,b)[LEGS[i]+'_foot_link'];v=m.meshes[LEGS[i]+'_foot_link'].vertices
    return float(np.min(v@tf[2,:3]+tf[2,3]))
   grid=np.linspace(*m.limits[3*i+2],33);prev=grid[0];fp=floor(prev)
   for lower in grid[1:]:
    f=floor(lower)
    if fp*f<=0:
     root=brentq(floor,prev,lower,xtol=1e-13);q=[float(hip),float(u),float(root)]
     if leg_clear(m,i,q,m.body_height):found.append(q)
    prev,fp=lower,f
  families[LEGS[i]]=found;print('mesh foot family',LEGS[i],len(found),flush=True)
 # Select candidates away from limits, and preserve several distinct hip signs.
 for feet in [('lf','rf'),('rh','lh'),LEGS]:
  pool=[]
  for i in (0,2):
   samples=families[LEGS[i]] if LEGS[i] in feet else [base['q'][i*3:i*3+3]]
   samples=sorted(samples,key=lambda q:-min(np.minimum(np.array(q)-m.limits[i*3:i*3+3,0],m.limits[i*3:i*3+3,1]-np.array(q))))[:8]
   pool.append(samples)
  for f,r in itertools.product(*pool):
   q=f+[-f[0],f[1],f[2]]+r+[-r[0],r[1],r[2]]
   v=m.evaluate(q,b,'BODY_AND_FOOT_SUPPORT',feet);v['name']='BODY_PLUS_'+'_'.join(feet);rows.append(v)
  print('combination',feet,sum(v['valid'] for v in rows if v['active_feet']==list(feet)),flush=True)
 # Test contact additions through a raised-foot clearance path in joint space
 # while the base supports the COM. This path is a candidate, not an assumed safe interpolation.
 attempts=[]
 for dest in [r for r in rows if r['valid']][:12]:
  frames=[]
  for s in np.linspace(0,1,51):
   q=(1-s)*np.array(base['q'])+s*np.array(dest['q']);active=dest['active_feet'] if s==1 else []
   regime='BODY_AND_FOOT_SUPPORT' if active else 'BODY_SUPPORT'
   v=m.evaluate(q,b,regime,active);frames.append(v)
   if not v['valid']:break
  attempts.append({'to':dest['name'],'endpoint':dest,'frames':frames,'valid':len(frames)==51 and all(v['valid'] for v in frames)})
 # Lift-off proof obligation: body+four feet -> feet support. Hold actual footprint.
 lifts=[]
 for dest in [r for r in rows if r['valid'] and len(r['active_feet'])==4][:8]:
  targets=np.array(dest['contacts']);targets[:,2]=0;seed=np.array(dest['q']);frames=[]
  for h in np.linspace(.001,.06,60):
   body=transform(xyz=(0,0,h));q=general_ik(m,targets,body,seed)
   if q is None:frames.append({'valid':False,'errors':['IK_FAILURE'],'height':float(h)});break
   v=m.evaluate(q,body,'FOOT_SUPPORT',LEGS);v['max_delta_rad']=float(np.max(abs(q-seed)));seed=q;frames.append(v)
   if not v['valid']:break
  lifts.append({'endpoint':dest,'frames':frames,'valid':len(frames)==60 and all(v['valid'] for v in frames)})
 save('contact_modes',{'provenance':{'sources':m.sources,'generator_sha256':sha(__file__),'model_sha256':sha(ROOT/'06_Software/Matdog_Core/pose_audit/model.py')},'search':{'hip_samples':9,'upper_samples':31,'lower_brackets':32,'foot_ground_root_tolerance_rad':1e-13},'per_leg_mesh_contact_solutions':families,'combined_candidates':rows,'contact_addition_attempts':attempts,'lift_off_attempts':lifts,'global_absence_claim':False})
if __name__=='__main__':main()

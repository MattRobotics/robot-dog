"""Search quasi-static body shifts before each explicit foot-contact change."""
import json,itertools
import numpy as np
from model import Model,LEGS,transform,hull_margin,sha,PATCH_TOL
from survey import OUT,save,general_ik

def support_without(m,q,body,excluded):
 tf=m.fk(q,body);points=[]
 for i,l in enumerate(LEGS):
  if i==excluded:continue
  n=l+'_foot_link';v=m.meshes[n].vertices@tf[n][:3,:3].T+tf[n][:3,3];points.extend(v[v[:,2]<=PATCH_TOL])
 com,_=m.com(tf);return hull_margin(points,com)[0] if len(points)>=3 else None

def main():
 m=Model();modes=json.loads((OUT/'contact_modes.json').read_text());env=json.loads((OUT/'envelope.json').read_text());goal=np.array(env['poses']['LOW_C4']['contacts']);goal[:,2]=0;routes=[]
 for lift in modes['lift_off_attempts'][:1]:
  frames=list(lift['frames']);seed=np.array(frames[-1]['q']);body=np.array(frames[-1]['body']);targets=np.array(frames[-1]['contacts']);targets[:,2]=0;events=[];valid=True
  for leg in (2,3,0,1):
   if not valid:break
   current=body[:3,3].copy();options=sorted(itertools.product(np.arange(-.04,.041,.01),repeat=2),key=lambda xy:((xy[0]-current[0])**2+(xy[1]-current[1])**2,xy))
   chosen=None
   for x,y in options:
    proposed=transform(xyz=(x,y,.06));q=general_ik(m,targets,proposed,seed)
    if q is None:continue
    margin=support_without(m,q,proposed,leg)
    if margin is None or margin<.005:continue
    v=m.evaluate(q,proposed,'FOOT_SUPPORT',LEGS)
    if v['valid']:chosen=proposed;break
   if chosen is None:events.append({'leg':LEGS[leg],'status':'NO_BODY_SHIFT_WITH_5MM_MODEL_SUPPORT_MARGIN'});valid=False;break
   old=body.copy()
   for s in np.linspace(0,1,21)[1:]:
    body=transform(xyz=(1-s)*old[:3,3]+s*chosen[:3,3]);q=general_ik(m,targets,body,seed)
    if q is None:valid=False;events.append({'status':'SHIFT_IK_FAILURE'});break
    v=m.evaluate(q,body,'FOOT_SUPPORT',LEGS);v['segment']='shift_for_'+LEGS[leg];v['max_joint_delta_rad']=float(np.max(abs(q-seed)));seed=q;frames.append(v)
    if not v['valid']:valid=False;break
   if not valid:break
   start=targets[leg].copy();end=goal[leg]
   for s in np.linspace(0,1,41)[1:]:
    moving=targets.copy();smooth=10*s**3-15*s**4+6*s**5;moving[leg]=(1-smooth)*start+smooth*end;moving[leg,2]=.025*16*s*s*(1-s)*(1-s)
    q=general_ik(m,moving,body,seed)
    if q is None:valid=False;events.append({'status':'SWING_IK_FAILURE'});break
    active=LEGS if s==1 else [l for i,l in enumerate(LEGS) if i!=leg]
    v=m.evaluate(q,body,'FOOT_SUPPORT',active);v['segment']='swing_'+LEGS[leg];v['max_joint_delta_rad']=float(np.max(abs(q-seed)));seed=q;frames.append(v)
    if not v['valid']:valid=False;break
   events.append({'leg':LEGS[leg],'body_shift':chosen[:3,3].tolist(),'status':'PASS_SAMPLES' if valid else 'FAILED','contacts_during_swing':[l for i,l in enumerate(LEGS) if i!=leg]});targets[leg]=end
  if valid:
   old=body.copy()
   for s in np.linspace(0,1,41)[1:]:
    body=transform(xyz=(1-s)*old[:3,3]+s*np.array([0,0,.1]));q=general_ik(m,goal,body,seed)
    if q is None:valid=False;break
    v=m.evaluate(q,body,'FOOT_SUPPORT',LEGS);v['segment']='finish_LOW_C4';v['max_joint_delta_rad']=float(np.max(abs(q-seed)));seed=q;frames.append(v)
    if not v['valid']:valid=False;break
  routes.append({'start':lift['endpoint'],'events':events,'frames':frames,'all_samples_valid':valid,'continuous_collision_free_proved':False,'scope':'quasi-static CAD/URDF support and mesh-contact samples; no actuator/dynamic claim'})
  print('support-transfer',len(frames),valid,events,flush=True)
 save('support_transfer',{'provenance':{'sources':m.sources,'generator_sha256':sha(__file__)},'routes':routes,'startup_reassessment':'STILL_REQUIRED_UNTIL_CONTINUOUS_AND_CONTACT_TRANSFER_VALIDATION'})
if __name__=='__main__':main()

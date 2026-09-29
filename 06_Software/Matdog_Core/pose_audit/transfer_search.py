"""Explicit contact-mode path candidates, independently checked at every frame."""
import json
import numpy as np
from model import Model,LEGS,transform,sha
from survey import OUT,save,general_ik

def main():
 m=Model();modes=json.loads((OUT/'contact_modes.json').read_text());env=json.loads((OUT/'envelope.json').read_text());goal=np.array(env['poses']['LOW_C4']['contacts']);goal[:,2]=0;routes=[]
 for lift in modes['lift_off_attempts']:
  frames=list(lift['frames']);seed=np.array(frames[-1]['q']);body=np.array(frames[-1]['body']);targets=np.array(frames[-1]['contacts']);targets[:,2]=0;segments=[{'regime':'BODY_AND_FOOT_SUPPORT','contacts':['base']+list(LEGS),'event':'initial rest'},{'regime':'FOOT_SUPPORT','contacts':list(LEGS),'event':'base lift-off','frames':len(frames)}]
  valid=lift['valid']
  for leg in (2,3,0,1):
   if not valid:break
   start=targets[leg].copy();end=goal[leg];count=0
   for s in np.linspace(0,1,41)[1:]:
    moving=targets.copy();smooth=10*s**3-15*s**4+6*s**5;moving[leg]=(1-smooth)*start+smooth*end;moving[leg,2]=.025*16*s*s*(1-s)*(1-s)
    q=general_ik(m,moving,body,seed)
    if q is None:frames.append({'valid':False,'errors':['IK_FAILURE'],'leg':LEGS[leg],'progress':float(s)});valid=False;break
    active=LEGS if s==1 else [l for i,l in enumerate(LEGS) if i!=leg]
    v=m.evaluate(q,body,'FOOT_SUPPORT',active);v['max_joint_delta_rad']=float(np.max(abs(q-seed)));v['event']='swing_'+LEGS[leg];v['swing_progress']=float(s);frames.append(v);count+=1;seed=q
    if not v['valid']:valid=False;break
   segments.append({'regime':'FOOT_SUPPORT','contacts':[l for i,l in enumerate(LEGS) if i!=leg],'event':'remove/swing/add '+LEGS[leg],'frames':count})
   targets[leg]=end
  if valid:
   for h in np.linspace(.06,.1,41)[1:]:
    body=transform(xyz=(0,0,h));q=general_ik(m,goal,body,seed)
    if q is None:frames.append({'valid':False,'errors':['IK_FAILURE']});valid=False;break
    v=m.evaluate(q,body,'FOOT_SUPPORT',LEGS);v['max_joint_delta_rad']=float(np.max(abs(q-seed)));frames.append(v);seed=q
    if not v['valid']:valid=False;break
   segments.append({'regime':'FOOT_SUPPORT','contacts':list(LEGS),'event':'rise to LOW_C4','frames':40})
  routes.append({'start':lift['endpoint'],'segments':segments,'frames':frames,'all_samples_valid':valid,'continuous_collision_free_proved':False,'reason':'Sampled mesh/contact checks do not certify swept collision or contact force/patch transfer; exact G3 fixture uses cylinder references, this survey uses mesh support'})
  print('transfer',len(frames),valid,frames[-1].get('errors'),flush=True)
 save('transfer_search',{'provenance':{'sources':m.sources,'generator_sha256':sha(__file__)},'routes':routes,'startup_reassessment':'STILL_REQUIRED','scope':'OFFLINE CONTACT-MODE RESEARCH ONLY; no gait scheduler or runtime transition installed'})
if __name__=='__main__':main()

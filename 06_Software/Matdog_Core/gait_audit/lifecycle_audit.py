"""Full canonical mesh audit of start/stop samples and exact transition joins."""
import sys,json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from core import Core,params,lifecycle,ROOT
from survey import save,initialize,serial
sys.path.insert(0,str(ROOT/'06_Software/Matdog_Core/pose_audit'))
from model import Model,LEGS

def run(kind):
 model=Model();core=Core();p=params(kind=kind,height=.1,x=.01,duty=.8 if kind==0 else .6,sway=.004 if kind==0 else 0)
 # 50 ms at period=1 s is a geometric sampling interval, not an execution rate.
 times=np.linspace(0,7,141);frames=lifecycle(core,p,times);results=[];failure_counts={}
 for f in frames:
  active=[l for i,l in enumerate(LEGS) if f['leg_phase'][i,1] or f['leg_phase'][i,2]]
  a=model.evaluate(f['q'],f['body'],'FOOT_SUPPORT',active,full=True)
  geometric=[e for e in a['errors'] if kind==0 or e!='SUPPORT_INVALID']
  for e in geometric:failure_counts[e.split(':')[0]]=failure_counts.get(e.split(':')[0],0)+1
  results.append(dict(time_s=f['time_s'],state=f['state'],q=f['q'],qdot=f['qdot'],qddot=f['qddot'],body=f['body'],active_feet=active,contacts_world_m=a['contacts'],targets_world_m=f['feet'],com_world_m=a['com_world_m'],support_margin_m=a['support_margin_m'],min_separation=a['min_separation'],ground_min_m=a['ground_min_m'],errors=a['errors'],geometric_errors=geometric))
 return dict(type='WALK' if kind==0 else 'TROT',parameters=p,period_s=1.,stop_request_time_s=3.2,final_stand_time_s=7.,complete_sampled_geometry_valid=not failure_counts,failure_counts=failure_counts,frames=results,scope='Sampled geometry only. Kinematic lifecycle joins checked separately by independent derivatives oracle.')
def main():
 with ProcessPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,(0,1)))
 save('lifecycle_audit.json',dict(cases=results))
 for r in results:print(r['type'],r['failure_counts'],flush=True)
if __name__=='__main__':main()

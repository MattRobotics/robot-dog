"""Continue preserved routes: original-footprint rise and folded joint-space detours.
No gait, timing law, hardware command, or continuous-collision claim.
"""
import json
import numpy as np
from model import Model,LEGS,transform,sha
from survey import OUT,save,general_ik,leg_clear

def leg_route(m,index,start,goal,seed):
 rng=np.random.default_rng(seed);nodes=[np.array(start)];parents=[-1]
 def edge(a,b):
  for s in np.linspace(0,1,max(2,int(np.max(abs(b-a))/.025)+2)):
   if not leg_clear(m,index,(1-s)*a+s*b,m.body_height):return False
  return True
 for trial in range(2000):
  target=np.array(goal) if trial%5==0 else rng.uniform(m.limits[index*3:index*3+3,0],m.limits[index*3:index*3+3,1])
  k=int(np.argmin([np.linalg.norm(n-target) for n in nodes]));delta=target-nodes[k];dist=np.linalg.norm(delta)
  if dist<1e-12:continue
  new=nodes[k]+delta*min(1,.15/dist)
  if not edge(nodes[k],new):continue
  nodes.append(new);parents.append(k)
  if np.linalg.norm(new-goal)<.3 and edge(new,np.array(goal)):
   path=[np.array(goal)];k=len(nodes)-1
   while k>=0:path.append(nodes[k]);k=parents[k]
   return list(reversed(path)),trial+1,len(nodes)
 return None,2000,len(nodes)

def main():
 m=Model();modes=json.loads((OUT/'contact_modes.json').read_text());rest=json.loads((OUT/'rest_search.json').read_text())['selected'];rises=[];detours=[]
 for lift in modes['lift_off_attempts']:
  start=lift['endpoint'];q=np.array(start['q']);target=np.array(start['contacts']);target[:,2]=0;frames=[start];reason=None
  for h in np.linspace(.0001,.15,151):
   body=transform(xyz=(0,0,h));n=general_ik(m,target,body,q)
   if n is None:reason={'height_m':float(h),'failure':'IK_OPTIMIZER_OR_LIMIT'};break
   v=m.evaluate(n,body,'FOOT_SUPPORT',LEGS);v['progress']=float(h/.15);v['max_joint_delta_rad']=float(max(abs(n-q)));frames.append(v);q=n
   if not v['valid']:reason={'height_m':float(h),'failure':v['errors']};break
  rises.append({'frames':frames,'all_samples_valid':reason is None,'failure':reason,'fixed_quantity':'analytical reference XY; actual mesh minimum Z; mesh support location may migrate','continuous_collision_free_proved':False})
  print('rise',len(frames),reason,flush=True)
 for family,lift in enumerate(modes['lift_off_attempts']):
  goal=np.array(lift['endpoint']['q']);q=np.array(rest['q']);frames=[rest];attempts=[];failed=False
  for index in (2,0):
   path=None
   for seed in (17,31,73):
    path,iterations,nodes=leg_route(m,index,q[index*3:index*3+3],goal[index*3:index*3+3],seed)
    attempts.append({'leg':LEGS[index],'seed':seed,'iterations':iterations,'nodes':nodes,'found':path is not None})
    if path is not None:break
   if path is None:failed=True;break
   old=q.copy()
   for a,b in zip(path,path[1:]):
    for s in np.linspace(0,1,max(2,int(np.max(abs(b-a))/.025)+2))[1:]:
     v3=(1-s)*a+s*b;q[index*3:index*3+3]=v3;mirror=1 if index==0 else 3;q[mirror*3:mirror*3+3]=[-v3[0],v3[1],v3[2]]
     # Explicit contact set from the endpoint/ground geometry; unexpected leg
     # ground contact is still rejected, and every resulting whole pose is checked.
     tf=m.fk(q,transform(xyz=(0,0,m.body_height)));active=[]
     for l in LEGS:
      t=tf[l+'_foot_link'];z=np.min(m.meshes[l+'_foot_link'].vertices@t[2,:3]+t[2,3])
      if abs(z)<=1e-5:active.append(l)
     r=m.evaluate(q,transform(xyz=(0,0,m.body_height)),'BODY_AND_FOOT_SUPPORT' if active else 'BODY_SUPPORT',active)
     r['segment']='fold_'+LEGS[index];frames.append(r)
     if not r['valid']:failed=True;break
    if failed:break
   if failed:break
  detours.append({'family':family,'search':attempts,'frames':frames,'all_samples_valid':not failed,'continuous_collision_free_proved':False,'contact_events':'active foot mesh patches added only at ground boundary; body stays supported'})
  print('detour',family,len(frames),not failed,attempts,flush=True)
 save('route_extension',{'provenance':{'sources':m.sources,'generator_sha256':sha(__file__),'inputs':{n:sha(OUT/(n+'.json')) for n in ('contact_modes','rest_search')}},'original_footprint_rises':rises,'body_supported_detours':detours})
if __name__=='__main__':main()

"""Apply the preserved body-only connection protocol to every valid body-only rest candidate.

The original contact/route searches seeded only rest_search['selected']. This reruns the same
direct-interpolation and body-supported leg-detour protocol (same targets, seeds, trial counts)
from all valid BODY_SUPPORT candidates. Bounded failure is not a global impossibility proof.
"""
import json,concurrent.futures,argparse
import numpy as np
from model import Model,LEGS,transform,sha
from survey import OUT,save,leg_clear
from route_extension import leg_route
MODEL=None
SEEDS=(17,31,73)
def initialize():
 global MODEL;MODEL=Model()

def first_failure(frame,index,progress):
 return {'frame':index,'progress':progress,'errors':frame['errors'],'collisions':frame['collisions'],'min_separation':frame['min_separation']}

def direct(task):
 start,dest=task;m=MODEL;body=transform(xyz=(0,0,m.body_height));frames=0;failure=None
 for k,s in enumerate(np.linspace(0,1,51)):
  q=(1-s)*np.array(start['q'])+s*np.array(dest['q']);active=dest['active_feet'] if s==1 else []
  v=m.evaluate(q,body,'BODY_AND_FOOT_SUPPORT' if active else 'BODY_SUPPORT',active);frames+=1
  if not v['valid']:failure=first_failure(v,k,float(s));break
 return {'target':dest['name'],'target_active_feet':dest['active_feet'],'frames_evaluated':frames,'valid':failure is None,'first_failure':failure}

def detour(task):
 start,goal=task;m=MODEL;body=transform(xyz=(0,0,m.body_height));q=np.array(start['q']);goal=np.array(goal);attempts=[];frames=[start];failure=None
 for index in (2,0):
  path=None
  for seed in SEEDS:
   path,iterations,nodes=leg_route(m,index,q[index*3:index*3+3],goal[index*3:index*3+3],seed)
   attempts.append({'leg':LEGS[index],'seed':seed,'iterations':iterations,'nodes':nodes,'found':path is not None})
   if path is not None:break
  if path is None:failure={'stage':'leg_route_'+LEGS[index],'reason':'no collision-free single-leg route within bounded search'};break
  for a,b in zip(path,path[1:]):
   for s in np.linspace(0,1,max(2,int(np.max(abs(b-a))/.025)+2))[1:]:
    v3=(1-s)*a+s*b;q[index*3:index*3+3]=v3;mirror=1 if index==0 else 3;q[mirror*3:mirror*3+3]=[-v3[0],v3[1],v3[2]]
    tf=m.fk(q,body);active=[l for l in LEGS if abs(np.min(m.meshes[l+'_foot_link'].vertices@tf[l+'_foot_link'][2,:3]+tf[l+'_foot_link'][2,3]))<=1e-5]
    v=m.evaluate(q,body,'BODY_AND_FOOT_SUPPORT' if active else 'BODY_SUPPORT',active);v['segment']='fold_'+LEGS[index];frames.append(v)
    if not v['valid']:failure=first_failure(v,len(frames)-1,None);break
   if failure:break
  if failure:break
 result={'attempts':attempts,'frames_evaluated':len(frames),'all_samples_valid':failure is None,'failure':failure}
 if failure is None:result['frames']=frames
 return result

def run(task):
 kind,payload=task
 return (direct if kind=='direct' else detour)(payload)

def main():
 argparse.ArgumentParser().parse_args()
 rest=json.loads((OUT/'rest_search.json').read_text());modes=json.loads((OUT/'contact_modes.json').read_text());library={p['name']:p for p in json.loads((OUT/'pose_library.json').read_text())['poses']}
 starts=[(i,r) for i,r in enumerate(rest['candidates']) if r.get('valid') and r.get('regime')=='BODY_SUPPORT']
 destinations=[r for r in modes['combined_candidates'] if r['valid']][:12];goals=[(i,l['endpoint']) for i,l in enumerate(modes['lift_off_attempts'])]
 tasks=[];index=[]
 for si,(_,s) in enumerate(starts):
  for d in destinations:tasks.append(('direct',(s,d)));index.append((si,'direct',d['name']))
  for gi,g in goals:tasks.append(('detour',(s,g['q'])));index.append((si,'detour',gi))
 with concurrent.futures.ProcessPoolExecutor(max_workers=4,initializer=initialize) as pool:results=list(pool.map(run,tasks))
 rows=[]
 for si,(pointer,s) in enumerate(starts):
  rows.append({'source_pointer':'/candidates/'+str(pointer),'is_REST_GROUND':s['q']==library['REST_GROUND']['q'],'is_REST_GROUND_MAX_SEPARATION':s['q']==rest['selected']['q'],'q':s['q'],'joint_margin_rad':s['joint_margin_rad'],'min_separation_m':s['min_separation']['distance_m'],'support_margin_m':s['support_margin_m'],
   'direct_contact_additions':[r for (k,kind,_),r in zip(index,results) if k==si and kind=='direct'],
   'body_supported_detours':[{'target_family':t,**r} for (k,kind,t),r in zip(index,results) if k==si and kind=='detour']})
 model=Model();free={}
 for leg in (2,0):
  rng=np.random.default_rng(1);hits=sum(leg_clear(model,leg,rng.uniform(model.limits[leg*3:leg*3+3,0],model.limits[leg*3:leg*3+3,1]),model.body_height) for _ in range(300));free[LEGS[leg]]={'uniform_joint_box_samples':300,'seed':1,'isolated_leg_clear_fraction_at_base_ground_height':hits/300}
 for r,(_,s) in zip(rows,starts):r['start_isolated_leg_clear']={l:leg_clear(model,i,np.array(s['q'])[i*3:i*3+3],model.body_height) for i,l in enumerate(LEGS)}
 connected=[r['source_pointer'] for r in rows if any(x['valid'] for x in r['direct_contact_additions']) or any(x['all_samples_valid'] for x in r['body_supported_detours'])]
 save('rest_connectivity',{'provenance':{'canonical_sources':Model().sources,'inputs':{n:sha(OUT/n) for n in ('rest_search.json','contact_modes.json','pose_library.json')},'generator_sha256':sha(__file__),'model_sha256':sha(__import__('pathlib').Path(__file__).with_name('model.py')),'route_extension_sha256':sha(__import__('pathlib').Path(__file__).with_name('route_extension.py'))},
  'protocol':'direct joint interpolation (51 samples) to every valid contact-mode endpoint plus body-supported single-leg RRT-style detours (rear then front, seeds 17/31/73, 2000 trials) to both four-foot endpoints; identical to contact_modes.py and route_extension.py but applied to every valid body-only start','starts':rows,'isolated_leg_free_space':free,'free_space_scope':'leg vs base_link/self and ground penetration only; other legs are ignored by the search and by this estimate','tree_growth':'attempt records list RRT node counts; saturation at tens of nodes means the reachable clear component from the start is small under the 0.15 rad step','starts_with_any_valid_connection':connected,'global_absence_claim':False})
 print('REST_CONNECTIVITY starts',len(rows),'connected',connected,flush=True)
if __name__=='__main__':main()

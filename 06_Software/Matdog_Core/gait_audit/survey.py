"""Deterministic bounded gait grid. Screening and full mesh validation are distinct."""
import argparse,csv,json,sys,hashlib,itertools,os
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from core import Core,params,ROOT,FIELDS,STATUS,HERE,MOTION
sys.path.insert(0,str(ROOT/'06_Software/Matdog_Core/pose_audit'))
from model import Model,LEGS
OUT=ROOT/'09_Logs/Validation_Reports/G4_Gait_Envelope'
_model=None;_core=None

def initialize():
 global _model,_core
 _model=Model();_core=Core()
def serial(value):
 if isinstance(value,np.ndarray):return value.tolist()
 if isinstance(value,np.generic):return value.item()
 raise TypeError(type(value).__name__)
def save(name,data): (OUT/name).write_text(json.dumps(data,indent=2,default=serial,allow_nan=False)+'\n')
def phases(p,n=40,start=1,end=2):
 offsets=[0,.5,.75,.25] if p[0]==0 else [0,.5,0,.5]
 events=[s for k in range(int(start),int(end)+2) for o in offsets for s in (k-o,k+p[3]-o) if start<=s<=end]
 return sorted(set(round(float(s),12) for s in [*np.linspace(start,end,n+1),*events]))
def classify(errors,kind):
 result=[]
 for e in errors:
  if e=='SUPPORT_INVALID' and kind==1:continue
  result.append('GROUND_COLLISION' if e.startswith('GROUND_PENETRATION') else 'CONTACT_INVALID' if e.startswith(('UNDECLARED_GROUND_CONTACT','MISSING_MESH_SUPPORT')) else 'JOINT_LIMIT_BLOCKED' if e=='JOINT_LIMIT' else e)
 return sorted(set(result))
def check_frame(f,p,full):
 active=[l for i,l in enumerate(LEGS) if f['leg_phase'][i,1] or f['leg_phase'][i,2]]
 a=_model.evaluate(f['q'],f['body'],'FOOT_SUPPORT',active,full=full)
 # Quasi-static WALK requires strictly positive modeled support margin.
 if p[0]==0 and (a['support_margin_m'] is None or a['support_margin_m']<=0) and 'SUPPORT_INVALID' not in a['errors']:a['errors'].append('SUPPORT_INVALID')
 return a,classify(a['errors'],p[0])
def evaluate(task):
 idx,p,full,n=task;p=np.array(p);old=None;errors=set();first=None
 metrics=dict(min_joint_margin_rad=None,max_condition=0.,max_contact_residual_m=0.,max_stance_drift_m=0.,min_self_separation_m=None,min_nonfoot_ground_m=None,min_support_margin_m=None,peak_qdot_rad_s=np.zeros(12),peak_qddot_rad_s2=np.zeros(12),branch_changes=np.zeros(4,dtype=int),weakest_phase=None,weakest_support_count=None)
 frames=[];geometry_checked=0
 for s in phases(p,n):
  f=_core.frame(p,s,previous=old)
  if 'q' not in f:errors.add(f['status']);first=first or dict(phase=s,errors=[f['status']],leg=f['failed_leg']);break
  metrics['min_joint_margin_rad']=f['joint_margin'] if metrics['min_joint_margin_rad'] is None else min(metrics['min_joint_margin_rad'],f['joint_margin'])
  metrics['max_condition']=max(metrics['max_condition'],max(f['condition']));metrics['max_contact_residual_m']=max(metrics['max_contact_residual_m'],f['residual'])
  metrics['peak_qdot_rad_s']=np.maximum(metrics['peak_qdot_rad_s'],abs(f['qdot']));metrics['peak_qddot_rad_s2']=np.maximum(metrics['peak_qddot_rad_s2'],abs(f['qddot']))
  if old is not None:
   metrics['branch_changes']+=(f['branches'].reshape(4,2)!=old['branches'].reshape(4,2)).any(axis=1)
  old=f
  a,why=check_frame(f,p,full);geometry_checked+=1
  errors.update(why)
  if why:
   first=first or dict(phase=s,errors=a['errors'])
   # Failed grid cases are evidence of a failure, not complete-cycle metrics.
   if not full:break
  nonfoot=min(z for name,z in a['ground_min_m'].items() if not name.endswith('_foot_link'))
  metrics['min_nonfoot_ground_m']=nonfoot if metrics['min_nonfoot_ground_m'] is None else min(nonfoot,metrics['min_nonfoot_ground_m'])
  if a['support_margin_m'] is not None and (metrics['min_support_margin_m'] is None or a['support_margin_m']<metrics['min_support_margin_m']):
   metrics['min_support_margin_m']=a['support_margin_m'];metrics['weakest_phase']=s%1;metrics['weakest_support_count']=len(a['active_feet'])
  if full:
   d=a['min_separation']['distance_m'];metrics['min_self_separation_m']=d if metrics['min_self_separation_m'] is None else min(d,metrics['min_self_separation_m'])
   frames.append(dict(phase=s,q=f['q'],qdot=f['qdot'],qddot=f['qddot'],body=f['body'],contacts_world_m=a['contacts'],targets_world_m=f['feet'],leg_phase=f['leg_phase'],branches=f['branches'],condition=f['condition'],joint_margin_rad=f['joint_margin'],residual_m=f['residual'],errors=a['errors'],classification=why,support_margin_m=a['support_margin_m'],active_feet=a['active_feet'],support_polygon=a['support_polygon'],com_world_m=a['com_world_m'],min_separation=a['min_separation'],ground_min_m=a['ground_min_m']))
 if full:
  for a,b in zip(frames,frames[1:]):
   for i in range(4):
    if a['leg_phase'][i,1] and b['leg_phase'][i,1] and b['leg_phase'][i,0]>=a['leg_phase'][i,0]:metrics['max_stance_drift_m']=max(metrics['max_stance_drift_m'],float(np.linalg.norm(np.array(a['contacts_world_m'][i])-b['contacts_world_m'][i])))
 return dict(id=idx,parameters=dict(zip(FIELDS,p)),period_s=1.,resolution_intervals=n,classification=sorted(errors) or ['KINEMATICALLY_VALID'],validation='FULL_SAMPLED_MESH' if full else 'SCREEN_ONLY_NO_SELF_COLLISION_CHECK',complete=not errors,geometry_samples=geometry_checked,first_failure=first,metrics=metrics,frames=frames if full else None,dynamic_stability='NOT_YET_PROVEN' if p[0]==1 else 'QUASI_STATIC_MODEL_ONLY')

def grid():
 cases=[]
 for k,h,x,lift,duty in itertools.product((0,1),(.08,.1,.12,.14,.15),(-.1,-.08,-.06,-.04,-.02,-.01,0,.01,.02,.04,.06,.08,.1),(.005,.01,.02,.03),(0,1,2)):
  d=(.75,.8,.9)[duty] if k==0 else (.5,.6,.8)[duty]
  cases.append(params(kind=k,height=h,x=x,lift=lift,duty=d,sway=.004 if k==0 and d>.75 else 0))
 for k,h,y,yaw in itertools.product((0,1),(.08,.1,.12,.14,.15),(-.02,-.005,-.001,0,.001,.005,.02),(-.15,-.05,-.01,0,.01,.05,.15)):
  if y==0 and yaw==0:continue
  cases.append(params(kind=k,height=h,x=.01,y=y,yaw=yaw,duty=.8 if k==0 else .6,sway=.004 if k==0 else 0))
 # Explicit challenging geometric cases identify IK, limit and conditioning classes.
 cases.extend([params(height=.3),params(lift=.15),params(x=.3),params(condition=1)])
 return [list(p) for p in sorted(set(tuple(p) for p in cases))]
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--stage',choices=['screen','full'],required=True);ap.add_argument('--workers',type=int,default=4);args=ap.parse_args()
 if args.stage=='screen':
  cases=grid();results=[]
  with ProcessPoolExecutor(max_workers=args.workers,initializer=initialize) as pool:
   for j,r in enumerate(pool.map(evaluate,[(i,p,False,40) for i,p in enumerate(cases)],chunksize=4)):
    results.append(r)
    if j%100==0:print('screen',j,'/',len(cases),flush=True)
  save('screen.json',dict(scope='Bounded screening grid; full mesh frontier checks separate. Early rejection metrics cover prefix only.',cases=results))
  with (OUT/'envelope_grid.csv').open('w') as f:
   writer=csv.writer(f);writer.writerow(['id',*FIELDS,'classification','samples','first_failure_phase'])
   for r in results:writer.writerow([r['id'],*[r['parameters'][k] for k in FIELDS],';'.join(r['classification']),r['geometry_samples'],None if r['first_failure'] is None else r['first_failure']['phase']])
  print('SCREEN',len(results),Counter(e for r in results for e in r['classification']))
 else:
  results=json.loads((OUT/'screen.json').read_text())['cases'];selected={}
  # Fully check the largest screen-passing fore/aft cases at each height.
  for kind,height,direction in itertools.product((0,1),(.08,.1,.12,.14,.15),(-1,1)):
   group=[r for r in results if r['complete'] and r['parameters']['type']==kind and r['parameters']['height_m']==height and r['parameters']['advance_y_m']==r['parameters']['yaw_rad']==0 and r['parameters']['advance_x_m']*direction>0]
   group.sort(key=lambda r:(-direction*r['parameters']['advance_x_m'],r['parameters']['lift_m'],abs(r['parameters']['duty']-(.8 if kind==0 else .6))))
   for r in group[:1]:selected[r['id']]=r
  # Representative small gait at 100 mm, both types; and contrasting failed cases.
  for kind in (0,1):
   for r in results:
    p=r['parameters']
    if p['type']==kind and p['height_m']==.1 and p['advance_x_m']==.01 and p['advance_y_m']==p['yaw_rad']==0 and p['lift_m']==.01 and p['duty']==(.8 if kind==0 else .6):selected[r['id']]=r
   for r in results:
    p=r['parameters']
    if p['type']==kind and p['height_m']==.15 and p['advance_x_m']==.02 and p['advance_y_m']==p['yaw_rad']==0 and p['lift_m']==.01 and p['duty']==(.8 if kind==0 else .6):selected[r['id']]=r
  tasks=[]
  for i,r in sorted(selected.items()):
   p=[r['parameters'][k] for k in FIELDS];representative=p[1]==.1 and p[4]==.01 and p[2]==.01
   tasks.append((i,p,True,200 if representative else 80))
  full=[]
  with ProcessPoolExecutor(max_workers=args.workers,initializer=initialize) as pool:
   for r in pool.map(evaluate,tasks):
    full.append(r);save('full_cases.json',dict(cases=full,planned_cases=len(tasks),complete=len(full)==len(tasks)))
    print('full',r['id'],r['classification'],'samples',r['geometry_samples'],flush=True)
  save('definitions.json',dict(leg_order=list(LEGS),walk_offsets=[0,.5,.75,.25],trot_offsets=[0,.5,0,.5],walk_order=['rh','rf','lh','lf'],periods_evaluated_s=[.5,1,2,4],heights_m=[.08,.1,.12,.14,.15],stance='WORLD_LOCKED_G2_REFERENCE',swing='quintic XY; 64 h u^3(1-u)^3 Z',support='CAD/URDF COM, intentional finite mesh patches',trot_dynamic_stability='NOT_YET_PROVEN',canonical_sources=_sources()))
def _sources():
 m=Model();return {**m.sources,**{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(HERE.glob('*.py'))+sorted(HERE.glob('*.cpp'))+sorted(MOTION.glob('Gait.*'))+sorted(MOTION.glob('Locomotion.*'))}}
if __name__=='__main__':main()

"""Deterministic finer axial grids, pure lateral/yaw cases, and rejected stress cases."""
import itertools,json
from concurrent.futures import ProcessPoolExecutor
from core import params,FIELDS
from survey import OUT,save,evaluate,initialize

def main():
 cases=[]
 for kind,h,x in itertools.product((0,1),(.08,.1,.12),(-.07,-.065,-.055,-.05,-.045,-.035,-.03,-.025,-.0175,-.015,-.0125,.0125,.015,.0175,.025,.03,.035,.045,.05,.055,.065,.07)):
  cases.append(params(kind=kind,height=h,x=x,lift=.005,duty=.8 if kind==0 else .6,sway=.004 if kind==0 else 0).tolist())
 for kind,h,axis,value in itertools.product((0,1),(.08,.1,.12,.14,.15),('y','yaw'),(-.02,-.005,-.001,.001,.005,.02)):
  kw={'y':value} if axis=='y' else {'yaw':value}
  cases.append(params(kind=kind,height=h,x=0,duty=.8 if kind==0 else .6,sway=.004 if kind==0 else 0,**kw).tolist())
 for kind,h,lift in itertools.product((0,1),(.06,.16,.18,.20,.21,.22,.24),(.005,.04,.06,.08)):
  cases.append(params(kind=kind,height=h,lift=lift,duty=.8 if kind==0 else .6,sway=.004 if kind==0 else 0).tolist())
 with ProcessPoolExecutor(max_workers=2,initializer=initialize) as pool:screen=list(pool.map(evaluate,[(3000+i,p,False,80) for i,p in enumerate(cases)],chunksize=4))
 save('refinement_screen.json',dict(cases=screen,scope='Finer bounded screen, pure lateral/yaw and stress probes; same unchanged collision policy.'))
 tasks={}
 for kind,h,direction in itertools.product((0,1),(.08,.1,.12),(-1,1)):
  group=[r for r in screen if r['complete'] and r['parameters']['type']==kind and r['parameters']['height_m']==h and r['parameters']['advance_x_m']*direction>0]
  if group:
   r=max(group,key=lambda r:direction*r['parameters']['advance_x_m']);tasks[r['id']]=(r['id'],list(r['parameters'].values()),True,80)
 # Validate the useful 80 mm forward fallback even if the first frontier fails.
 tasks[4000]=(4000,params(height=.08,x=.01,lift=.01,sway=.004).tolist(),True,100)
 full=[]
 with ProcessPoolExecutor(max_workers=2,initializer=initialize) as pool:
  for r in pool.map(evaluate,list(tasks.values())):
   full.append(r);save('refinement_full.json',dict(cases=full,planned_cases=len(tasks),complete=len(full)==len(tasks)));print('refined',r['id'],r['classification'],flush=True)
if __name__=='__main__':main()

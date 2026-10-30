#!/usr/bin/env python3
"""Post-candidate study ONLY. No firmware target or collision-box changes.
Search minimal paired rear UPPER adjustment, then UPPER+LOWER with a 12-tick
mechanical reserve. Final full-mesh 3D support check is distinct from sampling.
"""
from geometry import *
from flat_bound import boxes,limits
from scipy.optimize import differential_evolution
out=Path(sys.argv[1]);tick=2*math.pi/4096
lo=max(limits[8]['limits_rad'][0],limits[11]['limits_rad'][0])+12*tick
hi=min(boxes['kEntry'][1,8],boxes['kEntry'][1,11])-12*tick
urlo=max(boxes['kEntry'][0,7],boxes['kEntry'][0,10])+12*tick
urhi=min(boxes['kEntry'][1,7],boxes['kEntry'][1,10])-12*tick
# A level belly with body z=0 is separately excluded numerically by flat_bound.
def evaluate(ur,lr):
 q=entry_q(.17,ur=ur,lr=lr);r=rest(q)
 clearance=min(v for k,v in r['low'].items() if k!='base_link' and not k.endswith('foot_link'))
 return q,r,clearance
# Reject lower/upper link ground contact and loss of the intended front belly support.
def score(v):
 _,r,c=evaluate(*v)
 return abs(r['pitch'])+100*max(0,r['belly_gap']-1e-7)+100*max(0,.001-c)
nominal=evaluate(math.pi/2,-89*math.pi/180)
one=min((evaluate(u,-89*math.pi/180) for u in np.linspace(urlo,urhi,401)),key=lambda v:abs(v[1]['pitch'])+100*max(0,v[1]['belly_gap']-1e-7)+100*max(0,.001-v[2]))
r=differential_evolution(score,[(urlo,urhi),(lo,hi)],seed=29,popsize=8,maxiter=60,tol=1e-8,polish=True)
two=evaluate(*r.x)
result={'implemented':False,'criterion':'belly contact, non-foot/non-base floor clearance >=1 mm; finite optimization, not a global proof','rear_lower_12_tick_reserved_min_rad':lo,'variants':{}}
for name,(q,_,_) in [('current',nominal),('rear_upper_only',one),('rear_upper_lower',two)]:
 rr=rest3d(q);V,c=geometry(q);rr['q_rad']=q.tolist();rr['rear_upper_change_rad']=float(q[7]-math.pi/2);rr['rear_lower_change_rad']=float(q[8]+89*math.pi/180);rr['solver_converged']=bool(r.success) if name=='rear_upper_lower' else None
 result['variants'][name]=rr
(out/'rear_settle_study.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

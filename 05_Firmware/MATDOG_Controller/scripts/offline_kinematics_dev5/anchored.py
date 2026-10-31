"""Finite max tripod margin before first rear lift, four contacts held at entry.
Allows body x/y/z, roll/pitch; existing production tilted IK, no new solver.
"""
from common import *
from scipy.optimize import minimize
from scipy.spatial import ConvexHull
out=Path(sys.argv[1]);lib=bridge(out);head=knot(lib,0);rr=rest(head[:12]);anchors=feet(lib,head,np.array([rr['pitch'],0,rr['z']]))
# Use the full 13.9mm support strip, not point contacts (optimistic support).
cp=np.concatenate([anchors[i,:2]+[0,d] for i in [0,1,3] for d in [-.00695,.00695]]).reshape(-1,2);hull=ConvexHull(cp)
lo=np.maximum(LOW,boxes['kDeploy'][0])+4*TICK;hi=np.minimum(HIGH,boxes['kDeploy'][1])-4*TICK
cache={}
def calc(p):
 key=tuple(p)
 if key in cache:return cache[key]
 q=np.zeros(12)
 for l in range(4):
  a=np.zeros(3)
  if not lib.ik6(l,np.ascontiguousarray(anchors[l]),np.ascontiguousarray(p),np.ascontiguousarray(head[3*l:3*l+3]),a):cache[key]=(None,-1e3);return cache[key]
  q[3*l:3*l+3]=a
 _,com=geometry(q);R=rot('0 1 0',p[1])@rot('1 0 0',p[0]);cw=R@com+p[2:5];margin=float(np.min(-(hull.equations[:,:2]@cw[:2]+hull.equations[:,2])))*1000
 cache[key]=(q,margin);return q,margin
bounds=[(-.08,.08),(-.05,.25),(-.05,.05),(-.02,.02),(.005,.06)]
def constraints(p):
 q,_=calc(p)
 return np.r_[q-lo,hi-q] if q is not None else np.full(24,-10.)
sol=[]
for dx in [-.005,0,.005]:
 p=np.array([0,rr['pitch'],dx,0,rr['z']]);r=minimize(lambda p:-calc(p)[1],p,method='SLSQP',bounds=bounds,constraints=[{'type':'ineq','fun':constraints}],options={'maxiter':200,'ftol':1e-9});q,m=calc(r.x);sol.append({'converged':bool(r.success),'message':r.message,'pose_roll_pitch_x_y_z':r.x,'q':q,'support_margin_mm':m,'min_constraint_rad':float(min(constraints(r.x)))})
result={'classification':'FINITE_LOCAL_SEARCH_NOT_GLOBAL_PROOF','anchors':anchors,'support_strip_m':.0139,'joint_reserve_ticks':4,'solutions':sol,'meaning':'If the optimum remains negative, these searches found no stable first hind lift with all entry contacts anchored. Unanchored endpoint feasibility is not a connected no-slip route.'};dump(out/'anchored-search.json',result);print(json.dumps(result,indent=2,default=lambda a:a.tolist()))

"""Finite searches, not global feasibility certificates; no production mutation."""
from common import *
from scipy.spatial import ConvexHull
from scipy.optimize import differential_evolution
out=Path(sys.argv[1]);lib=bridge(out)
# Support plane touching the three stationary finite CAD feet, including cases
# with COM outside their support polygon. Unlike rest3d it must not silently
# reselect the supposedly airborne foot to restore equilibrium.
def tripod(q,free=2):
 V,c=geometry(q);names=[n for n in V if n.endswith('foot_link')];support=[n for i,n in enumerate(names) if i!=free];pts=np.concatenate([V[n] for n in support]);h=ConvexHull(pts);candidates=[]
 for e in h.equations:
  n,d=e[:3],e[3]
  if n[2]>-.85:continue
  up=-n;z=-d
  if any(abs(float((V[k]@up+z).min()))>2e-5 for k in support):continue
  candidates.append((np.linalg.norm(up-np.array([-.1,0,.995])),up,z))
 if not candidates:return None
 _,up,z=min(candidates,key=lambda a:a[0]);pitch=math.asin(-up[0]);roll=math.atan2(up[1],up[2]);R=rot('0 1 0',pitch)@rot('1 0 0',roll);W={n:v@R.T+[0,0,z] for n,v in V.items()};low={n:float(v[:,2].min()) for n,v in W.items()};cp=np.concatenate([W[k][W[k][:,2]<2e-5] for k in support]);cw=R@c+[0,0,z];h=ConvexHull(cp[:,:2]);margin=float(np.min(-(h.equations[:,:2]@cw[:2]+h.equations[:,2])))
 return {'margin_mm':margin*1000,'swing_gap_mm':low[names[free]]*1000,'body_gap_mm':low['base_link']*1000,'nonfoot_gap_mm':1000*min(v for n,v in low.items() if n!='base_link' and not n.endswith('foot_link')),'pitch_deg':pitch*180/math.pi,'roll_deg':roll*180/math.pi,'com':cw,'q':q,'low':low}
q0=knot(lib,0);q1=knot(lib,107);baseline={}
for k in [0,30,42,65,107]:baseline[k]=tripod(knot(lib,k)[:12])
print('BASELINE TRIPOD',[(k,v['margin_mm']) for k,v in baseline.items()],flush=True)
# Finite search allowing all 12 axes inside unchanged DEPLOY, not just timing.
# This is deliberately more freedom than the old front-advance variant.
box=boxes['kDeploy'];reserve=4*TICK;lo=np.maximum(LOW,box[0])+reserve;hi=np.minimum(HIGH,box[1])-reserve
best=None
calls=0
def score(q):
 global best,calls
 calls+=1;r=tripod(q)
 if r is None:return 1000
 # Desired: >=3mm swing clearance, positive belly/link clearance, >=5mm support.
 penalty=max(0,3-r['swing_gap_mm'])*3+max(0,-r['body_gap_mm'])*3+max(0,1-r['nonfoot_gap_mm'])*3
 value=-r['margin_mm']+penalty
 if best is None or value<best[0]:best=(value,r)
 return value
r=differential_evolution(score,list(zip(lo,hi)),seed=30,popsize=6,maxiter=65,tol=.002,polish=False)
result={'classification':'FINITE_SEARCH_NOT_GLOBAL_PROOF','reserve_ticks':4,'tripod_baseline':baseline,'search':{'converged':bool(r.success),'calls':calls,'best_score':best[0],'best':best[1]}}
dump(out/'tripod-search.json',result);print('TRIPOD',json.dumps(result['search'],default=lambda a:a.tolist() if isinstance(a,np.ndarray) else float(a)),flush=True)
# PRONE: rerun the unintegrated rear proposals with dev.5's FRONT LOWER floor.
variants={};floor=-1.5246163880149817
for name,u,l,h in [('dev5',math.pi/2,-89*math.pi/180,.17),('rear_upper_m100mrad',math.pi/2-.10,-89*math.pi/180,.17),('old_upper_only',1.375651796828815,-89*math.pi/180,.17),('old_upper_lower',1.4012319312094104,-1.5723302303484568,.17)]:
 q=entry_q(h,ur=u,lr=l,lf=floor);rr=rest3d(q);rr['q']=q;rr['margin_ticks']=margin(q,boxes['kEntry']);variants[name]=rr
lo=max(LOW[8],LOW[11])+12*TICK;hi=min(boxes['kEntry'][1,8],boxes['kEntry'][1,11])-12*TICK
bounds=[(.12,.19707963267948966-12*TICK),(1.3562634015954635+12*TICK,1.7853292519943296-12*TICK),(lo,hi)]
def pscore(p):
 h,u,l=p;q=entry_q(.17,ur=u,lr=l,lf=floor);q[6]=-h;q[9]=h;r=rest(q);clear=min(v for n,v in r['low'].items() if n!='base_link' and not n.endswith('foot_link'))
 return abs(r['pitch'])+100*max(0,r['belly_gap']-1e-7)+100*max(0,.0015-clear)
r=differential_evolution(pscore,bounds,seed=30,popsize=8,maxiter=70,tol=1e-7,polish=True);h,u,l=r.x;q=entry_q(.17,ur=u,lr=l,lf=floor);q[6]=-h;q[9]=h;v=rest3d(q);v.update(q=q,margin_ticks=margin(q,boxes['kEntry']),converged=bool(r.success));variants['rear_three_axis_reserved12']=v
for k,v in variants.items():print('PRONE',k,'gap',v['belly_max_gap']*1000,'support',v['support_margin']*1000,'force',v['belly_force_N_minmax'],'margin',v['margin_ticks'],flush=True)
dump(out/'prone-variants.json',variants)

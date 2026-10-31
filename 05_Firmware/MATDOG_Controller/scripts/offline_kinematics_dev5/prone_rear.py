"""REV01-FIX rigid PRONE rear support: how far rear HIP/UPPER/LOWER can flatten the belly.
Finite search and rigid CAD support only; gen-1 limits unchanged, no friction, compliance or dynamics."""
from common import *
from scipy.optimize import differential_evolution
out=Path(sys.argv[1]);lib=bridge(out);floor=-1.5246163880149817;reserve=12*TICK;base=entry_q(.17,lf=floor);rows=[]
def q_of(h,u,l):
 q=base.copy();q[6]=-h;q[9]=h;q[7]=q[10]=u;q[8]=q[11]=l;return q
def ev(q):
 r=rest3d(q);f=feet(lib,q,np.array([r['pitch'],0,r['z']]));non=min(v for n,v in r['low'].items() if n!='base_link' and not n.endswith('foot_link'))
 return r,non,(f[0,0]-f[2,0])*1000
# Gen-1 window of the two rear legs, reserve 12 ticks; URDF LOWER floor -89 deg route floor kept as an option.
lo8=max(LOW[8],LOW[11])+reserve;hi7=min(HIGH[7],HIGH[10])-reserve
cases=[('kEntry',max(-boxes['kEntry'][0,6],boxes['kEntry'][1,9])-reserve,boxes['kEntry'][0,7]+reserve,boxes['kEntry'][1,7]-reserve,max(lo8,max(boxes['kEntry'][0,8],boxes['kEntry'][0,11])+reserve)),
       ('hip_0.30',.30,1.20,1.80,lo8),('hip_0.45',.45,1.10,1.80,lo8),('hip_0.60',.60,1.00,1.80,lo8)]
for name,hh,ul,uh,ll in cases:
 def score(p):
  h,u,l=p;q=q_of(h,u,l)
  try:r,non,sep=ev(q)
  except (AssertionError,Exception):return 100
  return r['pitch']*1000+100*max(0,.0015-non)*1000+100*max(0,r['belly_gap']-1e-6)*1000
 s=differential_evolution(score,[(.05,hh),(ul,uh),(ll,-1.30)],seed=30,popsize=8,maxiter=40,tol=1e-5,polish=False)
 h,u,l=s.x;q=q_of(h,u,l);r,non,sep=ev(q)
 row=dict(case=name,hip=h,upper=u,lower=l,pitch=r['pitch'],gap_mm=r['belly_max_gap']*1000,nonfoot_mm=non*1000,sep_mm=sep,support_mm=r['support_margin']*1000,belly_N=r['belly_force_N_minmax'],contacts=r['contacts'],low={k:v*1000 for k,v in r['low'].items()})
 rows.append(row);print(name,'h %.3f u %.3f l %.3f pitch %.4f gap %.2f nonfoot %.2f sep %.1f support %.1f belly %.1f'%(h,u,l,r['pitch'],row['gap_mm'],row['nonfoot_mm'],sep,row['support_mm'],r['belly_force_N_minmax'][0]),[c.replace('_link','') for c in r['contacts']],flush=True)
dump(out/'prone-rear.json',rows)

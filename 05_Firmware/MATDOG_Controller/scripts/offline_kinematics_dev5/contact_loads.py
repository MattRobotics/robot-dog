"""REV01-FIX rigid static contact shares along the frozen DEPLOY/LIFT route.
Rigid CAD support, LP force bounds; friction, compliance and dynamics not modelled."""
from common import *
from scipy.optimize import linprog
from scipy.spatial import ConvexHull
from geometry import inert
M=sum(m for _,m,_ in inert)
def shares(q):
 r=rest3d(q);V,c=geometry(q);R=rot('0 1 0',r['pitch'])@rot('1 0 0',r['roll']);W={k:v@R.T+[0,0,r['z']] for k,v in V.items()};cw=R@c+[0,0,r['z']]
 names=[k for k,v in W.items() if v[:,2].min()<2e-5];cp=[];own=[]
 for i,k in enumerate(names):
  p=W[k][W[k][:,2]<2e-5][:,:2];cp.append(p);own+= [i]*len(p)
 cp=np.concatenate(cp);own=np.array(own);A=np.c_[np.ones(len(cp)),cp].T;b=M*9.80665*np.r_[1,cw[:2]];out={}
 for i,k in enumerate(names):
  lo=linprog((own==i).astype(float),A_eq=A,b_eq=b,bounds=(0,None),method='highs');hi=linprog(-(own==i).astype(float),A_eq=A,b_eq=b,bounds=(0,None),method='highs')
  out[k.replace('_link','')]=[round(lo.fun,2),round(-hi.fun,2)]
 return r,out,cw
lib=bridge(sys.argv[1]);rows=[]
for k in [0,30,42,50,58,65,72,80,100,107,114,125,134,146,160]:
 q=knot(lib,k)
 try:r,o,cw=shares(q[:12])
 except AssertionError as e:rows.append(dict(knot=k,status='NO_STABLE_FACE'));print(k,'NO_STABLE_FACE');continue
 f=feet(lib,q,np.array([q[12],q[13],q[14]]));sep=(f[2,0]-f[0,0])*1000
 rows.append(dict(knot=k,route_pitch=q[12],rigid_pitch=r['pitch'],belly_gap_mm=r['belly_gap']*1000,support_margin_mm=r['support_margin']*1000,sep_mm=sep,com_x_mm=cw[0]*1000,forces_N=o))
 print(k,'pitch %.3f/%.3f gap %.1f sm %.1f sep %.1f comx %.1f'%(q[12],r['pitch'],r['belly_gap']*1000,r['support_margin']*1000,sep,cw[0]*1000),o,flush=True)
dump(Path(sys.argv[1])/'contact-loads.json',rows)

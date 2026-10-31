"""Quasi-static friction model on the joint-linear paths between actual DEPLOY vias. Same assumptions as deploy_friction.py."""
from common import *
from geometry import inert
from scipy.optimize import minimize
out=Path(sys.argv[1]);lib=bridge(out);M=sum(m for _,m,_ in inert);R=.0149
def run(V,lr=0.,sub=10):
 B=np.zeros(3);prev=None;slip=np.zeros(4);peak=0;comc={}
 for a,b in zip(V,V[1:]):
  c0=geometry(a[:12])[1][0];c1=geometry(b[:12])[1][0]
  for t in np.linspace(0,1,sub,endpoint=False):
   q=a*(1-t)+b*t;f=feet(lib,q,np.array([q[12],0,0]));rr=f[:,:2];al=np.array([q[1]+q[2],q[4]+q[5],q[7]+q[8],q[10]+q[11]]);mat=rr-np.c_[R*al,np.zeros(4)]
   if prev is not None:
    d=mat-prev;c=c0*(1-t)+c1*t;xf=(rr[0,0]+rr[1,0])/2;xr=(rr[2,0]+rr[3,0])/2;ff=(c-xr)/(xf-xr);w=np.array([ff/2,ff/2,(1-ff)/2,(1-ff)/2])*M*9.80665*(1+np.array([-1,1,1,-1])*lr)
    cs,sn=math.cos(B[2]),math.sin(B[2]);Rb=np.array([[cs,-sn],[sn,cs]]);dw=(Rb@d.T).T;arm=(Rb@np.c_[-rr[:,1],rr[:,0]].T).T
    cost=lambda u:float(np.sum(w*np.sqrt(np.sum((dw+u[:2]+u[2]*arm)**2,axis=1)+1e-12)))
    x0=min([np.zeros(3),*[np.r_[-dw[i],0] for i in range(4)]],key=cost);r=minimize(cost,x0,method='Nelder-Mead',options=dict(xatol=1e-8,fatol=1e-11,maxiter=500,initial_simplex=x0+np.vstack([np.zeros(3),np.diag([3e-4,3e-4,3e-3])])));u=r.x
    slip+=np.sqrt(np.sum((dw+u[:2]+u[2]*arm)**2,axis=1));B=B+u;peak=max(peak,abs(B[2]))
   prev=mat
 return dict(body_x_mm=B[0]*1000,body_y_mm=B[1]*1000,yaw_deg=math.degrees(B[2]),yaw_peak_deg=math.degrees(peak),slip_mm=slip*1000,total_mm=float(slip.sum()*1000))
base=np.array([knot(lib,k) for k in (0,30,42,43,50,58,65,72,80,100,107)]);rows={}
def show(n,V,**k):
 r=run(V,**k);rows[n]=r;print('%-22s body x %+6.1f y %+5.1f yaw %+5.2f peak %4.2f | slip LF %5.1f RF %5.1f RH %5.1f LH %5.1f total %6.1f'%(n,r['body_x_mm'],r['body_y_mm'],r['yaw_deg'],r['yaw_peak_deg'],*r['slip_mm'],r['total_mm']),flush=True)
show('pair',base)
for f in sys.argv[2:]:
 d=json.loads(Path(f).read_text());V=np.array(d['vias']);n=Path(f).stem;print(n,'margin',round(d['min_margin_ticks'],1),'spread mm',round(d['contact_spread_mm'],2));show(n,V);show(n+' lr.15',V,lr=.15)
dump(out/'deploy-friction-path.json',rows)

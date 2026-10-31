"""Quasi-static DEPLOY foot/body sliding, REV01-FIX CAD and inertials: at each small step the planar body
motion minimises the Coulomb friction power (Peshkin-Sanderson). Uniform mu unless stated; no compliance,
belly contact or dynamics. Compares the frozen pair route with rear legs advanced alternately."""
from common import *
from geometry import inert
from scipy.optimize import minimize
out=Path(sys.argv[1]);lib=bridge(out);M=sum(m for _,m,_ in inert);R=.0149
K=list(range(0,108));Q=[knot(lib,k) for k in K]
def rel(q):
 """contact material x,y in body frame per leg (level projection of the route pose), and rigid sagittal loads."""
 p=np.array([q[12],0,0]);f=feet(lib,q,p);alpha=np.array([q[1]+q[2],q[4]+q[5],q[7]+q[8],q[10]+q[11]])
 return f[:,:2],alpha
T=[rel(q) for q in Q]
COM=[geometry(q[:12])[1][0] for q in Q]
def interp(s,i):
 s=np.clip(s,0,len(K)-1);a=int(math.floor(s));b=min(a+1,len(K)-1);w=s-a
 return T[a][0][i]*(1-w)+T[b][0][i]*w,T[a][1][i]*(1-w)+T[b][1][i]*w
def com_x(s):
 return COM[int(np.clip(round(s),0,len(K)-1))]
def loads(sa,lr=0.):
 # sagittal statics on the mean rear/front contact x, equal sides, lr shifts share from left to right
 pts=[interp(s,i)[0] for i,s in enumerate(sa)];xf=(pts[0][0]+pts[1][0])/2;xr=(pts[2][0]+pts[3][0])/2;c=com_x(np.mean(sa));ff=(c-xr)/(xf-xr)
 W=M*9.80665;n=np.array([ff/2,ff/2,(1-ff)/2,(1-ff)/2])*W
 side=np.array([-1,1,1,-1])*lr;return n*(1+side)
def sim(schedule,lr=0.,mu=(1,1,1,1)):
 """schedule(t) -> 4 route coordinates per leg, t in [0,1]. Returns totals."""
 B=np.zeros(3);P=[];prev=None;slip=np.zeros(4);xy=None;N=[];
 ts=np.linspace(0,1,241);foot=None;trace=[]
 for t in ts:
  sa=schedule(t);rr=[];mat=[]
  for i,s in enumerate(sa):
   p,a=interp(s,i);rr.append(p);mat.append(p-np.array([R*a,0]))
  rr=np.array(rr);mat=np.array(mat)
  if prev is not None:
   d=mat-prev;w=loads(sa,lr)*np.array(mu)
   def cost(u,eps=1e-9):
    c,s_=math.cos(B[2]),math.sin(B[2]);Rb=np.array([[c,-s_],[s_,c]])
    v=(Rb@d.T).T+u[:2]+u[2]*(Rb@np.c_[-rr[:,1],rr[:,0]].T).T
    return float(np.sum(w*np.sqrt(np.sum(v*v,axis=1)+eps)))
   best=None
   starts=[np.zeros(3),*[np.r_[-(d[i]),0] for i in range(4)]]
   x0=min(starts,key=cost)
   best=minimize(cost,x0,method='Nelder-Mead',options=dict(xatol=1e-8,fatol=1e-10,maxiter=400,initial_simplex=x0+np.vstack([np.zeros(3),np.diag([2e-4,2e-4,2e-3])])))
   u=best.x;c,s_=math.cos(B[2]),math.sin(B[2]);Rb=np.array([[c,-s_],[s_,c]])
   v=(Rb@d.T).T+u[:2]+u[2]*(Rb@np.c_[-rr[:,1],rr[:,0]].T).T
   slip+=np.sqrt(np.sum(v*v,axis=1));B=B+u;trace.append(B.copy())
  prev=mat
 tr=np.array(trace);return dict(body_x_mm=B[0]*1000,body_y_mm=B[1]*1000,yaw_deg=math.degrees(B[2]),yaw_peak_deg=math.degrees(np.abs(tr[:,2]).max()),slip_mm=slip*1000,total_slip_mm=float(slip.sum()*1000))
def pair(t):
 s=t*107;return [s,s,s,s]
def stagger(n,a=30,b=80,first=2):
 """Between route knots a..b the two rear legs advance in n alternate stages; front legs and the rest follow the route."""
 def f(t):
  s=t*107
  if s<=a or s>=b:return [s,s,s,s]
  u=(s-a)/(b-a)*2*n;st=int(min(u,2*n-1));frac=u-st
  dk=(b-a)/n;i=st//2
  lead=a+dk*i+(dk*frac if st%2==0 else dk);lag=a+dk*i+(dk*frac if st%2==1 else 0)
  r=[s,s,s,s];r[first]=lead;r[5-first]=lag;return r
 return f
rows={}
def show(n,r):
 rows[n]=r;print('%-28s body x %+6.1f y %+5.1f yaw %+5.2f peak %4.2f | slip LF %5.1f RF %5.1f RH %5.1f LH %5.1f total %6.1f'%(n,r['body_x_mm'],r['body_y_mm'],r['yaw_deg'],r['yaw_peak_deg'],*r['slip_mm'],r['total_slip_mm']),flush=True)
show('pair (dev.5 route)',sim(pair))
for n in (1,2,3,4,6):show('stagger n=%d'%n,sim(stagger(n)))
for lr in (.1,.2):
 show('pair lr %.1f'%lr,sim(pair,lr=lr))
 for n in (3,4):show('stagger n=%d lr %.1f'%(n,lr),sim(stagger(n),lr=lr))
for mu in ((.8,.8,1,1),(1.2,1.2,1,1),(1,1,.7,.7),(1,1,1.3,1.3)):
 show('pair mu %s'%str(mu),sim(pair,mu=mu));show('stagger n=4 mu %s'%str(mu),sim(stagger(4),mu=mu))
dump(out/'deploy-friction.json',dict(classification='QUASI_STATIC_COULOMB_MODEL_NOT_HARDWARE',rows=rows))

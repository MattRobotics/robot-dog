#!/usr/bin/env python3
"""Bound rigid flat-belly feasibility in each EXISTING collision box.
An upper bound below zero for any leg proves that box cannot place the whole
leg above the planar belly. Independent joint progress; no terrain/sensor claim.
"""
from geometry import *
import re
from scipy.optimize import differential_evolution
fw=root/'05_Firmware/MATDOG_Controller'
limits=np.array(json.loads((Path(__file__).parent/'gen1_geometry.json').read_text())['joints'],dtype=object)
source=(fw/'src/simple/CollisionData.h').read_text()
boxes={name:np.array([[float(v) for v in b.split(',')] for b in [lo,hi]]) for name,lo,hi in re.findall(r'constexpr Box (\w+)=\{\{([^}]+)\},\{([^}]+)\}\}',source)}

def vertices(leg):
 # Coordinate vectors in each source link's frame, before its joint rotations.
 result=[];hipz=.0465 if leg in ('lf','rf') else .0265;side=1 if leg in ('lf','lh') else -1
 for part in ('upper_leg','lower_leg','foot'):
  v=meshes[leg+'_'+part+'_link'][:,:3].copy()
  if part=='foot':v+=np.array([.107,-side*.0015,-.0499])
  result.append((part,v))
 return hipz,side*.048,result

def floor(leg,q):
 h,u,l=q;hipz,y0,parts=vertices(leg);vals=[]
 for part,v in parts:
  a=u if part=='upper_leg' else u+l
  zz=-v[:,0]*np.sin(a)+v[:,2]*np.cos(a)
  if part!='upper_leg':zz-=.09*np.cos(u)
  vals.append(hipz+(y0+v[:,1])*np.sin(h)+zz*np.cos(h))
 return min(z.min() for z in vals)

def trig(lo,hi,sin=False):
 f=np.sin if sin else np.cos;values=[f(lo),f(hi)];offset=math.pi/2 if sin else 0
 for k in range(math.ceil((lo-offset)/math.pi),math.floor((hi-offset)/math.pi)+1):values.append(f(offset+k*math.pi))
 return min(values),max(values)
def scale(v,a,b):return np.minimum(v*a,v*b),np.maximum(v*a,v*b)
def product(a,b,c,d):
 v=np.array([a*c,a*d,b*c,b*d]);return v.min(axis=0),v.max(axis=0)
def bound(leg,lo,hi):
 sh=trig(lo[0],hi[0],True);ch=trig(lo[0],hi[0]);cu=trig(lo[1],hi[1]);hipz,y0,parts=vertices(leg);tops=[]
 for part,v in parts:
  a,b=(lo[1],hi[1]) if part=='upper_leg' else (lo[1]+lo[2],hi[1]+hi[2]);sa=trig(a,b,True);ca=trig(a,b)
  za,zb=scale(-v[:,0],*sa);zc,zd=scale(v[:,2],*ca);za+=zc;zb+=zd
  if part!='upper_leg':za-=.09*cu[1];zb-=.09*cu[0]
  _,top=product(za,zb,*ch);_,yy=scale(y0+v[:,1],*sh);tops.append(float((hipz+yy+top).min()))
 return min(tops)

def prove(leg,lo,hi,ceiling):
 # Every rejected cell has upper bound < ceiling, with 1e-10 m guard.
 pending=[(lo,hi)];count=0;worst=-math.inf
 while pending:
  a,b=pending.pop();upper=bound(leg,a,b)+1e-10;count+=1
  if upper<ceiling:worst=max(worst,upper);continue
  if count>1000000:return {'proved':False,'cells':count,'remaining':len(pending)}
  j=int(np.argmax(b-a));mid=(a[j]+b[j])/2;c=b.copy();c[j]=mid;d=a.copy();d[j]=mid;pending.extend([(a,c),(d,b)])
 return {'proved':True,'cells':count,'upper_bound_m':worst,'ceiling_m':ceiling}

if __name__=='__main__':
 output=Path(sys.argv[1]);results={}
 for name,box in boxes.items():
  candidates=[]
  for leg,i in [('lf',0),('rh',6)]:
   lo=np.maximum(box[0,i:i+3],[j['limits_rad'][0] for j in limits[i:i+3]]);hi=np.minimum(box[1,i:i+3],[j['limits_rad'][1] for j in limits[i:i+3]])
   r=differential_evolution(lambda q:-floor(leg,q),list(zip(lo,hi)),seed=29,popsize=8,maxiter=70,tol=1e-7,polish=True)
   candidates.append((float(-r.fun),leg,lo,hi,r.x))
  value,leg,lo,hi,q=min(candidates,key=lambda c:c[0]);ceiling=min(-.001,value+.002)
  certificate=prove(leg,lo,hi,ceiling) if value<-.001 else {'proved':False}
  results[name]={'blocking_leg':leg,'best_sample_floor_m':value,'argmax_q':q.tolist(),'domain_lo':lo.tolist(),'domain_hi':hi.tolist(),'certificate':certificate}
  print(name,results[name],flush=True)
 output.write_text(json.dumps(results,indent=2)+'\n')

"""Independent-footprint height bounds and multiple seated/stretch intent families."""
import json,itertools,math
import numpy as np
from scipy.optimize import differential_evolution
from model import Model,LEGS,transform,sha
from survey import OUT,save,general_ik

def main():
 m=Model();env=json.loads((OUT/'envelope.json').read_text());modes=json.loads((OUT/'contact_modes.json').read_text());rows=[]
 seed=env['poses']['LOW_C4']['q'];targets=np.array(env['poses']['LOW_C4']['contacts']);targets[:,2]=0
 for pitch,height in itertools.product(np.deg2rad([-5,-10,-15,-20,-30,-45]),[.06,.08,.10,.12]):
  body=transform((0,pitch,0),(0,0,height));q=general_ik(m,targets,body,seed)
  r=m.evaluate(q,body,'FOOT_SUPPORT',LEGS) if q is not None else {'valid':False,'errors':['IK_OR_OPTIMIZER_FAILURE'],'body':body.tolist()}
  r['intent']='SEATED_OR_PITCHED_RESEARCH';rows.append(r)
 print('seated family',sum(r['valid'] for r in rows),flush=True)
 lows=[]
 for lift in modes['lift_off_attempts']:
  start=lift['endpoint'];targets0=np.array(start['contacts']);targets0[:,2]=0
  for height in [1e-5,.0001,.001,.005,.01]:
   body=transform(xyz=(0,0,height));q=general_ik(m,targets0,body,start['q'])
   r=m.evaluate(q,body,'FOOT_SUPPORT',LEGS) if q is not None else {'valid':False,'errors':['IK_OR_OPTIMIZER_FAILURE']}
   r['height_m']=height;lows.append(r)
 # Multi-start bounded search maximizes each leg's downward mesh extent.
 opt=[]
 for i in (0,2):
  def objective(q3):
   q=np.zeros(12);q[3*i:3*i+3]=q3;tf=m.fk(q)[LEGS[i]+'_foot_link'];v=m.meshes[LEGS[i]+'_foot_link'].vertices
   return float(np.min(v@tf[2,:3]+tf[2,3]))
  for seednum in [17,31]:
   result=differential_evolution(objective,m.limits[3*i:3*i+3],seed=seednum,popsize=8,maxiter=60,tol=1e-8,polish=True,workers=1)
   opt.append({'leg':LEGS[i],'q':result.x.tolist(),'maximum_downward_extent_m':-float(result.fun),'optimizer_seed':seednum})
 f=opt[0]['q'];r=opt[2]['q'];q=np.array(f+[-f[0],f[1],f[2]]+r+[-r[0],r[1],r[2]])
 common=min(x['maximum_downward_extent_m'] for x in opt)-1e-5;tf=m.fk(q,transform(xyz=(0,0,common)));target_hi=m.contacts(tf);target_hi[:,2]=0;highs=[]
 for h in np.arange(common-.03,common+1e-6,.002):
  body=transform(xyz=(0,0,h));solution=general_ik(m,target_hi,body,q)
  v=m.evaluate(solution,body,'FOOT_SUPPORT',LEGS) if solution is not None else {'valid':False,'errors':['IK_OR_OPTIMIZER_FAILURE']}
  v['height_m']=float(h);highs.append(v)
 # Analytic necessary upper bound from every distal mesh vertex, lower-joint
 # interval, unconstrained upper/hip rotations. It is not a feasible-pose claim.
 bounds={}
 for leg in LEGS:
  hip=m.joints[leg+'_hip_joint'].origin_xyz;upper=m.joints[leg+'_upper_leg_joint'].origin_xyz
  a=-m.joints[leg+'_lower_leg_joint'].origin_xyz[2];origin=np.array(m.joints[leg+'_foot_joint'].origin_xyz)
  v=m.meshes[leg+'_foot_link'].vertices+origin;d=upper[1]+v[:,1];lo,hi=m.limits[3*LEGS.index(leg)+2];maximum=[]
  for lower in [lo,hi]:maximum.append(v[:,0]*np.sin(lower)-v[:,2]*np.cos(lower))
  phase=np.arctan2(v[:,0],-v[:,2]);phase=np.where(phase>np.pi,phase-2*np.pi,phase)
  maxdot=np.maximum(*maximum);inside=(phase>=lo)&(phase<=hi);maxdot[inside]=np.hypot(v[inside,0],v[inside,2])
  radius=np.sqrt(d*d+a*a+v[:,0]**2+v[:,2]**2+2*a*maxdot)
  bounds[leg]=float(radius.max()-hip[2])
 save('expanded_envelope',{'provenance':{'sources':m.sources,'generator_sha256':sha(__file__)},'seated_candidates':rows,'near_ground_four_foot_candidates':lows,'maximum_extension_search':opt,'high_candidates':highs,'necessary_body_height_upper_bound_m':min(bounds.values()),'per_leg_upper_bounds_m':bounds,'scope':'Observed feasible extrema and necessary upper bound; finite searches are not proofs of global exclusion'})
 print('low valid',[r['height_m'] for r in lows if r['valid']],'high valid',[r['height_m'] for r in highs if r['valid']],'upper',bounds,flush=True)
if __name__=='__main__':main()

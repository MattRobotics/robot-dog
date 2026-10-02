"""Independent SE(2), polynomial, URDF FK and numerical contact-IK checks."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares
from core import Core,params,ROOT,lifecycle
sys.path.insert(0,str(ROOT/'06_Software/Matdog_Core/pose_audit'))
from model import Model,LEGS
OUT=ROOT/'09_Logs/Validation_Reports/G4_Gait_Envelope'

def reference(p,s,feet0):
 kind,h,lift,duty,dx,dy,yaw,sway,_=p
 offsets=[0,.5,.75,.25] if kind==0 else [0,.5,0,.5]
 twist=np.array([[0,-yaw,dx],[yaw,0,dy],[0,0,0.]])
 def pose(t):return expm(twist*t)
 def anchor(i,td):return feet0[i] if td<=0 else np.r_[(pose(td+duty/2)@np.r_[feet0[i,:2],1])[:2],0]
 tf=pose(s);body=np.eye(4);body[:2,:2]=tf[:2,:2];body[:2,3]=tf[:2,2];body[2,3]=h
 if kind==0 and sway:
  k=int(np.floor(s*4));u=(s-k/4)/(duty-.75);a=(-1)**k*sway
  shift=a if u>=1 else a*(-1+2*np.polynomial.polynomial.polyval(u,[0,0,0,10,-15,6]))
  body[:2,3]+=body[:2,0]*shift
 feet=[]
 for i,o in enumerate(offsets):
  k=np.floor(s+o);phase=s+o-k;td=k-o;a=anchor(i,td)
  if phase<duty:feet.append(a)
  else:
   u=(phase-duty)/(1-duty);blend=np.polynomial.polynomial.polyval(u,[0,0,0,10,-15,6])
   f=a+(anchor(i,td+1)-a)*blend;f[2]=64*lift*(u*(1-u))**3;feet.append(f)
 return body,np.array(feet)

def main():
 core=Core();model=Model();feet0=core.frame(params(),0)['feet'];metrics=dict(max_body=0.,max_target_m=0.,max_urdf_contact_m=0.,max_qdot_error=0.,max_qddot_error=0.,max_numeric_ik_rad=0.,max_periodic_q_rad=0.,max_world_stance_drift_m=0.)
 count=0;dls=0
 configurations=[params(kind=k,height=h,duty=.8 if k==0 else .6,x=x,y=y,yaw=yaw,sway=.004 if k==0 else 0) for k in (0,1) for h in (.08,.1,.12,.15) for x,y,yaw in ((.01,0,0),(-.01,0,0),(.005,.001,.005),(.005,-.001,-.005))]
 for p in configurations:
  previous=None;last=None
  for s in np.linspace(0,3,121):
   f=core.frame(p,s,previous=previous)
   if 'q' not in f: # Nominal-strip limits can reject otherwise feasible commands.
    assert f['status'] in ('CONTACT_INVALID','JOINT_LIMIT_BLOCKED','IK_UNREACHABLE');break
   assert np.isfinite(np.r_[f['q'],f['qdot'],f['qddot']]).all()
   repeat=core.frame(p,s,previous=previous);assert np.array_equal(f['seed'],repeat['seed']);assert np.array_equal(f['qddot'],repeat['qddot'])
   b,feet=reference(p,s,feet0);metrics['max_body']=max(metrics['max_body'],float(np.max(abs(b-f['body']))));metrics['max_target_m']=max(metrics['max_target_m'],float(np.max(abs(feet-f['feet']))))
   actual=model.contacts(model.fk(f['q'],f['body']));metrics['max_urdf_contact_m']=max(metrics['max_urdf_contact_m'],float(np.max(np.linalg.norm(actual-f['feet'],axis=1))))
   if last is not None:
    for i in range(4):
     if f['leg_phase'][i,1] and last['leg_phase'][i,1] and f['leg_phase'][i,0]>=last['leg_phase'][i,0]:metrics['max_world_stance_drift_m']=max(metrics['max_world_stance_drift_m'],float(np.linalg.norm(actual[i]-last_actual[i])))
   slower=core.frame(p,s,period=2,previous=previous);np.testing.assert_allclose(f['qdot'],2*slower['qdot'],atol=1e-12);np.testing.assert_allclose(f['qddot'],4*slower['qddot'],atol=1e-12)
   if s>=1:
    cyclic=core.frame(p,s+1,previous=f);np.testing.assert_allclose(f['q'],cyclic['q'],atol=2e-12);metrics['max_periodic_q_rad']=max(metrics['max_periodic_q_rad'],float(max(abs(f['q']-cyclic['q']))))
   previous=last=f;last_actual=actual;count+=1
  for s in (1.123,1.373,1.623,1.873):
   f=core.frame(p,s)
   if 'q' not in f:continue
   eps=2e-5;a=core.frame(p,s-eps,previous=f);b=core.frame(p,s+eps,previous=f)
   if 'q' not in a or 'q' not in b:continue
   metrics['max_qdot_error']=max(metrics['max_qdot_error'],float(max(abs((b['q']-a['q'])/(2*eps)-f['qdot']))))
   metrics['max_qddot_error']=max(metrics['max_qddot_error'],float(max(abs((b['qdot']-a['qdot'])/(2*eps)-f['qddot']))))
  # Independent numerical solve using URDF rotation chains, not the C++ formula.
  if p[1]==.1 and p[5]==0:
   f=core.frame(p,1.137)
   def residual(q):return (model.contacts(model.fk(q,f['body']))-f['feet']).ravel()
   result=least_squares(residual,f['q']+np.tile([.0001,-.0002,.0001],4),bounds=(model.limits[:,0],model.limits[:,1]),gtol=1e-13,ftol=1e-13,xtol=1e-13)
   assert result.success and max(abs(residual(result.x)))<1e-9
   metrics['max_numeric_ik_rad']=max(metrics['max_numeric_ik_rad'],float(max(abs(result.x-f['q']))));dls+=4
 assert metrics['max_body']<1e-12 and metrics['max_target_m']<1e-12 and metrics['max_urdf_contact_m']<1e-9
 assert metrics['max_qdot_error']<2e-5 and metrics['max_qddot_error']<.002 and metrics['max_numeric_ik_rad']<1e-7
 # Signed planar commands tested independently against mirrored SE(2), not
 # presumed joint symmetry across front/rear geometry or inertial asymmetry.
 # Start/stop joins and time scaling, independently differentiated in time.
 # C2 joins can have a jerk jump: Richardson extrapolation cancels the O(h)
 # central qdot-difference bias at those joins (ordinary interior error is O(h²)).
 lifecycle_metrics={'max_qdot_error':0.,'max_qddot_error':0.,'frames':0}
 for kind in (0,1):
  p=params(kind=kind,height=.1,duty=.8 if kind==0 else .6,sway=.004 if kind==0 else 0)
  eps=1e-5;joins=(1.,3.,4.,6.)
  times=sorted(set([0.,3.2,7.]+[t+d for t in joins for d in (-eps,-eps/2,0,eps/2,eps)]))
  frames=lifecycle(core,p,times);slow=lifecycle(core,p,np.array(times)*2,period=2,stop_at=6.4)
  for f,b in zip(frames,slow):
   np.testing.assert_allclose(f['q'],b['q'],atol=2e-12);np.testing.assert_allclose(f['qdot'],2*b['qdot'],atol=1e-11);np.testing.assert_allclose(f['qddot'],4*b['qddot'],atol=1e-10)
  for t in joins:
   i=times.index(t);a,ah,f,bh,b=frames[i-2:i+3]
   lifecycle_metrics['max_qdot_error']=max(lifecycle_metrics['max_qdot_error'],float(max(abs((b['q']-a['q'])/(2*eps)-f['qdot']))))
   lifecycle_metrics['max_qddot_error']=max(lifecycle_metrics['max_qddot_error'],float(max(abs(2*(bh['qdot']-ah['qdot'])/eps-(b['qdot']-a['qdot'])/(2*eps)-f['qddot']))))
  np.testing.assert_allclose(frames[0]['q'],frames[-1]['q'],atol=1e-12)
  assert frames[-1]['state']==3 and max(abs(frames[-1]['qdot']))==0 and max(abs(frames[-1]['qddot']))==0
  lifecycle_metrics['frames']+=len(frames)
 assert lifecycle_metrics['max_qdot_error']<1e-6 and lifecycle_metrics['max_qddot_error']<.001
 report=dict(status='PASS',lifecycle=lifecycle_metrics,sample_count=count,numerical_ik_legs=dls,metrics=metrics,scope='Kinematic oracle. Mesh/support validity is a separate survey gate.')
 # The checkpoint's oracle_results.json is preserved; a final-source run is written beside it.
 name=sys.argv[sys.argv.index('--output')+1] if '--output' in sys.argv else 'oracle_results.json'
 (OUT/name).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()

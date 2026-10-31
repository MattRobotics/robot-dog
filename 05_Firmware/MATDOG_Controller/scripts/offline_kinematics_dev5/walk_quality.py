"""REV01-FIX WALK STANDARD: reference continuity at the 50 Hz release rate and static support with URDF inertials
on the production gait. Model only; servo tracking, compliance and dynamics not included."""
from common import *
from scipy.spatial import ConvexHull
import numpy as np
names=['LF_H','LF_U','LF_L','RF_H','RF_U','RF_L','RH_H','RH_U','RH_L','LH_H','LH_U','LH_L']
dt=.02;res={}
for st,f in zip(sys.argv[2::2],sys.argv[3::2]):
 a=np.loadtxt(f);c=a[:,0];b=a[:,1:4];q=a[:,4:16];ft=a[:,16:].reshape(len(a),4,4)
 v=np.diff(q,axis=0)/dt;ac=np.diff(v,axis=0)/dt;je=np.diff(ac,axis=0)/dt
 ia=np.unravel_index(np.abs(ac).argmax(),ac.shape);ij=np.unravel_index(np.abs(je).argmax(),je.shape);iv=np.unravel_index(np.abs(v).argmax(),v.shape)
 # static support: COM from URDF inertials, shifted by dcx; polygon of stance contacts
 def margin(dcx=0.,dcy=0.):
  out=[]
  for k in range(0,len(a),3):
   _,com=geometry(q[k]);cw=com[:2]+b[k,:2]+[dcx,dcy];pts=ft[k][ft[k][:,3]>.5][:,:2]
   if len(pts)<3:out.append((-1,c[k],len(pts)));continue
   h=ConvexHull(pts);out.append((float(np.min(-(h.equations[:,:2]@cw+h.equations[:,2]))),c[k],len(pts)))
  return min(out)
 m=margin();res[st]=dict(vmax=float(np.abs(v).max()),vjoint=names[iv[1]],vcycle=float(c[iv[0]]),amax=float(np.abs(ac).max()),ajoint=names[ia[1]],acycle=float(c[ia[0]]),jmax=float(np.abs(je).max()),jjoint=names[ij[1]],jcycle=float(c[ij[0]]),step_ticks=float(np.abs(np.diff(q,axis=0)).max()/TICK),support_mm=m[0]*1000,support_cycle=m[1],sens={f'{dx*1000:+.0f}mm':margin(dx)[0]*1000 for dx in (-.01,-.005,.005,.01)})
 r=res[st];print(st,'vmax %.3f %s c%.3f | amax %.2f %s c%.3f | jerk %.0f %s c%.3f | step %.1f tick | support %.2f mm c%.3f | COMx shift %s'%(r['vmax'],r['vjoint'],r['vcycle'],r['amax'],r['ajoint'],r['acycle'],r['jmax'],r['jjoint'],r['jcycle'],r['step_ticks'],r['support_mm'],r['support_cycle'],{k:round(x,1) for k,x in r['sens'].items()}),flush=True)
dump(Path(sys.argv[1])/'walk-quality.json',res)

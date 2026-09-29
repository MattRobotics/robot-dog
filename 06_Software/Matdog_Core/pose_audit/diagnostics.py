"""Additional diagnostics for preserved samples; does not replace search evidence."""
import json,argparse
from pathlib import Path
import numpy as np
import fcl
import trimesh
from scipy.spatial import ConvexHull
from model import Model,LEGS,TOL,PATCH_TOL,sha
from survey import OUT,save

def nearest_patch_point(vertices,point):
 """Nearest point on the convex near-bottom support patch (10 um band).
 The centroid depends on tessellation density; it is not a contact-model error.
 """
 v=np.unique(vertices,axis=0)
 if len(v)==1:return v[0]
 center=v.mean(axis=0);_,s,axes=np.linalg.svd(v-center,full_matrices=False)
 if len(s)<2 or s[1]<1e-12:
  t=(v-center)@axes[0];a=v[np.argmin(t)];b=v[np.argmax(t)];d=b-a
  return a+np.clip(np.dot(point-a,d)/np.dot(d,d),0,1)*d
 xy=(v-center)@axes[:2].T;h=ConvexHull(xy);ordered=v[h.vertices]
 triangles=np.array([[ordered[0],ordered[i],ordered[i+1]] for i in range(1,len(ordered)-1)])
 candidates=trimesh.triangles.closest_point(triangles,np.repeat(np.asarray(point)[None,:],len(triangles),axis=0))
 return candidates[np.argmin(np.linalg.norm(candidates-point,axis=1))]

def diagnostics(m,q,body,collisions=()):
 q=np.asarray(q);body=np.asarray(body);tf=m.fk(q,body);refs=m.contacts(tf);feet={};jac={}
 for i,l in enumerate(LEGS):
  n=l+'_foot_link';t=tf[n];v=m.meshes[n].vertices@t[:3,:3].T+t[:3,3];minimum=float(v[:,2].min());lowest=v[np.argmin(v[:,2])];patch=v[v[:,2]<=minimum+PATCH_TOL];center=patch.mean(axis=0);delta=center-refs[i]
  active=v[v[:,2]<=PATCH_TOL] if minimum>=-TOL else np.empty((0,3))
  nearest=nearest_patch_point(patch,refs[i]);patch_delta=nearest-refs[i]
  feet[l]={'analytical_reference_world_m':refs[i].tolist(),'lowest_vertex_world_m':lowest.tolist(),'lowest_patch_centroid_world_m':center.tolist(),'lowest_patch_vertex_count':len(patch),'lowest_patch_extent_m':np.ptp(patch,axis=0).tolist(),'actual_ground_patch_world_m':active.tolist(),'reference_to_patch_centroid_m':delta.tolist(),'centroid_displacement_m':float(np.linalg.norm(delta)),'nearest_lowest_patch_world_m':nearest.tolist(),'reference_to_nearest_patch_m':patch_delta.tolist(),'discrepancy_m':float(np.linalg.norm(patch_delta)),'vertical_discrepancy_m':minimum-float(refs[i,2]),'definition':'nearest point on convex hull of vertices within 10 um of lowest Z; separate centroid is tessellation-dependent and its migration is not material-point slip'}
  J=np.zeros((3,3));eps=1e-6
  for j in range(3):
   plus=q.copy();minus=q.copy();plus[3*i+j]+=eps;minus[3*i+j]-=eps
   J[:,j]=(m.contacts(m.fk(plus,body))[i]-m.contacts(m.fk(minus,body))[i])/(2*eps)
  s=np.linalg.svd(J,compute_uv=False)
  jac[l]={'singular_values_m_per_rad':s.tolist(),'condition_number':float(s[0]/s[-1]) if s[-1]>1e-14 else None,'near_singular':bool(s[-1]<1e-5),'jacobian_determinant':float(np.linalg.det(J))}
 collision_details=[]
 for a,b in collisions:
  oa=fcl.CollisionObject(m.bvh[a],fcl.Transform(tf[a][:3,:3],tf[a][:3,3]));ob=fcl.CollisionObject(m.bvh[b],fcl.Transform(tf[b][:3,:3],tf[b][:3,3]));r=fcl.CollisionResult();fcl.collide(oa,ob,fcl.CollisionRequest(num_max_contacts=128,enable_contact=True),r)
  collision_details.append({'pair':[a,b],'minimum_separation_m':0.0,'reported_triangle_penetration_max_m':max((float(c.penetration_depth) for c in r.contacts),default=None),'depth_scope':'FCL triangle contact estimate, not a solid minimum translation distance','cause':'joint-space folding sweep crosses nonadjacent link volumes; exact pair identifies body/foot or front/rear involvement'})
 return {'feet':feet,'conditioning':jac,'conditioning_scope':'central-difference analytical G2 cross-section reference FK Jacobian; not nonsmooth mesh-edge IK Jacobian','joint_margins_rad':np.minimum(q-m.limits[:,0],m.limits[:,1]-q).tolist(),'collision_details':collision_details}

def walk(obj,path=''):
 if isinstance(obj,dict):
  if all(k in obj for k in ('q','body','regime')):yield path,obj
  for k,v in obj.items():yield from walk(v,path+'/'+str(k))
 elif isinstance(obj,list):
  for i,v in enumerate(obj):yield from walk(v,path+'/'+str(i))

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--sources',nargs='+',default=['rest_search','envelope','expanded_envelope','contact_modes','transitions','transfer_search','support_transfer','route_extension']);parser.add_argument('--output',default='candidate_diagnostics');args=parser.parse_args()
 m=Model();files=args.sources;rows=[];inputs={}
 for name in files:
  p=OUT/(name+'.json')
  if not p.exists():continue
  inputs[p.name]=sha(p);data=json.loads(p.read_text());count=0
  for path,r in walk(data):
   detail=diagnostics(m,r['q'],r['body'],r.get('collisions',[]));actual=[n for n,z in r['ground_min_m'].items() if -TOL<=z<=TOL];allowed=({'base_link'} if r['regime'] in ('BODY_SUPPORT','BODY_AND_FOOT_SUPPORT') else set())|{l+'_foot_link' for l in r['active_feet']}
   rows.append({'source':p.name,'pointer':path,'intended_contacts':sorted(allowed),'actual_contacts':actual,'forbidden_contacts':sorted(set(actual)-allowed),'valid_in_preserved_search':r['valid'],**detail});count+=1
  print(name,count,flush=True)
 save(args.output,{'provenance':{'canonical_sources':m.sources,'inputs':inputs,'tool_sources':{p.name:sha(p) for p in [Path(__file__), Path(__file__).with_name('model.py')]}},'records':rows})
if __name__=='__main__':main()

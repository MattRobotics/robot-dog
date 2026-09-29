"""Explicit mesh pair policy, support tolerances, and q=0 ground audit."""
import itertools,json
import numpy as np
from model import Model,LEGS,TOL,PATCH_TOL,transform,sha
from survey import OUT,save

def main():
 m=Model();pairs=[]
 for a,b in itertools.combinations(m.meshes,2):
  key=tuple(sorted((a,b)));pairs.append({'links':[a,b],'policy':'EXCLUDED_ADJACENT_DESIGNED_INTERFACE' if key in m.excluded else 'CHECK_TRIANGLE_INTERSECTION_AND_SOLID_CONTAINMENT','reason':m.excluded.get(key,'nonadjacent canonical solids')})
 q=np.zeros(12);tf=m.fk(q);low={}
 for l in LEGS:
  t=tf[l+'_foot_link'];low[l]=float(np.min(m.meshes[l+'_foot_link'].vertices@t[2,:3]+t[2,3]))
 h=-min(low.values());feet=[l for l,z in low.items() if abs(z+h)<=TOL]
 q0=m.evaluate(q,transform(xyz=(0,0,h)),'FOOT_SUPPORT',feet)
 save('collision_policy',{'provenance':{'canonical_sources':m.sources,'generator_sha256':sha(__file__),'model_sha256':sha(__import__('pathlib').Path(__file__).with_name('model.py'))},'mesh_inventory':[{'link':n,'triangles':len(mesh.faces),'vertices':len(mesh.vertices),'watertight':bool(mesh.is_watertight)} for n,mesh in m.meshes.items()],'pairs':pairs,'ground_penetration_and_undeclared_contact_tolerance_m':TOL,'declared_support_patch_band_m':PATCH_TOL,'support_definition':'convex hull of near-ground mesh vertices for explicitly declared base/feet only; CAD COM projection inside hull','exclusion_limit':'Entire direct URDF adjacency pairs are excluded for designed bearing/fork or fixed assembly interfaces. These results do not certify internal adjacent-link clearances or manufacturing tolerances. All 120 nonadjacent pairs are checked; no nonadjacent exception is introduced.','minimum_separation_scope':'minimum over tested nonadjacent triangle pairs; zero for triangle overlap; containment also rejects a candidate','q0_first_ground_contact':q0,'q0_height_m':h,'q0_active_feet':feet,'q0_body_ground':m.evaluate(q,transform(xyz=(0,0,m.body_height)),'BODY_SUPPORT')})
 print('POLICY',len(pairs),'q0',h,feet,q0['errors'],flush=True)
if __name__=='__main__':main()

"""Search an equivalent 150 mm endpoint without demanding the C4 footprint."""
import json,itertools
import numpy as np
from model import Model,LEGS,transform,sha
from survey import OUT,save,general_ik

def main():
 m=Model();source=json.loads((OUT/'route_extension.json').read_text());routes=[]
 for i,rise in enumerate(source['original_footprint_rises']):
  start=rise['frames'][-1];q=np.array(start['q']);targets=np.array(start['contacts']);targets[:,2]=0;options=[];chosen=None
  for x,y in sorted(itertools.product(np.arange(-.04,.041,.005),repeat=2),key=lambda xy:(xy[0]**2+xy[1]**2,xy)):
   body=transform(xyz=(x,y,.15));n=general_ik(m,targets,body,q)
   if n is None:options.append({'xy':[float(x),float(y)],'status':'IK_FAILURE'});continue
   v=m.evaluate(n,body,'FOOT_SUPPORT',LEGS);options.append({'xy':[float(x),float(y)],'status':'VALID_STATIC' if v['valid'] else v['errors']})
   if v['valid']:chosen=v;break
  frames=[start];failure=None
  if chosen is not None:
   for s in np.linspace(0,1,101)[1:]:
    body=transform(xyz=(1-s)*np.array(start['body'])[:3,3]+s*np.array(chosen['body'])[:3,3]);n=general_ik(m,targets,body,q)
    if n is None:failure='IK_FAILURE';break
    v=m.evaluate(n,body,'FOOT_SUPPORT',LEGS);frames.append(v);q=n
    if not v['valid']:failure=v['errors'];break
  routes.append({'family':i,'endpoint_search':options,'endpoint':chosen,'frames':frames,'all_samples_valid':chosen is not None and failure is None,'failure':failure,'continuous_collision_free_proved':False,'contact_model':'fixed cross-section reference XY and actual mesh minimum Z; patch migration permitted, not G2 strip-contact startup'})
  print('equivalent',i,len(options),len(frames),chosen is not None,failure,flush=True)
 save('equivalent_stand',{'provenance':{'canonical_sources':m.sources,'source_sha256':sha(OUT/'route_extension.json'),'generator_sha256':sha(__file__)},'routes':routes})
if __name__=='__main__':main()

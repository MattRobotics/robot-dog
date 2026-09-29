"""Recheck every unique preserved whole-robot sample, retaining original artifacts."""
import json,concurrent.futures,hashlib,argparse
import numpy as np
from model import Model,sha
from diagnostics import walk
from survey import OUT,save
MODEL=None
def initialize():
 global MODEL;MODEL=Model()
def check(item):
 key,r=item;v=MODEL.evaluate(r['q'],np.array(r['body']),r['regime'],r['active_feet'])
 return key,{'valid':v['valid'],'errors':v['errors'],'min_separation':v['min_separation'],'max_ground_delta_m':max(abs(v['ground_min_m'][n]-r['ground_min_m'][n]) for n in v['ground_min_m']),'com_delta_m':float(np.linalg.norm(np.array(v['com_world_m'])-r['com_world_m']))}
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--sources',nargs='+',default=['rest_search','envelope','expanded_envelope','contact_modes','transitions','transfer_search','support_transfer','route_extension']);parser.add_argument('--output',default='revalidation');args=parser.parse_args()
 inputs={};unique={};refs=[]
 for name in args.sources:
  p=OUT/(name+'.json');inputs[p.name]=sha(p)
  for path,r in walk(json.loads(p.read_text())):
   key=hashlib.sha256(json.dumps([r['q'],r['body'],r['regime'],r['active_feet']],sort_keys=True).encode()).hexdigest();unique[key]=r;refs.append({'file':p.name,'pointer':path,'key':key,'preserved_valid':r['valid']})
 with concurrent.futures.ProcessPoolExecutor(max_workers=4,initializer=initialize) as pool:results=dict(pool.map(check,unique.items()))
 changes=[{**r,'current':results[r['key']]} for r in refs if r['preserved_valid']!=results[r['key']]['valid']]
 m=Model();save(args.output,{'provenance':{'canonical_sources':m.sources,'inputs':inputs,'generator_sha256':sha(__file__),'model_sha256':sha(__import__('pathlib').Path(__file__).with_name('model.py'))},'sample_references':len(refs),'unique_samples':len(unique),'classification_changes':changes,'max_com_delta_m':max(r['com_delta_m'] for r in results.values()),'max_ground_delta_m':max(r['max_ground_delta_m'] for r in results.values()),'results':results,'references':refs})
 print('REVALIDATION',len(refs),len(unique),'classification changes',len(changes),flush=True)
if __name__=='__main__':main()

"""Dense full-mesh re-validation of representative candidates (new samples, saved separately)."""
import json,sys
from concurrent.futures import ProcessPoolExecutor
from core import FIELDS
from survey import evaluate,initialize,save,OUT
# id -> (source file, intervals). Candidates are read from preserved raw artifacts, never edited.
CANDIDATES={287:('full_cases.json',400),1238:('full_cases.json',200),3105:('refinement_full.json',200),1046:('full_cases.json',200),3084:('refinement_full.json',200)}
def main():
 tasks=[]
 for i,(src,n) in CANDIDATES.items():
  case=next(c for c in json.loads((OUT/src).read_text())['cases'] if c['id']==i)
  tasks.append((i,[case['parameters'][k] for k in FIELDS],True,n))
 results=[]
 with ProcessPoolExecutor(max_workers=4,initializer=initialize) as pool:
  for r in pool.map(evaluate,tasks):
   results.append(r);print('dense',r['id'],r['classification'],r['geometry_samples'],flush=True)
   save('dense_representatives.json',dict(scope='New denser full-mesh samples of preserved candidates; same G3.5 model and 1 um policy',planned_cases=len(tasks),complete=len(results)==len(tasks),cases=results))
if __name__=='__main__':main()

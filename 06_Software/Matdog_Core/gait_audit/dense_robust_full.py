"""Full canonical-mesh validation (400 intervals) of density-robust candidates from robust_search.json."""
import json,sys
from concurrent.futures import ProcessPoolExecutor
from core import FIELDS
from survey import evaluate,initialize,save,OUT
# Ids index robust_search.json (stage 2 survivors): WALK 80 mm; TROT 80 mm (lift 10 and 5) and 120 mm.
CANDIDATES=(357,61,29,309)
def main():
 search=json.loads((OUT/'robust_search.json').read_text())
 assert set(CANDIDATES)<=set(search['robust_ids'])
 tasks=[(i,[search['cases'][i]['parameters'][k] for k in FIELDS],True,400) for i in CANDIDATES]
 results=[]
 with ProcessPoolExecutor(max_workers=4,initializer=initialize) as pool:
  for r in pool.map(evaluate,tasks):
   results.append(r);print('full400',r['id'],r['classification'],r['geometry_samples'],flush=True)
   save('dense_robust_full.json',dict(scope='New full-mesh samples (400 intervals) of density-robust ground-screen survivors; ids index robust_search.json',planned_cases=len(tasks),complete=len(results)==len(tasks),cases=results))
if __name__=='__main__':main()

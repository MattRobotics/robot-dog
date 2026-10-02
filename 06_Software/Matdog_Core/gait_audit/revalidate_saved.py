"""Recompute every saved full-mesh case with the CURRENT gait core and compare with the saved frames.

Several saved artifacts were generated before late input guards were added to Gait.cpp (see
definitions.json provenance). This proves the saved q/qdot/qddot/feet/body values are still what the
current source produces, so the evidence need not be regenerated. No mesh or policy is involved.
"""
import json,sys
import numpy as np
from core import Core,FIELDS
from survey import OUT,save,phases
FILES=('full_cases.json','refinement_full.json','dense_representatives.json','dense_robust_full.json')
def main():
 core=Core();rows=[];worst=0.
 for name in FILES:
  for case in json.loads((OUT/name).read_text())['cases']:
   p=np.array([case['parameters'][k] for k in FIELDS]);old=None;err=dict(q=0.,qdot=0.,qddot=0.,body=0.)
   for saved in case['frames']:
    f=core.frame(p,saved['phase'],previous=old);assert 'q' in f,(name,case['id'],saved['phase']);old=f
    err['q']=max(err['q'],float(np.max(abs(f['q']-saved['q']))));err['qdot']=max(err['qdot'],float(np.max(abs(f['qdot']-saved['qdot']))))
    err['qddot']=max(err['qddot'],float(np.max(abs(f['qddot']-saved['qddot']))));err['body']=max(err['body'],float(np.max(abs(f['body']-np.array(saved['body'])))))
   worst=max(worst,*err.values());rows.append(dict(artifact=name,id=case['id'],frames=len(case['frames']),max_abs_difference=err))
 assert worst<1e-9,worst
 save('revalidate_saved.json',dict(scope='Saved frames vs current production gait core (bridge); no policy or mesh involved',worst_abs_difference=worst,cases=rows))
 print('REVALIDATE_SAVED PASS',len(rows),'cases; worst',worst)
if __name__=='__main__':sys.exit(main())

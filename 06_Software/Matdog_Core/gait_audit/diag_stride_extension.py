"""DIAGNOSTIC: extend pure fore/aft stride beyond the +/-100 mm/cycle edge of the saved grids.

The micrometre-quarantine screen passed up to the grid edge for TROT at every height, so the grid edge, not
the robot, was the limit there. This ground/IK/limit/support screen (no self-collision; micrometre foot events
quarantined as in diag_screen.py) finds the first kinematic category that stops a larger stride. Not acceptance.
"""
import itertools,sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import survey
from core import params,FIELDS
from survey import save
from diag_screen import diag
def main():
 cases=[]
 for kind,h,x in itertools.product((0,1),(.08,.1,.12,.14,.15),[s*v for s in (-1,1) for v in (.1,.12,.14,.16,.18,.2,.25,.3)]):
  cases.append(params(kind=kind,height=h,x=x,lift=.01,duty=.8 if kind==0 else .6,sway=.004 if kind==0 else 0).tolist())
 with ProcessPoolExecutor(max_workers=4,initializer=survey.initialize) as pool:
  rows=list(pool.map(diag,[(5000+i,p,40) for i,p in enumerate(cases)],chunksize=2))
 save('diag_stride_extension.json',dict(scope='DIAGNOSTIC stride extension, no self-collision, micrometre foot events quarantined; not acceptance',cases=rows))
 for r in rows:
  p=r['parameters'];print('WALK' if p['type']==0 else 'TROT',p['height_m'],'%+.2f'%p['advance_x_m'],r['categories'],'margin %.3f'%r['min_joint_margin_rad'] if r['min_joint_margin_rad'] is not None else '')
if __name__=='__main__':sys.exit(main())

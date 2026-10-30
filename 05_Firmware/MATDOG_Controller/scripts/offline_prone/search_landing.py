#!/usr/bin/env python3
"""Offline feasibility search inside unchanged ENTRY and gen1 limits."""
from geometry import *
from scipy.optimize import differential_evolution

def qfrom(p):
 hf,hr,uf,ur,lf,lr=p
 return np.array([hf,uf,lf,-hf,uf,lf,-hr,ur,lr,hr,ur,lr])
def score(p,details=False):
 q=qfrom(p);V,c=geometry(q);low={n:float(v[:,2].min()) for n,v in V.items() if n!='base_link'}
 # Keep the full rigid leg assembly above the planar ventral support, not only feet.
 return low if details else -min(low.values())
# 12 mrad margin from the existing common ENTRY bounds and gen1 lower limits.
bounds=[(-.18,.18),(-.18,.18),(1.259,1.664),(1.369,1.773),(-1.568,-1.343),(-1.578,-1.351)]
r=differential_evolution(score,bounds,seed=20261029,popsize=10,maxiter=100,tol=1e-7,polish=True)
print('optimization',r.success,r.fun,r.x.tolist());print('q',qfrom(r.x).tolist());print('level_clearances',score(r.x,True));print('rest',rest(qfrom(r.x)))
# A rounded interior target is easier to audit than an optimizer corner.
for p in [[.17,.17,1.46,1.38,-1.56,-1.56],[.16,.16,1.46,1.40,-1.56,-1.56],[.17,.17,1.62,1.70,-1.56,-1.56]]:
 print('rounded',p,score(p,True),rest(qfrom(p)))

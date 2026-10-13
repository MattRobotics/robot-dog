#!/usr/bin/env python3
"""Continuous triangle separation for adjacent revolutes in their hinge frame.

Exact trigonometric projection extrema replace the whole-link Lipschitz budget.
No joint-core exclusion, mesh trimming, altered tolerance or assembly exception.
Every canonical triangle remains; an unresolved fit/intersection is BLOCKED.
"""
import argparse,hashlib,json,math,time
from pathlib import Path
import numpy as np
from paired_prone_model import PairModel,ROOT
from matdog_geometry_mesh_kernel import _triangle_grid_index,_iter_candidate_batches

ROUND=1e-9

def trig_range(c,a,b,lo,hi):
    first=c+a*np.cos(lo)+b*np.sin(lo);last=c+a*np.cos(hi)+b*np.sin(hi)
    lower=np.minimum(first,last);upper=np.maximum(first,last)
    phi=np.arctan2(b,a);r=np.hypot(a,b)
    for k in range(-2,3):
        maximum=phi+2*np.pi*k;minimum=maximum+np.pi
        upper=np.where((maximum>=lo)&(maximum<=hi),np.maximum(upper,c+r),upper)
        lower=np.where((minimum>=lo)&(minimum<=hi),np.minimum(lower,c-r),lower)
    return lower,upper

def coefficients(points,axis):
    unit=axis/np.linalg.norm(axis)
    parallel=np.einsum('...j,j->...',points,unit)[...,None]*unit
    return parallel,points-parallel,np.cross(unit,points)

def projected_ranges(c,a,b,axes,lo,hi):
    # N triangle pairs, three vertices, one independent direction per pair.
    proj=lambda x:np.einsum('nij,nj->ni',x,axes)
    low,high=trig_range(proj(c),proj(a),proj(b),lo,hi)
    return low.min(axis=1),high.max(axis=1)

def bound(A,C,U,V,lo,hi):
    B=C+U*np.cos((lo+hi)/2)+V*np.sin((lo+hi)/2)
    ea=np.roll(A,-1,axis=1)-A;eb=np.roll(B,-1,axis=1)-B
    na=np.cross(ea[:,0],ea[:,1]);nb=np.cross(eb[:,0],eb[:,1])
    axes=[np.broadcast_to(np.eye(3)[i],na.shape) for i in range(3)]+[na,nb]
    axes += [np.cross(ea[:,i],eb[:,j]) for i in range(3) for j in range(3)]
    axes += [np.cross(n,e[:,i]) for n,e in [(na,ea),(nb,eb)] for i in range(3)]
    result=np.zeros(len(A))
    for axis in axes:
        length=np.linalg.norm(axis,axis=1)
        unit=np.divide(axis,length[:,None],out=np.zeros_like(axis),where=length[:,None]>1e-15)
        ap=np.einsum('nij,nj->ni',A,unit);blo,bhi=projected_ranges(C,U,V,unit,lo,hi)
        result=np.maximum(result,np.maximum(ap.min(axis=1)-bhi,blo-ap.max(axis=1)))
    return result-ROUND

def certify(scene,joint,lo,hi,budget_s):
    j=scene.model.joints[joint];pair=(j.parent_link,j.child_link)
    # Canonical collision origins/rpy are zero; assert rather than assume.
    for link in pair:
        g=scene.model.collision_geometry(link)
        assert tuple(g.origin_xyz)==(0.,0.,0.) and tuple(g.origin_rpy)==(0.,0.,0.)
    assert tuple(j.origin_rpy)==(0.,0.,0.)
    A=scene.mesh(pair[0]).triangles_local;B=scene.mesh(pair[1]).triangles_local
    C,U,V=coefficients(B,np.array(j.axis_xyz));C=C+np.array(j.origin_xyz)
    low,high=trig_range(C,U,V,lo,hi)
    # Enclosing swept boxes encoded as degenerate triangles for the grid only.
    boxes=np.stack((low.min(axis=1),high.max(axis=1),low.min(axis=1)),axis=1)
    grids=[_triangle_grid_index(t,.01,0.) for t in (A,boxes)]
    start=time.monotonic();checks=0;minimum=math.inf;candidates=0
    for ia,ib in _iter_candidate_batches(*grids,100000000,'adjacent axis '+joint,2048):
        candidates+=len(ia);stack=[(ia,ib,lo,hi,0)]
        while stack:
            x,y,l,h,depth=stack.pop();g=bound(A[x],C[y],U[y],V[y],l,h);checks+=len(x)
            good=g>0
            if good.any():minimum=min(minimum,float(g[good].min()))
            x=x[~good];y=y[~good]
            if len(x)==0:continue
            if depth>=30 or time.monotonic()-start>budget_s:
                return dict(status='BLOCKED',reason='UNRESOLVED_CANONICAL_TRIANGLES',pair=pair,
                  triangle_pair=[int(x[0]),int(y[0])],angle_interval=[l,h],checks=checks,candidates=candidates,
                  excluded_triangles=0,intentional_intersections_assumed=False)
            mid=(l+h)/2;stack.extend(((x,y,l,mid,depth+1),(x,y,mid,h,depth+1)))
    return dict(status='PASS_CONTINUOUS',pair=pair,checks=checks,candidates=candidates,
      minimum_projection_bound_m=None if math.isinf(minimum) else minimum,
      excluded_triangles=0,intentional_intersections_assumed=False)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--budget-s',type=float,default=90);args=parser.parse_args()
    assert not args.output.exists(),'preserve evidence'
    path=ROOT/'09_Logs/Validation_Reports/Motion_Night_Run_2026-10-12/continuous_pair_path.json'
    raw=path.read_bytes();d=json.loads(raw);m=PairModel();rows=[]
    for leg in ('lf','rf','rh','lh'):
        for part,field in [('hip','h'),('upper_leg','uf' if leg in ('lf','rf') else 'ur'),('lower_leg','lf' if leg in ('lf','rf') else 'lr')]:
            sign=-1 if part=='hip' and leg in ('lf','lh') else 1
            angles=[sign*s[k][field] for s in d['segments'] for k in ('start_parameters','end_parameters')]
            joint=leg+'_'+part+'_joint';r=certify(m.scene,joint,min(angles),max(angles),args.budget_s)
            rows.append(dict(joint=joint,domain_rad=[min(angles),max(angles)],**r));print(joint,r['status'],r['checks'],flush=True)
            args.output.write_text(json.dumps(dict(schema='MATDOG_ADJACENT_AXIS_INTERVAL_V1',source_sha256=hashlib.sha256(raw).hexdigest(),trajectory_changed=False,
              all_adjacent_swept_certified=len(rows)==12 and all(x['status']=='PASS_CONTINUOUS' for x in rows),roundoff_m=ROUND,
              hardware_authorized=False,rows=rows),indent=2)+'\n')
if __name__=='__main__':main()

"""Offline Phase B: measured radial/axial clearance of the 8 base pins in the HIP sockets (REV01-FIX), V5 kernel primitives.
usage: pin_clearance.py <repo_copy> <urdf_variant_dir_name> <out_json>"""
import sys, json
from pathlib import Path
import numpy as np
repo=Path(sys.argv[1]).resolve(); var=sys.argv[2]; out=Path(sys.argv[3])
sys.path.insert(0,str(repo/"06_Software/Matdog_Core/calibration"))
import matdog_geometry_mesh_kernel as mk
U=repo/"03_CAD/URDF"; S=(0.001,)*3
AX={"lf":(0.1125,0.0475,0.0487),"rf":(0.1125,-0.0475,0.0487),"rh":(-0.1125,-0.0475,0.0287),"lh":(-0.1125,0.0475,0.0287)}
NOM={ # nominal CAD (d_pin mm, h_pin mm) per end, inner/outer
 "lf":{"inner":(5.55,3.45),"outer":(5.70,3.15)},"rf":{"inner":(5.55,3.45),"outer":(5.70,3.15)},
 "rh":{"inner":(5.70,3.85),"outer":(5.55,3.15)},"lh":{"inner":(5.70,3.85),"outer":(5.55,3.15)}}
B=mk.load_mesh(U/var/"meshes/collision/base_link.stl",scale=S,use_cache=False).triangles_local
res={}
for h,ax in AX.items():
    ax=np.array(ax); H=mk.load_mesh(U/f"matt_robodog_rev00/meshes/collision/{h}_hip_link.stl",scale=S,use_cache=False).triangles_local+ax
    sgn=np.sign(ax[0])
    def r_of(T): return np.hypot(T[...,1]-ax[1],T[...,2]-ax[2])
    xb=np.abs(B[...,0]); rb=r_of(B); xh=np.abs(H[...,0]); rh=r_of(H)
    for end,(xlo,xhi) in (("inner",(0.0935,0.0980)),("outer",(0.1260,0.1305))):
        # pin triangles: all vertices r<=3.0 mm, |x| inside end band
        pin=(rb.max(1)<=3.0e-3)&(xb.min(1)>=xlo)&(xb.max(1)<=xhi+0.0005)
        P=B[pin]
        # socket triangles of the hip in the same band (wall + floor), r<=3.2 mm
        sk=(rh.max(1)<=3.2e-3)&(xh.min(1)>=xlo-0.0005)&(xh.max(1)<=xhi+0.0005)
        Sk=H[sk]
        # pin tip x: the extreme |x| toward the socket floor
        px=np.abs(P[...,0]).ravel(); pr=r_of(P).ravel()
        tip = px.min() if end=="inner" else px.max()
        rp_max=pr.max()
        side=(r_of(Sk).min(1)>=2.9e-3)&(np.ptp(np.abs(Sk[...,0]),axis=1)>1e-6)  # socket wall: spans x, r>=2.9
        floor=(np.ptp(np.abs(Sk[...,0]),axis=1)<1e-9)
        # floor plane x: nearest to pin tip
        fx=np.abs(Sk[floor][...,0]).ravel()
        floor_x = (fx.max() if end=="inner" else fx.min()) if len(fx) else None
        # wall vertex radii (about axis) within the pin-length band
        wall=Sk[side]; wr=r_of(wall).ravel()
        r_wall_min=wr.min(); r_wall_med=float(np.median(wr))
        # exact surface distance pin(all P) vs socket (all Sk) with kernel triangle_triangle_distance
        pmin,pmax=P.min(1),P.max(1); best=(1e9,None); best_side=1e9
        # side-only distance: pin side triangles (not cap) vs wall triangles
        pin_cap=(np.ptp(np.abs(P[...,0]),axis=1)<1e-9)
        for i,t in enumerate(P):
            ok=np.all((Sk.min(1)<=pmax[i]+1e-3)&(Sk.max(1)>=pmin[i]-1e-3),axis=1)
            for j in np.nonzero(ok)[0]:
                d=mk.triangle_triangle_distance(t,Sk[j])
                if d<best[0]: best=(d,(i,int(j)))
                if (not pin_cap[i]) and side[j] and d<best_side: best_side=d
        # axial: cap-to-floor
        cap_x=np.abs(P[pin_cap][...,0]).ravel()
        axial=abs(floor_x-(cap_x.min() if end=="inner" else cap_x.max())) if floor_x is not None and len(cap_x) else None
        d_nom,h_nom=NOM[h][end]
        root = np.abs(B[...,0][(rb.max(1)<=3.0e-3)]) 
        res[f"{h}_{end}"]={"pin_tri":int(pin.sum()),"socket_tri":int(sk.sum()),
          "pin_diameter_max_mm":round(2*rp_max*1e3,4),"pin_diameter_nominal_mm":d_nom,
          "pin_tip_abs_x_mm":round(tip*1e3,4),"socket_floor_abs_x_mm":None if floor_x is None else round(floor_x*1e3,4),
          "socket_wall_r_min_vertex_mm":round(r_wall_min*1e3,4),"socket_wall_r_median_mm":round(r_wall_med*1e3,4),
          "radial_clearance_vertex_based_mm":round((r_wall_min-rp_max)*1e3,4),
          "radial_clearance_surface_min_mm":round(best_side*1e3,4),
          "axial_clearance_cap_to_floor_mm":None if axial is None else round(axial*1e3,4),
          "min_3d_distance_pin_to_socket_mm":round(best[0]*1e3,4)}
        print(h,end,json.dumps(res[f"{h}_{end}"]))
out.write_text(json.dumps(res,indent=1))

"""Offline canonical triangle-mesh, contact and CAD/URDF COM model. No hardware."""
from pathlib import Path
import sys, hashlib, itertools, xml.etree.ElementTree as ET
import numpy as np
import trimesh
import fcl
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'06_Software/Matdog_Core/kinematics'))
from matdog_urdf_fk import load_urdf_joints, canonical_urdf_path
from matdog_offline_collision_contact_policy import _transform_from_xyz_rpy
LEGS=('lf','rf','rh','lh')
TOL=1e-6
PATCH_TOL=1e-5

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def vec(node,attr,default='0 0 0'):return np.array([float(x) for x in (node.get(attr,default) if node is not None else default).split()])
def origin(node):return np.array(_transform_from_xyz_rpy(tuple(vec(node,'xyz')),tuple(vec(node,'rpy'))))
def transform(rpy=(0,0,0),xyz=(0,0,0)):return np.array(_transform_from_xyz_rpy(tuple(xyz),tuple(rpy)))
def hull_margin(points,point):
    points=np.unique(np.array(points)[:,:2],axis=0)
    if len(points)<3:return None,points.tolist()
    try:h=ConvexHull(points)
    except Exception:return None,points.tolist()
    return float(np.min(-(h.equations[:,:2]@np.array(point)[:2]+h.equations[:,2]))),points[h.vertices].tolist()

class Model:
    def __init__(self):
        self.urdf=canonical_urdf_path(ROOT);root=ET.parse(self.urdf).getroot()
        self.joints=load_urdf_joints(self.urdf)
        self.meshes={};self.bvh={};self.components={};self.inertial={};self.sources={str(self.urdf.relative_to(ROOT)):sha(self.urdf)}
        self.joint_order=[f'{l}_{j}_joint' for l in LEGS for j in ('hip','upper_leg','lower_leg')]
        self.limits=np.array([[self.joints[n].lower_limit_rad,self.joints[n].upper_limit_rad] for n in self.joint_order])
        for link in root.findall('link'):
            name=link.get('name');parts=[]
            for c in link.findall('collision'):
                mesh=c.find('geometry/mesh')
                if mesh is None:raise ValueError('Expected canonical collision mesh')
                uri=mesh.get('filename');path=self.urdf.parent/uri.split('matt_robodog_rev00/')[-1]
                if not path.exists():path=self.urdf.parent/uri
                m=trimesh.load_mesh(path,process=True);m.apply_scale(vec(mesh,'scale','1 1 1'));m.apply_transform(origin(c.find('origin')))
                parts.append(m);self.sources[str(path.relative_to(ROOT))]=sha(path)
            m=trimesh.util.concatenate(parts);self.meshes[name]=m
            if not m.is_watertight:raise ValueError('Canonical mesh is not watertight: '+name)
            self.components[name]=np.array([part.vertices[0] for part in m.split(only_watertight=False)])
            b=fcl.BVHModel();b.beginModel(len(m.vertices),len(m.faces));b.addSubModel(m.vertices,m.faces);b.endModel();self.bvh[name]=b
            inertial=link.find('inertial');self.inertial[name]=(float(inertial.find('mass').get('value')),vec(inertial.find('origin'),'xyz'))
        self.excluded={tuple(sorted((j.parent_link,j.child_link))):'URDF direct joint adjacency: designed bearing/fork interface' for j in self.joints.values()}
        self.pairs=[(a,b) for a,b in itertools.combinations(self.meshes,2) if tuple(sorted((a,b))) not in self.excluded]
        contact_source=ROOT/'06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml'
        self.sources[str(contact_source.relative_to(ROOT))]=sha(contact_source)
        self.body_height=-float(self.meshes['base_link'].vertices[:,2].min())
        self.base_patch=self.meshes['base_link'].vertices[self.meshes['base_link'].vertices[:,2]<=-self.body_height+PATCH_TOL]
    def fk(self,q,body=None):
        values=dict(zip(self.joint_order,q));tf={'base_link':np.eye(4) if body is None else body};remaining=list(self.joints.values())
        while remaining:
            for j in remaining[:]:
                if j.parent_link not in tf:continue
                motion=np.eye(4)
                if j.joint_type!='fixed':motion=trimesh.transformations.rotation_matrix(values[j.name],j.axis_xyz)
                tf[j.child_link]=tf[j.parent_link]@transform(j.origin_rpy,j.origin_xyz)@motion;remaining.remove(j)
        return tf
    def com(self,tf):
        total=sum(m for m,c in self.inertial.values())
        return sum(m*(tf[n][:3,:3]@c+tf[n][:3,3]) for n,(m,c) in self.inertial.items())/total,total
    def contacts(self,tf):
        # Canonical rigid cylinder, values loaded from the G2 Python model below.
        from matdog_foot_contact import load_foot_contact_model
        # See canonical loader at initialization call boundary; no copied offsets.
        if not hasattr(self,'contact_geometry'):self.contact_geometry=load_foot_contact_model(ROOT)
        g=self.contact_geometry
        result=[]
        for leg in LEGS:
            t=tf[leg+'_foot_link'];axis=t[:3,:3]@np.array(g.cylinder_axis_in_foot_link_unit);n=np.array([0.,0.,1.]);down=-(n-axis*np.dot(axis,n));down/=np.linalg.norm(down)
            result.append(t[:3,3]+t[:3,:3]@np.array(g.cylinder_center_in_foot_link_m)+g.cylinder_radius_m*down)
        return np.array(result)
    def evaluate(self,q,body,regime,active_feet=(),full=True,base_contact=None):
        q=np.asarray(q);body=np.asarray(body)
        if q.shape!=(12,) or not np.isfinite(q).all():raise ValueError('12 finite semantic angles required')
        if (body.shape!=(4,4) or not np.isfinite(body).all() or
            not np.allclose(body[3],[0,0,0,1],atol=1e-12,rtol=0) or
            not np.allclose(body[:3,:3].T@body[:3,:3],np.eye(3),atol=1e-12,rtol=0) or
            abs(np.linalg.det(body[:3,:3])-1)>1e-12):raise ValueError('Rigid finite body transform required')
        tf=self.fk(q,body);ground={};points=[];errors=[]
        margin=float(np.min(np.minimum(q-self.limits[:,0],self.limits[:,1]-q)))
        if margin < -1e-12:errors.append('JOINT_LIMIT')
        expected_base=regime in ('BODY_SUPPORT','BODY_AND_FOOT_SUPPORT')
        if base_contact is None:base_contact=expected_base
        if regime!='TRANSITIONAL_SUPPORT' and base_contact!=expected_base:errors.append('INVALID_SUPPORT_DECLARATION')
        if regime=='TRANSITIONAL_SUPPORT' and not (base_contact or active_feet):errors.append('INVALID_SUPPORT_DECLARATION')
        allowed=({'base_link'} if base_contact else set())|{l+'_foot_link' for l in active_feet}
        if len(set(active_feet))!=len(active_feet) or any(l not in LEGS for l in active_feet):errors.append('INVALID_SUPPORT_DECLARATION')
        if regime=='BODY_SUPPORT' and active_feet:errors.append('INVALID_SUPPORT_DECLARATION')
        if regime=='BODY_AND_FOOT_SUPPORT' and not active_feet:errors.append('INVALID_SUPPORT_DECLARATION')
        if regime=='FREE_SPACE' and active_feet:errors.append('INVALID_SUPPORT_DECLARATION')
        if regime=='FOOT_SUPPORT' and not active_feet:errors.append('INVALID_SUPPORT_DECLARATION')
        if regime not in ('BODY_SUPPORT','BODY_AND_FOOT_SUPPORT','FOOT_SUPPORT','FREE_SPACE','TRANSITIONAL_SUPPORT'):errors.append('INVALID_REGIME')
        for n,m in self.meshes.items():
            vertices=m.vertices@tf[n][:3,:3].T+tf[n][:3,3];z=float(vertices[:,2].min());ground[n]=z
            if z < -TOL:errors.append('GROUND_PENETRATION:'+n)
            elif z<=TOL and n not in allowed:errors.append('UNDECLARED_GROUND_CONTACT:'+n)
            if n in allowed:
                if z>PATCH_TOL:errors.append('MISSING_MESH_SUPPORT:'+n)
                else:points.extend(vertices[vertices[:,2]<=PATCH_TOL])
        contacts=self.contacts(tf)
        tilts={l:float(np.arcsin(np.clip(abs(tf[l+'_foot_link'][2,1]),0,1))) for l in LEGS}
        collisions=[];minimum=None
        if full:
            objects={n:fcl.CollisionObject(self.bvh[n],fcl.Transform(t[:3,:3],t[:3,3])) for n,t in tf.items()}
            for a,b in self.pairs:
                result=fcl.CollisionResult();hit=fcl.collide(objects[a],objects[b],fcl.CollisionRequest(),result)
                if hit:
                    collisions.append([a,b]);minimum={'pair':[a,b],'distance_m':0.0};continue
                # Closed triangle surfaces can be disjoint while one solid contains another.
                for outer,inner in ((a,b),(b,a)):
                    relative=np.linalg.inv(tf[outer])@tf[inner]
                    seeds=self.components[inner]@relative[:3,:3].T+relative[:3,3]
                    bounds=self.meshes[outer].bounds
                    inside_box=np.all((seeds>bounds[0])&(seeds<bounds[1]),axis=1)
                    if inside_box.any() and self.meshes[outer].contains(seeds[inside_box]).any():
                        collisions.append([a,b]);break
                d=float(fcl.distance(objects[a],objects[b],fcl.DistanceRequest(),fcl.DistanceResult()))
                if minimum is None or d<minimum['distance_m']:minimum={'pair':[a,b],'distance_m':d}
            if collisions:errors.append('SELF_COLLISION')
        com,mass=self.com(tf);support,polygon=hull_margin(points,com) if len(points)>=3 else (None,[])
        if regime!='FREE_SPACE' and (support is None or support<0):errors.append('SUPPORT_INVALID')
        return dict(q=q.tolist(),body=body.tolist(),regime=regime,active_feet=list(active_feet),base_contact=bool(base_contact),ground_min_m=ground,
          contacts=contacts.tolist(),contact_axis_tilt_rad=tilts,collisions=collisions,min_separation=minimum,com_world_m=com.tolist(),mass_kg=mass,
          support_polygon=polygon,support_margin_m=support,joint_margin_rad=margin,errors=errors,valid=not errors)

"""Deterministic mesh-ground, self-collision, COM and posture survey (offline)."""
import argparse,json,itertools,math,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from model import Model,ROOT,LEGS,TOL,PATCH_TOL,transform,hull_margin,sha
from ik import ContactIK
OUT=ROOT/'09_Logs/Validation_Reports/G35_Pose_Audit'
def save(name,data):
    OUT.mkdir(parents=True,exist_ok=True);(OUT/(name+'.json')).write_text(json.dumps(data,sort_keys=True,indent=2,allow_nan=False)+'\n')
def leg_ground(model,index,q3,height):
    q=np.zeros(12);q[3*index:3*index+3]=q3;tf=model.fk(q,transform(xyz=(0,0,height)));leg=LEGS[index]
    names=[leg+'_'+n+'_link' for n in ('hip','upper_leg','lower_leg','foot')]
    ground={n:float((model.meshes[n].vertices@tf[n][:3,:3].T+tf[n][:3,3])[:,2].min()) for n in names}
    return ground,tf,names

def leg_clear(model,index,q3,height):
    import fcl
    ground,tf,names=leg_ground(model,index,q3,height)
    if min(ground.values()) < -TOL:return False
    names=['base_link']+names
    objs={n:fcl.CollisionObject(model.bvh[n],fcl.Transform(tf[n][:3,:3],tf[n][:3,3])) for n in names}
    return not any(fcl.collide(objs[a],objs[b],fcl.CollisionRequest(),fcl.CollisionResult()) for a,b in itertools.combinations(names,2) if tuple(sorted((a,b))) not in model.excluded)

def general_ik(m,targets,body,seed):
    # Separate offline optimizer; G2 production IK is never generalized.
    q=np.array(seed).copy()
    for i in range(4):
        def fun(v):
            temp=q.copy();temp[i*3:i*3+3]=v
            tf=m.fk(temp,body);reference=m.contacts(tf)[i];name=LEGS[i]+'_foot_link'
            vertices=m.meshes[name].vertices@tf[name][:3,:3].T+tf[name][:3,3]
            return np.array([reference[0]-targets[i,0],reference[1]-targets[i,1],vertices[:,2].min()-targets[i,2]])
        r=least_squares(fun,q[i*3:i*3+3],bounds=(m.limits[i*3:i*3+3,0],m.limits[i*3:i*3+3,1]),xtol=1e-12,ftol=1e-12,gtol=1e-12,max_nfev=120)
        if np.linalg.norm(r.fun)>1e-8:return None
        q[i*3:i*3+3]=r.x
    return q

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--stage',choices=['rest','envelope','transitions','all'],default='all');args=parser.parse_args()
    m=Model();ik=ContactIK();prov={'canonical_sources':m.sources,'tool_sources':{p.name:sha(p) for p in Path(__file__).parent.glob('*.py')},'model_com':'CAD/URDF MODEL COM','ground_tolerance_m':TOL,'patch_tolerance_m':PATCH_TOL}
    a=json.loads(next((ROOT/'09_Logs/Validation_Reports').glob('*C4A_offline_safe_stand_candidate.json')).read_text())
    targets=np.array([a['legs'][l]['target_contact_reference_world_m'] for l in LEGS])
    if args.stage in ('rest','all'):
        mesh=m.meshes['base_link'];mask=(mesh.triangles[:,:,2]<=-m.body_height+PATCH_TOL).all(axis=1);patch=mesh.submesh([np.where(mask)[0]],append=True)
        bodypatch={'height_m':m.body_height,'minimum_local_z_m':-m.body_height,'patch_area_m2':float(patch.area),'patch_triangles':len(patch.faces),'connected_patches':len(patch.split(only_watertight=False)),'polygon_xy':hull_margin(m.base_patch,[0,0])[1]}
        families={};candidates=[]
        # Symmetry is used only LF/RF and RH/LH. Their own meshes are checked on every combined candidate.
        for i in (0,2):
            found=[];total=0
            for h,u,l in itertools.product(np.linspace(-.52,.52,5),np.linspace(*m.limits[i*3+1],35),np.linspace(*m.limits[i*3+2],35)):
                total+=1;g,tf,names=leg_ground(m,i,(h,u,l),m.body_height)
                if min(g.values())<1e-5:continue
                if leg_clear(m,i,(h,u,l),m.body_height):found.append({'q':[float(h),float(u),float(l)],'clearance':min(g.values())})
            families[LEGS[i]]={'tested':total,'valid_per_leg':found};print('rest leg',LEGS[i],len(found),flush=True)
        fronts=sorted(families['lf']['valid_per_leg'],key=lambda r:-r['clearance'])
        rears=sorted(families['rh']['valid_per_leg'],key=lambda r:-r['clearance'])
        # Keep separate hip/topology families, not just one maximum-clearance point.
        for fsign,rsign in itertools.product((-1,0,1),repeat=2):
            fa=[r for r in fronts if np.sign(r['q'][0])==fsign][:3];ra=[r for r in rears if np.sign(r['q'][0])==rsign][:3]
            for f,r in itertools.product(fa,ra):
                fq=f['q'];rq=r['q'];q=fq+[-fq[0],fq[1],fq[2]]+rq+[-rq[0],rq[1],rq[2]]
                v=m.evaluate(q,transform(xyz=(0,0,m.body_height)),'BODY_SUPPORT');v['name']='BODY_ONLY';candidates.append(v)
        foot_solutions={}
        # Contact-at-rest: sweep planar X and both IK seeds; each leg's exact geometry.
        for i,l in enumerate(LEGS):
            rows=[];hx=m.joints[l+'_hip_joint'].origin_xyz[0]
            for x in np.linspace(hx-.24,hx+.24,481):
                for seed in ((0,2,.6),(0,1,-1.5)):
                    q,s=ik.leg(i,(x,.094 if l in ('lf','lh') else -.094,-m.body_height),seed)
                    if q is not None and leg_clear(m,i,q,m.body_height):rows.append({'x':float(x),'q':q.tolist()})
            foot_solutions[l]=rows;print('rest foot',l,len(rows),flush=True)
        valid=[r for r in candidates if r['valid']]
        if not valid:raise ValueError('No validated rest in systematic search')
        # Prefer wide nonadjacent separation, then joint margin; fixed tie ordering.
        best=max(valid,key=lambda r:(r['min_separation']['distance_m'],r['joint_margin_rad']))
        best['name']='REST_GROUND'
        for active in [('lf','rf'),('rh','lh'),LEGS]:
            if all(foot_solutions[l] for l in active):
                q=np.array(best['q'])
                for l in active:q[3*LEGS.index(l):3*LEGS.index(l)+3]=foot_solutions[l][len(foot_solutions[l])//2]['q']
                v=m.evaluate(q,transform(xyz=(0,0,m.body_height)),'BODY_AND_FOOT_SUPPORT',active);v['name']='BODY_PLUS_'+'_'.join(active);candidates.append(v)
            else:candidates.append({'name':'BODY_PLUS_'+'_'.join(active),'valid':False,'errors':['NO_CONTACT_CANDIDATE_IN_DECLARED_SEARCH'],'global_impossibility_proved':False})
        save('rest_search',{'provenance':prov,'body_patch':bodypatch,'search':families,'foot_search':foot_solutions,'candidates':candidates,'selected':best})
    if args.stage in ('envelope','all'):
        rows=[];poses={}
        for dim,values in [('height',np.arange(.002,.232,.002)),('x',np.arange(-.1,.101,.005)),('y',np.arange(-.06,.061,.005)),('yaw',np.deg2rad(np.arange(-40,41,2))),('roll',np.deg2rad(np.arange(-20,21,2))),('pitch',np.deg2rad(np.arange(-20,21,2)))]:
            for value in values:
                xyz=[0,0,.15];rpy=[0,0,0]
                if dim=='height':xyz[2]=float(value)
                elif dim in ('x','y'):xyz[0 if dim=='x' else 1]=float(value)
                else:rpy[('roll','pitch','yaw').index(dim)]=float(value)
                b=transform(rpy,xyz)
                seed,_=ik.pose(targets,transform(xyz=(0,0,.15)))
                q0=ik.pose(targets,b)[0] if dim not in ('roll','pitch') else seed
                q=general_ik(m,targets,b,q0) if q0 is not None else None
                if q is None:r={'valid':False,'errors':['IK_REACHABILITY_OR_LIMIT_OR_OPTIMIZER_FAILURE'],'body':b.tolist()}
                else:r=m.evaluate(q,b,'FOOT_SUPPORT',LEGS)
                r.update(dimension=dim,value=float(value));rows.append(r)
            print('envelope',dim,sum(r['valid'] for r in rows if r['dimension']==dim),flush=True)
        for name,h in [('LOW_C4',.1),('STAND',.15),('LOW_CROUCH',.06),('XGO_HEIGHT_LOW_INTENT',.15*60/110)]:
            b=transform(xyz=(0,0,h));q,s=ik.pose(targets,b);r=m.evaluate(q,b,'FOOT_SUPPORT',LEGS);r['name']=name;poses[name]=r
        # Independently specified research intent families, never claimed recovered XGO endpoints.
        for name,rpy,h,dx in [('SIT_CANDIDATE',(0,-.3,0),.07,0),('STRETCH_CANDIDATE',(0,.2,0),.1,-.03),('CRAWL_READY',(0,0,0),.06,0),('BEG_CANDIDATE',(0,-.6,0),.07,0),('ROLL_PREPARATION',(0.3,0,0),.1,0)]:
            b=transform(rpy,(dx,0,h));seed=poses['LOW_C4']['q'];q=general_ik(m,targets,b,seed)
            r=m.evaluate(q,b,'FOOT_SUPPORT',LEGS) if q is not None else {'valid':False,'errors':['IK_REACHABILITY_OR_LIMIT_OR_OPTIMIZER_FAILURE'],'body':b.tolist(),'q':seed}
            r.update(name=name,provenance_scope='MATDOG_RESEARCH_INTENT_NOT_RECOVERED_XGO_GEOMETRY');poses[name]=r
        extrema={d:([min(r['value'] for r in rows if r['dimension']==d and r['valid']),max(r['value'] for r in rows if r['dimension']==d and r['valid'])] if any(r['valid'] and r['dimension']==d for r in rows) else None) for d in ('height','x','y','yaw','roll','pitch')}
        save('envelope',{'provenance':prov,'scope':'One-axis slices at C4 footprint, other dimensions held at stand; sampled bounds, not global extrema','extrema':extrema,'rows':rows,'poses':poses})
    if args.stage in ('transitions','all'):
        rest=json.loads((OUT/'rest_search.json').read_text())['selected'];env=json.loads((OUT/'envelope.json').read_text());edges=[]
        for start,end in [('LOW_CROUCH','LOW_C4'),('LOW_C4','STAND'),('STAND','STRETCH_CANDIDATE'),('STAND','SIT_CANDIDATE'),('STAND','CRAWL_READY')]:
            a=env['poses'][start];b=env['poses'][end];frames=[];seed=np.array(a['q'])
            if a['valid'] and b['valid']:
                # Body-pose interpolation through RPY, physical contacts held fixed, IK at every sample.
                from scipy.spatial.transform import Rotation
                ra=Rotation.from_matrix(np.array(a['body'])[:3,:3]).as_euler('xyz');rb=Rotation.from_matrix(np.array(b['body'])[:3,:3]).as_euler('xyz')
                for s in np.linspace(0,1,51):
                    body=transform((1-s)*ra+s*rb,(1-s)*np.array(a['body'])[:3,3]+s*np.array(b['body'])[:3,3]);q=general_ik(m,targets,body,seed)
                    if q is None:frames.append({'valid':False,'errors':['IK_FAILURE']});break
                    v=m.evaluate(q,body,'FOOT_SUPPORT',LEGS);v['delta_max_rad']=float(np.max(abs(q-seed)));seed=q;frames.append(v)
            edges.append({'from':start,'to':end,'reversible_geometrically':True,'regimes':['FOOT_SUPPORT'],'frames':frames,'status':'VALID_SEQUENCE_CANDIDATE' if frames and all(v['valid'] for v in frames) else 'UNPROVEN_OR_INVALID','continuous_collision_free_proved':False})
            print('transition',start,end,edges[-1]['status'],flush=True)
        # Contact-changing routes cannot be asserted merely from static endpoints.
        edges.append({'from':'REST_GROUND','to':'LOW_C4','status':'UNPROVEN','regimes':['BODY_SUPPORT','BODY_AND_FOOT_SUPPORT','FOOT_SUPPORT'],'required_contact_events':['add front feet','add rear feet','base lift-off'],'reason':'Rear contact-at-body-height and connecting folded branch require further route search; no valid sampled route established','continuous_collision_free_proved':False})
        save('transitions',{'provenance':prov,'edges':edges,'startup_reassessment':'STILL_REQUIRED'})
if __name__=='__main__':main()

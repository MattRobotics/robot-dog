#!/usr/bin/env python3
"""MATDOG-original two-pair keypoint compiler, offline and non-actuating.

Two UPPER/LOWER profiles, independently calibrated semantic HIP neutrals.
Validate keypoints before generating any trajectory. No servo tick mapping,
bus access, permission grant, arbitrary body rock or 12-DOF pose optimizer.
"""
from pathlib import Path
from dataclasses import dataclass, asdict
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent
URDF=ROOT/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
sys.path.insert(0,str(ROOT/'06_Software/Matdog_Core/calibration'))
from matdog_geometry_scene_v5 import RobotSceneV5

LEGS=('lf','rf','rh','lh')
PAIRS={'rear':('rh','lh'),'front':('rf','lf')}
JOINTS=tuple(f'{leg}_{part}_joint' for leg in LEGS for part in ('hip','upper_leg','lower_leg'))
GROUND_TOLERANCE_M=1e-9  # unchanged from the existing prone keypoint checker

def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def r_y(a): return np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])

def trig_intervals(a,b,c,lo,hi):
    """Closed intervals where c+a*cos(x)+b*sin(x)>=0 in the given domain."""
    radius=math.hypot(a,b)
    if radius<1e-15:return [(lo,hi)] if c>=0 else []
    threshold=-c/radius
    if threshold>1:return []
    if threshold<=-1:return [(lo,hi)]
    phi=math.atan2(b,a);half=math.acos(threshold)
    return [(max(lo,phi+2*math.pi*k-half),min(hi,phi+2*math.pi*k+half)) for k in range(-2,3)
            if max(lo,phi+2*math.pi*k-half)<=min(hi,phi+2*math.pi*k+half)]

def intersect_intervals(first,second):
    return [(max(a,x),min(b,y)) for a,b in first for x,y in second if max(a,x)<=min(b,y)]

@dataclass(frozen=True)
class PairProfile:
    upper_rad: float
    lower_rad: float

@dataclass(frozen=True)
class Observation:
    q_rad: dict
    base_z_m: float
    roll_rad: float=0.
    pitch_rad: float=0.
    provenance: str='UNKNOWN'
    coherent: bool=False
    fresh: bool=False
    belly_contact_observed: bool=False
    installed_mapping_verified: bool=False
    after_torque_loss_reobserved: bool=False

class PairModel:
    def __init__(self):
        self.scene=RobotSceneV5.from_urdf(URDF)
        self.xml=ET.parse(URDF).getroot()
        self.nodes={x.get('name'):x for x in self.xml.findall('joint')}
        assert len(self.scene.model.collision_link_names)==17
        self._verify_chain()
        # At the unique in-limit solution sin(h)=0, upper hinge axes are parallel
        # to base Y and the upper origins have no HIP-induced lateral-plane tilt.
        # This establishes URDF neutral only, never encoder zero or an installed
        # mapping. A future runtime consumer must resolve each side separately.
        self.neutral_hip_rad={leg:0. for leg in LEGS}
        self.belly_z_m=-float(self.scene.world_broad_phase('base_link',self.scene.home_pose()).hull_vertices_world[:,2].min())

    def _verify_chain(self):
        for leg in LEGS:
            for part,axis in (('hip','1 0 0'),('upper_leg','0 1 0'),('lower_leg','0 1 0')):
                node=self.nodes[f'{leg}_{part}_joint']
                assert node.find('axis').get('xyz')==axis
                assert node.find('origin').get('rpy')=='0 0 0'
            for part in ('hip','upper_leg','lower_leg','foot'):
                geometry=self.scene.model.collision_geometry(f'{leg}_{part}_link')
                assert tuple(geometry.origin_xyz)==(0.,0.,0.) and tuple(geometry.origin_rpy)==(0.,0.,0.)
                assert self.nodes[f'{leg}_{part}_joint'].find('origin').get('rpy')=='0 0 0'
            assert tuple(self.scene.model.joints[leg+'_lower_leg_joint'].origin_xyz)==(0.,0.,-.09)
            upper_origin=self.scene.model.joints[leg+'_upper_leg_joint'].origin_xyz
            assert upper_origin[0]==upper_origin[2]==0.
        for pair in PAIRS.values():
            right,left=pair
            for part in ('hip','upper_leg','lower_leg','foot'):
                a=np.asarray(self.scene.model.joints[f'{right}_{part}_joint'].origin_xyz)
                b=np.asarray(self.scene.model.joints[f'{left}_{part}_joint'].origin_xyz)
                assert np.max(np.abs(a*np.array([1,-1,1])-b))<1e-12
        assert self.scene.model.joints['lf_hip_joint'].origin_xyz[2]-.020==self.scene.model.joints['rh_hip_joint'].origin_xyz[2]

    def frame(self,rear:PairProfile,front:PairProfile,hips=None):
        """Construct all 12 values atomically, then verify every individual limit."""
        profiles={'rear':rear,'front':front}; hips=self.neutral_hip_rad if hips is None else hips
        if set(hips)!=set(LEGS): raise ValueError('four observed/verified HIP values required')
        q={}
        for pair,legs in PAIRS.items():
            p=profiles[pair]
            for leg in legs:
                q.update({leg+'_hip_joint':hips[leg],leg+'_upper_leg_joint':p.upper_rad,leg+'_lower_leg_joint':p.lower_rad})
        if set(q)!=set(JOINTS): raise ValueError('incomplete atomic joint frame')
        for name,value in q.items():
            joint=self.scene.model.joints[name]
            if not math.isfinite(value) or not joint.lower_limit_rad<=value<=joint.upper_limit_rad:
                raise ValueError('individual joint limit: '+name)
        return q

    def mirror_residual(self,q):
        result={}
        reflection=np.diag([1.,-1.,1.])
        for pair,(right,left) in PAIRS.items():
            errors=[]
            for part in ('hip','upper_leg','lower_leg','foot'):
                a=self.scene.collision_transform(f'{right}_{part}_link',q)
                b=self.scene.collision_transform(f'{left}_{part}_link',q)
                errors.extend([float(np.max(np.abs(reflection@a.translation-b.translation))),
                               float(np.max(np.abs(reflection@a.rotation@reflection-b.rotation)))])
            result[pair]=max(errors)
        return result

    def keypoint(self,q,z,phase):
        # This is a flat-body keypoint. Inclined/body support inference is not
        # silently inserted; measured inclined starts require another assessment.
        ground={}
        transforms={}
        for link in self.scene.model.collision_link_names:
            tf=self.scene.collision_transform(link,q)
            points=self.scene.mesh(link).triangles_local.reshape(-1,3)@tf.rotation.T+tf.translation+[0,0,z]
            index=int(np.argmin(points[:,2]));tri,vertex=divmod(index,3)
            minimum=float(points[index,2])
            ground[link]={'minimum_z_m':minimum,'lowest_world_vertex_m':points[index].tolist(),
                          'triangle_index_0based':tri,'vertex_index_0based':vertex,
                          'mesh_ground_clear':minimum>=-GROUND_TOLERANCE_M,
                          'contact':'EXPECTED_BELLY_CONTACT' if link=='base_link' else ('GEOMETRIC_NEAR_PLANE' if abs(minimum)<=1e-5 else 'CLEAR' if minimum>0 else 'PENETRATION'),
                          'bearing_load':'UNVERIFIED'}
            transform=np.eye(4);transform[:3,:3]=tf.rotation;transform[:3,3]=tf.translation+[0,0,z]
            transforms[link]=transform.tolist()
        intersect,pair=self.scene.first_collision_at_pose(q)
        a,b,worst=self.scene.worst_pair_at_pose(q)
        residual=self.mirror_residual(q)
        violations=[{'kind':'GROUND_PENETRATION','link':link,**value} for link,value in ground.items() if not value['mesh_ground_clear']]
        if intersect: violations.append({'kind':'LINK_INTERSECTION','pair':pair})
        if max(residual.values())>1e-10: violations.append({'kind':'PAIR_FK_ASYMMETRY','residuals':residual})
        return {'phase':phase,'q_rad':q,'q_order':JOINTS,'base_z_m':z,'pitch_rad':0.,'roll_rad':0.,
            'provenance':'SYNTHETIC_KEYPOINT_FROM_CANONICAL_ANATOMICAL_CONSTRAINTS_NOT_HARDWARE_READBACK',
            'transforms_world':transforms,'ground':ground,'mirror_fk_max_residual':residual,
            'self_intersection':intersect,'worst_nonadjacent_pair':asdict(worst),
            'violations':violations,'geometry_pass':not violations,'motion_eligible':False}

    def synthetic_folded_keypoint(self):
        """One anatomical hypothesis; no prior failure trajectory is reused.

        Both UPPER pivot segments horizontal; each LOWER at its maximum URDF
        fold. Initial mirrored HIP per pair is the smallest scalar angle making
        both real collision meshes ground-clear. Not a observed rest pose.
        """
        profiles={pair:PairProfile(math.pi/2,max(self.scene.model.joints[f'{leg}_lower_leg_joint'].lower_limit_rad for leg in legs)) for pair,legs in PAIRS.items()}
        hips={leg:0. for leg in LEGS}
        for pair,(right,left) in PAIRS.items():
            hi=min(self.scene.model.joints[left+'_hip_joint'].upper_limit_rad,-self.scene.model.joints[right+'_hip_joint'].lower_limit_rad)
            allowed=[(0.,hi)]
            zero=self.frame(profiles['rear'],profiles['front'],{**hips,right:0.,left:0.})
            for leg,sign in ((right,-1.),(left,1.)):
                hip_origin=np.asarray(self.scene.model.joints[leg+'_hip_joint'].origin_xyz)
                for part in ('hip','upper_leg','lower_leg','foot'):
                    link=leg+'_'+part+'_link';tf=self.scene.collision_transform(link,zero)
                    vertices=self.scene.mesh(link).triangles_local.reshape(-1,3)@tf.rotation.T+tf.translation-hip_origin
                    for p in vertices:
                        arcs=trig_intervals(float(p[2]),float(sign*p[1]),float(hip_origin[2]+self.belly_z_m+GROUND_TOLERANCE_M),0.,hi)
                        allowed=intersect_intervals(allowed,arcs)
                        if not allowed:raise ValueError('synthetic folded C0 has no paired HIP clearance root')
            h=min(a for a,b in allowed)
            if h>0:h=math.nextafter(h,math.inf)
            hips[right]=-h;hips[left]=h
        return self.frame(profiles['rear'],profiles['front'],hips)

    def upper_feasibility_certificate(self,leg,lower_rad):
        """Exact interval intersection of per-vertex trigonometric inequalities.

        For HIP=0, level belly contact and fixed LOWER, vertex height is
        c + a*cos(UPPER) + b*sin(UPPER). This proves an empty interval without
        sampling joint poses or generating alternative stand candidates.
        """
        j=self.scene.model.joints[leg+'_upper_leg_joint'];lo,hi=j.lower_limit_rad,j.upper_limit_rad
        c=self.scene.model.joints[leg+'_hip_joint'].origin_xyz[2]+self.belly_z_m+GROUND_TOLERANCE_M
        interval=[(lo,hi)]; changes=[]; upper_domain=None; witnesses=[]; tested=0
        for part in ('upper_leg','lower_leg','foot'):
            link=leg+'_'+part+'_link';vertices=self.scene.mesh(link).triangles_local.reshape(-1,3)
            if part=='upper_leg':w=vertices
            else:
                offset=np.asarray(self.scene.model.joints[leg+'_foot_joint'].origin_xyz) if part=='foot' else np.zeros(3)
                w=(vertices+offset)@r_y(lower_rad).T+self.scene.model.joints[leg+'_lower_leg_joint'].origin_xyz
            for index,p in enumerate(w):
                tested+=1;allowed=trig_intervals(float(p[2]),float(-p[0]),c,lo,hi)
                intersect=intersect_intervals(interval,allowed)
                if intersect!=interval:
                    changes.append({'link':link,'triangle_index_0based':index//3,'vertex_index_0based':index%3,
                        'local_vertex_m':vertices[index].tolist(),'height_coefficients_m':{'c_includes_tolerance':c,'a_cos':float(p[2]),'b_sin':float(-p[0])},
                        'interval_before_rad':interval,'vertex_allowed_rad':allowed,'interval_after_rad':intersect})
                if upper_domain is not None and part=='lower_leg':
                    # A single lower vertex that excludes even the upper-only
                    # domain proves a necessary minimum LOWER opening change.
                    only=intersect_intervals(upper_domain,allowed)
                    local=vertices[index]
                    if not only and local[0]>0 and local[2]<0:
                        critical=math.atan2(local[0],-local[2])
                        limit=critical-hi
                        if min(a for a,b in upper_domain)+lower_rad>-math.pi/2 and limit>lower_rad:
                            witnesses.append({'link':link,'local_vertex_m':local.tolist(),'upper_only_domain_rad':upper_domain,
                              'derivative_equation':'dZ/dLOWER=-vx*cos(UPPER+LOWER)-vz*sin(UPPER+LOWER)',
                              'lower_opening_still_infeasible_through_rad':limit,
                              'necessary_lower_change_rad_exceeds':limit-lower_rad,
                              'bound_is_necessary_not_sufficient_for_alignment':True})
                interval=intersect
                if not interval:break
            if part=='upper_leg':upper_domain=interval.copy()
            if not interval:break
        return {'leg':leg,'hip_neutral_rad':0.,'body_z_m':self.belly_z_m,'pitch_rad':0.,'roll_rad':0.,
           'fixed_lower_rad':lower_rad,'upper_joint_range_rad':[lo,hi],'vertices_processed_until_certificate':tested,
           'constraints_changing_interval':changes,'feasible_upper_intervals_rad':interval,
           'continuous_upper_domain_proved_empty':not interval,'minimum_opening_necessary_bounds':witnesses,
           'not_a_general_impossibility_proof_for_all_rest_poses':True}

    def prealign(self,observation:Observation):
        reject=lambda reason: {'state':'REJECTED','reason':reason,'actuator_targets':[],'trajectory_frames':[],
                              'motion_eligible':False,'hardware_authorized':False}
        if not observation.coherent or not observation.fresh or set(observation.q_rad)!=set(JOINTS):return reject('OBSERVATION_REQUIRED')
        if not all(math.isfinite(v) for v in (*observation.q_rad.values(),observation.base_z_m,observation.roll_rad,observation.pitch_rad)):
            return reject('INVALID_OBSERVATION')
        if any(not self.scene.model.joints[name].lower_limit_rad<=value<=self.scene.model.joints[name].upper_limit_rad for name,value in observation.q_rad.items()):
            return reject('OBSERVED_JOINT_LIMIT')
        if observation.provenance not in ('SYNTHETIC','EXTERNAL_MEASURED'):return reject('PROVENANCE_REQUIRED')
        if observation.provenance=='EXTERNAL_MEASURED' and not observation.installed_mapping_verified:return reject('INSTALLED_NEUTRAL_MAPPING_UNVERIFIED')
        if observation.provenance=='EXTERNAL_MEASURED' and not observation.after_torque_loss_reobserved:return reject('TORQUE_LOSS_REOBSERVATION_REQUIRED')
        if not observation.belly_contact_observed:return reject('BELLY_CONTACT_WITNESS_REQUIRED')
        if abs(observation.roll_rad)>1e-10 or abs(observation.pitch_rad)>1e-10:return reject('NONLEVEL_START_REQUIRES_MEASURED_CONTACT_ASSESSMENT')
        if abs(observation.base_z_m-self.belly_z_m)>1e-9:return reject('BELLY_PLANE_MODEL_MISMATCH')
        q=observation.q_rad
        profiles={}
        for pair,(a,b) in PAIRS.items():
            if any(abs(q[a+'_'+part+'_joint']-q[b+'_'+part+'_joint'])>1e-10 for part in ('upper_leg','lower_leg')):
                return reject('OBSERVED_PAIR_ALIGNMENT_ADAPTATION_REQUIRED')
            profiles[pair]=PairProfile(q[a+'_upper_leg_joint'],q[a+'_lower_leg_joint'])
        try:
            target=self.frame(profiles['rear'],profiles['front'])
        except ValueError:return reject('INDIVIDUAL_JOINT_LIMIT')
        key=self.keypoint(target,observation.base_z_m,'PRE_ALIGN_NEUTRAL_TENTATIVE_KEYPOINT')
        certificates=[self.upper_feasibility_certificate(leg,profiles['rear'].lower_rad) for leg in PAIRS['rear']]
        if not key['geometry_pass']:
            result=reject('NEUTRAL_PREALIGN_KEYPOINT_INFEASIBLE')
            result.update({'tentative_keypoint':key,'rear_upper_domain_certificates':certificates,
                           'following_phases':'REAR_DEPLOY / FRONT_DEPLOY / LOAD_TRANSFER / RISE / WALK NOT GENERATED'})
            return result
        # Keypoint alone is not a swept HIP alignment / loaded contact proof.
        result=reject('PREALIGN_SWEPT_AND_LOAD_EVIDENCE_REQUIRED')
        result.update({'tentative_keypoint':key,'rear_upper_domain_certificates':certificates})
        return result


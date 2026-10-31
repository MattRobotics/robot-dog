#!/usr/bin/env python3
"""Independent XML/STL/FK regressions of saved proofs, no search or actuation."""
import hashlib,json,math,os,unittest,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from paired_prone_model import ROOT,URDF,JOINTS,LEGS
from paired_acquisition import Acquisition,OUT
from continuous_pair_path import Continuous
from core_python import origin

class FrozenGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a=Acquisition();cls.c=Continuous();cls.cert=json.loads((OUT/'continuous_pair_path.json').read_text());cls.walk=json.loads((OUT/'walk_after_stand.json').read_text());cls.keys=json.loads((OUT/'acquisition_keypoints.json').read_text())['waypoints'];cls.xml=ET.parse(URDF).getroot();cls.joints=cls.xml.findall('joint');cls.mesh={}
        for link in cls.xml.findall('link'):
            mesh=link.find('collision/geometry/mesh');filename=mesh.get('filename');path=URDF.parent/filename;scale=np.array([float(v) for v in mesh.get('scale','1 1 1').split()]);raw=path.read_bytes();n=int.from_bytes(raw[80:84],'little');assert len(raw)==84+50*n
            dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')]);vertices=np.frombuffer(raw[84:],dtype=dtype,count=n)['vertices'].astype(float).reshape(-1,3)*scale;cls.mesh[link.get('name')]=vertices
    def manual_fk(self,q,p=0,x=0,z=0):
        R=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]]);root=np.eye(4);root[:3,:3]=R;root[:3,3]=[x,0,z];links={'base_link':root}
        for j in self.joints:
            axis=np.array([float(v) for v in j.find('axis').get('xyz').split()]) if j.get('type')=='revolute' else np.zeros(3);angle=q.get(j.get('name'),0);skew=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]]);rot=np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)
            tf=np.eye(4);tf[:3,:3]=rot;tf[:3,3]=[float(v) for v in j.find('origin').get('xyz').split()];self.assertEqual(j.find('origin').get('rpy'),'0 0 0');links[j.find('child').get('link')]=links[j.find('parent').get('link')]@tf
        return links
    def test_01_geometry_identity_and_17_full_meshes(self):
        self.assertEqual(len(self.mesh),17);self.assertEqual(sum(len(v)//3 for v in self.mesh.values()),119696);self.assertEqual(hashlib.sha256(URDF.read_bytes()).hexdigest(),'3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59')
        pins=json.loads((OUT/'export_manifest.json').read_text())['source_pins']
        for p,h in pins.items():self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(),h)
    def test_02_synthetic_origin_and_individual_limits(self):
        self.assertFalse(self.cert['motion_eligible']);self.assertTrue(self.cert['synthetic_initial_pose']);self.assertFalse(self.cert['load_distribution_proven']);self.assertEqual(self.cert['MOTION_AUTHORIZED'],0)
        for node in self.cert['nodes']:
            q=self.a.q(node['parameters'])
            for j in self.joints:
                if j.get('type')=='revolute':
                    limit=j.find('limit');self.assertGreaterEqual(q[j.get('name')],float(limit.get('lower'))-1e-12);self.assertLessEqual(q[j.get('name')],float(limit.get('upper'))+1e-12)
    def test_03_full_vertex_plane_extrema_match_hull_and_cpp_fk(self):
        cases=[w['parameters'] for w in self.keys]
        cases.extend({k:(s['start_parameters'][k]+s['end_parameters'][k])/2 for k in s['start_parameters']} for s in self.cert['segments'][::16])
        for v in cases:
            q=self.a.q(v);lift,gaps=self.c.corrected(v);links=self.manual_fk(q,v['p'],v['x'],v['z']+lift)
            for n,vertices in self.mesh.items():
                tf=links[n];points=vertices@tf[:3,:3].T+tf[:3,3];self.assertGreaterEqual(points[:,2].min(),-1e-9);self.assertAlmostEqual(points[:,2].min(),gaps[n],places=12)
            base=self.manual_fk(q)
            for i,l in enumerate(LEGS):
                t,r=origin(i,q[l+'_upper_leg_joint'],q[l+'_lower_leg_joint'],q[l+'_hip_joint']);np.testing.assert_allclose(t,base[l+'_foot_link'][:3,3],atol=1e-12);np.testing.assert_allclose(r,base[l+'_foot_link'][:3,:3],atol=1e-12)
    def test_04_true_fk_mirror_and_hip_fixed_after_pre_align(self):
        mirror=np.diag([1,-1,1]);started=False
        for node in self.cert['nodes']:
            v=node['parameters'];q=self.a.q(v);links=self.manual_fk(q)
            for right,left in (('rh','lh'),('rf','lf')):
                for part in ('hip','upper_leg','lower_leg','foot'):
                    a=links[right+'_'+part+'_link'];b=links[left+'_'+part+'_link'];np.testing.assert_allclose(mirror@a[:3,3],b[:3,3],atol=1e-12);np.testing.assert_allclose(mirror@a[:3,:3]@mirror,b[:3,:3],atol=1e-12)
            if node['phase']!='PRE_ALIGN':started=True
            if started:self.assertTrue(all(abs(q[l+'_hip_joint'])<1e-14 for l in LEGS))
    def test_05_continuous_seams_final_height_and_prerequisites(self):
        self.assertEqual(len(self.cert['segments']),251);self.assertEqual(self.cert['interval_evaluations'],315545)
        for a,b in zip(self.cert['segments'],self.cert['segments'][1:]):self.assertEqual(a['end_parameters'],b['start_parameters'])
        for s in self.cert['segments']:
            self.assertEqual(s['swept_certificate']['pairs'],120);self.assertGreater(s['swept_certificate']['minimum_swept_separation_lower_bound_m'],0);self.assertLessEqual(s['ground_certificate']['maximum_contact_gap_upper_bound_m'],1e-5)
        self.assertAlmostEqual(self.cert['nodes'][-1]['parameters']['z'],.15,places=12)
        # Deliberately no promotion of a positive bound into the 3mm policy.
        self.assertLess(min(s['swept_certificate']['minimum_swept_separation_lower_bound_m'] for s in self.cert['segments']),.003)
    def test_06_walk_entry_stop_and_rearm_geometry(self):
        self.assertTrue(self.walk['sampled_geometry_pass']);self.assertEqual(len(self.walk['samples']),214);self.assertLess(self.walk['stand_entry_mesh_reference_alignment_delta_rad'],1e-10);self.assertEqual(self.walk['samples'][-1]['phase'],'SIMULATED_REARM_STAND');self.assertFalse(self.walk['rearm_is_authority_grant']);self.assertFalse(self.walk['loaded_tracking_proven'])
        for f in self.walk['samples']:
            self.assertFalse(f['self_intersection']);self.assertGreater(f['assumed_support_polygon_margin_m'],0)
            for g in f['ground'].values():self.assertGreaterEqual(g['minimum_z_m'],-1e-9)
        q0=self.walk['samples'][-2]['q_rad'];q1=self.walk['samples'][-1]['q_rad'];self.assertLess(max(abs(q0[j]-q1[j]) for j in JOINTS),1e-10)
    def test_07_intermediate_suffix_and_unknown_contact_not_promoted(self):
        manifest=json.loads((OUT/'export_manifest.json').read_text());i=manifest['intermediate_restart_source_segment'];self.assertAlmostEqual(self.cert['segments'][i]['start_parameters']['z'],.05,places=12);self.assertTrue(manifest['intermediate_restart_certificate_reused'])
        trace=[json.loads(l) for l in (OUT/'paired_motion_trace.jsonl').read_text().splitlines()];self.assertTrue(all('SYNTHETIC' in f['evidence'] for f in trace));self.assertTrue(any(f['stand_state']=='INTERMEDIATE_RESTART_50MM' for f in trace));self.assertTrue(all(p['state']=='UNKNOWN' for f in trace if f['stand_state']=='PRE_ALIGN' for p in f['telemetry']['feet'].values()))
        self.assertTrue(all(f['telemetry']['base_position_m'][2]==f['telemetry']['base_height_m'] for f in trace));self.assertTrue(all(a['now_ms']<=b['now_ms'] for a,b in zip(trace,trace[1:])))
if __name__=='__main__':unittest.main(verbosity=2)

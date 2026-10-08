"""Targeted proof-tool checks; no serial, GPIO or servo backend."""
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import matdog_startup_geometry_v5 as startup
from matdog_geometry_mesh_kernel import triangle_triangle_distance, MeshKernelError


def separation(value,kind='LOWER_BOUND',status='SEPARATED_AABB'):
    return SimpleNamespace(clearance_m=value,clearance_kind=kind,status=status)


class FakeScene:
    def __init__(self,result):
        self.result=result
        self.model=SimpleNamespace(
            relevant_actuated_joints_for_pair=lambda *p:('joint',),
            relevant_actuated_joints_for_link=lambda l:('joint',) if l=='a' else (),
            joint_chain_by_link={'a':('joint',),'b':()},
            collision_geometry=lambda l:SimpleNamespace(origin_xyz=(0.,0.,0.)))
    def mesh(self,l):return SimpleNamespace(hull_vertices_local=np.array([[1.,0.,0.]]))
    def full_pose(self,p):return p
    def check_link_pair(self,*args,**kwargs):return self.result


class ProofTests(unittest.TestCase):
    def test_triangle_bounds_never_exceed_exact_distance(self):
        rng=np.random.default_rng(20261003)
        a=rng.normal(size=(400,3,3))*.04
        b=rng.normal(size=(400,3,3))*.04+rng.normal(size=(400,1,3))*.15
        lower=startup.triangle_separation_bounds(a,b)
        for index,bound in enumerate(lower):
            exact=triangle_triangle_distance(a[index],b[index])
            self.assertGreaterEqual(bound,0.)
            self.assertLessEqual(bound,exact+1e-10,(index,bound,exact))
    def test_parallel_and_coplanar_separation(self):
        a=np.array([[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]]*3)
        b=a+np.array([[[0.,0.,.005]],[[2.,0.,0.]],[[0.,0.,0.]]])
        bound=startup.triangle_separation_bounds(a,b)
        self.assertAlmostEqual(bound[0],.005-1e-9)
        self.assertAlmostEqual(bound[1],1.-1e-9)
        self.assertEqual(bound[2],0.)
    def test_complete_box_uses_motion_bound(self):
        checker=startup.ContinuousCheck(FakeScene(separation(.02)))
        result=checker.check(('a','b'),{'joint':(-10,10)})
        self.assertEqual(result['status'],'PASS_CONTINUOUS_BOX')
        self.assertAlmostEqual(checker.minimum_certified,.02-10*startup.TICK)
    def test_lower_bound_below_policy_is_not_called_collision(self):
        checker=startup.ContinuousCheck(FakeScene(separation(.001)))
        with patch.object(startup,'mesh_separation_bound',return_value=separation(.001)):
            result=checker.check(('a','b'),{'joint':(-10,10)},max_boxes=2)
        self.assertEqual(result['status'],'UNRESOLVED')
    def test_exact_unsafe_clearance_and_collision_are_rejected(self):
        for result,status in [(separation(.002,'EXACT'),'CLEARANCE_BELOW_POLICY'),
                              (separation(None,status='INTERSECTING'),'COLLISION')]:
            checker=startup.ContinuousCheck(FakeScene(result))
            with patch.object(startup,'mesh_separation_bound',return_value=result):
                self.assertEqual(checker.check(('a','b'),{'joint':(-10,10)})['status'],status)
    def test_kernel_resource_failure_is_fail_closed(self):
        checker=startup.ContinuousCheck(FakeScene(separation(.001)))
        with patch.object(startup,'mesh_separation_bound',side_effect=MeshKernelError('resource cap')):
            result=checker.check(('a','b'),{'joint':(-10,10)},max_boxes=2)
        self.assertEqual(result['status'],'UNRESOLVED')
    def test_observed_positions_and_compiled_direction(self):
        profile=(Path(startup.URDF).parents[3]/'05_Firmware/MATDOG_Controller/src/actuator/CalibrationGeometryProfileData.h').read_text()
        self.assertIn('"NEW03"}, 21, 1, EncoderDirectionSource::HISTORICAL_SLOT_UNCHANGED',profile)
        self.assertEqual((2348-1997)*1,351)
        self.assertEqual((1080-2106)*-1,1026)
        self.assertEqual((1665-2058)*-1,393)
        for stage,intervals in startup.STAGES:
            for joint,tick in [('rf_lower_leg_joint',351),('rf_upper_leg_joint',1026),('rh_upper_leg_joint',393)]:
                if stage=='RF_LOWER':
                    lo,hi=intervals[joint];self.assertTrue(lo<=tick<=hi)


if __name__=='__main__':unittest.main()

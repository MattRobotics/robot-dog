"""Deterministic analytical fixtures; invented numbers are test fixtures only."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from model import (Body, PENDING, ROOT, URDF_REL, aggregate, at_pose, body_dict,
                   c4e, fk, load_baseline, matvec, principal_moments, project_support,
                   resolve_budget, rotate_tensor, rotation, sensitivity, validate_tensor)
from sources import source_audit
from contract import candidate_payload, canonical_json, string_field

I = ((2., .2, .1), (.2, 3., -.3), (.1, -.3, 4.))
Z = ((0., 0., 0.), (0., 0., 0.), (0., 0., 0.))


class TensorTests(unittest.TestCase):
    def assertVec(self, a, b, places=12):
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, places=places)

    def assertMat(self, a, b, places=12):
        for x, y in zip(a, b):
            self.assertVec(x, y, places)

    def test_two_point_masses_parallel_axis(self):
        t = aggregate([Body("a", 1., (-1., 0., 0.), Z), Body("b", 3., (1., 0., 0.), Z)])
        self.assertEqual(t.mass, 4.)
        self.assertVec(t.com, (.5, 0., 0.))
        self.assertMat(t.inertia, ((0.,0.,0.),(0.,3.,0.),(0.,0.,3.)))

    def test_off_diagonal_parallel_axis_sign(self):
        t = aggregate([Body("a", 1., (-1., -2., 0.), Z), Body("b", 1., (1., 2., 0.), Z)])
        self.assertMat(t.inertia, ((8.,-4.,0.),(-4.,2.,0.),(0.,0.,10.)))

    def test_rotation_90_degrees(self):
        t = rotate_tensor(((2.,0.,0.),(0.,3.,0.),(0.,0.,4.)), rotation((0.,0.,math.pi/2)))
        self.assertMat(t, ((3.,0.,0.),(0.,2.,0.),(0.,0.,4.)))

    def test_general_rotation_preserves_eigenvalues(self):
        self.assertVec(principal_moments(I), principal_moments(rotate_tensor(I, rotation((.37,-.59,1.18)))))

    def test_aggregation_is_rigid_transform_covariant(self):
        bodies = [Body("a",2.,(.7,-.2,.4),I), Body("b",3.,(-.3,.8,-.6),I)]
        total = aggregate(bodies)
        r, t = rotation((.31,-.73,1.04)), (.3,.7,-.2)
        moved = [Body(b.name,b.mass,tuple(x+y for x,y in zip(matvec(r,b.com),t)),
                      rotate_tensor(b.inertia,r)) for b in bodies]
        result = aggregate(moved)
        self.assertVec(result.com, tuple(x+y for x,y in zip(matvec(r,total.com),t)))
        self.assertMat(result.inertia, rotate_tensor(total.inertia,r))

    def test_lumping_associativity(self):
        a,b,c = Body("a",1.,(1.,2.,3.),I), Body("b",2.,(-1.,2.,0.),I), Body("c",3.,(0.,-2.,1.),I)
        direct, lumped = aggregate([a,b,c]), aggregate([aggregate([a,b]),c])
        self.assertVec(direct.com,lumped.com)
        self.assertMat(direct.inertia,lumped.inertia)

    def test_nonphysical_rejected(self):
        for tensor in [((1.,0.,0.),(0.,1.,0.),(0.,0.,3.)),
                       ((-1.,0.,0.),(0.,1.,0.),(0.,0.,1.)),
                       ((1.,.1,0.),(.2,1.,0.),(0.,0.,1.)),
                       ((float("nan"),0.,0.),(0.,1.,0.),(0.,0.,1.))]:
            with self.subTest(tensor=tensor), self.assertRaises(ValueError):
                validate_tensor(tensor)

    def test_tiny_nonphysical_rejected(self):
        with self.assertRaises(ValueError):
            validate_tensor(((-1e-20,0.,0.),(0.,1e-20,0.),(0.,0.,1e-20)))

    def test_semidefinite_point_mass_accepted_explicit_fixture(self):
        self.assertFalse(validate_tensor(Z)["positive_definite"])

    def test_bad_mass_and_shapes_rejected(self):
        for mass in [0, -1, True, float("inf"), "1"]:
            with self.subTest(mass=mass), self.assertRaises(ValueError):
                Body("a",mass,(0.,0.,0.),I)
        with self.assertRaises(ValueError):
            Body("a",1.,(0.,0.),I)

    def test_duplicate_empty_aggregate_rejected(self):
        b = Body("a",1.,(0.,0.,0.),I)
        for bodies in ([], [b,b]):
            with self.assertRaises(ValueError):
                aggregate(bodies)


class BaselineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.links = load_baseline()
        cls.pose = {n: 0. for n,j in fk.load_urdf_joints(ROOT/URDF_REL).items() if j.joint_type != "fixed"}

    def test_excel_urdf_agreement_and_symmetry(self):
        audit = source_audit()
        self.assertEqual(len(audit["links"]),17)
        self.assertEqual(sum(r["historical_evidence"] == "DIRECT CAD" for r in audit["links"]),5)
        self.assertEqual(len(audit["local_symmetry"]),12)

    def test_historical_mass_and_pose_specific_com(self):
        total = at_pose(self.links,self.pose)
        self.assertAlmostEqual(total.mass,2.48,places=12)
        # Independently sum q=0 translations, using the fixed tree's explicit offsets.
        c = [1.282*v for v in self.links["base_link"].com]
        for leg in ("lf","rf","rh","lh"):
            x = .1125 if leg in ("lf","rf") else -.1125
            y = .0475 if leg in ("lf","lh") else -.0475
            z = .0465 if leg in ("lf","rf") else .0265
            sy = 1 if y > 0 else -1
            for family,offset in [("hip",(x,y,z)),("upper_leg",(x,y+sy*.048,z)),
                                  ("lower_leg",(x,y+sy*.048,z-.09)),
                                  ("foot",(x+.107,y+sy*(.048-.0015),z-.09-.0499))]:
                b = self.links[f"{leg}_{family}_link"]
                for k in range(3): c[k] += b.mass*(b.com[k]+offset[k])
        for k in range(3): self.assertAlmostEqual(total.com[k],c[k]/2.48,places=14)

    def test_pose_required_missing_extra_limit_nonfinite(self):
        for p in [{}, {**self.pose,"unknown":0.}, {**self.pose,"lf_hip_joint":2.},
                  {**self.pose,"lf_hip_joint":float("nan")}]:
            with self.assertRaises(ValueError): at_pose(self.links,p)

    def test_urdf_inertial_rpy_not_ignored(self):
        import xml.etree.ElementTree as ET
        tree = ET.parse(ROOT/URDF_REL)
        node = tree.getroot().find("link/inertial/origin")
        node.set("rpy","0 0 1.5707963267948966")
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/"fixture.urdf"
            tree.write(p)
            rotated = load_baseline(p)["base_link"]
        expected = rotate_tensor(self.links["base_link"].inertia, rotation((0.,0.,math.pi/2)))
        for row1,row2 in zip(rotated.inertia,expected):
            for a,b in zip(row1,row2): self.assertAlmostEqual(a,b,places=14)

    def test_joint_origin_rotation_precedes_axis_motion(self):
        # Synthetic tree: Rz(90) Rx(90), two unit masses, no MATDOG measurements.
        xml = '''<robot name="fixture">
        <link name="base_link"><inertial><origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="1"/><inertia ixx="2" ixy="0" ixz="0" iyy="3" iyz="0" izz="4"/></inertial></link>
        <link name="child"><inertial><origin xyz="1 0 0" rpy="0 0 0"/>
        <mass value="1"/><inertia ixx="2" ixy="0" ixz="0" iyy="3" iyz="0" izz="4"/></inertial></link>
        <joint name="joint" type="continuous"><parent link="base_link"/><child link="child"/>
        <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><axis xyz="1 0 0"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"fixture.urdf"; path.write_text(xml)
            total=at_pose(load_baseline(path),{"joint":math.pi/2},path)
        for actual,expected in zip(total.com,(.5,1.5,1.5)):
            self.assertAlmostEqual(actual,expected,places=12)
        expected=((15.,-1.5,-1.5),(-1.5,10.,-4.5),(-1.5,-4.5,12.))
        for actual,row in zip(total.inertia,expected):
            for a,b in zip(actual,row): self.assertAlmostEqual(a,b,places=12)


class BudgetTests(unittest.TestCase):
    def setUp(self):
        self.links = load_baseline()
        self.budget = json.loads((HERE/"data/mass_budget_rev01.json").read_text())

    def complete_empty(self):
        b = copy.deepcopy(self.budget)
        b["inventory_review"] = "SYNTHETIC TEST ONLY, complete inventory fixture"
        b["assembly_configuration_description"] = "SYNTHETIC TEST ONLY, retained CAD fixture"
        for r in b["link_reviews"]:
            r.update(decision="retain_historical_offline",evidence="synthetic test")
        for r in b["components"]:
            r.update(disposition="absent",evidence="synthetic test absence")
        return b

    def addon(self,b,disposition="add"):
        b["components"][0].update(disposition=disposition,parent_link="base_link",mass_kg=.2,
                                  com_in_parent_m=[.3,-.1,.2],inertia_at_com_component_kg_m2=I,
                                  component_rpy_in_parent_rad=[.2,.3,.4],evidence="synthetic fixture",
                                  coverage_note="SYNTHETIC disjoint addition or full replacement")

    def test_pending_blocks_total_and_never_treats_missing_as_zero(self):
        r = resolve_budget(self.links,self.budget)
        self.assertFalse(r["complete"])
        self.assertIsNone(r["result"])
        self.assertIsNone(r["links"])
        self.assertGreater(len(r["pending_fields"]),70)

    def test_single_missing_inertia_blocks_output(self):
        b=self.complete_empty(); self.addon(b)
        b["components"][0]["inertia_at_com_component_kg_m2"]=PENDING
        self.assertFalse(resolve_budget(self.links,b)["complete"])

    def test_still_pending_even_when_numeric_complete(self):
        r=resolve_budget(self.links,self.complete_empty())
        self.assertTrue(r["complete"])
        self.assertEqual(r["readiness"],"REV01_PENDING")
        self.assertFalse(r["hardware_verified"])
        self.assertFalse(r["command_eligible"])

    def test_addition_lumped_to_parent_17_links(self):
        b=self.complete_empty(); self.addon(b)
        r=resolve_budget(self.links,b)
        self.assertEqual(len(r["links"]),17)
        self.assertAlmostEqual(sum(v.mass for v in r["links"].values()),2.68,places=12)
        self.assertNotEqual(r["links"]["base_link"].com,self.links["base_link"].com)
        for body in r["links"].values():
            for i in range(3):
                for j in range(3): self.assertEqual(body.inertia[i][j],body.inertia[j][i])

    def test_complete_link_replacement_no_double_count(self):
        b=self.complete_empty(); self.addon(b,"replace_link")
        b["link_reviews"][0]["decision"]="replace_complete_link"
        r=resolve_budget(self.links,b)
        self.assertAlmostEqual(sum(v.mass for v in r["links"].values()),2.48-1.282+.2,places=12)

    def test_replacement_requires_matching_review(self):
        b=self.complete_empty(); self.addon(b,"replace_link")
        with self.assertRaises(ValueError): resolve_budget(self.links,b)

    def test_duplicate_replacement_rejected(self):
        b=self.complete_empty(); self.addon(b,"replace_link")
        b["link_reviews"][0]["decision"]="replace_complete_link"
        b["components"][1] = {**b["components"][0], "id":"different"}
        with self.assertRaises(ValueError): resolve_budget(self.links,b)

    def test_invalid_revision_units_parent_mass_tensor(self):
        cases=[]
        b=self.complete_empty(); b["readiness"]="REV01_VERIFIED"; cases.append(b)
        b=self.complete_empty(); b["units"]["length"]="mm"; cases.append(b)
        b=self.complete_empty(); self.addon(b); b["components"][0]["parent_link"]="neck_link"; cases.append(b)
        b=self.complete_empty(); self.addon(b); b["components"][0]["mass_kg"]=-.2; cases.append(b)
        b=self.complete_empty(); self.addon(b); b["components"][0]["inertia_at_com_component_kg_m2"]=[[1,0,0],[0,1,0],[0,0,3]]; cases.append(b)
        for b in cases:
            with self.subTest(b=b), self.assertRaises(ValueError): resolve_budget(self.links,b)

    def test_duplicate_id_and_review_coverage_rejected(self):
        b=self.complete_empty(); b["components"][1]["id"]=b["components"][0]["id"]
        with self.assertRaises(ValueError): resolve_budget(self.links,b)
        b=self.complete_empty(); b["link_reviews"].pop()
        with self.assertRaises(ValueError): resolve_budget(self.links,b)

    def test_absent_and_included_require_evidence(self):
        b=self.complete_empty(); b["components"][0]["evidence"]=PENDING
        self.assertFalse(resolve_budget(self.links,b)["complete"])
        b=self.complete_empty(); b["components"][0].update(disposition="included_in_link",parent_link="base_link",coverage_note=PENDING)
        self.assertFalse(resolve_budget(self.links,b)["complete"])


class SupportTests(unittest.TestCase):
    square=[(-1.,-1.,0.),(1.,-1.,0.),(1.,1.,0.),(-1.,1.,0.)]

    def project(self,com=(.2,.3,2.),contacts=None,**kw):
        args=dict(com_base=com,translation_world=(0.,0.,0.),rpy_world=(0.,0.,0.),
                  contacts_world=self.square if contacts is None else contacts,
                  plane_origin_world=(0.,0.,0.),plane_normal_world=(0.,0.,1.),
                  gravity_world=(0.,0.,-1.),plane_tolerance_m=1e-10)
        args.update(kw)
        return project_support(**args)

    def test_horizontal_matches_c4e(self):
        p=self.project()
        self.assertEqual(p["projected_com_world_m"],(.2,.3,0.))
        m=c4e._polygon_signed_margin((.2,.3),c4e._convex_hull([(v[0],v[1]) for v in self.square]))
        self.assertAlmostEqual(p["signed_margin_m"],m,places=14)

    def test_outside_signed_negative(self):
        self.assertLess(self.project(com=(2.,0.,1.))["signed_margin_m"],0.)

    def test_tilted_projection_along_gravity_not_normal(self):
        # Plane z=x/2, so vertical gravity leaves x and y unchanged.
        contacts=[(x,y,.5*x) for x,y,_ in self.square]
        p=self.project(contacts=contacts,plane_normal_world=(-.5,0.,1.))
        for a,b in zip(p["projected_com_world_m"],(.2,.3,.1)):
            self.assertAlmostEqual(a,b,places=14)

    def test_rotated_body_and_translation(self):
        p=self.project(com=(.2,0.,1.),rpy_world=(0.,0.,math.pi/2),translation_world=(.1,.2,0.))
        for a,b in zip(p["projected_com_world_m"],(.1,.4,0.)):
            self.assertAlmostEqual(a,b,places=14)

    def test_degeneracy_parallel_gravity_and_nonplanar(self):
        for kw in ({"contacts":[(0.,0.,0.),(1.,0.,0.),(2.,0.,0.)]},
                   {"contacts":self.square[:2]}, {"gravity_world":(1.,0.,0.)},
                   {"contacts":[*self.square[:3],(0.,0.,1.)]},
                   {"plane_normal_world":(0.,0.,0.)}):
            with self.subTest(kw=kw), self.assertRaises(ValueError): self.project(**kw)

    def test_geometric_pass_never_hardware_eligible(self):
        p=self.project()
        self.assertTrue(p["geometrically_inside"])
        self.assertFalse(p["command_eligible"])


class SensitivityContractTests(unittest.TestCase):
    def test_sensitivity_exact_and_finite_difference(self):
        m,c,dm,r=2.,(.1,-.2,.3),.4,(.8,.4,-.1)
        s=sensitivity(m,c,dm,r)
        total=aggregate([Body("a",m,c,I),Body("b",dm,r,I)])
        for k in range(3): self.assertAlmostEqual(total.com[k]-c[k],s["delta_com_m"][k],places=14)
        eps=1e-6
        plus=sensitivity(m,c,dm+eps,r); minus=sensitivity(m,c,dm-eps,r)
        for k in range(3):
            self.assertAlmostEqual((plus["delta_com_m"][k]-minus["delta_com_m"][k])/(2*eps),
                                   s["d_com_d_mass_m_per_kg"][k],places=9)

    def test_sensitivity_mass_at_com_no_shift_and_zero(self):
        self.assertEqual(sensitivity(2.,(.1,.2,.3),.5,(.1,.2,.3))["delta_com_m"],(0.,0.,0.))
        self.assertEqual(sensitivity(2.,(.1,.2,.3),0.,(1.,2.,3.))["delta_com_m"],(0.,0.,0.))
        with self.assertRaises(ValueError): sensitivity(2.,(0.,0.,0.),-.1,(1.,1.,1.))

    def test_contract_hash_and_readiness_and_string_capacities(self):
        links=load_baseline()
        pose={n:0. for n,j in fk.load_urdf_joints(ROOT/URDF_REL).items() if j.joint_type!="fixed"}
        out=candidate_payload(links,pose,"REV00-historical","offline test","HISTORICAL_REV00")
        self.assertEqual(out["payload"]["sourceSha256"],hashlib.sha256(canonical_json(out["hash_preimage"]).encode()).hexdigest())
        self.assertEqual(out["payload"]["linkCount"],17)
        self.assertFalse(out["runtime_eligible"])
        with self.assertRaises(ValueError): candidate_payload(links,pose,"REV01","test","REV01_VERIFIED")
        self.assertEqual(string_field("a"*31,32),"a"*31)
        for value in ("a"*32,"é"*16,"a\0b"):
            with self.assertRaises(ValueError): string_field(value,32)


if __name__ == "__main__":
    unittest.main()

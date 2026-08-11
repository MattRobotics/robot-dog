#!/usr/bin/env python3
"""Offline G5 tests for the additive, model-driven V5 core."""

from __future__ import annotations

import ast
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = CALIBRATION_DIR.parents[2]
URDF_PATH = REPO_ROOT / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
COLLISION_DIR = URDF_PATH.parent / "meshes/collision"

import sys

for path in (CALIBRATION_DIR, CALIBRATION_DIR.parent / "kinematics"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from matdog_geometry_mesh_kernel import load_mesh  # noqa: E402
from matdog_geometry_model_v5 import (  # noqa: E402
    GeometryModelError,
    RELATION_BODY_VS_BRANCH,
    RELATION_CROSS_BRANCH,
    RELATION_SAME_BRANCH,
    load_robot_geometry_model,
)
from matdog_geometry_scene import RobotScene  # noqa: E402
from matdog_geometry_scene_v5 import RobotSceneV5  # noqa: E402


def _single_link_urdf(
    mesh_path: Path,
    *,
    scale: str = "1 1 1",
    origin_xyz: str = "0 0 0",
    origin_rpy: str = "0 0 0",
) -> str:
    return f"""<?xml version="1.0"?>
<robot name="synthetic">
  <link name="opaque_body">
    <collision>
      <origin xyz="{origin_xyz}" rpy="{origin_rpy}"/>
      <geometry><mesh filename="{mesh_path}" scale="{scale}"/></geometry>
    </collision>
  </link>
</robot>
"""


def _renamed_three_joint_urdf(mesh_path: Path, *, omit_middle_hardware: bool = False) -> str:
    links = "\n".join(
        f"""  <link name="node_{index}"><collision><geometry>
    <mesh filename="{mesh_path}" scale="0.001 0.001 0.001"/>
  </geometry></collision></link>"""
        for index in range(5)
    )
    joints = []
    for index, motor_id in enumerate((903, 902, 901)):
        hardware = (
            ""
            if omit_middle_hardware and index == 1
            else f"<hardware><motorId>{motor_id}</motorId><motorDirection>{1 if index != 1 else -1}</motorDirection></hardware>"
        )
        joints.append(
            f"""  <joint name="axis_{index}" type="revolute">
    <parent link="node_{index}"/><child link="node_{index + 1}"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1"/>{hardware}
  </joint>"""
        )
    joints.append(
        """  <joint name="terminal_attachment" type="fixed">
    <parent link="node_3"/><child link="node_4"/>
  </joint>"""
    )
    return "<?xml version=\"1.0\"?>\n<robot name=\"renamed\">\n" + links + "\n" + "\n".join(joints) + "\n</robot>\n"


class TestCurrentUrdfModelFirstSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = load_robot_geometry_model(URDF_PATH)

    def test_exactly_twelve_actuated_joints_from_model_metadata(self):
        self.assertEqual(len(self.model.actuated_joint_names), 12)
        self.assertEqual(
            {self.model.joints[name].motor_id for name in self.model.actuated_joint_names},
            {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43},
        )
        self.assertTrue(
            all(
                self.model.joints[name].motor_direction in (-1, 1)
                for name in self.model.actuated_joint_names
            )
        )

    def test_four_topology_branches_of_three_actuators(self):
        self.assertEqual(len(self.model.articulated_branches), 4)
        self.assertEqual(
            [len(branch.joint_names) for branch in self.model.articulated_branches],
            [3, 3, 3, 3],
        )
        self.assertEqual(
            [branch.sort_motor_id for branch in self.model.articulated_branches],
            [13, 23, 33, 43],
        )

    def test_pair_relation_is_topology_derived(self):
        first, second = self.model.articulated_branches[:2]
        self.assertEqual(
            self.model.pair_relation(self.model.root_link, first.link_names[0]),
            RELATION_BODY_VS_BRANCH,
        )
        self.assertEqual(
            self.model.pair_relation(first.link_names[0], first.link_names[-1]),
            RELATION_SAME_BRANCH,
        )
        self.assertEqual(
            self.model.pair_relation(first.link_names[-1], second.link_names[-1]),
            RELATION_CROSS_BRANCH,
        )

    def test_motor_direction_is_read_from_urdf(self):
        by_id = {
            self.model.joints[name].motor_id: self.model.joints[name].motor_direction
            for name in self.model.actuated_joint_names
        }
        self.assertEqual(by_id[13], 1)
        self.assertEqual(by_id[22], -1)
        self.assertEqual(by_id[43], -1)
        self.assertEqual(by_id[42], 1)


class TestSyntheticModelDataControlsV5(unittest.TestCase):
    def _write(self, directory: Path, text: str, name: str = "model.urdf") -> Path:
        path = directory / name
        path.write_text(text, encoding="utf-8")
        return path

    def test_renamed_model_is_selected_without_matdog_name_conventions(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = self._write(
                Path(temporary),
                _renamed_three_joint_urdf(COLLISION_DIR / "lf_foot_link.stl"),
            )
            model = load_robot_geometry_model(path, expected_actuated_joint_count=3)
            self.assertEqual(model.actuated_joint_names, ("axis_0", "axis_1", "axis_2"))
            self.assertEqual(len(model.articulated_branches), 1)
            self.assertEqual(model.articulated_branches[0].joint_names, model.actuated_joint_names)

    def test_incomplete_revolute_motor_metadata_is_an_explicit_stop(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = self._write(
                Path(temporary),
                _renamed_three_joint_urdf(
                    COLLISION_DIR / "lf_foot_link.stl",
                    omit_middle_hardware=True,
                ),
            )
            with self.assertRaisesRegex(GeometryModelError, "cannot be selected unambiguously"):
                load_robot_geometry_model(path, expected_actuated_joint_count=3)

    def test_collision_filename_changes_loaded_geometry_without_python_edit(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            first = self._write(
                directory,
                _single_link_urdf(COLLISION_DIR / "lf_foot_link.stl", scale="0.001 0.001 0.001"),
                "first.urdf",
            )
            second = self._write(
                directory,
                _single_link_urdf(COLLISION_DIR / "lf_hip_link.stl", scale="0.001 0.001 0.001"),
                "second.urdf",
            )
            scene_first = RobotSceneV5.from_urdf(first, expected_actuated_joint_count=0)
            scene_second = RobotSceneV5.from_urdf(second, expected_actuated_joint_count=0)
            self.assertNotEqual(
                scene_first.mesh("opaque_body").sha256,
                scene_second.mesh("opaque_body").sha256,
            )
            self.assertNotEqual(
                scene_first.mesh("opaque_body").triangle_count,
                scene_second.mesh("opaque_body").triangle_count,
            )

    def test_per_mesh_scale_and_collision_origin_are_honored(self):
        scale = (0.002, 0.003, 0.004)
        with tempfile.TemporaryDirectory() as temporary:
            path = self._write(
                Path(temporary),
                _single_link_urdf(
                    COLLISION_DIR / "lf_foot_link.stl",
                    scale="0.002 0.003 0.004",
                    origin_xyz="0.1 -0.2 0.3",
                    origin_rpy=f"0 0 {math.pi / 2.0}",
                ),
            )
            scene = RobotSceneV5.from_urdf(path, expected_actuated_joint_count=0)
            raw = load_mesh(COLLISION_DIR / "lf_foot_link.stl", use_cache=False)
            np.testing.assert_allclose(
                scene.mesh("opaque_body").triangles_local,
                raw.triangles_local * np.asarray(scale),
                atol=0.0,
                rtol=0.0,
            )
            transform = scene.collision_transform("opaque_body", {})
            np.testing.assert_allclose(transform.translation, (0.1, -0.2, 0.3), atol=1e-15)
            np.testing.assert_allclose(
                transform.rotation,
                ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
                atol=1e-15,
            )


class TestCurrentGeometryCompatibility(unittest.TestCase):
    def test_v5_fk_and_meshes_match_phase1b_for_current_identity_origins(self):
        old = RobotScene(REPO_ROOT)
        new = RobotSceneV5.from_urdf(URDF_PATH)
        pose = new.home_pose()
        for index, joint_name in enumerate(new.model.actuated_joint_names):
            pose[joint_name] = 0.01 * (index - 5)
        for link_name in new.model.collision_link_names:
            old_transform = old.link_transform(link_name, pose)
            new_transform = new.collision_transform(link_name, pose)
            np.testing.assert_allclose(old_transform.rotation, new_transform.rotation, atol=1e-15)
            np.testing.assert_allclose(old_transform.translation, new_transform.translation, atol=1e-15)
            self.assertEqual(old.mesh(link_name).sha256, new.mesh(link_name).sha256)

    def test_v5_core_has_no_legacy_hardware_or_policy_import(self):
        filenames = (
            "matdog_geometry_model_v5.py",
            "matdog_geometry_scene_v5.py",
            "matdog_geometry_contact_search_v5.py",
            "matdog_geometry_profile_v5.py",
        )
        forbidden_imports = {
            "matdog_geometry_contact_search",
            "matdog_geometry_path_planner",
            "matdog_geometry_profile",
        }
        for filename in filenames:
            source = (CALIBRATION_DIR / filename).read_text(encoding="utf-8")
            self.assertNotIn("LF_V25_HARDWARE_EVIDENCE", source)
            tree = ast.parse(source)
            imported = {
                node.module
                for node in ast.walk(tree)
                if isinstance(node, ast.ImportFrom) and node.module is not None
            }
            self.assertTrue(forbidden_imports.isdisjoint(imported), (filename, imported))


if __name__ == "__main__":
    unittest.main()

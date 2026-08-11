#!/usr/bin/env python3
"""Focused offline architecture tests for canonical/replay separation."""

from __future__ import annotations

from dataclasses import replace
import inspect
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

import matdog_geometry_compiler_v5 as compiler  # noqa: E402
from matdog_geometry_contact_search_v5 import (  # noqa: E402
    PATH_DOMAIN_DIRECT_TO_GEOMETRIC_TARGET,
    EndpointSpecV5,
)


def _specs() -> tuple[EndpointSpecV5, ...]:
    return tuple(
        EndpointSpecV5(
            endpoint_id=f"joint_{joint_index}:{side}",
            joint_name=f"joint_{joint_index}",
            side=side,
            branch_root_joint_name=f"branch_{joint_index // 3}",
            structural_depth=joint_index % 3,
            motor_id=100 + joint_index,
            motor_direction=1,
            urdf_lower_rad=-1.0,
            urdf_upper_rad=1.0,
            active_link_pair=(f"parent_{joint_index}", f"child_{joint_index}"),
        )
        for joint_index in range(12)
        for side in ("min", "max")
    )


class TestCanonicalCompilerBoundary(unittest.TestCase):
    def test_compiler_api_cannot_accept_a_g4_profile(self):
        parameters = inspect.signature(compiler.run_geometry_compiler_v5).parameters
        self.assertNotIn("g4_reference_profile_path", parameters)
        self.assertNotIn("g4_profile_path", parameters)

    def test_default_tasks_are_exactly_24_context_free_direct_sweeps(self):
        scene = SimpleNamespace(model=object())
        with patch.object(compiler, "load_endpoint_specs", return_value=_specs()):
            tasks = compiler._default_tasks(scene)
        self.assertEqual(len(tasks), 24)
        self.assertTrue(all(task.context_pose_rad == {} for task in tasks))
        self.assertTrue(
            all(
                task.path_domain_mode == PATH_DOMAIN_DIRECT_TO_GEOMETRIC_TARGET
                for task in tasks
            )
        )
        compiler._assert_canonical_context_free(tasks)

    def test_canonical_gate_rejects_even_zero_valued_legacy_context_keys(self):
        scene = SimpleNamespace(model=object())
        with patch.object(compiler, "load_endpoint_specs", return_value=_specs()):
            tasks = compiler._default_tasks(scene)
        contaminated = (replace(tasks[0], context_pose_rad={"other_joint": 0.0}),) + tasks[1:]
        with self.assertRaisesRegex(
            compiler.GeometryCompilerV5Error,
            "24 context-free",
        ):
            compiler._assert_canonical_context_free(contaminated)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Fast, offline tests for the integrated Geometry Compiler V5 runner.

No test executes collision geometry or a real C/D benchmark.  Repository
boundaries and publication use temporary directories; the integrated-flow
tests replace every expensive geometry boundary with deterministic mocks.
"""

from __future__ import annotations

from contextlib import ExitStack
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import patch


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

import matdog_geometry_full_runner_v5 as runner  # noqa: E402
from matdog_geometry_path_planner_v5 import (  # noqa: E402
    PARKING_SCHEMA_V2,
    PARKING_V2_SEMANTIC_KEYS,
    ParkingPlannerParametersV5,
)
from matdog_geometry_process_workers_v5 import (  # noqa: E402
    GeometryInputFingerprintV5,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _profile_payload(value: str, *, audit: str) -> dict:
    return {
        "schema_version": "matdog.calibration_geometry_profile.v5",
        "generation_metadata": {"audit": audit},
        "semantic_content_sha256": value * 64,
        "geometry_fact": {"value": value},
    }


def _parking_payload(value: str, *, audit: str) -> dict:
    semantic = {
        "schema_version": PARKING_SCHEMA_V2,
        "input_geometry_profile": {
            "schema_version": "matdog.calibration_geometry_profile.v5",
            "semantic_content_sha256": "a" * 64,
        },
        "provenance": {"source": value},
        "parameters": {"step": 1},
        "summary": {"count": 24},
        "plans": [{"canonical_endpoint_index": index, "value": value} for index in range(24)],
    }
    return {
        **semantic,
        "generation_metadata": {"audit": audit},
        "semantic_content_sha256": value * 64,
    }


def _write_integrated_bundle(
    repo: Path,
    *,
    benchmark: str = "C",
    json_values: dict[str, dict] | None = None,
) -> Path:
    json_values = json_values or {}
    artifacts = {}
    for name in sorted(runner.EXPECTED_BUNDLE_ARTIFACT_KEYS):
        suffix = ".json" if name.endswith("profile") or name.endswith("json") else ".md"
        path = repo / f"run_{name}{suffix}"
        if suffix == ".json":
            _write_json(path, json_values.get(name, {"name": name}))
        else:
            path.write_text(f"# {name}\n", encoding="utf-8")
        artifacts[name] = {
            "relative_path": path.name,
            "file_sha256": _sha256(path),
        }
    manifest = {
        "schema_version": runner.RUN_SCHEMA_VERSION,
        "benchmark_id": benchmark,
        "execution": {"worker_count": 1 if benchmark == "C" else 4},
        "provenance": {},
        "artifacts": artifacts,
        "bundle_validity": {
            "marker": "RUN_MANIFEST_PUBLISHED_LAST",
            "manifest_required": True,
            "artifact_count": len(runner.EXPECTED_BUNDLE_ARTIFACT_KEYS),
            "artifact_hash_algorithm": "SHA256",
        },
    }
    manifest["manifest_content_sha256"] = runner._manifest_content_sha256(manifest)
    manifest_path = repo / "run_RUN_MANIFEST.json"
    _write_json(manifest_path, manifest)
    return manifest_path


class TestIntegratedOutputPaths(unittest.TestCase):
    def test_paths_are_unique_repo_bound_and_suffix_constrained(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            prefix = repo / "artifacts" / "run_C"
            paths = runner.integrated_output_paths(prefix, repo)

            self.assertEqual(len(paths.all()), 9)
            self.assertEqual(len(set(paths.all())), 9)
            self.assertTrue(all(path.is_relative_to(repo) for path in paths.all()))
            self.assertTrue(paths.endpoint_profile.name.endswith("_ENDPOINT_PROFILE.json"))
            self.assertTrue(paths.run_manifest.name.endswith("_RUN_MANIFEST.json"))
            self.assertFalse(any(path.exists() for path in paths.all()))

            with self.assertRaisesRegex(runner.GeometryFullRunnerV5Error, "escapes"):
                runner.integrated_output_paths(repo.parent / "outside", repo)
            with self.assertRaisesRegex(runner.GeometryFullRunnerV5Error, "suffix-free"):
                runner.integrated_output_paths(repo / "run.json", repo)

    def test_existing_member_rejects_whole_bundle_before_publication(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            prefix = repo / "out" / "run"
            paths = runner.integrated_output_paths(prefix, repo)
            paths.parking_json.parent.mkdir(parents=True)
            paths.parking_json.write_bytes(b"protected")

            with self.assertRaisesRegex(runner.GeometryFullRunnerV5Error, "overwrite"):
                runner.integrated_output_paths(prefix, repo)
            self.assertEqual(paths.parking_json.read_bytes(), b"protected")
            self.assertEqual(
                [path for path in paths.all() if path.exists()],
                [paths.parking_json],
            )


class TestNoClobberBundlePublication(unittest.TestCase):
    def test_success_is_complete_and_a_second_publish_never_overwrites(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary).resolve() / "bundle"
            payloads = {
                parent / "a.json": b"alpha",
                parent / "b.md": b"beta",
                parent / "manifest.json": b"gamma",
            }
            manifest_path = parent / "manifest.json"
            runner._publish_no_clobber_bundle(
                payloads,
                manifest_path=manifest_path,
            )
            self.assertEqual(
                {path: path.read_bytes() for path in payloads},
                payloads,
            )
            self.assertEqual(list(parent.glob(".matdog-v5-stage-*")), [])

            with self.assertRaisesRegex(runner.GeometryFullRunnerV5Error, "appeared"):
                runner._publish_no_clobber_bundle(
                    {path: b"replacement" for path in payloads},
                    manifest_path=manifest_path,
                )
            self.assertEqual(
                {path: path.read_bytes() for path in payloads},
                payloads,
            )

    def test_link_failure_rolls_back_every_published_target_and_stage(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary).resolve() / "bundle"
            payloads = {
                parent / "a.json": b"alpha",
                parent / "b.json": b"beta",
                parent / "manifest.json": b"gamma",
            }
            real_link = os.link
            calls = 0

            def fail_second_link(source, destination):
                nonlocal calls
                calls += 1
                if calls == 2:
                    raise OSError("synthetic publication failure")
                return real_link(source, destination)

            with patch.object(runner.os, "link", side_effect=fail_second_link):
                with self.assertRaisesRegex(OSError, "synthetic"):
                    runner._publish_no_clobber_bundle(
                        payloads,
                        manifest_path=parent / "manifest.json",
                    )

            self.assertFalse(any(path.exists() for path in payloads))
            self.assertEqual(list(parent.glob(".matdog-v5-stage-*")), [])

    def test_empty_or_multi_directory_bundle_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            with self.assertRaisesRegex(runner.GeometryFullRunnerV5Error, "empty"):
                runner._publish_no_clobber_bundle(
                    {},
                    manifest_path=root / "manifest.json",
                )
            with self.assertRaisesRegex(
                runner.GeometryFullRunnerV5Error,
                "one directory",
            ):
                runner._publish_no_clobber_bundle(
                    {
                        root / "a" / "one": b"1",
                        root / "b" / "manifest.json": b"2",
                    },
                    manifest_path=root / "b" / "manifest.json",
                )

    def test_manifest_is_the_last_visible_link(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary).resolve() / "bundle"
            manifest_path = parent / "run_RUN_MANIFEST.json"
            payloads = {
                parent / "z-data.json": b"z",
                manifest_path: b"manifest",
                parent / "a-data.json": b"a",
            }
            destinations = []
            real_link = os.link

            def observe_link(source, destination):
                destinations.append(Path(destination))
                return real_link(source, destination)

            with patch.object(runner.os, "link", side_effect=observe_link):
                runner._publish_no_clobber_bundle(
                    payloads,
                    manifest_path=manifest_path,
                )
            self.assertEqual(destinations[-1], manifest_path)


class TestIntegratedBundleValidityMarker(unittest.TestCase):
    def test_complete_manifest_marked_bundle_is_accepted(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            manifest_path = _write_integrated_bundle(repo)
            manifest, digest, payloads, snapshots = (
                runner._load_validated_integrated_bundle(
                    manifest_path,
                    repo,
                    expected_benchmark="C",
                )
            )
            self.assertEqual(manifest["benchmark_id"], "C")
            self.assertEqual(digest, _sha256(manifest_path))
            self.assertEqual(set(payloads), runner.EXPECTED_BUNDLE_ARTIFACT_KEYS)
            self.assertEqual(set(snapshots), runner.EXPECTED_BUNDLE_ARTIFACT_KEYS)

    def test_visible_data_without_manifest_is_not_a_valid_bundle(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            (repo / "orphan_ENDPOINT_PROFILE.json").write_text("{}\n")
            with self.assertRaisesRegex(
                runner.GeometryFullRunnerV5Error,
                "no validity-marker manifest",
            ):
                runner._load_validated_integrated_bundle(
                    repo / "missing_RUN_MANIFEST.json",
                    repo,
                )

    def test_tampered_member_and_unexpected_manifest_member_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            manifest_path = _write_integrated_bundle(repo)
            manifest = json.loads(manifest_path.read_text())
            endpoint_path = repo / manifest["artifacts"]["endpoint_profile"][
                "relative_path"
            ]
            endpoint_path.write_bytes(b"tampered")
            with self.assertRaisesRegex(
                runner.GeometryFullRunnerV5Error,
                "artifact SHA mismatch",
            ):
                runner._load_validated_integrated_bundle(manifest_path, repo)

        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            manifest_path = _write_integrated_bundle(repo)
            manifest = json.loads(manifest_path.read_text())
            manifest["artifacts"]["unexpected"] = deepcopy(
                manifest["artifacts"]["endpoint_report"]
            )
            manifest["manifest_content_sha256"] = runner._manifest_content_sha256(
                manifest
            )
            _write_json(manifest_path, manifest)
            with self.assertRaisesRegex(
                runner.GeometryFullRunnerV5Error,
                "artifact set",
            ):
                runner._load_validated_integrated_bundle(manifest_path, repo)

    def test_manifest_content_hash_and_missing_member_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            manifest_path = _write_integrated_bundle(repo)
            manifest = json.loads(manifest_path.read_text())
            manifest["benchmark_id"] = "D"
            _write_json(manifest_path, manifest)
            with self.assertRaisesRegex(
                runner.GeometryFullRunnerV5Error,
                "manifest content SHA is invalid",
            ):
                runner._load_validated_integrated_bundle(manifest_path, repo)

        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            manifest_path = _write_integrated_bundle(repo)
            manifest = json.loads(manifest_path.read_text())
            missing = repo / manifest["artifacts"]["endpoint_report"][
                "relative_path"
            ]
            missing.unlink()
            with self.assertRaisesRegex(
                runner.GeometryFullRunnerV5Error,
                "artifact is missing",
            ):
                runner._load_validated_integrated_bundle(manifest_path, repo)

    def test_determinism_snapshot_detects_manifest_and_member_changes(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            manifest_path = _write_integrated_bundle(repo)
            manifest_bytes = manifest_path.read_bytes()
            _manifest, manifest_sha, _payloads, snapshots = (
                runner._load_validated_integrated_bundle(manifest_path, repo)
            )
            determinism = {
                "status": "PASS",
                "reference_manifest_relative_path": manifest_path.name,
                "reference_manifest_file_sha256": manifest_sha,
                "reference_artifacts": snapshots,
            }
            runner._verify_determinism_reference_snapshot(determinism, repo)

            manifest_path.write_bytes(manifest_bytes + b" ")
            with self.assertRaisesRegex(
                runner.GeometryFullRunnerV5Error,
                "Benchmark C manifest changed",
            ):
                runner._verify_determinism_reference_snapshot(determinism, repo)

            manifest_path.write_bytes(manifest_bytes)
            report_path = repo / snapshots["endpoint_report"]["relative_path"]
            report_path.write_bytes(report_path.read_bytes() + b"tamper")
            with self.assertRaisesRegex(
                runner.GeometryFullRunnerV5Error,
                "Benchmark C artifact changed",
            ):
                runner._verify_determinism_reference_snapshot(determinism, repo)


class TestSyntheticCgroupContract(unittest.TestCase):
    def setUp(self):
        self.memory_max = 6 * 1024**3
        self.valid = {
            "memory_max": str(self.memory_max),
            "memory_swap_max": "0",
            "memory_peak_bytes": 1024,
            "memory_swap_peak_bytes": 0,
            "memory_events": {"oom": 0, "oom_kill": 0},
            "process_cpu_affinity": [0, 1, 2, 3],
        }

    def test_exact_six_gib_zero_swap_contract_passes(self):
        runner._validate_cgroup_contract(
            self.valid,
            required_memory_max_bytes=self.memory_max,
            required_memory_swap_max_bytes=0,
            required_cpu_affinity=(0, 1, 2, 3),
        )

    def test_limit_swap_peak_and_oom_failures_are_hard_stops(self):
        cases = (
            ({**self.valid, "memory_max": "max"}, "memory.max"),
            ({**self.valid, "memory_swap_max": "max"}, "memory.swap.max"),
            ({**self.valid, "memory_swap_peak_bytes": 1}, "swap peak"),
            (
                {**self.valid, "memory_events": {"oom": 1, "oom_kill": 0}},
                "OOM event",
            ),
            (
                {**self.valid, "memory_events": {"oom": 0, "oom_kill": 1}},
                "OOM event",
            ),
            ({**self.valid, "process_cpu_affinity": [0, 1]}, "CPU affinity"),
        )
        for telemetry, message in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(runner.GeometryFullRunnerV5Error, message):
                    runner._validate_cgroup_contract(
                        telemetry,
                        required_memory_max_bytes=self.memory_max,
                        required_memory_swap_max_bytes=0,
                        required_cpu_affinity=(0, 1, 2, 3),
                    )


class TestSemanticPayloadBoundaries(unittest.TestCase):
    def test_profile_runtime_metadata_and_digest_are_excluded(self):
        first = _profile_payload("a", audit="workers=1")
        second = deepcopy(first)
        second["generation_metadata"] = {"audit": "workers=4"}
        second["semantic_content_sha256"] = "different"
        self.assertEqual(
            runner._semantic_profile_payload(first),
            runner._semantic_profile_payload(second),
        )
        second["geometry_fact"]["value"] = "changed"
        self.assertNotEqual(
            runner._semantic_profile_payload(first),
            runner._semantic_profile_payload(second),
        )

    def test_parking_v2_uses_the_closed_semantic_key_set(self):
        first = _parking_payload("b", audit="workers=1")
        second = deepcopy(first)
        second["generation_metadata"] = {"audit": "workers=4"}
        second["semantic_content_sha256"] = "different"
        second["future_audit_only"] = "ignored by closed projection"
        projected = runner._semantic_parking_payload(second)
        self.assertEqual(set(projected), set(PARKING_V2_SEMANTIC_KEYS))
        self.assertEqual(projected, runner._semantic_parking_payload(first))

        second["plans"][0]["value"] = "semantic change"
        self.assertNotEqual(
            runner._semantic_parking_payload(first),
            runner._semantic_parking_payload(second),
        )
        v1 = deepcopy(first)
        v1["schema_version"] = "matdog.geometry_path_parking.v1"
        with self.assertRaisesRegex(runner.GeometryFullRunnerV5Error, "parking v2"):
            runner._semantic_parking_payload(v1)


class TestBenchmarkContract(unittest.TestCase):
    def _call(self, repo: Path, *, benchmark: str, workers: int, reference=None):
        return runner.run_integrated_geometry_v5(
            repo_root=repo,
            urdf_path=repo / "robot.urdf",
            g4_reference_profile_path=repo / "g4.json",
            output_prefix=repo / "out" / "run",
            benchmark_id=benchmark,
            workers=workers,
            parameters=ParkingPlannerParametersV5(),
            determinism_reference_manifest=reference,
        )

    def test_c_and_d_worker_reference_matrix_rejects_invalid_combinations_early(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            reference = repo / "C_RUN_MANIFEST.json"
            cases = (
                ("C", 4, None, "requires workers=1"),
                ("D", 1, reference, "requires workers=4"),
                ("D", 4, None, "requires the Benchmark C"),
                ("C", 1, reference, "cannot consume"),
                ("X", 1, None, "must be C or D"),
            )
            for benchmark, workers, manifest, message in cases:
                with self.subTest(benchmark=benchmark, workers=workers):
                    with self.assertRaisesRegex(runner.GeometryFullRunnerV5Error, message):
                        self._call(
                            repo,
                            benchmark=benchmark,
                            workers=workers,
                            reference=manifest,
                        )

    def test_cgroup_limits_require_explicit_cgroup_mode(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            with self.assertRaisesRegex(
                runner.GeometryFullRunnerV5Error,
                "cgroup memory limits",
            ):
                runner.run_integrated_geometry_v5(
                    repo_root=repo,
                    urdf_path=repo / "robot.urdf",
                    g4_reference_profile_path=repo / "g4.json",
                    output_prefix=repo / "out" / "run",
                    benchmark_id="C",
                    workers=1,
                    parameters=ParkingPlannerParametersV5(),
                    required_memory_max_bytes=6 * 1024**3,
                )

    def _run_fully_mocked(self, repo: Path, *, benchmark: str):
        workers = 1 if benchmark == "C" else 4
        reference = None if benchmark == "C" else repo / "C_RUN_MANIFEST.json"
        fingerprint = GeometryInputFingerprintV5(
            urdf_sha256="1" * 64,
            collision_mesh_sha256=(("link", "2" * 64),),
            q0_status_by_joint=tuple(
                (f"joint_{index}", "SEPARATED_AABB") for index in range(12)
            ),
        )
        model = SimpleNamespace(
            actuated_joint_names=tuple(f"joint_{index}" for index in range(12)),
            collision_link_names=("link",),
        )
        scene = SimpleNamespace(
            model=model,
            mesh=lambda _name: SimpleNamespace(
                triangle_count=100,
                stl_path=repo / "mesh.stl",
            ),
        )
        oracle = {
            "status": "PASS",
            "endpoint_count": 24,
            "geometric_contact_found_count": 24,
            "no_geometric_contact_count": 0,
            "path_obstruction_count": 6,
            "path_obstruction_precedes_contact_count": 2,
        }
        endpoint_profile = {
            "schema_version": "matdog.calibration_geometry_profile.v5",
            "generation_metadata": {},
            "semantic_content_sha256": "a" * 64,
            "geometry_fact": "same",
            "endpoint_searches": [
                {"search_context": {"joint_positions_rad": {}}}
                for _index in range(24)
            ],
        }
        compiler_run = SimpleNamespace(
            profile=endpoint_profile,
            analyses=tuple(range(24)),
            runtime_seconds=1.25,
        )
        replay_run = SimpleNamespace(
            analyses=tuple(range(24)),
            comparison=oracle,
            runtime_seconds=0.75,
        )
        plans = tuple(
            SimpleNamespace(
                evaluated_1d_candidates=1,
                evaluated_2d_candidates=0,
                one_dof_search_grid_rad={"joint": (0.0, 1.0)},
                two_dof_search_grid_rad={},
            )
            for _index in range(24)
        )

        def parking_builder(*_args, **_kwargs):
            return {
                "schema_version": PARKING_SCHEMA_V2,
                "generation_metadata": {},
                "semantic_content_sha256": "b" * 64,
                "parameters": {
                    "obstruction_bisection_resolution_rad": (
                        runner.DEFAULT_OBSTRUCTION_BISECTION_RESOLUTION_RAD
                    ),
                    "max_obstruction_bisection_iterations": (
                        runner.DEFAULT_MAX_OBSTRUCTION_BISECTION_ITERATIONS
                    ),
                },
                "summary": {"endpoint_plan_count": 24},
            }

        def derive(profile, _parking):
            combined = deepcopy(profile)
            combined["semantic_content_sha256"] = "c" * 64
            return combined

        source_manifest = {"source.py": "3" * 64}
        determinism = {
            "status": "PASS",
            "endpoint_profile_semantic_payload_equal": True,
            "parking_semantic_payload_equal": True,
            "combined_profile_semantic_payload_equal": True,
            "g4_g7_oracle_payload_equal": True,
        }
        with ExitStack() as stack:
            stack.enter_context(
                patch.object(runner, "geometry_source_manifest", return_value=source_manifest)
            )
            stack.enter_context(
                patch.object(runner, "_sha256_file", return_value=runner.EXPECTED_G4_FILE_SHA256)
            )
            stack.enter_context(
                patch.object(runner.RobotSceneV5, "from_urdf", return_value=scene)
            )
            stack.enter_context(
                patch.object(runner, "scene_input_fingerprint", return_value=fingerprint)
            )
            stack.enter_context(patch.object(runner, "clear_mesh_cache"))
            compiler = stack.enter_context(
                patch.object(runner, "run_geometry_compiler_v5", return_value=compiler_run)
            )
            replay = stack.enter_context(
                patch.object(runner, "run_g4_replay_v5", return_value=replay_run)
            )
            stack.enter_context(
                patch.object(
                    runner,
                    "_compare_canonical_contacts_to_replay",
                    return_value={"status": "PASS", "endpoint_count": 24},
                )
            )
            stack.enter_context(
                patch.object(runner.os, "sched_getaffinity", return_value={0, 1, 2, 3})
            )
            stack.enter_context(patch.object(runner, "validate_pure_geometry_profile"))
            stack.enter_context(
                patch.object(runner, "find_geometry_mismatches_v5", return_value=[])
            )
            task_loader = stack.enter_context(
                patch.object(
                    runner,
                    "endpoint_parking_tasks_from_profile",
                    return_value=tuple(range(24)),
                )
            )
            stack.enter_context(
                patch.object(runner, "execute_parking_tasks", return_value=plans)
            )
            stack.enter_context(
                patch.object(
                    runner,
                    "validate_endpoint_path_plan_consistency",
                    return_value={"status": "PASS", "endpoint_count": 24},
                )
            )
            stack.enter_context(
                patch.object(runner, "build_parking_artifact_v2", side_effect=parking_builder)
            )
            stack.enter_context(patch.object(runner, "validate_parking_artifact"))
            stack.enter_context(
                patch.object(
                    runner,
                    "derive_geometry_profile_with_path_plans",
                    side_effect=derive,
                )
            )
            stack.enter_context(
                patch.object(runner, "render_geometry_report_v5", return_value="profile report")
            )
            stack.enter_context(
                patch.object(runner, "render_g4_g7_report", return_value="oracle report")
            )
            stack.enter_context(
                patch.object(runner, "render_parking_report", return_value="parking report")
            )
            compare = stack.enter_context(
                patch.object(runner, "_compare_with_reference", return_value=determinism)
            )
            stack.enter_context(
                patch.object(
                    runner,
                    "_read_cgroup_v2",
                    return_value={
                        "memory_max": str(runner.CANONICAL_MEMORY_MAX_BYTES),
                        "memory_swap_max": "0",
                        "memory_swap_peak_bytes": 0,
                        "memory_events": {},
                        "process_cpu_affinity": [0, 1, 2, 3],
                    },
                )
            )
            stack.enter_context(
                patch.object(runner, "_verify_determinism_reference_snapshot")
            )
            publish = stack.enter_context(
                patch.object(runner, "_publish_no_clobber_bundle")
            )
            resource_contract = (
                {
                    "require_cgroup_v2": True,
                    "required_memory_max_bytes": runner.CANONICAL_MEMORY_MAX_BYTES,
                    "required_memory_swap_max_bytes": 0,
                    "required_cpu_affinity": runner.CANONICAL_PHYSICAL_CPU_AFFINITY,
                }
                if benchmark == "D"
                else {}
            )
            manifest = runner.run_integrated_geometry_v5(
                repo_root=repo,
                urdf_path=repo / "robot.urdf",
                g4_reference_profile_path=repo / "g4.json",
                output_prefix=repo / "out" / f"run_{benchmark}",
                benchmark_id=benchmark,
                workers=workers,
                parameters=ParkingPlannerParametersV5(),
                determinism_reference_manifest=reference,
                **resource_contract,
            )
            self.assertNotIn("g4_reference_profile_path", compiler.call_args.kwargs)
            self.assertIs(
                task_loader.call_args.args[1],
                endpoint_profile,
            )
            self.assertEqual(
                replay.call_args.kwargs["g4_profile_path"],
                repo / "g4.json",
            )
        return manifest, compare, publish

    def test_fully_mocked_c_and_d_preserve_workers_reference_and_parent_publish_contract(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            for benchmark, workers in (("C", 1), ("D", 4)):
                with self.subTest(benchmark=benchmark):
                    manifest, compare, publish = self._run_fully_mocked(
                        repo,
                        benchmark=benchmark,
                    )
                    self.assertEqual(manifest["benchmark_id"], benchmark)
                    self.assertEqual(manifest["execution"]["worker_count"], workers)
                    self.assertEqual(manifest["gates"]["worker_writes"], 0)
                    self.assertEqual(manifest["gates"]["parent_bundle_publications"], 1)
                    self.assertEqual(
                        manifest["semantic_content_sha256"],
                        {
                            "endpoint_profile": "a" * 64,
                            "parking_v2": "b" * 64,
                            "combined_profile": "c" * 64,
                        },
                    )
                    if benchmark == "C":
                        self.assertEqual(
                            manifest["resource_enforcement"],
                            {
                                "memory_swap_oom": "NOT_REQUIRED_NOT_ENFORCED",
                                "cpu_affinity": "NOT_REQUIRED_NOT_ENFORCED",
                                "cpuset_cpus_effective_role": "NOT_COLLECTED",
                            },
                        )
                    else:
                        self.assertEqual(
                            manifest["resource_enforcement"]["memory_swap_oom"],
                            "CGROUP_V2_VALIDATED",
                        )
                        self.assertEqual(
                            manifest["resource_enforcement"]["cpu_affinity"],
                            "PROCESS_SCHED_AFFINITY_INHERITED_BY_SPAWN_WORKERS",
                        )
                    publish.assert_called_once()
                    published_payloads = publish.call_args.args[0]
                    self.assertEqual(len(published_payloads), 9)
                    self.assertTrue(
                        publish.call_args.kwargs["manifest_path"].name.endswith(
                            "_RUN_MANIFEST.json"
                        )
                    )
                    if benchmark == "C":
                        compare.assert_not_called()
                        self.assertEqual(
                            manifest["determinism"]["status"],
                            "REFERENCE_CAPTURED",
                        )
                    else:
                        compare.assert_called_once()
                        self.assertEqual(manifest["determinism"]["status"], "PASS")


class TestDeterminismReference(unittest.TestCase):
    def test_c_reference_accepts_audit_differences_and_rejects_semantic_difference(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary).resolve()
            endpoint = _profile_payload("a", audit="C")
            parking = _parking_payload("b", audit="C")
            combined = _profile_payload("c", audit="C")
            oracle = {
                "status": "PASS",
                "value": 1,
                "replay_execution_provenance": {
                    "workers": 1,
                    "path_domain_mode": "FULL_ENDPOINT_ENVELOPE",
                    "g4_file_sha256": "5" * 64,
                },
            }
            semantic_sources = {"semantic.py": "1" * 64}
            replay_sources = {"oracle.py": "4" * 64}
            execution_sources = {"report.py": "2" * 64}
            input_files = {"robot.urdf": "3" * 64}
            values = {
                "endpoint_profile": endpoint,
                "parking_json": parking,
                "combined_profile": combined,
                "oracle_json": oracle,
            }
            manifest_path = _write_integrated_bundle(
                repo,
                json_values=values,
            )
            manifest = json.loads(manifest_path.read_text())
            manifest["provenance"] = {
                "canonical_semantic_source_file_sha256": semantic_sources,
                "g4_replay_source_file_sha256": replay_sources,
                "execution_source_file_sha256": execution_sources,
                "input_file_sha256": input_files,
            }
            manifest["manifest_content_sha256"] = runner._manifest_content_sha256(
                manifest
            )
            _write_json(manifest_path, manifest)

            current_endpoint = deepcopy(endpoint)
            current_endpoint["generation_metadata"] = {"audit": "D"}
            current_parking = deepcopy(parking)
            current_parking["generation_metadata"] = {"audit": "D"}
            current_combined = deepcopy(combined)
            current_combined["generation_metadata"] = {"audit": "D"}
            current_oracle = deepcopy(oracle)
            current_oracle["replay_execution_provenance"]["workers"] = 4
            with (
                patch.object(runner, "validate_pure_geometry_profile"),
                patch.object(runner, "validate_parking_artifact"),
                patch.object(
                    runner,
                    "derive_geometry_profile_with_path_plans",
                    return_value=combined,
                ),
            ):
                comparison = runner._compare_with_reference(
                    reference_manifest_path=manifest_path,
                    repo_root=repo,
                    endpoint_profile=current_endpoint,
                    parking_artifact=current_parking,
                    combined_profile=current_combined,
                    oracle=current_oracle,
                    semantic_source_manifest=semantic_sources,
                    g4_replay_source_manifest=replay_sources,
                    execution_source_manifest=execution_sources,
                    input_file_manifest=input_files,
                )
                self.assertEqual(comparison["status"], "PASS")
                self.assertTrue(
                    all(
                        value is True
                        for key, value in comparison.items()
                        if key.endswith("_equal")
                    )
                )

                current_combined["geometry_fact"]["value"] = "different"
                with self.assertRaisesRegex(
                    runner.GeometryFullRunnerV5Error,
                    "semantic determinism mismatch",
                ):
                    runner._compare_with_reference(
                        reference_manifest_path=manifest_path,
                        repo_root=repo,
                        endpoint_profile=current_endpoint,
                        parking_artifact=current_parking,
                        combined_profile=current_combined,
                        oracle=current_oracle,
                        semantic_source_manifest=semantic_sources,
                        g4_replay_source_manifest=replay_sources,
                        execution_source_manifest=execution_sources,
                        input_file_manifest=input_files,
                    )

                current_combined["geometry_fact"]["value"] = "c"
                current_oracle["value"] = 2
                with self.assertRaisesRegex(
                    runner.GeometryFullRunnerV5Error,
                    "semantic determinism mismatch",
                ):
                    runner._compare_with_reference(
                        reference_manifest_path=manifest_path,
                        repo_root=repo,
                        endpoint_profile=current_endpoint,
                        parking_artifact=current_parking,
                        combined_profile=current_combined,
                        oracle=current_oracle,
                        semantic_source_manifest=semantic_sources,
                        g4_replay_source_manifest=replay_sources,
                        execution_source_manifest=execution_sources,
                        input_file_manifest=input_files,
                    )


if __name__ == "__main__":
    unittest.main()

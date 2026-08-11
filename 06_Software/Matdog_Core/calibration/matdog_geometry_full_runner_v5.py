#!/usr/bin/env python3
"""Canonical G11/G12 Geometry Compiler V5 integrated benchmark runner.

The parent validates immutable inputs, computes endpoint geometry and G4 replay,
computes topology-driven path/parking, verifies deterministic semantic content,
and publishes one no-clobber artifact bundle.  Workers receive no output paths
and never serialize canonical artifacts.

Offline only: no Station, serial, servo command, EEPROM or hardware access.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any


THREAD_ENVIRONMENT_NAMES = (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
)
for _thread_environment_name in THREAD_ENVIRONMENT_NAMES:
    os.environ[_thread_environment_name] = "1"


from matdog_geometry_compiler_v5 import run_geometry_compiler_v5  # noqa: E402
from matdog_geometry_contact_search_v5 import EndpointAnalysisV5  # noqa: E402
from matdog_geometry_g4_oracle_v5 import (  # noqa: E402
    EXPECTED_G4_CONTENT_SHA256,
    render_g4_g7_report,
    run_g4_replay_v5,
)
from matdog_geometry_mesh_kernel import clear_mesh_cache  # noqa: E402
from matdog_geometry_path_planner_v5 import (  # noqa: E402
    DEFAULT_MAX_OBSTRUCTION_BISECTION_ITERATIONS,
    DEFAULT_OBSTRUCTION_BISECTION_RESOLUTION_RAD,
    PARKING_SCHEMA_V2,
    PARKING_V2_SEMANTIC_KEYS,
    ParkingPlannerParametersV5,
    build_parking_artifact_v2,
    derive_geometry_profile_with_path_plans,
    endpoint_parking_tasks_from_profile,
    render_parking_report,
    validate_endpoint_path_plan_consistency,
    validate_parking_artifact,
)
from matdog_geometry_process_workers_v5 import (  # noqa: E402
    execute_parking_tasks,
    scene_input_fingerprint,
)
from matdog_geometry_profile_v5 import (  # noqa: E402
    PURE_GEOMETRY_SOURCE_FILES,
    find_geometry_mismatches_v5,
    geometry_source_manifest,
    validate_pure_geometry_profile,
)
from matdog_geometry_report_v5 import render_geometry_report_v5  # noqa: E402
from matdog_geometry_scene_v5 import RobotSceneV5  # noqa: E402


EXPECTED_G4_FILE_SHA256 = (
    "f1b059a58c51508345ec583cc421cf9ca66ec8e9ac5547649c05bbf755e5c5fa"
)
CANONICAL_MEMORY_MAX_BYTES = 6 * 1024 * 1024 * 1024
CANONICAL_MEMORY_SWAP_MAX_BYTES = 0
CANONICAL_PHYSICAL_CPU_AFFINITY = (0, 1, 2, 3)
RUN_SCHEMA_VERSION = "matdog.geometry_compiler_v5.integrated_run.v2"
EXPECTED_BUNDLE_ARTIFACT_KEYS = frozenset(
    {
        "endpoint_profile",
        "endpoint_report",
        "oracle_json",
        "oracle_report",
        "parking_json",
        "parking_report",
        "combined_profile",
        "combined_report",
    }
)
FINAL_SOURCE_FILES = tuple(
    dict.fromkeys(
        (
            *PURE_GEOMETRY_SOURCE_FILES,
            "matdog_geometry_path_planner_v5.py",
            "matdog_geometry_full_runner_v5.py",
        )
    )
)
G4_REPLAY_SOURCE_FILES = (
    "matdog_geometry_mesh_kernel.py",
    "matdog_geometry_model_v5.py",
    "matdog_geometry_scene_v5.py",
    "matdog_geometry_contact_search_v5.py",
    "matdog_geometry_process_workers_v5.py",
    "matdog_geometry_profile_v5.py",
    "matdog_geometry_g4_oracle_v5.py",
)
FINAL_EXECUTION_SOURCE_FILES = tuple(
    dict.fromkeys(
        (
            *FINAL_SOURCE_FILES,
            *G4_REPLAY_SOURCE_FILES,
            "matdog_geometry_report_v5.py",
        )
    )
)


class GeometryFullRunnerV5Error(RuntimeError):
    """A canonical integrated-run hard gate failed."""


@dataclass(frozen=True)
class IntegratedOutputPathsV5:
    endpoint_profile: Path
    endpoint_report: Path
    oracle_json: Path
    oracle_report: Path
    parking_json: Path
    parking_report: Path
    combined_profile: Path
    combined_report: Path
    run_manifest: Path

    def all(self) -> tuple[Path, ...]:
        return tuple(getattr(self, field) for field in self.__dataclass_fields__)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _input_file_manifest(paths: tuple[Path, ...], repo_root: Path) -> dict[str, str]:
    return {
        _relative(path, repo_root): _sha256_file(path)
        for path in paths
    }


def _json_bytes(value: dict[str, Any]) -> bytes:
    return (
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    ).encode("utf-8")


def _manifest_content_sha256(manifest: dict[str, Any]) -> str:
    content = {
        key: value
        for key, value in manifest.items()
        if key != "manifest_content_sha256"
    }
    canonical = json.dumps(
        content,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _inside_repo(path: Path, repo_root: Path, *, label: str) -> Path:
    resolved = Path(path).resolve()
    try:
        resolved.relative_to(repo_root)
    except ValueError as exc:
        raise GeometryFullRunnerV5Error(
            f"STOP: {label} escapes the explicit repository root: {resolved}"
        ) from exc
    return resolved


def _relative(path: Path, repo_root: Path) -> str:
    return str(_inside_repo(path, repo_root, label="artifact").relative_to(repo_root))


def integrated_output_paths(prefix: Path, repo_root: Path) -> IntegratedOutputPathsV5:
    prefix = _inside_repo(prefix, repo_root, label="artifact prefix")
    if prefix.name in {"", ".", ".."} or prefix.suffix:
        raise GeometryFullRunnerV5Error(
            "STOP: artifact prefix must be a suffix-free file prefix"
        )
    base = str(prefix)
    paths = IntegratedOutputPathsV5(
        endpoint_profile=Path(base + "_ENDPOINT_PROFILE.json"),
        endpoint_report=Path(base + "_ENDPOINT_REPORT.md"),
        oracle_json=Path(base + "_G4_G7_ORACLE.json"),
        oracle_report=Path(base + "_G4_G7_ORACLE.md"),
        parking_json=Path(base + "_PATH_PARKING.json"),
        parking_report=Path(base + "_PATH_PARKING.md"),
        combined_profile=Path(base + "_COMBINED_PROFILE.json"),
        combined_report=Path(base + "_COMBINED_REPORT.md"),
        run_manifest=Path(base + "_RUN_MANIFEST.json"),
    )
    resolved = paths.all()
    if len(set(resolved)) != len(resolved):
        raise GeometryFullRunnerV5Error("STOP: integrated output paths alias")
    existing = [str(path) for path in resolved if path.exists()]
    if existing:
        raise GeometryFullRunnerV5Error(
            f"STOP: refusing to overwrite integrated output(s): {existing}"
        )
    return paths


def _semantic_profile_payload(profile: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in profile.items()
        if key not in {"generation_metadata", "semantic_content_sha256"}
    }


def _semantic_parking_payload(artifact: dict[str, Any]) -> dict[str, Any]:
    if artifact.get("schema_version") != PARKING_SCHEMA_V2:
        raise GeometryFullRunnerV5Error("STOP: canonical comparison requires parking v2")
    return {key: artifact[key] for key in PARKING_V2_SEMANTIC_KEYS}


def _semantic_oracle_payload(oracle: dict[str, Any]) -> dict[str, Any]:
    """Exclude only replay execution materialization from oracle equality."""

    provenance = oracle.get("replay_execution_provenance")
    if not isinstance(provenance, dict):
        raise GeometryFullRunnerV5Error(
            "STOP: G4 replay oracle lacks execution provenance"
        )
    return {
        **oracle,
        "replay_execution_provenance": {
            key: value for key, value in provenance.items() if key != "workers"
        },
    }


def _read_cgroup_v2() -> dict[str, Any]:
    membership = Path("/proc/self/cgroup").read_text(encoding="utf-8").splitlines()
    relative = None
    for line in membership:
        parts = line.split(":", 2)
        if len(parts) == 3 and parts[0] == "0" and parts[1] == "":
            relative = parts[2].lstrip("/")
            break
    if relative is None:
        raise GeometryFullRunnerV5Error("STOP: process is not in a cgroup-v2 hierarchy")
    cgroup_path = (Path("/sys/fs/cgroup") / relative).resolve()

    def read(name: str) -> str | None:
        path = cgroup_path / name
        return path.read_text(encoding="utf-8").strip() if path.is_file() else None

    events_text = read("memory.events") or ""
    events = {}
    for line in events_text.splitlines():
        key, value = line.split()
        events[key] = int(value)
    return {
        "version": 2,
        "relative_path": "/" + relative,
        "memory_max": read("memory.max"),
        "memory_swap_max": read("memory.swap.max"),
        "memory_peak_bytes": int(read("memory.peak") or 0),
        "memory_swap_peak_bytes": int(read("memory.swap.peak") or 0),
        "memory_events": events,
        "cpuset_cpus_effective": read("cpuset.cpus.effective"),
        "process_cpu_affinity": sorted(os.sched_getaffinity(0)),
    }


def _validate_cgroup_contract(
    telemetry: dict[str, Any],
    *,
    required_memory_max_bytes: int | None,
    required_memory_swap_max_bytes: int | None,
    required_cpu_affinity: tuple[int, ...] | None,
) -> None:
    if required_memory_max_bytes is not None and telemetry.get("memory_max") != str(
        required_memory_max_bytes
    ):
        raise GeometryFullRunnerV5Error(
            "STOP: cgroup memory.max does not match the required process-tree envelope: "
            f"expected={required_memory_max_bytes}, observed={telemetry.get('memory_max')}"
        )
    if required_memory_swap_max_bytes is not None and telemetry.get(
        "memory_swap_max"
    ) != str(required_memory_swap_max_bytes):
        raise GeometryFullRunnerV5Error(
            "STOP: cgroup memory.swap.max does not match the required value: "
            f"expected={required_memory_swap_max_bytes}, observed={telemetry.get('memory_swap_max')}"
        )
    if telemetry.get("memory_swap_peak_bytes") != 0:
        raise GeometryFullRunnerV5Error(
            f"STOP: non-zero swap peak: {telemetry.get('memory_swap_peak_bytes')}"
        )
    events = telemetry.get("memory_events", {})
    if events.get("oom", 0) or events.get("oom_kill", 0):
        raise GeometryFullRunnerV5Error(
            f"STOP: cgroup reports an OOM event: {events}"
        )
    if required_cpu_affinity is not None and telemetry.get(
        "process_cpu_affinity"
    ) != list(required_cpu_affinity):
        raise GeometryFullRunnerV5Error(
            "STOP: process CPU affinity does not match the physical-core target: "
            f"expected={list(required_cpu_affinity)}, "
            f"observed={telemetry.get('process_cpu_affinity')}"
        )


def _parse_cpu_affinity(value: str) -> tuple[int, ...]:
    cpus: set[int] = set()
    try:
        for part in value.split(","):
            if "-" in part:
                start_text, end_text = part.split("-", 1)
                start = int(start_text)
                end = int(end_text)
                if start < 0 or end < start:
                    raise ValueError
                cpus.update(range(start, end + 1))
            else:
                cpu = int(part)
                if cpu < 0:
                    raise ValueError
                cpus.add(cpu)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"invalid CPU affinity list: {value!r}"
        ) from exc
    if not cpus:
        raise argparse.ArgumentTypeError("CPU affinity list is empty")
    return tuple(sorted(cpus))


def _publish_no_clobber_bundle(
    payloads: dict[Path, bytes],
    *,
    manifest_path: Path,
) -> None:
    """Publish data first and the validity-marker manifest last."""

    if not payloads:
        raise GeometryFullRunnerV5Error("STOP: empty artifact bundle")
    parents = {path.parent.resolve() for path in payloads}
    if len(parents) != 1:
        raise GeometryFullRunnerV5Error("STOP: canonical bundle must use one directory")
    parent = parents.pop()
    manifest_path = Path(manifest_path).resolve()
    if manifest_path not in payloads or manifest_path.parent != parent:
        raise GeometryFullRunnerV5Error(
            "STOP: bundle validity-marker manifest is missing or outside its bundle"
        )
    parent.mkdir(parents=True, exist_ok=True)
    existing = [str(path) for path in payloads if path.exists()]
    if existing:
        raise GeometryFullRunnerV5Error(
            f"STOP: output appeared before publication: {existing}"
        )
    stage_dir = Path(tempfile.mkdtemp(prefix=".matdog-v5-stage-", dir=parent))
    staged: dict[Path, Path] = {}
    published: list[Path] = []
    try:
        for index, (target, payload) in enumerate(sorted(payloads.items())):
            stage = stage_dir / f"{index:02d}.stage"
            with stage.open("xb") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            staged[target] = stage
        data_targets = sorted(path for path in staged if path != manifest_path)
        for target in data_targets:
            stage = staged[target]
            os.link(stage, target)
            published.append(target)
        directory_descriptor = os.open(parent, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
        os.link(staged[manifest_path], manifest_path)
        published.append(manifest_path)
        directory_descriptor = os.open(parent, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    except BaseException:
        for target in reversed(published):
            try:
                target.unlink()
            except FileNotFoundError:
                pass
        raise
    finally:
        for stage in staged.values():
            try:
                stage.unlink()
            except FileNotFoundError:
                pass
        try:
            stage_dir.rmdir()
        except FileNotFoundError:
            pass


def _load_validated_integrated_bundle(
    manifest_path: Path,
    repo_root: Path,
    *,
    expected_benchmark: str | None = None,
) -> tuple[dict[str, Any], str, dict[str, bytes], dict[str, dict[str, str]]]:
    """Load one complete manifest-marked bundle from one byte snapshot."""

    path = _inside_repo(manifest_path, repo_root, label="integrated bundle manifest")
    if not path.is_file():
        raise GeometryFullRunnerV5Error(
            f"STOP: integrated bundle has no validity-marker manifest: {path}"
        )
    manifest_payload = path.read_bytes()
    manifest_file_sha = _sha256_bytes(manifest_payload)
    try:
        manifest = json.loads(manifest_payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise GeometryFullRunnerV5Error(
            f"STOP: invalid integrated bundle manifest JSON: {path}"
        ) from exc
    if manifest.get("schema_version") != RUN_SCHEMA_VERSION:
        raise GeometryFullRunnerV5Error(
            "STOP: integrated bundle manifest schema is not the corrected v2 contract"
        )
    if manifest.get("manifest_content_sha256") != _manifest_content_sha256(manifest):
        raise GeometryFullRunnerV5Error(
            "STOP: integrated bundle manifest content SHA is invalid"
        )
    if expected_benchmark is not None and manifest.get("benchmark_id") != expected_benchmark:
        raise GeometryFullRunnerV5Error(
            f"STOP: expected Benchmark {expected_benchmark} bundle"
        )
    validity = manifest.get("bundle_validity")
    if validity != {
        "marker": "RUN_MANIFEST_PUBLISHED_LAST",
        "manifest_required": True,
        "artifact_count": len(EXPECTED_BUNDLE_ARTIFACT_KEYS),
        "artifact_hash_algorithm": "SHA256",
    }:
        raise GeometryFullRunnerV5Error(
            "STOP: integrated bundle validity-marker contract is incomplete"
        )
    records = manifest.get("artifacts")
    if not isinstance(records, dict) or set(records) != EXPECTED_BUNDLE_ARTIFACT_KEYS:
        raise GeometryFullRunnerV5Error(
            "STOP: integrated bundle artifact set is incomplete or unexpected"
        )

    payloads: dict[str, bytes] = {}
    snapshots: dict[str, dict[str, str]] = {}
    observed_paths: set[Path] = set()
    for name in sorted(EXPECTED_BUNDLE_ARTIFACT_KEYS):
        record = records[name]
        if not isinstance(record, dict) or set(record) != {
            "relative_path",
            "file_sha256",
        }:
            raise GeometryFullRunnerV5Error(
                f"STOP: invalid integrated bundle record for {name!r}"
            )
        path_value = record.get("relative_path")
        if not isinstance(path_value, str) or Path(path_value).is_absolute():
            raise GeometryFullRunnerV5Error(
                f"STOP: invalid integrated artifact path for {name!r}"
            )
        artifact_path = _inside_repo(
            repo_root / path_value,
            repo_root,
            label=f"integrated bundle {name}",
        )
        if artifact_path.parent != path.parent or artifact_path in observed_paths:
            raise GeometryFullRunnerV5Error(
                f"STOP: integrated artifact aliases or escapes its bundle: {name}"
            )
        observed_paths.add(artifact_path)
        if not artifact_path.is_file():
            raise GeometryFullRunnerV5Error(
                f"STOP: integrated bundle artifact is missing: {artifact_path}"
            )
        payload = artifact_path.read_bytes()
        observed_sha = _sha256_bytes(payload)
        if observed_sha != record.get("file_sha256"):
            raise GeometryFullRunnerV5Error(
                f"STOP: integrated bundle artifact SHA mismatch: {artifact_path}"
            )
        payloads[name] = payload
        snapshots[name] = {
            "relative_path": _relative(artifact_path, repo_root),
            "file_sha256": observed_sha,
        }
    return manifest, manifest_file_sha, payloads, snapshots


def _compare_with_reference(
    *,
    reference_manifest_path: Path,
    repo_root: Path,
    endpoint_profile: dict[str, Any],
    parking_artifact: dict[str, Any],
    combined_profile: dict[str, Any],
    oracle: dict[str, Any],
    semantic_source_manifest: dict[str, str],
    g4_replay_source_manifest: dict[str, str],
    execution_source_manifest: dict[str, str],
    input_file_manifest: dict[str, str],
) -> dict[str, Any]:
    path = _inside_repo(
        reference_manifest_path,
        repo_root,
        label="determinism reference manifest",
    )
    manifest, manifest_file_sha, bundle_payloads, bundle_snapshots = (
        _load_validated_integrated_bundle(
            path,
            repo_root,
            expected_benchmark="C",
        )
    )
    if (
        manifest.get("schema_version") != RUN_SCHEMA_VERSION
        or manifest.get("benchmark_id") != "C"
        or manifest.get("execution", {}).get("worker_count") != 1
    ):
        raise GeometryFullRunnerV5Error(
            "STOP: D determinism reference is not a canonical workers=1 Benchmark C"
        )
    try:
        reference_endpoint = json.loads(bundle_payloads["endpoint_profile"])
        reference_parking = json.loads(bundle_payloads["parking_json"])
        reference_combined = json.loads(bundle_payloads["combined_profile"])
        reference_oracle = json.loads(bundle_payloads["oracle_json"])
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise GeometryFullRunnerV5Error(
            "STOP: corrected Benchmark C bundle contains invalid JSON"
        ) from exc
    validate_pure_geometry_profile(reference_endpoint)
    validate_parking_artifact(reference_parking)
    validate_pure_geometry_profile(reference_combined)
    reconstructed = derive_geometry_profile_with_path_plans(
        reference_endpoint,
        reference_parking,
    )
    if _semantic_profile_payload(reconstructed) != _semantic_profile_payload(
        reference_combined
    ):
        raise GeometryFullRunnerV5Error(
            "STOP: Benchmark C combined profile does not match its base/parking chain"
        )
    comparisons = {
        "endpoint_profile_semantic_payload_equal": (
            _semantic_profile_payload(reference_endpoint)
            == _semantic_profile_payload(endpoint_profile)
        ),
        "parking_semantic_payload_equal": (
            _semantic_parking_payload(reference_parking)
            == _semantic_parking_payload(parking_artifact)
        ),
        "combined_profile_semantic_payload_equal": (
            _semantic_profile_payload(reference_combined)
            == _semantic_profile_payload(combined_profile)
        ),
        "g4_replay_oracle_semantic_payload_equal": (
            _semantic_oracle_payload(reference_oracle)
            == _semantic_oracle_payload(oracle)
        ),
        "semantic_source_manifest_equal": (
            manifest.get("provenance", {}).get(
                "canonical_semantic_source_file_sha256"
            )
            == semantic_source_manifest
        ),
        "g4_replay_source_manifest_equal": (
            manifest.get("provenance", {}).get("g4_replay_source_file_sha256")
            == g4_replay_source_manifest
        ),
        "execution_source_manifest_equal": (
            manifest.get("provenance", {}).get("execution_source_file_sha256")
            == execution_source_manifest
        ),
        "input_file_manifest_equal": (
            manifest.get("provenance", {}).get("input_file_sha256")
            == input_file_manifest
        ),
    }
    if not all(comparisons.values()):
        raise GeometryFullRunnerV5Error(
            f"STOP: workers=1/workers=4 semantic determinism mismatch: {comparisons}"
        )
    return {
        "status": "PASS",
        "reference_manifest_relative_path": _relative(path, repo_root),
        "reference_manifest_file_sha256": manifest_file_sha,
        "reference_artifacts": bundle_snapshots,
        **comparisons,
    }


def _verify_determinism_reference_snapshot(
    determinism: dict[str, Any],
    repo_root: Path,
) -> None:
    if determinism.get("status") != "PASS":
        return
    manifest_path = _inside_repo(
        repo_root / determinism["reference_manifest_relative_path"],
        repo_root,
        label="determinism reference manifest",
    )
    if _sha256_file(manifest_path) != determinism.get(
        "reference_manifest_file_sha256"
    ):
        raise GeometryFullRunnerV5Error(
            "STOP: Benchmark C manifest changed before D publication"
        )
    for name, record in determinism.get("reference_artifacts", {}).items():
        artifact_path = _inside_repo(
            repo_root / record["relative_path"],
            repo_root,
            label=f"determinism reference {name}",
        )
        if _sha256_file(artifact_path) != record.get("file_sha256"):
            raise GeometryFullRunnerV5Error(
                f"STOP: Benchmark C artifact changed before D publication: {name}"
            )


def _artifact_record(path: Path, payload: bytes, repo_root: Path) -> dict[str, str]:
    return {
        "relative_path": _relative(path, repo_root),
        "file_sha256": _sha256_bytes(payload),
    }


def _compare_canonical_contacts_to_replay(
    canonical: tuple[EndpointAnalysisV5, ...],
    replay: tuple[EndpointAnalysisV5, ...],
) -> dict[str, Any]:
    """Prove that removing legacy context did not move active-pair contacts."""

    if len(canonical) != 24 or len(replay) != 24:
        raise GeometryFullRunnerV5Error(
            "STOP: pure/replay endpoint comparison requires 24/24 analyses"
        )
    replay_by_key = {
        (item.geometry.endpoint.joint_name, item.geometry.endpoint.side): item
        for item in replay
    }
    if len(replay_by_key) != 24:
        raise GeometryFullRunnerV5Error(
            "STOP: replay endpoint identities are not unique"
        )
    rows = []
    max_delta = 0.0
    for item in canonical:
        result = item.geometry
        key = (result.endpoint.joint_name, result.endpoint.side)
        reference = replay_by_key.get(key)
        if reference is None:
            raise GeometryFullRunnerV5Error(
                f"STOP: replay lacks canonical endpoint {result.endpoint.endpoint_id}"
            )
        other = reference.geometry
        tolerance = max(
            result.bisection_resolution_rad,
            other.bisection_resolution_rad,
        )
        if (
            result.status != other.status
            or result.contact_link_pair != other.contact_link_pair
            or (result.contact_angle_rad is None) != (other.contact_angle_rad is None)
        ):
            raise GeometryFullRunnerV5Error(
                f"STOP: pure-q0 contact identity/status differs from replay for "
                f"{result.endpoint.endpoint_id}"
            )
        delta = None
        if result.contact_angle_rad is not None:
            assert other.contact_angle_rad is not None
            delta = abs(result.contact_angle_rad - other.contact_angle_rad)
            if delta > tolerance:
                raise GeometryFullRunnerV5Error(
                    f"STOP: pure-q0 contact moved beyond declared resolution for "
                    f"{result.endpoint.endpoint_id}: delta={delta}, tolerance={tolerance}"
                )
            max_delta = max(max_delta, delta)
        rows.append(
            {
                "endpoint_id": result.endpoint.endpoint_id,
                "status": "PASS",
                "contact_angle_delta_rad": delta,
                "acceptance_tolerance_rad": tolerance,
            }
        )
    return {
        "status": "PASS",
        "endpoint_count": 24,
        "matching_endpoint_count": 24,
        "max_contact_angle_delta_rad": max_delta,
        "acceptance_basis": "MAX_DECLARED_BISECTION_RESOLUTION",
        "rows": rows,
    }


def run_integrated_geometry_v5(
    *,
    repo_root: Path,
    urdf_path: Path,
    g4_reference_profile_path: Path,
    output_prefix: Path,
    benchmark_id: str,
    workers: int,
    parameters: ParkingPlannerParametersV5,
    determinism_reference_manifest: Path | None = None,
    require_cgroup_v2: bool = False,
    required_memory_max_bytes: int | None = None,
    required_memory_swap_max_bytes: int | None = None,
    required_cpu_affinity: tuple[int, ...] | None = None,
) -> dict[str, Any]:
    repo_root = Path(repo_root).resolve()
    urdf_path = _inside_repo(urdf_path, repo_root, label="URDF")
    g4_path = _inside_repo(
        g4_reference_profile_path,
        repo_root,
        label="frozen G4 reference",
    )
    paths = integrated_output_paths(output_prefix, repo_root)
    if urdf_path in paths.all() or g4_path in paths.all():
        raise GeometryFullRunnerV5Error("STOP: input/output alias detected")
    if benchmark_id not in {"C", "D"}:
        raise GeometryFullRunnerV5Error("STOP: benchmark-id must be C or D")
    if (benchmark_id, workers) not in {("C", 1), ("D", 4)}:
        raise GeometryFullRunnerV5Error(
            "STOP: Benchmark C requires workers=1 and Benchmark D requires workers=4"
        )
    if benchmark_id == "D" and determinism_reference_manifest is None:
        raise GeometryFullRunnerV5Error(
            "STOP: Benchmark D requires the Benchmark C determinism manifest"
        )
    if benchmark_id == "D" and (
        not require_cgroup_v2
        or required_memory_max_bytes != CANONICAL_MEMORY_MAX_BYTES
        or required_memory_swap_max_bytes != CANONICAL_MEMORY_SWAP_MAX_BYTES
        or required_cpu_affinity != CANONICAL_PHYSICAL_CPU_AFFINITY
    ):
        raise GeometryFullRunnerV5Error(
            "STOP: Benchmark D requires cgroup v2 MemoryMax=6 GiB, "
            "MemorySwapMax=0 and physical CPU affinity 0-3"
        )
    if benchmark_id == "C" and determinism_reference_manifest is not None:
        raise GeometryFullRunnerV5Error(
            "STOP: Benchmark C cannot consume a determinism reference"
        )
    if not require_cgroup_v2 and (
        required_memory_max_bytes is not None
        or required_memory_swap_max_bytes is not None
    ):
        raise GeometryFullRunnerV5Error(
            "STOP: cgroup memory limits cannot be required without --require-cgroup-v2"
        )
    if required_cpu_affinity is not None and sorted(os.sched_getaffinity(0)) != list(
        required_cpu_affinity
    ):
        raise GeometryFullRunnerV5Error(
            "STOP: process CPU affinity does not match the required inherited "
            f"scheduler mask: expected={list(required_cpu_affinity)}, "
            f"observed={sorted(os.sched_getaffinity(0))}"
        )
    parameters.validate()

    total_started = time.perf_counter()
    generated_at = datetime.now(timezone.utc).isoformat()
    parent_pid = os.getpid()
    initial_sources = geometry_source_manifest(FINAL_SOURCE_FILES)
    initial_replay_sources = geometry_source_manifest(G4_REPLAY_SOURCE_FILES)
    initial_execution_sources = geometry_source_manifest(
        FINAL_EXECUTION_SOURCE_FILES
    )
    initial_g4_file_sha = _sha256_file(g4_path)
    if initial_g4_file_sha != EXPECTED_G4_FILE_SHA256:
        raise GeometryFullRunnerV5Error(
            "STOP: frozen G4 file SHA mismatch: "
            f"expected={EXPECTED_G4_FILE_SHA256}, observed={initial_g4_file_sha}"
        )

    preflight_scene = RobotSceneV5.from_urdf(urdf_path)
    initial_fingerprint = scene_input_fingerprint(preflight_scene)
    input_file_paths = (
        urdf_path,
        *(
            preflight_scene.mesh(link_name).stl_path
            for link_name in preflight_scene.model.collision_link_names
        ),
    )
    initial_input_files = _input_file_manifest(input_file_paths, repo_root)
    actuated_joint_names = preflight_scene.model.actuated_joint_names
    triangle_count = sum(
        preflight_scene.mesh(link_name).triangle_count
        for link_name in preflight_scene.model.collision_link_names
    )
    del preflight_scene
    clear_mesh_cache()

    compiler_run = run_geometry_compiler_v5(
        repo_root=repo_root,
        urdf_path=urdf_path,
        workers=workers,
        include_path_obstruction=True,
        source_files=FINAL_SOURCE_FILES,
    )
    endpoint_profile = compiler_run.profile
    canonical_contexts = [
        record.get("search_context", {}).get("joint_positions_rad")
        for record in endpoint_profile.get("endpoint_searches", [])
    ]
    if len(canonical_contexts) != 24 or any(context != {} for context in canonical_contexts):
        raise GeometryFullRunnerV5Error(
            "STOP: canonical endpoint profile is not 24/24 context-free"
        )

    replay_run = run_g4_replay_v5(
        repo_root=repo_root,
        urdf_path=urdf_path,
        g4_profile_path=g4_path,
        workers=workers,
        source_files=G4_REPLAY_SOURCE_FILES,
    )
    oracle = replay_run.comparison
    if oracle.get("status") != "PASS":
        raise GeometryFullRunnerV5Error("STOP: separated G4 replay oracle did not PASS")
    pure_replay_contacts = _compare_canonical_contacts_to_replay(
        compiler_run.analyses,
        replay_run.analyses,
    )
    endpoint_profile["generation_metadata"]["integrated_execution"] = {
        "benchmark_id": benchmark_id,
        "parent_writer_pid": parent_pid,
        "worker_model": "single_parent_process" if workers == 1 else "spawn_process_pool",
        "batch_strategy": "12_topology_derived_actuated_joint_batches_min_then_max",
        "canonical_result_sort_key": ["canonical_endpoint_index"],
        "numerical_thread_limits": {
            name: 1 for name in THREAD_ENVIRONMENT_NAMES
        },
    }
    validate_pure_geometry_profile(endpoint_profile)
    endpoint_payload = _json_bytes(endpoint_profile)
    endpoint_file_sha = _sha256_bytes(endpoint_payload)

    scene = RobotSceneV5.from_urdf(urdf_path)
    if scene_input_fingerprint(scene) != initial_fingerprint:
        raise GeometryFullRunnerV5Error(
            "STOP: input fingerprint changed between endpoint and parking phases"
        )
    mismatches = find_geometry_mismatches_v5(
        endpoint_profile,
        scene,
        repo_root=repo_root,
        source_files=FINAL_SOURCE_FILES,
    )
    if mismatches:
        raise GeometryFullRunnerV5Error(
            f"STOP: endpoint profile live provenance mismatch: {mismatches}"
        )
    parking_tasks = endpoint_parking_tasks_from_profile(scene, endpoint_profile)
    if scene.model.actuated_joint_names != actuated_joint_names:
        raise GeometryFullRunnerV5Error("STOP: actuated model ordering changed")
    del scene
    clear_mesh_cache()

    parking_started = time.perf_counter()
    plans = execute_parking_tasks(
        urdf_path,
        actuated_joint_names,
        initial_fingerprint,
        parking_tasks,
        workers=workers,
        parameters=parameters,
    )
    parking_runtime = time.perf_counter() - parking_started
    path_consistency = validate_endpoint_path_plan_consistency(
        endpoint_profile,
        plans,
        path_step_rad=parameters.path_step_rad,
    )

    postflight_scene = RobotSceneV5.from_urdf(urdf_path)
    if scene_input_fingerprint(postflight_scene) != initial_fingerprint:
        raise GeometryFullRunnerV5Error(
            "STOP: URDF/mesh/q0 fingerprint changed during integrated execution"
        )
    postflight_mismatches = find_geometry_mismatches_v5(
        endpoint_profile,
        postflight_scene,
        repo_root=repo_root,
        source_files=FINAL_SOURCE_FILES,
    )
    if postflight_mismatches:
        raise GeometryFullRunnerV5Error(
            f"STOP: post-parking provenance mismatch: {postflight_mismatches}"
        )
    del postflight_scene
    clear_mesh_cache()
    if (
        geometry_source_manifest(FINAL_SOURCE_FILES) != initial_sources
        or geometry_source_manifest(G4_REPLAY_SOURCE_FILES)
        != initial_replay_sources
        or geometry_source_manifest(FINAL_EXECUTION_SOURCE_FILES)
        != initial_execution_sources
        or _sha256_file(g4_path) != initial_g4_file_sha
    ):
        raise GeometryFullRunnerV5Error(
            "STOP: source or frozen G4 provenance changed during integrated execution"
        )

    parking_artifact = build_parking_artifact_v2(
        plans,
        geometry_profile_semantic_sha256=endpoint_profile[
            "semantic_content_sha256"
        ],
        audit_geometry_profile_reference={
            "relative_path": _relative(paths.endpoint_profile, repo_root),
            "file_sha256": endpoint_file_sha,
        },
        source_file_sha256=initial_sources,
        parameters=parameters,
        worker_count=workers,
        runtime_seconds=parking_runtime,
    )
    parking_artifact["generation_metadata"]["execution_audit"] = {
        "parent_writer_pid": parent_pid,
        "worker_model": "single_parent_process" if workers == 1 else "spawn_process_pool",
        "batch_strategy": "12_topology_derived_actuated_joint_batches_min_then_max",
        "batch_count": 12,
        "canonical_result_sort_key": ["canonical_endpoint_index"],
        "numerical_thread_limits": {
            name: 1 for name in THREAD_ENVIRONMENT_NAMES
        },
    }
    validate_parking_artifact(parking_artifact)
    expected_refinement_contract = {
        "obstruction_bisection_resolution_rad": (
            DEFAULT_OBSTRUCTION_BISECTION_RESOLUTION_RAD
        ),
        "max_obstruction_bisection_iterations": (
            DEFAULT_MAX_OBSTRUCTION_BISECTION_ITERATIONS
        ),
    }
    if any(
        parking_artifact["parameters"].get(key) != value
        for key, value in expected_refinement_contract.items()
    ):
        raise GeometryFullRunnerV5Error(
            "STOP: canonical parking artifact lacks the required obstruction "
            "refinement contract"
        )
    combined_profile = derive_geometry_profile_with_path_plans(
        endpoint_profile,
        parking_artifact,
    )
    combined_profile["generation_metadata"]["integrated_execution"][
        "parking_runtime_seconds"
    ] = round(parking_runtime, 6)
    validate_pure_geometry_profile(combined_profile)

    determinism = {"status": "REFERENCE_CAPTURED"}
    if determinism_reference_manifest is not None:
        determinism = _compare_with_reference(
            reference_manifest_path=determinism_reference_manifest,
            repo_root=repo_root,
            endpoint_profile=endpoint_profile,
            parking_artifact=parking_artifact,
            combined_profile=combined_profile,
            oracle=oracle,
            semantic_source_manifest=initial_sources,
            g4_replay_source_manifest=initial_replay_sources,
            execution_source_manifest=initial_execution_sources,
            input_file_manifest=initial_input_files,
        )

    endpoint_report = render_geometry_report_v5(endpoint_profile).encode("utf-8")
    oracle_payload = _json_bytes(oracle)
    oracle_report = render_g4_g7_report(oracle).encode("utf-8")
    parking_payload = _json_bytes(parking_artifact)
    parking_report = render_parking_report(parking_artifact).encode("utf-8")
    combined_payload = _json_bytes(combined_profile)
    combined_report = render_geometry_report_v5(combined_profile).encode("utf-8")
    materialized_payloads = {
        paths.endpoint_profile: endpoint_payload,
        paths.endpoint_report: endpoint_report,
        paths.oracle_json: oracle_payload,
        paths.oracle_report: oracle_report,
        paths.parking_json: parking_payload,
        paths.parking_report: parking_report,
        paths.combined_profile: combined_payload,
        paths.combined_report: combined_report,
    }
    artifacts = {
        name: _artifact_record(getattr(paths, name), materialized_payloads[getattr(paths, name)], repo_root)
        for name in (
            "endpoint_profile",
            "endpoint_report",
            "oracle_json",
            "oracle_report",
            "parking_json",
            "parking_report",
            "combined_profile",
            "combined_report",
        )
    }
    evaluated_1d = sum(plan.evaluated_1d_candidates for plan in plans)
    evaluated_2d = sum(plan.evaluated_2d_candidates for plan in plans)
    grid_entries = sum(
        sum(len(values) for values in plan.one_dof_search_grid_rad.values())
        + sum(len(values) for values in plan.two_dof_search_grid_rad.values())
        for plan in plans
    )
    final_scene = RobotSceneV5.from_urdf(urdf_path)
    if scene_input_fingerprint(final_scene) != initial_fingerprint:
        raise GeometryFullRunnerV5Error(
            "STOP: final URDF/mesh/q0 fingerprint changed before publication"
        )
    del final_scene
    clear_mesh_cache()
    total_runtime = time.perf_counter() - total_started
    cgroup = _read_cgroup_v2() if require_cgroup_v2 else None
    if cgroup is not None:
        _validate_cgroup_contract(
            cgroup,
            required_memory_max_bytes=required_memory_max_bytes,
            required_memory_swap_max_bytes=required_memory_swap_max_bytes,
            required_cpu_affinity=required_cpu_affinity,
        )
    manifest: dict[str, Any] = {
        "schema_version": RUN_SCHEMA_VERSION,
        "generated_at_utc": generated_at,
        "benchmark_id": benchmark_id,
        "execution": {
            "parent_writer_pid": parent_pid,
            "worker_count": workers,
            "worker_model": "single_parent_process" if workers == 1 else "spawn_process_pool",
            "physical_core_target": workers,
            "numerical_thread_limits": {
                name: 1 for name in THREAD_ENVIRONMENT_NAMES
            },
            "batch_strategy": "12_topology_derived_actuated_joint_batches_min_then_max_dynamic_scheduling",
            "canonical_result_sort_key": ["canonical_endpoint_index"],
        },
        "provenance": {
            "urdf_relative_path": _relative(urdf_path, repo_root),
            "urdf_sha256": initial_fingerprint.urdf_sha256,
            "collision_mesh_sha256": dict(initial_fingerprint.collision_mesh_sha256),
            "q0_status_by_joint": dict(initial_fingerprint.q0_status_by_joint),
            "g4_reference_relative_path": _relative(g4_path, repo_root),
            "g4_reference_file_sha256": initial_g4_file_sha,
            "g4_reference_content_sha256": EXPECTED_G4_CONTENT_SHA256,
            "input_file_sha256": initial_input_files,
            "canonical_semantic_source_file_sha256": initial_sources,
            "g4_replay_source_file_sha256": initial_replay_sources,
            "execution_source_file_sha256": initial_execution_sources,
        },
        "parameters": {
            "path_step_rad": parameters.path_step_rad,
            "obstruction_bisection_resolution_rad": (
                DEFAULT_OBSTRUCTION_BISECTION_RESOLUTION_RAD
            ),
            "max_obstruction_bisection_iterations": (
                DEFAULT_MAX_OBSTRUCTION_BISECTION_ITERATIONS
            ),
            "one_dof_grid_divisions": parameters.one_dof_grid_divisions,
            "two_dof_grid_divisions": parameters.two_dof_grid_divisions,
            "clearance_sample_stride": parameters.clearance_sample_stride,
        },
        "gates": {
            "q0_active_revolute_pairs_separated": "12/12",
            "g4_g7_oracle_status": oracle["status"],
            "canonical_endpoint_coverage": "24/24",
            "canonical_context_free_endpoint_searches": "24/24",
            "pure_q0_vs_g4_replay_contacts": "24/24",
            "endpoint_path_vs_parking_baseline_consistency": "24/24",
            "worker_writes": 0,
            "parent_bundle_publications": 1,
        },
        "semantic_content_sha256": {
            "endpoint_profile": endpoint_profile["semantic_content_sha256"],
            "parking_v2": parking_artifact["semantic_content_sha256"],
            "combined_profile": combined_profile["semantic_content_sha256"],
        },
        "determinism": determinism,
        "timings_seconds": {
            "canonical_endpoint_compute": round(compiler_run.runtime_seconds, 6),
            "noncanonical_g4_replay_compute": round(
                replay_run.runtime_seconds, 6
            ),
            "parking_compute": round(parking_runtime, 6),
            "integrated_internal_wall": round(total_runtime, 6),
        },
        "resources": {
            "cgroup_v2": cgroup,
            "triangle_count": triangle_count,
            "grid_entries_serialized": grid_entries,
            "evaluated_1d_candidates": evaluated_1d,
            "evaluated_2d_candidates": evaluated_2d,
            "collision_candidate_pairs": "NOT_INSTRUMENTED",
            "aabb_surviving_pairs": "NOT_INSTRUMENTED",
        },
        "parking_summary": parking_artifact["summary"],
        "path_consistency": path_consistency,
        "pure_q0_vs_g4_replay_contacts": pure_replay_contacts,
        "oracle_summary": {
            key: oracle[key]
            for key in (
                "endpoint_count",
                "geometric_contact_found_count",
                "no_geometric_contact_count",
                "path_obstruction_count",
                "path_obstruction_precedes_contact_count",
            )
        },
        "artifacts": artifacts,
        "constraints": {
            "hardware_used": False,
            "norma_core_modified": False,
            "merge_performed": False,
            "collision_candidate_cap_changed": False,
            "safety_threshold_relaxed": False,
        },
        "resource_enforcement": {
            "memory_swap_oom": (
                "CGROUP_V2_VALIDATED"
                if cgroup is not None
                else "NOT_REQUIRED_NOT_ENFORCED"
            ),
            "cpu_affinity": (
                "PROCESS_SCHED_AFFINITY_INHERITED_BY_SPAWN_WORKERS"
                if required_cpu_affinity is not None
                else "NOT_REQUIRED_NOT_ENFORCED"
            ),
            "cpuset_cpus_effective_role": (
                "TELEMETRY_ONLY" if cgroup is not None else "NOT_COLLECTED"
            ),
        },
        "bundle_validity": {
            "marker": "RUN_MANIFEST_PUBLISHED_LAST",
            "manifest_required": True,
            "artifact_count": len(EXPECTED_BUNDLE_ARTIFACT_KEYS),
            "artifact_hash_algorithm": "SHA256",
        },
    }
    manifest["manifest_content_sha256"] = _manifest_content_sha256(manifest)
    manifest_payload = _json_bytes(manifest)
    materialized_payloads[paths.run_manifest] = manifest_payload

    if (
        geometry_source_manifest(FINAL_SOURCE_FILES) != initial_sources
        or geometry_source_manifest(G4_REPLAY_SOURCE_FILES)
        != initial_replay_sources
        or geometry_source_manifest(FINAL_EXECUTION_SOURCE_FILES)
        != initial_execution_sources
        or _input_file_manifest(input_file_paths, repo_root) != initial_input_files
        or _sha256_file(g4_path) != initial_g4_file_sha
    ):
        raise GeometryFullRunnerV5Error(
            "STOP: final source/G4 provenance check failed before publication"
        )
    _verify_determinism_reference_snapshot(determinism, repo_root)
    integrated_output_paths(output_prefix, repo_root)
    _publish_no_clobber_bundle(
        materialized_payloads,
        manifest_path=paths.run_manifest,
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MATDOG Geometry Compiler V5 integrated C/D benchmark. Offline only."
    )
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--g4-reference-profile", type=Path, required=True)
    parser.add_argument("--artifact-prefix", type=Path, required=True)
    parser.add_argument("--benchmark-id", choices=("C", "D"), required=True)
    parser.add_argument("--workers", type=int, choices=(1, 4), required=True)
    parser.add_argument("--determinism-reference-manifest", type=Path)
    parser.add_argument("--path-step-rad", type=float, default=math.radians(1.0))
    parser.add_argument("--one-dof-grid-divisions", type=int, default=6)
    parser.add_argument("--two-dof-grid-divisions", type=int, default=4)
    parser.add_argument("--clearance-sample-stride", type=int, default=10)
    parser.add_argument("--require-cgroup-v2", action="store_true")
    parser.add_argument("--require-memory-max-bytes", type=int)
    parser.add_argument("--require-memory-swap-max-bytes", type=int)
    parser.add_argument("--require-cpu-affinity", type=_parse_cpu_affinity)
    args = parser.parse_args()

    parameters = ParkingPlannerParametersV5(
        path_step_rad=args.path_step_rad,
        one_dof_grid_divisions=args.one_dof_grid_divisions,
        two_dof_grid_divisions=args.two_dof_grid_divisions,
        clearance_sample_stride=args.clearance_sample_stride,
    )
    manifest = run_integrated_geometry_v5(
        repo_root=args.repo_root,
        urdf_path=args.urdf,
        g4_reference_profile_path=args.g4_reference_profile,
        output_prefix=args.artifact_prefix,
        benchmark_id=args.benchmark_id,
        workers=args.workers,
        parameters=parameters,
        determinism_reference_manifest=args.determinism_reference_manifest,
        require_cgroup_v2=args.require_cgroup_v2,
        required_memory_max_bytes=args.require_memory_max_bytes,
        required_memory_swap_max_bytes=args.require_memory_swap_max_bytes,
        required_cpu_affinity=args.require_cpu_affinity,
    )
    print("=== MATDOG GEOMETRY COMPILER V5 — INTEGRATED BENCHMARK ===")
    print(f"benchmark_id: {manifest['benchmark_id']}")
    print(f"workers: {manifest['execution']['worker_count']}")
    print(f"q=0 active pairs separated: {manifest['gates']['q0_active_revolute_pairs_separated']}")
    print(f"G4/G7 oracle: {manifest['gates']['g4_g7_oracle_status']}")
    for name, digest in manifest["semantic_content_sha256"].items():
        print(f"{name}_semantic_sha256: {digest}")
    print(
        "integrated_internal_wall_seconds: "
        f"{manifest['timings_seconds']['integrated_internal_wall']:.6f}"
    )
    print(f"determinism: {manifest['determinism']['status']}")
    print("NO HARDWARE USED. NO NORMA-CORE MODIFIED. NO MERGE PERFORMED.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

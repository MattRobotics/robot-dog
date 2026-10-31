#!/usr/bin/env python3
"""Historical calibration geometry basis (REV00) vs the operational model (REV01-FIX).

The hardware calibration chain (Geometry V5 bundle, CalibrationGeometryProfileData.h,
CalibrationSequencePlanData.h, startup reference, gen-1 NVS record) is bound to the REV00
inputs by SHA256 and by the geometry tag 3713f4ddc43b204e. Those artifacts are preserved
unchanged; this module lets their provenance checks run against the preserved REV00 inputs
instead of the live operational model, and proves the operational model differs from REV00
only by a closed, declared delta. It computes no geometry and validates nothing physically.

    python3 matdog_calibration_basis.py [--repo-root DIR] --check
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_REPO_ROOT = Path(__file__).resolve().parents[3]
BASIS_DIR = "03_CAD/URDF/historical_calibration_basis/rev00"
SCHEMA = "matdog.calibration_geometry_basis.v1"
URDF_NAME = "matt_robodog_rev00.urdf"
MANIFEST_NAME = "SHA256SUMS.txt"
COLLISION_PREFIX = "meshes/collision/"


class BasisError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_basis(repo_root: Path = DEFAULT_REPO_ROOT) -> dict:
    path = repo_root / BASIS_DIR / "BASIS.json"
    try:
        basis = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise BasisError(f"calibration basis unreadable: {path} ({exc})")
    if basis.get("schema") != SCHEMA or not basis.get("files"):
        raise BasisError(f"{path}: not a {SCHEMA} document")
    return basis


def model_dir(repo_root: Path, basis: dict) -> Path:
    return repo_root / basis["operational_model_dir"]


def basis_file(repo_root: Path, rel_in_model: str) -> Path:
    """The preserved REV00 copy of a superseded file. Its hash is verified on every use."""
    basis = load_basis(repo_root)
    record = basis["files"].get(rel_in_model)
    if record is None:
        raise BasisError(f"{rel_in_model} is not a declared REV00 basis file")
    path = repo_root / BASIS_DIR / rel_in_model
    if not path.is_file():
        raise BasisError(f"preserved REV00 basis file missing: {path}")
    actual = sha256_file(path)
    if actual != record["sha256"]:
        raise BasisError(f"preserved REV00 basis file altered: {rel_in_model}\n"
                         f"  declared {record['sha256']}\n  actual   {actual}")
    return path


def resolve_recorded_input(repo_root: Path, rel_repo_path: str, expected_sha: str) -> Path:
    """Path whose bytes hash to `expected_sha`. The live file when it still matches; the
    preserved REV00 copy only for the closed set of declared superseded files. Anything
    else that drifted stays a hard failure."""
    live = repo_root / rel_repo_path
    if live.is_file() and sha256_file(live) == expected_sha:
        return live
    basis = load_basis(repo_root)
    prefix = basis["operational_model_dir"] + "/"
    if rel_repo_path.startswith(prefix):
        rel = rel_repo_path[len(prefix):]
        if rel in basis["files"]:
            archived = basis_file(repo_root, rel)
            if sha256_file(archived) == expected_sha:
                return archived
    actual = sha256_file(live) if live.is_file() else "<missing>"
    raise BasisError(f"input drift: {rel_repo_path}\n  manifest {expected_sha}\n  actual   {actual}")


def _canon(element: ET.Element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(),
            tuple(_canon(child) for child in element))


def _manifest_entries(path: Path) -> dict[str, str]:
    out = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split(None, 1)
        if len(parts) == 2:
            out[parts[1].strip().lstrip("*")] = parts[0]
    return out


def verify_delta(repo_root: Path = DEFAULT_REPO_ROOT) -> list[str]:
    """Empty list = the operational model is REV00 plus exactly the declared delta."""
    problems: list[str] = []
    try:
        basis = load_basis(repo_root)
        for rel in basis["files"]:
            basis_file(repo_root, rel)
    except BasisError as exc:
        return [str(exc)]
    live_dir = model_dir(repo_root, basis)
    delta = basis["allowed_urdf_delta"]
    tol = float(delta["tolerance_m"])
    old_prefix, new_prefix = delta["visual_mesh_prefix"]

    rev00 = ET.parse(repo_root / BASIS_DIR / URDF_NAME).getroot()
    live = ET.parse(live_dir / URDF_NAME).getroot()

    seen = set()
    for joint in live.findall("joint"):
        name = joint.get("name")
        if name not in delta["hip_origin_z_m"]:
            continue
        seen.add(name)
        old_z, new_z = delta["hip_origin_z_m"][name]
        origin = joint.find("origin")
        x, y, z = (float(v) for v in origin.get("xyz").split())
        reference = next(j for j in rev00.findall("joint") if j.get("name") == name).find("origin")
        rx, ry, rz = (float(v) for v in reference.get("xyz").split())
        if abs(rz - old_z) > tol:
            problems.append(f"{name}: REV00 basis z {rz} != declared {old_z}")
        if abs(z - new_z) > tol:
            problems.append(f"{name}: operational z {z} != declared {new_z}")
        if abs(x - rx) > tol or abs(y - ry) > tol:
            problems.append(f"{name}: x/y changed ({x},{y}) vs REV00 ({rx},{ry})")
        origin.set("xyz", reference.get("xyz"))        # neutralise only the declared delta
    if seen != set(delta["hip_origin_z_m"]):
        problems.append(f"declared hip joints not found in the operational URDF: "
                        f"{sorted(set(delta['hip_origin_z_m']) - seen)}")

    for link in live.findall("link"):
        visual = link.find("visual")
        if visual is None:
            continue
        mesh = visual.find("geometry/mesh")
        filename = mesh.get("filename")
        if not filename.startswith(new_prefix):
            problems.append(f"{link.get('name')}: visual mesh {filename} is not under {new_prefix}")
            continue
        mesh.set("filename", old_prefix + filename[len(new_prefix):])   # neutralise declared delta

    if _canon(live) != _canon(rev00):
        problems.append("operational URDF differs from the REV00 basis beyond the declared delta "
                        "(hip origin Z + visual mesh paths): joint axes/limits/motor metadata, "
                        "inertials, collision geometry or any other element changed")

    # Collision meshes: every one except the declared superseded set must still be REV00's.
    rev00_manifest = _manifest_entries(repo_root / BASIS_DIR / MANIFEST_NAME)
    collision = {k: v for k, v in rev00_manifest.items()
                 if k.startswith(COLLISION_PREFIX) and k.endswith(".stl")}
    if len(collision) != 17:
        problems.append(f"REV00 manifest lists {len(collision)} collision meshes, expected 17")
    for rel, expected in sorted(collision.items()):
        if rel in basis["files"]:
            if expected != basis["files"][rel]["sha256"]:
                problems.append(f"{rel}: REV00 manifest disagrees with BASIS.json")
            continue
        path = live_dir / rel
        if not path.is_file() or sha256_file(path) != expected:
            problems.append(f"{rel}: not identical to the REV00 collision mesh")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--repo-root", type=Path, default=DEFAULT_REPO_ROOT)
    parser.add_argument("--check", action="store_true", required=True)
    args = parser.parse_args()
    problems = verify_delta(args.repo_root.resolve())
    if problems:
        print("CALIBRATION_BASIS = FAIL", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1
    print("CALIBRATION_BASIS = PASS (operational model = REV00 + declared delta only; "
          "preserved REV00 basis files hash-verified)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

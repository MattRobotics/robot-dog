"""Candidate field mapping to pinned PR #50. No G5-B/runtime wiring."""
import hashlib
import json

from model import ROOT, URDF_REL, at_pose

PR50_SHA = "0529e44303467c454b07eb7f0e24bedd07182f08"
HEADER_REL = "06_Software/Matdog_Core/stabilization_g5b/InertialInput.h"


def canonical_json(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def string_field(value, capacity):
    if not isinstance(value, str) or "\0" in value or len(value.encode("utf-8")) >= capacity:
        raise ValueError(f"C string requires fewer than {capacity} UTF-8 bytes")
    return value


def candidate_payload(links, pose, assembly_revision, provenance, readiness, source_hashes=None):
    if readiness not in {"HISTORICAL_REV00", "REV01_PENDING"}:
        raise ValueError("Offline exporter never emits REV01_VERIFIED")
    if len(links) != 17:
        raise ValueError("PR50 contract requires the bounded 17-link representation")
    total = at_pose(links, pose)
    payload = {"readiness": readiness,
               "assemblyRevision": string_field(assembly_revision, 32),
               "provenance": string_field(provenance, 96),
               "totalMassKg": total.mass, "totalComBaseM": total.com,
               "totalInertiaAtComBaseKgM2": total.inertia,
               "links": [{"linkName": string_field(b.name, 32), "massKg": b.mass,
                          "comInLinkM": b.com, "inertiaAtComKgM2": b.inertia}
                         for b in links.values()],
               "linkCount": len(links), "measuredAtS": 0.}
    # Canonical companion defines the pose and local inputs used for this aggregate.
    companion = {"schema": 1, "pinned_pr50_commit": PR50_SHA,
                 "semantic_joint_positions_rad": pose, "payload_without_hash": dict(payload),
                 "source_hashes": source_hashes if source_hashes is not None else {
                     URDF_REL: hashlib.sha256((ROOT/URDF_REL).read_bytes()).hexdigest()},
                 "measuredAtS_meaning": "0 = unavailable; historical/offline data, no fresh measurement",
                 "readiness_policy": "Never REV01_VERIFIED. Future qualification and time-domain mapping required."}
    payload["sourceSha256"] = hashlib.sha256(canonical_json(companion).encode()).hexdigest()
    return {"payload": payload, "hash_preimage": companion, "runtime_eligible": False}


def cpp_probe(candidate):
    """Host fixture compiled with the real PR50 header and transitive includes."""
    p = candidate["payload"]
    q = json.dumps
    lines = ['#include "InertialInput.h"', '#include <cassert>', '#include <cstring>',
             '#include <cmath>', 'using namespace matdog::stabilization;', 'int main() {',
             'static_assert(kMaxInertialLinks == 17);', 'InertialInput x{};',
             f'x.readiness = InertialRevision::{p["readiness"]};']
    for field, cap in (("assemblyRevision", 32), ("provenance", 96), ("sourceSha256", 65)):
        lines.extend([f'static_assert(sizeof(x.{field}) == {cap});',
                      f'std::strcpy(x.{field}, {q(p[field])});',
                      f'assert(std::strcmp(x.{field}, {q(p[field])}) == 0);'])
    lines.extend([f'x.linkCount = {p["linkCount"]};', f'x.totalMassKg = {p["totalMassKg"]!r};',
                  f'x.measuredAtS = {p["measuredAtS"]!r};'])
    for k, axis in enumerate("xyz"):
        lines.append(f'x.totalComBaseM.{axis} = {p["totalComBaseM"][k]!r};')
    for i in range(3):
        for j in range(3):
            lines.append(f'x.totalInertiaAtComBaseKgM2[{i}][{j}] = {p["totalInertiaAtComBaseKgM2"][i][j]!r};')
    for k, b in enumerate(p["links"]):
        lines.extend([f'std::strcpy(x.links[{k}].linkName, {q(b["linkName"])});',
                      f'x.links[{k}].massKg = {b["massKg"]!r};'])
        for i, axis in enumerate("xyz"):
            lines.append(f'x.links[{k}].comInLinkM.{axis} = {b["comInLinkM"][i]!r};')
        for i in range(3):
            for j in range(3):
                lines.append(f'x.links[{k}].inertiaAtComKgM2[{i}][{j}] = {b["inertiaAtComKgM2"][i][j]!r};')
    lines.extend(['double mass = 0;', 'for (unsigned k=0; k<x.linkCount; ++k) {',
                  'assert(std::strlen(x.links[k].linkName) < 32);', 'mass += x.links[k].massKg;',
                  'for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j)',
                  'assert(x.links[k].inertiaAtComKgM2[i][j] == x.links[k].inertiaAtComKgM2[j][i]);',
                  '}', 'assert(std::abs(mass-x.totalMassKg) < 1e-12);',
                  'assert(x.readiness != InertialRevision::REV01_VERIFIED);',
                  'SupportProjectionResult result{};',
                  'assert(result.status == SupportProjectionResult::Status::DEPENDENCY_MISSING);',
                  'return 0;', '}'])
    return "\n".join(lines) + "\n"

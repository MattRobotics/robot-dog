"""Read-only OOXML/URDF audit. Formula text and cached values stay distinct."""
from __future__ import annotations

import hashlib
from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as ET
from zipfile import ZipFile

from model import ROOT, URDF_REL, load_baseline, validate_tensor, rotate_tensor, matvec

XLSX_REL = "03_CAD/URDF/matt_robodog_rev00/reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx"
NS = {"s": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_xlsx(path):
    with ZipFile(path) as archive:
        strings = []
        if "xl/sharedStrings.xml" in archive.namelist():
            root = ET.fromstring(archive.read("xl/sharedStrings.xml"))
            strings = ["".join(n.itertext()) for n in root.findall("s:si", NS)]
        rels = ET.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
        targets = {r.attrib["Id"]: r.attrib["Target"] for r in rels}
        root = ET.fromstring(archive.read("xl/workbook.xml"))
        sheets = {}
        for sheet in root.findall("s:sheets/s:sheet", NS):
            rid = sheet.attrib["{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"]
            target = targets[rid]
            target = str(PurePosixPath("xl") / target) if not target.startswith("/") else target[1:]
            xml = ET.fromstring(archive.read(target))
            cells = {}
            for c in xml.findall("s:sheetData/s:row/s:c", NS):
                v = c.find("s:v", NS)
                kind = c.get("t")
                if kind == "inlineStr":
                    value = "".join(n.text or "" for n in c.findall("s:is//s:t", NS))
                elif v is None or v.text is None:
                    value = None
                elif kind == "s":
                    value = strings[int(v.text)]
                elif kind in {"str", "e"}:
                    value = v.text
                elif kind == "b":
                    value = v.text == "1"
                else:
                    value = float(v.text)
                f = c.find("s:f", NS)
                if value is not None or f is not None:
                    cells[c.attrib["r"]] = {"value": value, "formula": f.text if f is not None else None}
            sheets[sheet.attrib["name"]] = cells
        return sheets


def source_audit(repo=ROOT):
    sheets = read_xlsx(repo / XLSX_REL)
    links = load_baseline(repo / URDF_REL)
    raw = ET.parse(repo / URDF_REL).getroot()
    inertial = sheets["Inertial_Properties"]
    expected_headers = {"D4": "Mass\n[kg]", "E4": "CoM X\n[m]", "H4": "Ixx\n[kg·m²]"}
    for cell, expected in expected_headers.items():
        if inertial[cell]["value"] != expected:
            raise ValueError("Unexpected workbook units/layout")
    extracted = []
    for row in range(5, 22):
        get = lambda col: inertial[f"{col}{row}"]["value"]
        name = get("A")
        b = links[name]
        values = [b.mass, *b.com, b.inertia[0][0], b.inertia[0][1], b.inertia[0][2],
                  b.inertia[1][1], b.inertia[1][2], b.inertia[2][2]]
        if any(abs(actual-get(col)) > 1e-15 for actual, col in zip(values, "DEFGHIJKLM")):
            raise ValueError(f"Excel/URDF mismatch for {name}")
        node = raw.find(f"link[@name='{name}']/inertial/origin")
        extracted.append({"link_name": name, "mass_kg": b.mass,
                          "com_in_link_m": b.com, "inertial_origin_rpy_rad": node.attrib["rpy"],
                          "inertia_at_com_link_kg_m2": b.inertia,
                          "historical_evidence": get("N"), "derivation": get("O"),
                          "source_files_named_in_workbook": get("P"),
                          "source_reports_available_in_repository": False,
                          "workbook_range": f"Inertial_Properties!A{row}:Q{row}",
                          "assembly_evidence": "HARDWARE UNVERIFIED",
                          "tensor_validation": validate_tensor(b.inertia)})
    if len(links) != 17 or {r["link_name"] for r in extracted} != set(links):
        raise ValueError("17-link coverage required")
    mirror = ((1., 0., 0.), (0., -1., 0.), (0., 0., 1.))
    symmetry = []
    for leg in ("rf", "rh", "lh"):
        for family in ("hip", "upper_leg", "lower_leg", "foot"):
            seed = links[f"lf_{family}_link"]
            b = links[f"{leg}_{family}_link"]
            transform = mirror if leg != "lh" and family != "foot" else ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))
            expected_i = rotate_tensor(seed.inertia, transform)
            expected_c = matvec(transform, seed.com)
            ok = b.mass == seed.mass and b.com == expected_c and b.inertia == expected_i
            if not ok:
                raise ValueError("Historical local symmetry mismatch")
            symmetry.append({"link": b.name, "rule": "local_Y_reflection" if transform == mirror else "local_copy",
                             "historical_algebra_pass": ok, "real_assembly_verified": False})
    scales = [node.get("scale") for node in raw.findall("link/visual/geometry/mesh") +
              raw.findall("link/collision/geometry/mesh")]
    if len(scales) != 34 or set(scales) != {"0.001 0.001 0.001"}:
        raise ValueError("REV00 mesh SI scaling changed")
    control = lambda cell: sheets["Mass_Reconciliation"][cell]["value"]
    historical_controls = {"CAD_mass_kg": control("B5"), "weighed_robot_kg": control("B9"),
                           "weight_breakdown_kg": control("B8"),
                           "unallocated_CAD_minus_weighed_kg": control("B5")-control("B9"),
                           "neck_yaw_historical_mass_kg": control("B12"),
                           "current_configuration_verified": False,
                           "workbook_cells": "Mass_Reconciliation!B5,B8,B9,B12"}
    if abs(sum(b.mass for b in links.values())-control("B5")) > 1e-12:
        raise ValueError("Historical mass summary mismatch")
    return {"urdf": {"path": URDF_REL, "sha256": sha(repo/URDF_REL)},
            "workbook": {"path": XLSX_REL, "sha256": sha(repo/XLSX_REL)},
            "links": extracted, "local_symmetry": symmetry,
            "units": {"URDF": "kg, m, rad, kg*m^2", "STL_vertices": "mm", "mesh_scale": 0.001,
                      "CAD_CoM_and_gyration_radii": "mm -> m; squared radii require 1e-6",
                      "CAD_raw_global_inertia_unit": "PENDING_MEASUREMENT; header lacks unit, not used"},
            "coordinates": "X forward, Y left, Z up; right-handed; R_parent_from_child=Rz Ry Rx",
            "tensor_convention": "URDF off-diagonal tensor entries, not raw CAD products of inertia",
            "historical_controls": historical_controls,
            "documentation_conflict": "REV00 README Physical Data Status says CoM/inertia are not yet authoritative; URDF and workbook contain a historical CAD model. Original sources preserved.",
            "workbook_cells": sheets}

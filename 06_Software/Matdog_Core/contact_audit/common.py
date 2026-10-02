"""Shared helpers for the G4.1 contact reconciliation tools. Offline only; reads canonical files, never writes them."""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / '09_Logs/Validation_Reports/G41_Contact_Reconciliation'
G4 = ROOT / '09_Logs/Validation_Reports/G4_Gait_Envelope'
URDF_DIR = ROOT / '03_CAD/URDF/matt_robodog_rev00'
LEGS = ('lf', 'rf', 'rh', 'lh')
UM = 1e6
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/kinematics'))
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/gait_audit'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(name, data):
    (OUT / name).write_text(json.dumps(data, indent=2, allow_nan=False, default=_ser) + '\n')


def load(name):
    return json.loads((OUT / name).read_text())


def _ser(v):
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, np.generic):
        return v.item()
    raise TypeError(type(v).__name__)


def g2_cylinder():
    """G2 analytic contact cylinder (unchanged contract): centre, unit axis, radius, in foot_link metres."""
    from matdog_foot_contact import load_foot_contact_model
    g = load_foot_contact_model(ROOT)
    return dict(center=np.array(g.cylinder_center_in_foot_link_m), axis=np.array(g.cylinder_axis_in_foot_link_unit),
                radius=float(g.cylinder_radius_m), tread_width=float(g.total_tread_width_m), fillet=float(g.end_fillet_radius_m),
                support_width=float(g.central_rigid_support_width_m), tilt=float(g.nominal_strip_max_axis_tilt_from_ground_rad))


def foot_vertices(kind, leg, merge=True):
    """Foot mesh vertices in foot_link metres, exactly as the URDF places them (collision/visual origin and scale read from the URDF)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(URDF_DIR / 'matt_robodog_rev00.urdf').getroot()
    link = next(l for l in root.findall('link') if l.get('name') == f'{leg}_foot_link')
    node = link.find('collision' if kind == 'collision' else 'visual')
    mesh = node.find('geometry/mesh')
    xyz = np.array([float(x) for x in node.find('origin').get('xyz').split()])
    rpy = [float(x) for x in node.find('origin').get('rpy').split()]
    assert rpy == [0.0, 0.0, 0.0], rpy
    scale = np.array([float(x) for x in mesh.get('scale').split()])
    m = trimesh.load_mesh(URDF_DIR / mesh.get('filename'), process=False)
    v = np.asarray(m.vertices, dtype=np.float64) * scale + xyz
    if merge:
        v = np.unique(np.round(v, 12), axis=0)
    return v, dict(file=mesh.get('filename'), sha256=sha(URDF_DIR / mesh.get('filename')), faces=int(len(m.faces)), raw_vertices=int(len(m.vertices)), origin_xyz=xyz.tolist(), scale=scale.tolist())


def delta_um(vertices, center, radius, phi_deg, dtype=np.float64):
    """Mesh lowest height above the ground (um) for foot pitch phi when the analytic cylinder bottom sits exactly on the ground.

    u = (sin phi, 0, cos phi) is world-up in foot_link; the analytic bottom is at -R along -u from the centre c,
    so  delta = R + min_v u.(v - c).  A function of pitch only (rotation about the y axis).
    """
    phi = np.radians(np.atleast_1d(phi_deg)).astype(dtype)
    rel = (vertices - center).astype(dtype)
    out = np.empty(len(phi), dtype=dtype)
    for i in range(0, len(phi), 2000):
        p = phi[i:i + 2000]
        out[i:i + 2000] = np.min(np.sin(p)[:, None] * rel[None, :, 0] + np.cos(p)[:, None] * rel[None, :, 2], axis=1) + dtype(radius)
    return out.astype(np.float64) * UM

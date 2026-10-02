"""Vectorised foot-contact reference used by the G4.1 offline audits.

Reproduces the G2 analytic contact (cross-section contact centre, central support strip ends) for an arbitrary
cylinder reference, so the unchanged G2 nominal reference and a candidate registered reference can be compared
without touching the accepted G2 files. `tests` check agreement with matdog_foot_contact.contact_from_foot_pose.
"""
from dataclasses import dataclass
import numpy as np
from common import g2_cylinder, delta_um, foot_vertices, UM

UP = np.array([0.0, 0.0, 1.0])


@dataclass(frozen=True)
class CylinderRef:
    name: str
    center: np.ndarray
    axis: np.ndarray
    radius: float
    support_width: float

    def with_center(self, name, center, radius=None):
        return CylinderRef(name, np.asarray(center, dtype=np.float64), self.axis, self.radius if radius is None else radius, self.support_width)


def nominal():
    g = g2_cylinder()
    return CylinderRef('G2_NOMINAL', g['center'], g['axis'], g['radius'], g['support_width'])


def contact(tf_foot, ref):
    """World contact geometry for one foot: (cross_section_contact_centre, strip_end_a, strip_end_b)."""
    R_wf, t = tf_foot[:3, :3], tf_foot[:3, 3]
    axis = R_wf @ ref.axis
    down = -(UP - axis * (axis @ UP))
    down /= np.linalg.norm(down)
    centre_world = R_wf @ ref.center + t
    point = centre_world + ref.radius * down
    half = ref.axis * 0.5 * ref.support_width
    return point, point + R_wf @ half, point - R_wf @ half


def contacts_all(tf, ref):
    return {leg: contact(tf[leg + '_foot_link'], ref) for leg in ('lf', 'rf', 'rh', 'lh')}


class TreadMesh:
    """Foot collision vertices in foot_link, split into the tread surface (represented analytically) and the rest."""

    def __init__(self, ref=None, band=0.05e-3):
        ref = ref or nominal()
        v, self.meta = foot_vertices('collision', 'lf')
        r = np.hypot(v[:, 0] - ref.center[0], v[:, 2] - ref.center[2])
        self.tread = v[r >= ref.radius - band]
        self.rest = v[r < ref.radius - band]
        self.all = v

    @staticmethod
    def min_height(tf_foot, vertices):
        return float((vertices @ tf_foot[:3, :3].T + tf_foot[:3, 3])[:, 2].min())


def registered():
    """Candidate G2.1 reference: the nominal-radius cylinder translated by the minimax offset of the collision tread.

    The radius is raised by the largest radial excess of any tread vertex (about 0.6 nm) so that no mesh vertex lies
    outside the reference cylinder. Nothing in the accepted G2 files is changed; this object exists only in G4.1 tools.
    """
    import json
    from common import OUT
    mm = json.loads((OUT / 'root_cause.json').read_text())['meshes'][0]['nominal_radius_translated_circle_minimax']
    nom = nominal()
    centre = nom.center + np.array([mm['dx_um'] * 1e-6, 0.0, mm['dz_um'] * 1e-6])
    return nom.with_center('G2_1_REGISTERED_CANDIDATE', centre, nom.radius + max(mm['max_radial_excess_um'], 0.0) * 1e-6)

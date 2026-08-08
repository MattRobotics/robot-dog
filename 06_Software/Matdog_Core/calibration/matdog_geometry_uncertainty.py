#!/usr/bin/env python3
"""
MATDOG — Geometry Compiler manufacturing-tolerance / contact-sensitivity analysis.

Implements canonical handoff section 6 ("Tolleranza geometrica"): a
parametric handling of the +/-0.15 mm PPA+CF print tolerance, plus a local
numerical sensitivity estimate

    Delta q ~= Delta d / |d(clearance)/dq|

evaluated near each endpoint's converged contact angle, with explicit
numerical-stability checks (bounded step, minimum gradient magnitude
before trusting a division).

The +/-0.15 mm figure is NOT assumed to be the total assembly-level
worst case: assembly tolerances (bushings, screws, play, servo backlash)
are left as explicit unknowns unless supplied.

Offline only: no Station, serial, motor command or EEPROM access.
"""

from __future__ import annotations

from dataclasses import dataclass
import sys
from pathlib import Path

CALIBRATION_DIR = Path(__file__).resolve().parent

if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_geometry_contact_search import EndpointSpec, _pose_for_probe_angle  # noqa: E402
from matdog_geometry_scene import RobotScene  # noqa: E402


DEFAULT_PRINT_TOLERANCE_M = 0.00015
"""+/-0.15 mm PPA+CF print tolerance per printed part, from the canonical
handoff. This is a per-part input to the sensitivity estimate, not an
assumed total assembly-level worst case."""

DEFAULT_SENSITIVITY_STEP_RAD = 0.001
"""~0.057 deg finite-difference step for the local clearance gradient,
an order of magnitude coarser than the contact-search bisection
resolution (so both sample points are safely on the clear side of the
converged bracket) and an order of magnitude finer than one encoder tick
(~0.0879 deg)."""

DEFAULT_MIN_GRADIENT_M_PER_RAD = 1e-6
"""Below this |d(clearance)/dq|, the estimated Delta q blows up towards
1.5 m of angular uncertainty per metre of tolerance and is no longer a
meaningful figure; such cases are reported as numerically unstable
instead of an inflated number."""

UNRESOLVED_ASSEMBLY_TOLERANCE_NOTE = (
    "UNKNOWN: bushing play, screw/fastener tolerance, servo horn backlash "
    "and inter-part assembly stack-up are not modelled; only the nominal "
    "STL geometry and the per-part print tolerance are used here."
)


class UncertaintyAnalysisError(RuntimeError):
    """Errore nell'analisi di incertezza/sensibilita del Geometry Compiler."""


@dataclass(frozen=True)
class ManufacturingToleranceInputs:
    print_tolerance_m: float = DEFAULT_PRINT_TOLERANCE_M
    assembly_tolerance_note: str = UNRESOLVED_ASSEMBLY_TOLERANCE_NOTE


@dataclass(frozen=True)
class ContactSensitivityResult:
    contact_angle_rad: float
    contact_link_a: str
    contact_link_b: str
    """The exact same contact pair that defined the endpoint's converged
    result (canonical handoff section 8) -- the gradient below is always
    evaluated on this specific pair via `RobotScene.check_link_pair`,
    never on whichever pair happens to be worst overall at the probe
    pose (that pair can differ from the endpoint's own contact feature,
    e.g. near a low-clearance-but-unrelated self pose)."""
    finite_difference_step_rad: float
    clearance_near_m: float
    clearance_near_kind: str | None
    clearance_far_m: float
    clearance_far_kind: str | None
    gradient_m_per_rad: float | None
    gradient_stable: bool
    tolerance_used_m: float
    tolerance_budget_note: str
    """Explicit statement of what `tolerance_used_m` does and does not
    represent for this contact pair (canonical handoff section 8): two
    independently-printed parts participate in every contact pair here
    (`contact_link_a`, `contact_link_b`), each with its own +/-0.15mm
    PPA+CF print tolerance; `tolerance_used_m`/`estimated_uncertainty_rad`
    apply that figure as a single Delta d, i.e. as ONE part's tolerance,
    and do NOT sum or RSS-combine both parts' tolerances into a worst-case
    two-part figure -- neither combination is assumed silently. A
    sum-combined two-part estimate would scale `estimated_uncertainty_rad`
    by 2x; an RSS-combined one by sqrt(2)~=1.41x. Assembly-level
    tolerances (bushings, screws, backlash) are separately UNKNOWN, see
    ManufacturingToleranceInputs.assembly_tolerance_note."""
    estimated_uncertainty_rad: float | None
    unstable_reason: str | None


TOLERANCE_BUDGET_NOTE = (
    "tolerance_used_m is a single PART's +/-0.15mm PPA+CF print tolerance, applied as one "
    "Delta d to the two-point gradient; the contact pair has TWO independently-printed parts "
    "(contact_link_a, contact_link_b) and this figure is NOT summed or RSS-combined across "
    "them -- a summed two-part worst case would be ~2x estimated_uncertainty_rad, an "
    "RSS-combined one ~1.41x. Assembly-level tolerance (bushings, screws, servo backlash) is "
    "separately UNKNOWN."
)


def compute_contact_sensitivity(
    scene: RobotScene,
    endpoint: EndpointSpec,
    clear_angle_rad: float,
    other_legs_pose: dict[str, float],
    contact_link_a: str,
    contact_link_b: str,
    *,
    step_rad: float = DEFAULT_SENSITIVITY_STEP_RAD,
    tolerance_m: float = DEFAULT_PRINT_TOLERANCE_M,
    min_gradient_m_per_rad: float = DEFAULT_MIN_GRADIENT_M_PER_RAD,
) -> ContactSensitivityResult:
    """Estimate local angular uncertainty from the manufacturing tolerance
    via a numerical clearance gradient evaluated at two points strictly on
    the clear side of the converged contact bracket (`clear_angle_rad` and
    one step further back toward home), on the SAME (`contact_link_a`,
    `contact_link_b`) pair that defines the endpoint's own converged
    contact -- not on whatever pair `worst_pair_at_pose` would pick at
    these two probe poses, which is not guaranteed to be the same feature
    (canonical handoff section 8)."""
    sign = -1.0 if endpoint.side == "max" else 1.0
    angle_near = clear_angle_rad
    angle_far = clear_angle_rad + sign * step_rad

    pose_near = _pose_for_probe_angle(endpoint, angle_near, other_legs_pose)
    pose_far = _pose_for_probe_angle(endpoint, angle_far, other_legs_pose)

    result_near = scene.check_link_pair(contact_link_a, contact_link_b, pose_near, require_distance=True)
    result_far = scene.check_link_pair(contact_link_a, contact_link_b, pose_far, require_distance=True)

    if result_near.status == "INTERSECTING" or result_far.status == "INTERSECTING":
        return ContactSensitivityResult(
            contact_angle_rad=clear_angle_rad,
            contact_link_a=contact_link_a,
            contact_link_b=contact_link_b,
            finite_difference_step_rad=step_rad,
            clearance_near_m=result_near.clearance_m if result_near.clearance_m is not None else float("nan"),
            clearance_near_kind=result_near.clearance_kind,
            clearance_far_m=result_far.clearance_m if result_far.clearance_m is not None else float("nan"),
            clearance_far_kind=result_far.clearance_kind,
            gradient_m_per_rad=None,
            gradient_stable=False,
            tolerance_used_m=tolerance_m,
            tolerance_budget_note=TOLERANCE_BUDGET_NOTE,
            estimated_uncertainty_rad=None,
            unstable_reason=(
                "sensitivity probe point intersects on the endpoint's own contact pair; "
                "converged clear-side bracket assumption violated, cannot evaluate a "
                "clearance gradient here"
            ),
        )

    clearance_near = result_near.clearance_m
    clearance_far = result_far.clearance_m

    if clearance_near is None or clearance_far is None:
        return ContactSensitivityResult(
            contact_angle_rad=clear_angle_rad,
            contact_link_a=contact_link_a,
            contact_link_b=contact_link_b,
            finite_difference_step_rad=step_rad,
            clearance_near_m=clearance_near if clearance_near is not None else float("nan"),
            clearance_near_kind=result_near.clearance_kind,
            clearance_far_m=clearance_far if clearance_far is not None else float("nan"),
            clearance_far_kind=result_far.clearance_kind,
            gradient_m_per_rad=None,
            gradient_stable=False,
            tolerance_used_m=tolerance_m,
            tolerance_budget_note=TOLERANCE_BUDGET_NOTE,
            estimated_uncertainty_rad=None,
            unstable_reason="clearance not resolved at one of the two sensitivity probe points",
        )

    delta_clearance = clearance_far - clearance_near
    delta_angle = angle_far - angle_near
    gradient = delta_clearance / delta_angle

    if abs(gradient) < min_gradient_m_per_rad:
        return ContactSensitivityResult(
            contact_angle_rad=clear_angle_rad,
            contact_link_a=contact_link_a,
            contact_link_b=contact_link_b,
            finite_difference_step_rad=step_rad,
            clearance_near_m=clearance_near,
            clearance_near_kind=result_near.clearance_kind,
            clearance_far_m=clearance_far,
            clearance_far_kind=result_far.clearance_kind,
            gradient_m_per_rad=gradient,
            gradient_stable=False,
            tolerance_used_m=tolerance_m,
            tolerance_budget_note=TOLERANCE_BUDGET_NOTE,
            estimated_uncertainty_rad=None,
            unstable_reason=(
                f"|gradient|={abs(gradient):.3e} m/rad below stability floor "
                f"{min_gradient_m_per_rad:.3e} m/rad; contact direction is nearly "
                "tangential to this pair here, Delta d / |gradient| would be "
                "numerically meaningless"
            ),
        )

    estimated_uncertainty_rad = tolerance_m / abs(gradient)

    return ContactSensitivityResult(
        contact_angle_rad=clear_angle_rad,
        contact_link_a=contact_link_a,
        contact_link_b=contact_link_b,
        finite_difference_step_rad=step_rad,
        clearance_near_m=clearance_near,
        clearance_near_kind=result_near.clearance_kind,
        clearance_far_m=clearance_far,
        clearance_far_kind=result_far.clearance_kind,
        gradient_m_per_rad=gradient,
        gradient_stable=True,
        tolerance_used_m=tolerance_m,
        tolerance_budget_note=TOLERANCE_BUDGET_NOTE,
        estimated_uncertainty_rad=estimated_uncertainty_rad,
        unstable_reason=None,
    )

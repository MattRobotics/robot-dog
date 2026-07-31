# MATDOG frame convention

Status: `CURRENT_CANONICAL` for Milestone I.1. Numerical rows are in
`frame_registry.csv` and `joint_registry.csv`.

## Axes and units

MATDOG uses right-handed frames and SI units: metres for translation, radians
for rotation, seconds for time when time is explicitly modeled. In
`base_link`, +X is forward, +Y is left, and +Z is up. Joint positive rotation
is the URDF right-hand-rule direction about the recorded axis.

The axis claim is sourced to the pinned REV00 ADR section that explicitly
states the ROS X-forward/Y-left/Z-up convention. `base_link` is the actual root
frame of REV00 and is centered on the body
reference used by the model; it is not silently moved to a ground or center-of-
mass frame. The four leg chains inherit their hip-joint origins from it. Front
hips are at z=0.0465 m and rear hips at z=0.0265 m, a 0.0200 m front/rear
vertical asymmetry.

## Leg and foot frames

Leg order is `LF, RF, RH, LH`. Local joint frames are exactly the URDF joint
origins/axes; no mirrored sign convention is added. Each nominal foot frame is
the origin of `<leg>_foot_link`, attached to the lower link by a fixed joint:
front/back and left/right offsets are recorded per row in the frame registry.

The nominal foot-frame origin is a kinematic contact reference. It is not the
finite physical contact patch. Existing foot collision geometry models the
physical solid; contact-patch shape, compliance and load distribution are not
promoted to a point-frame fact.

## World, ground and collision

Every materialized registry row names its URDF source joint and source link;
parent, origin XYZ/RPY, units and state are checked against REV00. The REV00
URDF does not materialize a `world` link/joint. `world` and `ground_plane`
therefore have planned frame types, blank materialized source fields and
explicit `unknown` / `decision-required` states. The planned contract is a
right-handed world with ground plane `world Z=0`, but its robot placement is
not selected here.

Visual and collision geometry share each link frame in the current URDF and
use detailed STL meshes. They are suitable for offline geometry checks, but
are not a validated dynamic contact model or a set of reduced simulation
primitives.

#ifndef MATDOG_MOTION_CONTACT_MODE_H
#define MATDOG_MOTION_CONTACT_MODE_H
#include "Gait.h"
namespace matdog { namespace motion {
// G4.1 contact-mode contract. The mode of a leg is DECLARED by the schedule (its LegPhase); geometric proximity to the
// ground never changes it. LIFT_OFF and TOUCHDOWN are the exact schedule instants (zero foot velocity and acceleration,
// C2 swing); the three non-SWING modes are declared contact. Pure, deterministic, no allocation, no device dependency.
enum class ContactMode : uint8_t { STANCE, SWING, LIFT_OFF, TOUCHDOWN };
// Returns false (and leaves `mode` untouched) for an invalid parameter set or an inconsistent / non-finite phase.
bool contactMode(const LegPhase& phase, const GaitParameters& p, ContactMode& mode);
inline bool declaredContact(ContactMode m) { return m != ContactMode::SWING; }
} }
#endif

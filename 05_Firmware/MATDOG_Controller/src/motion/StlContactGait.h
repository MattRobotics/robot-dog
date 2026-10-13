#ifndef MATDOG_MOTION_STL_CONTACT_GAIT_H
#define MATDOG_MOTION_STL_CONTACT_GAIT_H
#include "PairedProneRoute.h"
namespace matdog { namespace motion {
// Same bounded five-iteration Cartesian reference adapter as frozen host G4.
// Derivatives evaluate the COMPLETE adapted map, including reference correction.
// Branch changes or a nonconvergent stencil refuse the frame. No load inference.
LocomotionFrame solveStlContactGait(const GaitParameters&,const CartesianSample&,const LocomotionFrame*,double derivativeStepS=.001);
// Position-only oracle boundary. It is never used by the executable planner:
// that planner must also qualify derivatives through solveStlContactGait.
LocomotionFrame stlContactPosition(const GaitParameters&,const CartesianSample&,const LocomotionFrame*);
} }
#endif

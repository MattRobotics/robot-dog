#pragma once
namespace matdog { namespace motion {
// Recovered verbatim from TimedStand: the STANDARD gait start/stop ramp.
struct TimeLawSample { double progress=0,velocity=0,acceleration=0; bool valid=false; };
TimeLawSample quinticTimeLaw(double u,double durationS);
} }

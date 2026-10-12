#ifndef MATDOG_CORE_MOTION_JSON_H
#define MATDOG_CORE_MOTION_JSON_H
#include "../motion/MotionContract.h"
namespace matdog { namespace core {
// Bounded, strict JSON adapter. Unknown/duplicate keys and trailing bytes fail;
// parsed values are never safety facts or actuator authority.
bool parseMotionJson(const char*,size_t,motion::MotionCommand*,const char** reason);
} }
#endif

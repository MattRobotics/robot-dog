#ifndef MATDOG_CORE_LOCAL_MOTION_COMMAND_H
#define MATDOG_CORE_LOCAL_MOTION_COMMAND_H
#include "../motion/MotionContract.h"
#include <cstdio>
namespace matdog { namespace core {
inline motion::MotionCommand localMotionCommand(motion::Intent intent,uint32_t sequence,uint32_t issued,uint32_t ttl){
 motion::MotionCommand c;c.owner=motion::ClientOwner::LOCAL;c.intent=intent;c.sequence=sequence;c.issued_at_ms=issued;c.ttl_ms=ttl;
 std::snprintf(c.id,sizeof(c.id),"local-%u-%lu",static_cast<unsigned>(intent),static_cast<unsigned long>(sequence));return c;
}
} }
#endif

// Host build of the QEMU Motion scenarios (run_qemu_motion.sh): same scenarios,
// same production sources, simulated clock; the cost counter is 0 here.
#include "motion_scenarios.h"
namespace qemu_motion { int64_t gClock=0; bool gSim=true; uint32_t cycles(){return 0;} void (*gProfile)(const char*,uint32_t)=nullptr; }
extern "C" int64_t esp_timer_get_time(){return qemu_motion::gClock;}
int main(){static qemu_motion::Sim s;std::printf("PLATFORM host\n");qemu_motion::run(s);std::printf("DONE\n");return 0;}

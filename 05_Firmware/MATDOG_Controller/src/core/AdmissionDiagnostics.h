#ifndef MATDOG_ADMISSION_DIAGNOSTICS_H
#define MATDOG_ADMISSION_DIAGNOSTICS_H
#include <cstdio>
#include <cstdint>
#include <cstring>
#include "../execution/MotionExecutor.h"
namespace matdog { namespace core {
constexpr size_t kAdmissionTraceBytes=768;
// No Arduino Print formatter. Whole-record capacity check; never wait/retry.
inline size_t formatAdmissionTrace(char* out,size_t cap,uint32_t now,uint32_t epoch,
 uint32_t generation,uint32_t lease,const char* reason,const execution::JointFeedback& fb){
 if(!out||!cap||!reason)return 0;
 out[0]=0;
 size_t r=0;while(r<64&&reason[r])++r;if(r==64)return 0;
 int n=snprintf(out,cap,"MOTION_ADMISSION now_ms=%lu epoch=%lu generation=%lu lease=%lu result=%s joints=",
  (unsigned long)now,(unsigned long)epoch,(unsigned long)generation,(unsigned long)lease,reason);
 if(n<0||static_cast<size_t>(n)>=cap){out[0]=0;return 0;}
 size_t used=static_cast<size_t>(n);
 for(unsigned i=0;i<12;++i){
  n=snprintf(out+used,cap-used,"%s%u:%lu:%ld:%u",i?",":"",i,
   (unsigned long)fb.sampled_at_ms[i],(long)static_cast<int32_t>(now-fb.sampled_at_ms[i]),fb.read_ok[i]?1:0);
  if(n<0||static_cast<size_t>(n)>=cap-used){out[0]=0;return 0;}used+=static_cast<size_t>(n);
 }
 if(used+2>=cap){out[0]=0;return 0;}out[used++]='\r';out[used++]='\n';out[used]=0;return used;
}
template<class Sink> bool writeAdmissionTrace(Sink& sink,const char* data,size_t used){
 if(!data||!used||used>=kAdmissionTraceBytes||sink.availableForWrite()<static_cast<int>(used))return false;
 return sink.write(reinterpret_cast<const uint8_t*>(data),used)==used;
}
} }
#endif

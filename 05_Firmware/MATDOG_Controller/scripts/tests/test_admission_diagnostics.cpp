#include "../../src/core/AdmissionDiagnostics.h"
#include <cassert>
#include <climits>
#include <cstring>
#include <cstdio>
struct Sink {
 int room=0,calls=0;size_t last=0;bool shortWrite=false;
 int availableForWrite(){return room;}
 size_t write(const uint8_t*,size_t n){++calls;last=n;return shortWrite?n-1:n;}
};
int main(){
 using namespace matdog::core;
 matdog::execution::JointFeedback fb;
 for(unsigned i=0;i<12;++i){fb.sampled_at_ms[i]=UINT32_MAX-i;fb.read_ok[i]=true;}
 char buffer[kAdmissionTraceBytes];char reason[64];memset(reason,'X',63);reason[63]=0;
 size_t n=formatAdmissionTrace(buffer,sizeof(buffer),INT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,reason,fb);
 assert(n&&n<sizeof(buffer)&&strstr(buffer,"11:")&&strstr(buffer,"result="));
 assert(strstr(buffer,"-2147483648"));assert(buffer[n-2]=='\r'&&buffer[n-1]=='\n');
 for(size_t cap=1;cap<=n;++cap){
  char b[kAdmissionTraceBytes+2];memset(b,'Z',sizeof(b));
  assert(!formatAdmissionTrace(b,cap,INT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,reason,fb));
  assert(b[0]==0&&b[cap]=='Z');
 }
 assert(!formatAdmissionTrace(nullptr,0,0,0,0,0,reason,fb));
 char tooLong[65];memset(tooLong,'X',64);tooLong[64]=0;
 assert(!formatAdmissionTrace(buffer,sizeof(buffer),0,0,0,0,tooLong,fb));
 n=formatAdmissionTrace(buffer,sizeof(buffer),0,0,0,0,"STALE_FEEDBACK",fb);
 Sink sink;sink.room=n-1;assert(!writeAdmissionTrace(sink,buffer,n)&&!sink.calls);
 sink.room=n;assert(writeAdmissionTrace(sink,buffer,n)&&sink.calls==1&&sink.last==n);
 assert(!writeAdmissionTrace(sink,buffer,0)&&sink.calls==1);
 assert(!writeAdmissionTrace(sink,buffer,SIZE_MAX)&&sink.calls==1);
 assert(!writeAdmissionTrace(sink,nullptr,n)&&sink.calls==1);
 sink.shortWrite=true;assert(!writeAdmissionTrace(sink,buffer,n)&&sink.calls==2);
 puts("ADMISSION_DIAGNOSTICS PASS: max fields, signed ages, all overflow caps, backpressure/no retry");
}

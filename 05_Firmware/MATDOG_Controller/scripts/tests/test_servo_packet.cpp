// Issue #52 (2026-10-15): src/servo/ServoPacket.h byte-for-byte against the
// INSTALLED SCServo 1.0.2 SyncWritePosEx (real library code, UART stub), and
// the bounded status parser against valid / corrupted / late / foreign
// replies. Host only: no device, no bus.
#include "SMS_STS.h"
#include "../../src/servo/ServoPacket.h"
#include <algorithm>
#include <cstdio>
#include <vector>
using namespace matdog::servo::packet;
static int checks=0,failures=0;
#define CHECK(c) do{++checks;if(!(c)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#c);}}while(0)
struct Capture:SMS_STS{
  std::vector<unsigned char> tx;int readcalls=0;
  int writeSCS(unsigned char* p,int n) override{tx.insert(tx.end(),p,p+n);return n;}
  int writeSCS(unsigned char c) override{tx.push_back(c);return 1;}
  int readSCS(unsigned char*,int) override{++readcalls;return 0;}
  void rFlushSCS() override{} void wFlushSCS() override{}
};
int main(){
  // Canonical leg allocation order is irrelevant to the packet; any 12 distinct ids.
  uint8_t ids[12]={13,12,11,23,22,21,33,32,31,43,42,41};uint16_t ticks[12];
  for(int i=0;i<12;++i)ticks[i]=static_cast<uint16_t>(2048+37*i-200);
  Capture lib;unsigned char acc[12];short pos[12];unsigned short spd[12];
  for(int i=0;i<12;++i){acc[i]=50;pos[i]=static_cast<short>(ticks[i]);spd[i]=1000;}
  lib.SyncWritePosEx(ids,12,pos,spd,acc);
  uint8_t ours[kGoalFrameBytes];const size_t n=buildGoalFrame(ids,ticks,12,1000,50,ours,sizeof(ours));
  CHECK(n==104&&lib.tx.size()==104&&std::equal(lib.tx.begin(),lib.tx.end(),ours)&&lib.readcalls==0);
  CHECK(ours[2]==0xFE&&ours[3]==100&&ours[4]==0x83&&ours[5]==41&&ours[6]==7);
  // Refusals: zero bytes.
  uint8_t dup[12];std::copy(ids,ids+12,dup);dup[11]=dup[0];CHECK(buildGoalFrame(dup,ticks,12,1000,50,ours,sizeof(ours))==0);
  uint8_t bc[12];std::copy(ids,ids+12,bc);bc[3]=0xFE;CHECK(buildGoalFrame(bc,ticks,12,1000,50,ours,sizeof(ours))==0);
  uint16_t big[12];std::copy(ticks,ticks+12,big);big[5]=4096;CHECK(buildGoalFrame(ids,big,12,1000,50,ours,sizeof(ours))==0);
  CHECK(buildGoalFrame(ids,ticks,12,0,50,ours,sizeof(ours))==0&&buildGoalFrame(ids,ticks,12,1000,50,ours,103)==0);
  // Torque OFF sync packet vs the library's generic syncWrite at address 40, width 1.
  Capture lib2;unsigned char zero[12]={0};lib2.syncWrite(ids,12,40,zero,1);
  uint8_t off[kTorqueOffBytesMax];const size_t m=buildTorqueOff(ids,12,off,sizeof(off));
  CHECK(m==lib2.tx.size()&&std::equal(lib2.tx.begin(),lib2.tx.end(),off)&&m==32);
  // Status parser: valid read reply of 15 bytes from id 13.
  auto reply=[](uint8_t id,const std::vector<uint8_t>& data,uint8_t err){std::vector<uint8_t> p={0xFF,0xFF,id,static_cast<uint8_t>(data.size()+2),err};
    p.insert(p.end(),data.begin(),data.end());p.push_back(checksum(p.data(),2,p.size()));return p;};
  std::vector<uint8_t> data(15);for(int i=0;i<15;++i)data[i]=static_cast<uint8_t>(i*7);
  auto ok=reply(13,data,0);StatusParser sp;sp.reset(13,15);for(auto b:ok)sp.feed(b);CHECK(sp.done()&&sp.servo_error==0&&sp.data()[3]==21);
  auto bad=ok;bad[8]^=1;sp.reset(13,15);for(auto b:bad)sp.feed(b);CHECK(sp.bad()&&sp.error==ParseError::CHECKSUM);
  sp.reset(12,15);for(auto b:ok)sp.feed(b);CHECK(sp.bad()&&sp.error==ParseError::WRONG_ID);
  auto shortlen=reply(13,std::vector<uint8_t>(14),0);sp.reset(13,15);for(auto b:shortlen)sp.feed(b);CHECK(sp.bad()&&sp.error==ParseError::LENGTH);
  auto err=reply(13,data,0x20);sp.reset(13,15);for(auto b:err)sp.feed(b);CHECK(sp.done()&&sp.servo_error==0x20);   // overload flag reported, not hidden
  sp.reset(13,15);for(size_t i=0;i+1<ok.size();++i)sp.feed(ok[i]);CHECK(!sp.done()&&!sp.bad());   // missing/late byte: never DONE
  sp.reset(13,15);for(auto b:ok)sp.feed(b);sp.feed(0xFF);CHECK(sp.bad());                         // trailing byte rejected
  // The stock library SyncRead parser accepts this corrupted checksum (audit E): not used.
  std::printf("SERVO_PACKET [MISURATO HOST] SyncWritePosEx 104 B identical to SCServo 1.0.2, torque-OFF sync 32 B identical, parser rejects checksum/id/length/trailing, %s checks=%d failures=%d\n",failures?"FAIL":"PASS",checks,failures);
  return failures?1:0;
}

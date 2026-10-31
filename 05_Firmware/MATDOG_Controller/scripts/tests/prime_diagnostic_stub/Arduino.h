#pragma once
// In-memory UART/device only. The real installed SCServo writes and the real
// production ServoBus/validated reads are linked by run_servo_goal_tests.sh.
// No OS serial/network/filesystem API exists in this fixture.
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>
inline uint64_t host_us=0;
inline unsigned long millis(){host_us+=10;return host_us/1000;}
inline unsigned long micros(){host_us+=10;return host_us;}
constexpr int SERIAL_8N1=0;
template<class T> T constrain(T x,T lo,T hi){return std::min(hi,std::max(lo,x));}
struct DebugSerial {
  std::string text;
  template<class... A> int printf(const char* fmt,A... args){
    char s[512];const int n=std::snprintf(s,sizeof(s),fmt,args...);text+=s;return n;
  }
};
inline DebugSerial Serial;
class HardwareSerial {
 public:
  enum Fault { NONE, NOT_APPLIED, DROP, CHECKSUM, WRONG_ID, LENGTH, STATUS, TRUNCATE, SWAP, SIGN_BIT };
  inline static HardwareSerial* bus=nullptr;
  explicit HardwareSerial(int=0){bus=this;}
  std::array<std::array<uint8_t,71>,254> regs{};
  std::deque<uint8_t> rx;
  std::vector<uint8_t> pending,last_write;
  std::vector<std::vector<uint8_t>> requests;
  Fault fault=NONE;
  int fault_id=-1,fault_address=-1;
  bool lose_ack=false,drop_all_reads=false;
  void begin(unsigned long,int,int,int){}
  void flush(){}
  int available(){return static_cast<int>(rx.size());}
  int read(){if(rx.empty())return -1;const int b=rx.front();rx.pop_front();return b;}
  static uint8_t checksum(const std::vector<uint8_t>& p){
    uint8_t sum=0;for(size_t i=2;i<p.size();++i)sum+=p[i];return static_cast<uint8_t>(~sum);
  }
  void reply(int id,std::vector<uint8_t> data,uint8_t status=0,Fault f=NONE){
    if(f==DROP)return;
    if(f==SWAP&&data.size()==2)std::swap(data[0],data[1]);
    if(f==SIGN_BIT&&data.size()==2)data[1]|=0x80;
    std::vector<uint8_t> p={255,255,static_cast<uint8_t>(f==WRONG_ID?id+1:id),static_cast<uint8_t>(data.size()+2),static_cast<uint8_t>(f==STATUS?1:status)};
    if(f==LENGTH)++p[3];
    p.insert(p.end(),data.begin(),data.end());p.push_back(checksum(p));
    if(f==CHECKSUM)p.back()^=1;
    if(f==TRUNCATE)p.pop_back();
    for(auto b:p)rx.push_back(b);
  }
  void dispatch(const std::vector<uint8_t>& p){
    requests.push_back(p);
    if(p.size()<6||p[0]!=255||p[1]!=255)return;
    auto body=p;body.pop_back();if(checksum(body)!=p.back())return;
    const int id=p[2],inst=p[4];
    if(id>=254)return; // frame bytes are checked separately against SCServo
    const int address=p[5];
    if(inst==3){
      last_write=p;
      for(size_t i=6;i+1<p.size();++i){
        const int reg=address+static_cast<int>(i)-6;
        const bool fault_word=fault_address==42||fault_address==46;
        if(reg<71&&!(fault==NOT_APPLIED&&id==fault_id&&(reg==fault_address||(fault_word&&reg==fault_address+1))))regs[id][reg]=p[i];
      }
      if(!lose_ack)reply(id,{});
    }else if(inst==2&&p.size()==8&&address+p[6]<=71){
      if(drop_all_reads)return;
      std::vector<uint8_t> data(regs[id].begin()+address,regs[id].begin()+address+p[6]);
      reply(id,data,0,id==fault_id&&address==fault_address?fault:NONE);
    }
  }
  size_t write(const uint8_t* p,size_t n){
    for(size_t i=0;i<n;++i){
      pending.push_back(p[i]);
      if(pending.size()>=4&&pending.size()==static_cast<size_t>(pending[3])+4){auto packet=pending;pending.clear();dispatch(packet);}
    }
    return n;
  }
};
// Same function-like macro as the real ESP32 core (cores/esp32/Arduino.h,
// makeWord in WMath.cpp). Without it the host build silently accepted a
// production identifier named word(...), which the device compiles to
// makeWord(address): PRIME readback then reported register addresses 42/46.
inline uint16_t makeWord(uint16_t w){return w;}
inline uint16_t makeWord(uint8_t h,uint8_t l){return static_cast<uint16_t>((h<<8)|l);}
#define word(...) makeWord(__VA_ARGS__)

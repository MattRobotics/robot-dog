#pragma once
// Host UART model with VIRTUAL TIME and per-byte arrival (Core Motion V5).
// No OS serial/network/filesystem. The production ServoBus and the installed
// SCServo SDK run against it; the servo model only decides WHEN reply bytes
// become readable. Every micros()/millis() call is a poll and costs spin_us of
// virtual time, so a bounded wait loop always terminates deterministically.
// All figures from this stub are SIMULATED, not ESP32 WCET.
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>
inline uint64_t host_us=0;
inline uint32_t spin_us=5;
inline unsigned long millis(){host_us+=spin_us;return static_cast<unsigned long>(host_us/1000);}
inline unsigned long micros(){host_us+=spin_us;return static_cast<unsigned long>(host_us);}
constexpr int SERIAL_8N1=0;
template<class T> T constrain(T x,T lo,T hi){return std::min(hi,std::max(lo,x));}
struct DebugSerial {
  std::string text;
  template<class... A> int printf(const char* fmt,A... args){char s[512];const int n=std::snprintf(s,sizeof(s),fmt,args...);text+=s;return n;}
};
inline DebugSerial Serial;
class HardwareSerial {
 public:
  struct Timed { uint64_t at; uint8_t b; };
  struct ReplyPlan {                 // how the next reply is delivered
    bool silent=false;               // no bytes at all
    uint32_t latency_us=60;          // request end -> first byte
    uint32_t gap_us=10;              // spacing between bytes (10 = back-to-back at 1 Mbaud)
    int slow_after=-1;               // from this byte index on, use slow_gap_us
    uint32_t slow_gap_us=0;
    int truncate_to=-1;              // deliver only this many bytes
    bool corrupt_checksum=false;
    bool wrong_id=false;
    bool no_ack=false;               // write ACK suppressed (applied, not acknowledged)
  };
  inline static HardwareSerial* bus=nullptr;
  explicit HardwareSerial(int=0){bus=this;}
  std::array<std::array<uint8_t,71>,254> regs{};
  std::deque<Timed> rx;
  ReplyPlan plan;
  std::vector<std::vector<uint8_t>> requests;      // every complete request that reached the wire
  std::vector<uint8_t> pending;
  size_t unflushed=0,tx_bytes_total=0;uint64_t last_at=0;
  bool flood=false;                                 // babbling bus: a byte is always readable
  unsigned bytes_read=0, jump_after_byte=0;
  uint32_t read_jump_us=0;
  size_t max_write=SIZE_MAX;                         // short UART write injection
  void begin(unsigned long,int,int,int){}
  void flush(){host_us+=unflushed*10;unflushed=0;}
  int available(){if(flood)return 1;int n=0;for(auto& t:rx)if(t.at<=host_us)++n;return n;}
  int read(){if(flood)return 0xAA;if(rx.empty()||rx.front().at>host_us)return -1;const int b=rx.front().b;rx.pop_front();if(++bytes_read==jump_after_byte)host_us+=read_jump_us;return b;}
  static uint8_t checksum(const std::vector<uint8_t>& p){uint8_t sum=0;for(size_t i=2;i<p.size();++i)sum+=p[i];return static_cast<uint8_t>(~sum);}
  void schedule(std::vector<uint8_t> p){
    if(plan.silent)return;
    if(plan.wrong_id)++p[2];
    if(plan.corrupt_checksum)p.back()^=1;
    size_t n=p.size();if(plan.truncate_to>=0)n=std::min<size_t>(n,plan.truncate_to);
    uint64_t t=std::max<uint64_t>(host_us+plan.latency_us,last_at+plan.gap_us);   // one wire: bytes never interleave
    for(size_t i=0;i<n;++i){
      rx.push_back({t,p[i]});
      last_at=t;t+=(plan.slow_after>=0&&static_cast<int>(i)>=plan.slow_after)?plan.slow_gap_us:plan.gap_us;
    }
  }
  std::vector<uint8_t> frame(int id,const std::vector<uint8_t>& data,uint8_t status=0){
    std::vector<uint8_t> p={255,255,static_cast<uint8_t>(id),static_cast<uint8_t>(data.size()+2),status};
    p.insert(p.end(),data.begin(),data.end());p.push_back(checksum(p));return p;
  }
  void dispatch(const std::vector<uint8_t>& p){
    requests.push_back(p);
    if(p.size()<6||p[0]!=255||p[1]!=255)return;
    auto body=p;body.pop_back();if(checksum(body)!=p.back())return;
    const int id=p[2],inst=p[4];
    if(id==254&&inst==0x83&&p.size()>=8){                // existing no-reply SYNC_WRITE (including SAFE_OFF)
      const unsigned address=p[5],width=p[6];if(!width)return;
      for(size_t i=7;i+width<p.size();i+=width+1)if(p[i]<254)
        for(unsigned j=0;j<width&&address+j<71;++j)regs[p[i]][address+j]=p[i+1+j];
      return;
    }
    if(id>=254)return;                                  // broadcast: no reply
    const int address=p[5];
    if(inst==3){                                           // WRITE: applied, then ACK
      for(size_t i=6;i+1<p.size();++i){const int reg=address+static_cast<int>(i)-6;if(reg<71)regs[id][reg]=p[i];}
      if(!plan.no_ack)schedule(frame(id,{}));
    }else if(inst==2&&p.size()==8&&address+p[6]<=71){
      schedule(frame(id,std::vector<uint8_t>(regs[id].begin()+address,regs[id].begin()+address+p[6])));
    }else if(inst==1){schedule(frame(id,{}));}              // PING
  }
  size_t write(const uint8_t* p,size_t n){
    n=std::min(n,max_write);
    unflushed+=n;tx_bytes_total+=n;
    for(size_t i=0;i<n;++i){
      pending.push_back(p[i]);
      if(pending.size()>=4&&pending.size()==static_cast<size_t>(pending[3])+4){auto packet=pending;pending.clear();dispatch(packet);}
    }
    return n;
  }
};
inline uint16_t makeWord(uint16_t w){return w;}
inline uint16_t makeWord(uint8_t h,uint8_t l){return static_cast<uint16_t>((h<<8)|l);}
#define word(...) makeWord(__VA_ARGS__)

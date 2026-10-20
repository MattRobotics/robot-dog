#ifndef HOST_SCSERVO_H
#define HOST_SCSERVO_H
#include <stdint.h>
#include <deque>
#include <vector>
inline uint32_t host_ms=0;
inline uint32_t millis() { return host_ms++; }
// Virtual-time spin clock: every call advances 250 us (so a bounded wait loop
// always terminates deterministically), host_ms follows it.
inline uint32_t host_sub_us=0;
inline uint32_t micros() { host_sub_us+=250; if(host_sub_us>=1000){host_sub_us-=1000;++host_ms;} return host_ms*1000+host_sub_us; }
struct HardwareSerial {
  std::deque<int> rx;
  std::vector<uint8_t> last_request;
  std::vector<uint8_t> reply;
  uint32_t arrive_at=0;
  int available() { return rx.size(); }   // a staged `reply` only appears when the request is read, like the original stub
  int read() {
    if(rx.empty() && !reply.empty() && host_ms>=arrive_at) { for(auto b:reply)rx.push_back(b);reply.clear(); }
    if(rx.empty()) return -1;
    int b=rx.front();rx.pop_front();return b;
  }
  size_t write(const uint8_t* p,size_t n){last_request.insert(last_request.end(),p,p+n);return n;}
};
constexpr uint8_t INST_READ=2;
typedef uint8_t u8;
// Minimal model of SCServo's SCSerial/SCS hooks: the SDK writes through the
// virtual writeSCS pieces exactly like SCS::writeBuf (6 bytes, data, checksum).
struct SMS_STS {
  unsigned long IOTimeOut=100;
  HardwareSerial* pSerial=nullptr;
  virtual ~SMS_STS()=default;
 protected:
  virtual int writeSCS(unsigned char* d,int n){return static_cast<int>(pSerial->write(d,n));}
  virtual int writeSCS(unsigned char b){return static_cast<int>(pSerial->write(&b,1));}
  virtual int readSCS(unsigned char*,int){return 0;}
  virtual void rFlushSCS(){while(pSerial->read()!=-1);}
  virtual void wFlushSCS(){}
  void writeBuf(uint8_t id,uint8_t address,uint8_t* width,uint8_t,uint8_t instruction) {
    pSerial->last_request.clear();
    uint8_t b[6]={255,255,id,4,instruction,address};
    writeSCS(b,6);writeSCS(width,1);
    writeSCS(static_cast<uint8_t>(~(id+4+instruction+address+*width)));
  }
};
#endif

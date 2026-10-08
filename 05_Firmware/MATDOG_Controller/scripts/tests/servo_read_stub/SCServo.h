#ifndef HOST_SCSERVO_H
#define HOST_SCSERVO_H
#include <stdint.h>
#include <deque>
#include <vector>
inline uint32_t host_ms=0;
inline uint32_t millis() { return host_ms++; }
struct HardwareSerial {
  std::deque<int> rx;
  std::vector<uint8_t> last_request;
  std::vector<uint8_t> reply;
  uint32_t arrive_at=0;
  int available() { return rx.size(); }
  int read() {
    if(rx.empty() && !reply.empty() && host_ms>=arrive_at) { for(auto b:reply)rx.push_back(b);reply.clear(); }
    if(rx.empty()) return -1;
    int b=rx.front();rx.pop_front();return b;
  }
};
constexpr uint8_t INST_READ=2;
struct SMS_STS {
  unsigned long IOTimeOut=100;
  HardwareSerial* pSerial=nullptr;
 protected:
  void writeBuf(uint8_t id,uint8_t address,uint8_t* width,uint8_t,uint8_t instruction) {
    uint8_t checksum=static_cast<uint8_t>(~(id+4+instruction+address+*width));
    pSerial->last_request={255,255,id,4,instruction,address,*width,checksum};
  }
};
#endif

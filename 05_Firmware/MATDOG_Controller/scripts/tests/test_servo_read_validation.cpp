#include <cstdio>
#include "../../src/servo/ValidatedServoRead.h"
using namespace matdog::servo;
int main() {
  int checks=0, failures=0;
  auto check=[&](bool condition){++checks;if(!condition)++failures;};
  auto packet=[](uint8_t id,uint8_t width,uint8_t status=0) {
    std::vector<uint8_t> v={255,255,id,static_cast<uint8_t>(width+2),status};
    for(unsigned i=0;i<width;++i)v.push_back(34+i);
    uint8_t sum=0;for(unsigned i=2;i<v.size();++i)sum+=v[i];v.push_back(~sum);return v;
  };
  for (uint8_t width : {uint8_t(1),uint8_t(2),uint8_t(10),uint8_t(15)}) {
    for(int fault=0;fault<8;++fault) {
      host_ms=0;HardwareSerial uart;ValidatedServoRead bus;bus.pSerial=&uart;
      uart.reply=packet(fault==1?22:21,width,fault==3?1:0);
      if(fault==2) uart.reply[3]++; // checksummed wrong length also rejected
      if(fault==4) uart.reply.back()^=1;
      if(fault==5) uart.reply.pop_back();
      if(fault==6) uart.reply.insert(uart.reply.begin(),0);
      if(fault==7) uart.arrive_at=200;
      uint8_t output[32];for(auto& b:output)b=99;
      const int n=bus.Read(21,63,output,width);
      check(n==(fault==0?width:0));check(host_ms<=101);
      check(uart.last_request==std::vector<uint8_t>({255,255,21,4,2,63,width,static_cast<uint8_t>(~(21+4+2+63+width))}));
      check(output[0]==(fault==0?34:99)); // no partial or previous data published
      if(fault==0)check(output[width-1]==34+width-1);
    }
  }
  host_ms=0;HardwareSerial uart;ValidatedServoRead bus;bus.pSerial=&uart;
  for(int i=0;i<65;++i)uart.rx.push_back(0);
  uint8_t result=99;check(bus.Read(21,63,&result,1)==0 && uart.last_request.empty() && result==99);
  uart.rx.clear();uart.rx.push_back(123);uart.reply=packet(21,1);
  check(bus.readByte(21,63)==34); // stale RX drained before request
  uart.reply=packet(22,1);check(bus.readByte(21,63)==-1);
  std::printf("test_servo_read_validation: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}

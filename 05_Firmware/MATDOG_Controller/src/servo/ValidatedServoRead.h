#ifndef MATDOG_SERVO_VALIDATED_READ_H
#define MATDOG_SERVO_VALIDATED_READ_H
#include <SCServo.h>
#include "ServoReadValidation.h"
namespace matdog { namespace servo {
// Keep existing SCServo writes. Strengthen all reads at this ownership boundary
// without editing the installed library or caching feedback from a previous call.
class ValidatedServoRead : public SMS_STS {
 public:
  int Read(uint8_t id, uint8_t address, uint8_t* data, uint8_t width) {
    return ReadCaptured(id,address,data,width,nullptr);
  }
  // Optional RAM copy of THIS response, including invalid/partial packets.
  // No second read, retry, UART ownership change or relaxed packet validation.
  int ReadCaptured(uint8_t id,uint8_t address,uint8_t* data,uint8_t width,diagnostics::ReadCapture* capture){
    using diagnostics::ReadValidity;
    if(capture){*capture=diagnostics::ReadCapture{};capture->id=id;capture->address=address;capture->width=width;}
    if (data == nullptr || width == 0 || width > 32 || pSerial == nullptr) {
      if(capture)capture->validity=ReadValidity::INVALID_ARGUMENT;
      return 0;
    }
    uint8_t drained = 0;
    while (pSerial->available() && drained < 64) { pSerial->read(); ++drained; }
    if(capture)capture->drained=drained;
    if (pSerial->available()) {if(capture)capture->validity=ReadValidity::RX_FLOOD;return 0;}
    writeBuf(id, address, &width, 1, INST_READ);
    uint8_t packet[38] = {0};
    uint8_t size = 0;
    const uint32_t started = millis();
    if(capture)capture->started_ms=started;
    while (size < width + 6 && millis() - started < IOTimeOut) {
      const int byte = pSerial->read();
      if (byte >= 0) packet[size++] = static_cast<uint8_t>(byte);
    }
    const auto validity=servoReadValidity(packet,size,id,width);
    if(capture){capture->size=size;capture->validity=validity;capture->finished_ms=millis();
      for(uint8_t i=0;i<size;++i)capture->bytes[i]=packet[i];}
    if (validity!=ReadValidity::VALID) return 0;
    for (uint8_t i = 0; i < width; ++i) data[i] = packet[i + 5];
    return width;
  }
  int readByte(uint8_t id, uint8_t address) {
    uint8_t data = 0;
    return Read(id, address, &data, 1) == 1 ? data : -1;
  }
  int readWord(uint8_t id, uint8_t address) {
    uint8_t data[2] = {0};
    return Read(id, address, data, 2) == 2 ? data[0] | (data[1] << 8) : -1;
  }
};
} }
#endif

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
    if (data == nullptr || width == 0 || width > 32 || pSerial == nullptr) return 0;
    uint8_t drained = 0;
    while (pSerial->available() && drained < 64) { pSerial->read(); ++drained; }
    if (pSerial->available()) return 0;  // bounded drain, fail closed on RX flood
    writeBuf(id, address, &width, 1, INST_READ);
    uint8_t packet[38] = {0};
    uint8_t size = 0;
    const uint32_t started = millis();
    while (size < width + 6 && millis() - started < IOTimeOut) {
      const int byte = pSerial->read();
      if (byte >= 0) packet[size++] = static_cast<uint8_t>(byte);
    }
    if (!validServoReadPacket(packet, size, id, width)) return 0;
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

#ifndef MATDOG_SERVO_READ_VALIDATION_H
#define MATDOG_SERVO_READ_VALIDATION_H
#include <stdint.h>
namespace matdog { namespace servo {
// A READ status packet carries no register address or transaction sequence.
// Match exact ID/length/status/checksum; transaction boundary drains old RX.
inline bool validServoReadPacket(const uint8_t* frame, uint8_t size,
                                 uint8_t id, uint8_t width) {
  if (frame == nullptr || size != width + 6 || frame[0] != 255 || frame[1] != 255 ||
      frame[2] != id || frame[3] != width + 2 || frame[4] != 0) return false;
  uint8_t sum = 0;
  for (uint8_t i = 2; i < size; ++i) sum = static_cast<uint8_t>(sum + frame[i]);
  return sum == 255;
}
} }
#endif

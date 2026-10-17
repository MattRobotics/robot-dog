#ifndef MATDOG_SERVO_READ_VALIDATION_H
#define MATDOG_SERVO_READ_VALIDATION_H
#include <stdint.h>
#include "../diagnostics/PrimeDiagnostic.h"
namespace matdog { namespace servo {
// A READ status packet carries no register address or transaction sequence.
// Match exact ID/length/status/checksum; transaction boundary drains old RX.
inline diagnostics::ReadValidity servoReadValidity(const uint8_t* frame, uint8_t size,
                                 uint8_t id, uint8_t width) {
  using diagnostics::ReadValidity;
  if(frame==nullptr)return ReadValidity::INVALID_ARGUMENT;
  if(size==0)return ReadValidity::NO_RESPONSE;
  if(size!=width+6)return ReadValidity::SHORT_PACKET;
  if(frame[0]!=255||frame[1]!=255)return ReadValidity::HEADER;
  if(frame[2]!=id)return ReadValidity::ID;
  if(frame[3]!=width+2)return ReadValidity::LENGTH;
  if(frame[4]!=0)return ReadValidity::STATUS;
  uint8_t sum = 0;
  for (uint8_t i = 2; i < size; ++i) sum = static_cast<uint8_t>(sum + frame[i]);
  return sum==255?ReadValidity::VALID:ReadValidity::CHECKSUM;
}
inline bool validServoReadPacket(const uint8_t* frame,uint8_t size,uint8_t id,uint8_t width){
 return servoReadValidity(frame,size,id,width)==diagnostics::ReadValidity::VALID;
}
} }
#endif

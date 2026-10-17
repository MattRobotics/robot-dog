#ifndef MATDOG_PRIME_DIAGNOSTIC_H
#define MATDOG_PRIME_DIAGNOSTIC_H
#include <stddef.h>
#include <stdint.h>
namespace matdog { namespace diagnostics {
enum class ReadValidity : uint8_t { NOT_ATTEMPTED, VALID, INVALID_ARGUMENT, RX_FLOOD, NO_RESPONSE,
 SHORT_PACKET, HEADER, ID, LENGTH, STATUS, CHECKSUM };
const char* toString(ReadValidity);
struct ReadCapture {
 uint8_t id=0,address=0,width=0,size=0,drained=0,bytes[38]{};
 ReadValidity validity=ReadValidity::NOT_ATTEMPTED;
 uint32_t started_ms=0,finished_ms=0;
};
struct GoalWriteCapture {
 bool attempted=false;
 uint8_t id=0,acc=0,mismatch=0; // bit0 position, bit1 speed, bit2 ACC
 uint16_t position=0,speed=0;
 int ack=-1,position_read=-1,speed_read=-1,acc_read=-1;
 uint32_t started_ms=0,write_done_ms=0,finished_ms=0;
 ReadCapture response{};
};
struct PrimeRejection {
 uint8_t slot=0,id=0,attempt=0;
 uint16_t target_position=0;
 uint32_t session_ms=0;
 const char* reason="NOT_SET";
 GoalWriteCapture goal{};
};
// Fixed RAM, no allocation, no overwrite. One trial <= 12*3 rejections.
// SAFE_OFF never clears this object; explicit safe-idle CLEAR or reset does.
class PrimeRejectionLog {
 public:
 static constexpr unsigned kCapacity=36;
 bool used=false,overflow=false;
 unsigned count() const {return count_;}
 bool full() const {return count_==kCapacity;}
 PrimeRejection* append(){if(full()){overflow=true;return nullptr;}return &records_[count_++];}
 const PrimeRejection* at(unsigned i) const {return i<count_?&records_[i]:nullptr;}
 void clear(){count_=0;used=false;overflow=false;}
 private:
 unsigned count_=0;
 PrimeRejection records_[kCapacity]{};
};
bool formatPrimeRejection(const PrimeRejection&,unsigned index,char* out,size_t capacity);
} }
#endif

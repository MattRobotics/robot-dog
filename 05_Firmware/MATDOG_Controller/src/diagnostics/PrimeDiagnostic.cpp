#include "PrimeDiagnostic.h"
#include <cstdio>
namespace matdog { namespace diagnostics {
const char* toString(ReadValidity v){
 switch(v){
 case ReadValidity::NOT_ATTEMPTED:return "NOT_ATTEMPTED";
 case ReadValidity::VALID:return "VALID";
 case ReadValidity::INVALID_ARGUMENT:return "INVALID_ARGUMENT";
 case ReadValidity::RX_FLOOD:return "RX_FLOOD";
 case ReadValidity::NO_RESPONSE:return "NO_RESPONSE";
 case ReadValidity::SHORT_PACKET:return "SHORT_PACKET";
 case ReadValidity::HEADER:return "BAD_HEADER";
 case ReadValidity::ID:return "WRONG_ID";
 case ReadValidity::LENGTH:return "WRONG_LENGTH";
 case ReadValidity::STATUS:return "SERVO_STATUS";
 case ReadValidity::CHECKSUM:return "BAD_CHECKSUM";
 }
 return "UNKNOWN";
}
bool formatPrimeRejection(const PrimeRejection& r,unsigned index,char* out,size_t cap){
 if(!out||!cap)return false;
 const auto& g=r.goal;const auto& p=g.response;
 // Decode the retained payload even if checksum/ID/status failed, clearly
 // labelled UNTRUSTED by decoded_valid=0. Never feed these values back into
 // the verdict. Missing/short payloads remain -1, not an invented zero.
 const bool payload=p.address==40&&p.width==10&&p.size>=15;
 const int raw_position=payload?p.bytes[7]|(p.bytes[8]<<8):-1;
 const int raw_speed=payload?p.bytes[11]|(p.bytes[12]<<8):-1;
 const int raw_acc=payload?p.bytes[6]:-1;
 const int n=std::snprintf(out,cap,
  "PRIME_RECORD index=%u slot=%u id=%u attempt=%u reason=%s session_ms=%lu target_position=%u capture=%u target_speed=%u target_acc=%u ack=%d write_start_ms=%lu write_done_ms=%lu read_start_ms=%lu read_done_ms=%lu done_ms=%lu address=%u width=%u rx_size=%u drained=%u validity=%s decoded_valid=%u position_read=%d speed_read=%d acc_read=%d raw_payload=%u raw_position=%d raw_speed=%d raw_acc=%d mismatch=0x%02x rx=",
  index,unsigned(r.slot),unsigned(r.id),unsigned(r.attempt),r.reason,(unsigned long)r.session_ms,unsigned(r.target_position),g.attempted?1u:0u,unsigned(g.speed),unsigned(g.acc),g.ack,
  (unsigned long)g.started_ms,(unsigned long)g.write_done_ms,(unsigned long)p.started_ms,(unsigned long)p.finished_ms,(unsigned long)g.finished_ms,
  unsigned(p.address),unsigned(p.width),unsigned(p.size),unsigned(p.drained),toString(p.validity),p.validity==ReadValidity::VALID?1u:0u,g.position_read,g.speed_read,g.acc_read,payload?1u:0u,raw_position,raw_speed,raw_acc,unsigned(g.mismatch));
 if(n<0||static_cast<size_t>(n)>=cap)return false;
 size_t used=static_cast<size_t>(n);
 for(unsigned i=0;i<p.size&&i<sizeof(p.bytes);++i){
  const int k=std::snprintf(out+used,cap-used,"%02X",unsigned(p.bytes[i]));
  if(k!=2||used+2>=cap)return false;
  used+=2;
 }
 return true;
}
} }

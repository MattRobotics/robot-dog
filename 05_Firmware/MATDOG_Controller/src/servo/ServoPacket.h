#ifndef MATDOG_SERVO_SERVO_PACKET_H
#define MATDOG_SERVO_SERVO_PACKET_H
// Pure ST3215/STS packet builders and a bounded status parser (Issue #52,
// 2026-10-15). No UART, no Arduino: host-tested byte for byte against the
// installed SCServo 1.0.2 sync position packet (scripts/tests/test_servo_packet.cpp).
//
// Only two multi-ID packets exist, both SYNC_WRITE (0x83) to an explicit list
// of distinct IDs - never an all-ID instruction, never EEPROM, never torque ON:
//   goalFrame  : SRAM 41..47 = ACC, GoalPosition(0..4095), GoalTime=0, GoalSpeed
//   torqueOff  : SRAM 40     = TorqueEnable 0 (de-escalation only)
// A SYNC_WRITE has no status reply; transmission is not application.
#include <stddef.h>
#include <stdint.h>
namespace matdog { namespace servo { namespace packet {

constexpr uint8_t kBroadcastId = 0xFE, kInstSync = 0x83, kInstRead = 0x02;
constexpr uint8_t kAddrTorqueEnable = 40, kAddrAcc = 41, kAddrPresentPosition = 56;
constexpr uint8_t kGoalWidth = 7;          // ACC, POS L/H, TIME L/H, SPEED L/H
constexpr unsigned kMaxFrameIds = 12;
// 4 header bytes + LEN; LEN = (width+1)*n + 4.
constexpr size_t kGoalFrameBytes = 4 + (kGoalWidth + 1) * kMaxFrameIds + 4;   // 104
constexpr size_t kTorqueOffBytesMax = 4 + 2 * 17 + 4;

inline uint8_t checksum(const uint8_t* p, size_t from, size_t to) {
  unsigned s = 0; for (size_t i = from; i < to; ++i) s += p[i]; return static_cast<uint8_t>(~s);
}
inline bool distinctIds(const uint8_t* ids, unsigned n) {
  for (unsigned i = 0; i < n; ++i) {
    if (ids[i] == kBroadcastId || ids[i] > 253) return false;
    for (unsigned j = 0; j < i; ++j) if (ids[j] == ids[i]) return false;
  }
  return true;
}
// Returns the packet length, 0 (nothing built) on any invalid input.
inline size_t buildGoalFrame(const uint8_t ids[], const uint16_t ticks[], unsigned n, uint16_t speed,
                             uint8_t acc, uint8_t* out, size_t cap) {
  if (!ids || !ticks || !out || n == 0 || n > kMaxFrameIds || !distinctIds(ids, n) || speed == 0 || speed > 0x7fff) return 0;
  const size_t len = (kGoalWidth + 1) * n + 4, total = len + 4;
  if (cap < total) return 0;
  for (unsigned i = 0; i < n; ++i) if (ticks[i] > 4095) return 0;
  size_t k = 0;
  out[k++] = 0xFF; out[k++] = 0xFF; out[k++] = kBroadcastId; out[k++] = static_cast<uint8_t>(len);
  out[k++] = kInstSync; out[k++] = kAddrAcc; out[k++] = kGoalWidth;
  for (unsigned i = 0; i < n; ++i) {
    out[k++] = ids[i]; out[k++] = acc;
    out[k++] = static_cast<uint8_t>(ticks[i] & 0xff); out[k++] = static_cast<uint8_t>(ticks[i] >> 8);
    out[k++] = 0; out[k++] = 0;
    out[k++] = static_cast<uint8_t>(speed & 0xff); out[k++] = static_cast<uint8_t>(speed >> 8);
  }
  out[k] = checksum(out, 2, k); ++k;
  return k == total ? k : 0;
}
inline size_t buildTorqueOff(const uint8_t ids[], unsigned n, uint8_t* out, size_t cap) {
  if (!ids || !out || n == 0 || n > 17 || !distinctIds(ids, n)) return 0;
  const size_t len = 2 * n + 4, total = len + 4;
  if (cap < total) return 0;
  size_t k = 0;
  out[k++] = 0xFF; out[k++] = 0xFF; out[k++] = kBroadcastId; out[k++] = static_cast<uint8_t>(len);
  out[k++] = kInstSync; out[k++] = kAddrTorqueEnable; out[k++] = 1;
  for (unsigned i = 0; i < n; ++i) { out[k++] = ids[i]; out[k++] = 0; }
  out[k] = checksum(out, 2, k); ++k;
  return k == total ? k : 0;
}
inline size_t buildRead(uint8_t id, uint8_t address, uint8_t width, uint8_t* out, size_t cap) {
  if (!out || cap < 8 || id >= kBroadcastId || width == 0 || width > 32) return 0;
  out[0] = 0xFF; out[1] = 0xFF; out[2] = id; out[3] = 4; out[4] = kInstRead; out[5] = address; out[6] = width;
  out[7] = checksum(out, 2, 7); return 8;
}
// Bounded status-packet parser (FF FF ID LEN ERR data[LEN-2] CHK). Fed one
// byte at a time; never reads past `width`, rejects any wrong header, ID,
// length or checksum. A non-zero servo ERROR byte is reported, not hidden.
enum class ParseState : uint8_t { HEADER1, HEADER2, ID, LEN, BODY, DONE, BAD };
enum class ParseError : uint8_t { NONE, HEADER, WRONG_ID, LENGTH, CHECKSUM, OVERFLOW_ };
struct StatusParser {
  uint8_t expected_id = 0, width = 0;
  uint8_t buf[40]{}; uint8_t n = 0, len = 0;
  ParseState state = ParseState::HEADER1; ParseError error = ParseError::NONE;
  uint8_t servo_error = 0;
  void reset(uint8_t id, uint8_t w) { *this = StatusParser{}; expected_id = id; width = w; }
  bool done() const { return state == ParseState::DONE; }
  bool bad() const { return state == ParseState::BAD; }
  void fail(ParseError e) { state = ParseState::BAD; error = e; }
  void feed(uint8_t b) {
    switch (state) {
      case ParseState::HEADER1: if (b == 0xFF) state = ParseState::HEADER2; else fail(ParseError::HEADER); break;
      case ParseState::HEADER2: if (b == 0xFF) state = ParseState::ID; else fail(ParseError::HEADER); break;
      case ParseState::ID: if (b == expected_id) { buf[0] = b; state = ParseState::LEN; } else fail(ParseError::WRONG_ID); break;
      case ParseState::LEN:
        if (b != width + 2) { fail(ParseError::LENGTH); break; }
        len = b; buf[1] = b; n = 2; state = ParseState::BODY; break;
      case ParseState::BODY:
        if (n >= sizeof(buf)) { fail(ParseError::OVERFLOW_); break; }
        buf[n++] = b;
        if (n == len + 2) {  // ID LEN ERR data... CHK
          if (checksum(buf, 0, n - 1) != buf[n - 1]) { fail(ParseError::CHECKSUM); break; }
          servo_error = buf[2]; state = ParseState::DONE;
        }
        break;
      case ParseState::DONE: fail(ParseError::OVERFLOW_); break;  // trailing byte
      case ParseState::BAD: break;
    }
  }
  const uint8_t* data() const { return buf + 3; }
};
} } }
#endif

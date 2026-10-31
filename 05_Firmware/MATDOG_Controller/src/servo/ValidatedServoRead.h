#ifndef MATDOG_SERVO_VALIDATED_READ_H
#define MATDOG_SERVO_VALIDATED_READ_H
#include <SCServo.h>
#include "BusTransaction.h"
#include "ServoReadValidation.h"
namespace matdog { namespace servo {
inline uint32_t busClockUs() { return static_cast<uint32_t>(micros()); }
// Keep existing SCServo writes. Strengthen all bus I/O at this ownership
// boundary without editing the installed library or caching feedback from a
// previous call. Core Motion V5: every SCServo byte (readSCS/writeSCS/rFlushSCS
// are the library's virtual hooks) is governed by the open BusTransaction -
// absolute budget, per-reply phase, STOP preemption, TX suppression once the
// transaction is closed. Outside a transaction the SDK may not touch the UART.
class ValidatedServoRead : public SMS_STS {
 public:
  ValidatedServoRead() { bus.bindClock(&busClockUs); }
  BusTransaction bus;
  // Operational recovery horizon, NOT a protocol guarantee. Reuse SCServo's
  // 100 ms host margin without spending it in a synchronous wait: the normal
  // 120 ms joint rotation retries. Late bytes seen before reissue are discarded.
  // FT-SCS cannot distinguish a same-ID/width reply first arriving AFTER this
  // horizon from a new one; this residual assumption needs hardware qualification.
  static constexpr uint32_t kReadRecoveryUs = 100000;
  bool replyPending(uint8_t id) const {
    if (id < 254) for (auto bits : pending_[id]) if (bits) return true;
    return false;
  }
  void discardReceived(uint8_t byte) { trackReply(byte); }

  // ---- SCServo hooks -------------------------------------------------------
  int readSCS(unsigned char* data, int length) override {
    int size = 0;
    if (pSerial == nullptr || length <= 0) return 0;
    while (size < length) {
      if (!bus.mayWait()) { bus.waitFailed(); break; }
      const int byte = pSerial->read();
      if (byte >= 0) { if (data) data[size] = static_cast<unsigned char>(byte); ++size; received(byte); }
    }
    if (size == length && !bus.mayWait()) { bus.waitFailed(); return 0; }
    return size;
  }
  int writeSCS(unsigned char* data, int length) override {
    if (data == nullptr || pSerial == nullptr || length <= 0 || !startPacketPiece(data, length)) return 0;
    const unsigned n = pSerial->write(data, length); sent(n, length); return static_cast<int>(n);
  }
  int writeSCS(unsigned char byte) override {
    return writeSCS(&byte, 1);
  }
  void rFlushSCS() override { drainRx(nullptr); }
  void wFlushSCS() override { endPacket(); }

  int Read(uint8_t id, uint8_t address, uint8_t* data, uint8_t width) {
    return ReadCaptured(id,address,data,width,nullptr);
  }
  // Optional RAM copy of THIS response, including invalid/partial packets.
  // No second read, retry, UART ownership change or relaxed packet validation.
  int ReadCaptured(uint8_t id,uint8_t address,uint8_t* data,uint8_t width,diagnostics::ReadCapture* capture){
    using diagnostics::ReadValidity;
    if(capture){*capture=diagnostics::ReadCapture{};capture->id=id;capture->address=address;capture->width=width;}
    if (data == nullptr || readWidthIndex(width) < 0 || pSerial == nullptr) {
      if(capture)capture->validity=ReadValidity::INVALID_ARGUMENT;
      return 0;
    }
    // Standalone use (tools, tests) opens its own bounded transaction; ServoBus
    // operations already hold one and share its absolute deadline.
    const bool own = !bus.open();
    if (own) {
      const uint32_t us = static_cast<uint32_t>(IOTimeOut) * 1000u;
      if (!bus.begin(us, us, false)) { if(capture)capture->validity=ReadValidity::NO_RESPONSE; return 0; }
    }
    uint8_t drained = 0;
    const bool drained_ok = drainRx(&drained);
    if(capture)capture->drained=drained;
    if (!drained_ok) {if(capture)capture->validity=ReadValidity::RX_FLOOD; if(own)bus.end(false); return 0;}
    uint8_t packet[38] = {0};
    uint8_t size = 0;
    const uint32_t started = millis();
    if(capture)capture->started_ms=started;
    request_width_ = width;
    const bool tx_allowed = bus.allowTx() && prepareReply(id, width);
    if (tx_allowed) {
      writeBuf(id, address, &width, 1, INST_READ);
      endPacket();
      while (size < width + 6) {
        if (!bus.mayWait()) { bus.waitFailed(); break; }
        const int byte = pSerial->read();
        if (byte >= 0) { packet[size++] = static_cast<uint8_t>(byte); received(byte); }
      }
    }
    auto validity=tx_allowed?servoReadValidity(packet,size,id,width):ReadValidity::NO_RESPONSE;
    if(capture){capture->size=size;capture->validity=validity;capture->finished_ms=millis();
      for(uint8_t i=0;i<size;++i)capture->bytes[i]=packet[i];}
    // A fully buffered reply can cross the deadline while this task is descheduled.
    // Packet structure alone never overrides the absolute transaction/phase budget.
    if (validity == ReadValidity::VALID && !bus.mayWait()) {
      bus.waitFailed(); validity = ReadValidity::DEADLINE;
      if (capture) capture->validity = validity;
    }
    if(own)bus.end(validity==ReadValidity::VALID);
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

 private:
  // All production READ shapes, including census, maintenance and calibration.
  // Keep direct indexing for ALL protocol IDs; no second servo allocation map.
  // Unsupported widths fail closed before TX, never alias another pending timer.
  static int readWidthIndex(uint8_t width) {
    switch(width){case 1:return 0;case 2:return 1;case 10:return 2;case 15:return 3;default:return -1;}
  }
  // Discard stale RX before a request (a late reply must never be parsed as the
  // next one). Bounded by BYTES and TIME: a bus that keeps talking is a fault.
  bool drainRx(uint8_t* drained_out) {
    uint8_t n = 0;
    if (drained_out) *drained_out = 0;
    if (pSerial == nullptr) return true;
    const uint32_t t0 = busClockUs();
    while (pSerial->available()) {
      if (n >= BusTransaction::kDrainMaxBytes || busDiffUs(busClockUs(), t0) >= static_cast<int32_t>(BusTransaction::kDrainMaxUs)) { bus.noteFlood(); return false; }
      const int byte = pSerial->read(); if (byte >= 0) trackReply(static_cast<uint8_t>(byte)); ++n;
    }
    if (drained_out) *drained_out = n;
    return true;
  }
  // One request may leave the UART in several writeSCS pieces; the permission to
  // send is decided once, at the first piece, so a deadline never cuts a packet.
  bool startPacketPiece(const uint8_t* data, int length) {
    if (tx_in_packet_) return !bus.closedForTx();
    if (tx_suppress_) return false;
    // Every ServoBus SDK request starts with writeBuf's complete header.
    if (length < 5 || data[0] != 255 || data[1] != 255 || data[2] >= 254) return false;
    tx_id_ = data[2]; tx_width_ = data[4] == INST_READ ? request_width_ : 0;
    if (!bus.allowTx()) { tx_suppress_ = true; return false; }
    if (!prepareReply(tx_id_, tx_width_)) { tx_suppress_ = true; return false; }
    tx_in_packet_ = true;
    return true;
  }
  bool prepareReply(uint8_t id, uint8_t width) {
    if (id >= 254 || (width && readWidthIndex(width) < 0)) {
      bus.notePendingRx(); return false;
    }
    const uint8_t bit = static_cast<uint8_t>(1u << (width % 8));
    if (pending_[id][width / 8] & bit) {
      // Preserve unresolved write ACKs. Only READ admission gains recovery;
      // STOP/SAFE_OFF and write verification keep their existing semantics.
      if (!width || static_cast<uint32_t>(busClockUs() - last_tx_end_us_[id][readWidthIndex(width)]) < kReadRecoveryUs) {
        bus.notePendingRx(); return false;
      }
      // Unlike available()/drain alone, flush(false) also purges the ESP32 RX
      // FIFO/ring. No remote cancellation is claimed. One UART owner and the
      // completed TX below bound this local purge; no sleep or extra READ.
      pSerial->flush(false);
      rx_size_ = 0;
      pending_[id][width / 8] &= static_cast<uint8_t>(~bit);
    }
    return true;
  }
  void sent(unsigned n, unsigned requested) {
    bus.onTx(n, requested);
    if (n) pending_[tx_id_][tx_width_ / 8] |= 1u << (tx_width_ % 8);
  }
  void received(uint8_t byte) { bus.onByte(); trackReply(byte); }
  void discardPrefix(unsigned n) {
    for (unsigned i = n; i < rx_size_; ++i) rx_[i-n] = rx_[i];
    rx_size_ -= n;
  }
  void trackReply(uint8_t byte) {
    if (rx_size_ == sizeof(rx_)) discardPrefix(1);
    rx_[rx_size_++] = byte;
    while (rx_size_ >= 4) {
      if (rx_[0] != 255 || rx_[1] != 255 || rx_[2] >= 254 || rx_[3] < 2 || rx_[3] > 34) { discardPrefix(1); continue; }
      const unsigned size = rx_[3] + 4;
      if (rx_size_ < size) return;
      uint8_t sum = 0; for (unsigned i = 2; i < size; ++i) sum += rx_[i];
      if (sum != 255) { discardPrefix(1); continue; }
      const uint8_t id = rx_[2], width = rx_[3] - 2;
      pending_[id][width / 8] &= static_cast<uint8_t>(~(1u << (width % 8)));
      discardPrefix(size);
    }
  }
  void endPacket() {
    if (tx_in_packet_) {
      // Account from physical TX completion, not from enqueue into TX FIFO.
      // A partial packet also leaves a debt; never refresh it on suppressed TX.
      pSerial->flush();
      if (tx_width_) last_tx_end_us_[tx_id_][readWidthIndex(tx_width_)] = busClockUs();
      bus.txDone();
    }
    tx_in_packet_ = false; tx_suppress_ = false;
  }
  bool tx_in_packet_ = false, tx_suppress_ = false;
  uint8_t request_width_ = 0, tx_id_ = 0, tx_width_ = 0;
  uint8_t pending_[254][5]{}; // one bit per reply width 0..32 (0 = ACK/Ping)
  // Match the pending identity: a successful width15 must not postpone
  // recovery of an unanswered width10 on the same servo (or vice versa).
  uint32_t last_tx_end_us_[254][4]{}; // byte, word, control block, fast block
  uint8_t rx_[38]{}, rx_size_ = 0;
};
} }
#endif

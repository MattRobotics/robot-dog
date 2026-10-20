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

  // ---- SCServo hooks -------------------------------------------------------
  int readSCS(unsigned char* data, int length) override {
    int size = 0;
    if (pSerial == nullptr || length <= 0) return 0;
    while (size < length) {
      const int byte = pSerial->read();
      if (byte >= 0) { if (data) data[size] = static_cast<unsigned char>(byte); ++size; bus.onByte(); continue; }
      if (!bus.mayWait()) { bus.waitFailed(); break; }
    }
    return size;
  }
  int writeSCS(unsigned char* data, int length) override {
    if (data == nullptr || pSerial == nullptr || !startPacketPiece()) return 0;
    return static_cast<int>(pSerial->write(data, length));
  }
  int writeSCS(unsigned char byte) override {
    if (pSerial == nullptr || !startPacketPiece()) return 0;
    return static_cast<int>(pSerial->write(&byte, 1));
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
    if (data == nullptr || width == 0 || width > 32 || pSerial == nullptr) {
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
    const bool tx_allowed = bus.allowTx();
    if (tx_allowed) {
      tx_in_packet_ = true;
      writeBuf(id, address, &width, 1, INST_READ);
      endPacket();
      while (size < width + 6) {
        const int byte = pSerial->read();
        if (byte >= 0) { packet[size++] = static_cast<uint8_t>(byte); bus.onByte(); continue; }
        if (!bus.mayWait()) { bus.waitFailed(); break; }
      }
    }
    const auto validity=tx_allowed?servoReadValidity(packet,size,id,width):ReadValidity::NO_RESPONSE;
    if(capture){capture->size=size;capture->validity=validity;capture->finished_ms=millis();
      for(uint8_t i=0;i<size;++i)capture->bytes[i]=packet[i];}
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
  // Discard stale RX before a request (a late reply must never be parsed as the
  // next one). Bounded by BYTES and TIME: a bus that keeps talking is a fault.
  bool drainRx(uint8_t* drained_out) {
    uint8_t n = 0;
    if (drained_out) *drained_out = 0;
    if (pSerial == nullptr) return true;
    const uint32_t t0 = busClockUs();
    while (pSerial->available()) {
      if (n >= BusTransaction::kDrainMaxBytes || busDiffUs(busClockUs(), t0) >= static_cast<int32_t>(BusTransaction::kDrainMaxUs)) { bus.noteFlood(); return false; }
      pSerial->read(); ++n;
    }
    if (drained_out) *drained_out = n;
    return true;
  }
  // One request may leave the UART in several writeSCS pieces; the permission to
  // send is decided once, at the first piece, so a deadline never cuts a packet.
  bool startPacketPiece() {
    if (tx_in_packet_) return true;
    if (tx_suppress_) return false;
    if (!bus.allowTx()) { tx_suppress_ = true; return false; }
    tx_in_packet_ = true;
    return true;
  }
  void endPacket() {
    if (tx_in_packet_) bus.txDone();
    tx_in_packet_ = false; tx_suppress_ = false;
  }
  bool tx_in_packet_ = false, tx_suppress_ = false;
};
} }
#endif

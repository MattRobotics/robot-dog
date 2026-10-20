#pragma once
#include "Arduino.h"
// Router tests never touch a bus: this models only the SCServo hook surface that
// ValidatedServoRead overrides (src/servo/ValidatedServoRead.h).
constexpr unsigned char INST_READ=2;
class SMS_STS {
 public: unsigned long IOTimeOut = 100; HardwareSerial* pSerial=nullptr;
 virtual ~SMS_STS() = default;
 protected:
  virtual int writeSCS(unsigned char*,int n) { return n; }
  virtual int writeSCS(unsigned char) { return 1; }
  virtual int readSCS(unsigned char*,int) { return 0; }
  virtual void rFlushSCS() {}
  virtual void wFlushSCS() {}
  void writeBuf(unsigned char,unsigned char,unsigned char*,unsigned char,unsigned char) {}
};

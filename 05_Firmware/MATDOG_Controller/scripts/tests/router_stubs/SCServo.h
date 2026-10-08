#pragma once
#include "Arduino.h"
constexpr unsigned char INST_READ=2;
class SMS_STS {
 public: unsigned long IOTimeOut = 100; HardwareSerial* pSerial=nullptr;
 protected: void writeBuf(unsigned char,unsigned char,unsigned char*,unsigned char,unsigned char) {}
};

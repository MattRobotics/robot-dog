#pragma once
// Host stub for compiling the installed SCServo 1.0.2 protocol layer
// (SCS.cpp, SMS_STS.cpp) in test_servo_packet.cpp. No UART exists.
#include <cstddef>
#include <cstdint>
#include <cstring>
class HardwareSerial{public:int read(){return -1;}std::size_t write(const unsigned char*,std::size_t n){return n;}std::size_t write(const unsigned char* p,int n){return write(p,static_cast<std::size_t>(n));}std::size_t write(unsigned char){return 1;}int available(){return 0;}};
inline unsigned long millis(){static unsigned long value=0;return ++value;}

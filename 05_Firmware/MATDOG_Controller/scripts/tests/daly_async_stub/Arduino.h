#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <deque>
inline uint32_t test_ms=0;
inline unsigned long millis(){return test_ms;}
constexpr int SERIAL_8N1=0;
class HardwareSerial {
 public:
  static inline HardwareSerial* uart=nullptr;
  std::deque<uint8_t> rx;std::vector<uint8_t> tx;int room=128;unsigned flushes=0;
  explicit HardwareSerial(int){uart=this;}
  void begin(unsigned long,int,int,int){}
  int available(){return int(rx.size());}
  int availableForWrite(){return room;}
  int read(){if(rx.empty())return -1;int b=rx.front();rx.pop_front();return b;}
  size_t write(const uint8_t* p,size_t n){tx.insert(tx.end(),p,p+n);return n;}
  void flush(){++flushes;test_ms+=9;}
};

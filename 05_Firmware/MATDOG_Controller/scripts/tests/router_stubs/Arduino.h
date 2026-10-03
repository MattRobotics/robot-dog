#ifndef MATDOG_TEST_ROUTER_ARDUINO_H
#define MATDOG_TEST_ROUTER_ARDUINO_H
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdarg>
#include <cctype>
#include <string>
#include <deque>
// Only platform transport is simulated; both router translation units are real.
class String {
 public:
  String() = default;
  String(const char* s) : text_(s) {}
  const char* c_str() const { return text_.c_str(); }
  size_t length() const { return text_.size(); }
  bool operator==(const String& s) const { return text_ == s.text_; }
  bool operator==(const char* s) const { return text_ == s; }
  String& operator+=(const char* s) { text_ += s; return *this; }
  bool startsWith(const char* s) const { return text_.rfind(s, 0) == 0; }
  void trim() {
    const auto first = text_.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) { text_.clear(); return; }
    text_ = text_.substr(first, text_.find_last_not_of(" \t\r\n") - first + 1);
  }
  void toUpperCase() { for (char& c : text_) c = std::toupper(static_cast<unsigned char>(c)); }
 private:
  std::string text_;
};
namespace router_test {
inline uint32_t now_ms = 1000;
inline unsigned hardware_calls = 0;
}
inline uint32_t millis() { return router_test::now_ms; }
class HardwareSerial { public: explicit HardwareSerial(int) {} int available() { return 0; } int read() { return -1; } };
class HostSerial {
 public:
  std::string output;
  std::deque<char> input;
  int available() const { return static_cast<int>(input.size()); }
  int read() { const char c = input.front(); input.pop_front(); return c; }
  int availableForWrite() const { return 4096; }
  void print(const char* s) { output += s; }
  void println(const char* s) { output += s; output += '\n'; }
  void println(const String& s) { println(s.c_str()); }
  void println() { output += '\n'; }
  void printf(const char* format, ...) {
    char buf[2048]; va_list args; va_start(args, format);
    std::vsnprintf(buf, sizeof(buf), format, args); va_end(args); output += buf;
  }
};
inline HostSerial Serial;
struct HostEsp { unsigned getFreeHeap() const { return 0; } unsigned getMinFreeHeap() const { return 0; } };
inline HostEsp ESP;
#endif

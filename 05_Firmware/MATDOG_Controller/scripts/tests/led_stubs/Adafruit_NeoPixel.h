#ifndef MATDOG_TEST_LED_NEOPIXEL_H
#define MATDOG_TEST_LED_NEOPIXEL_H

#include <Arduino.h>
#include <array>
#include <vector>

constexpr uint16_t NEO_GRB = 0;
constexpr uint16_t NEO_KHZ800 = 0;

namespace led_test {
struct TransportFrame {
  std::array<uint32_t, 12> pixels;
  uint8_t brightness;
};
inline unsigned begin_calls = 0;
inline std::vector<TransportFrame> frames;
inline void reset() {
  now_ms = 0;
  pin_modes.clear();
  begin_calls = 0;
  frames.clear();
}
}

// Only the used NeoPixel API is simulated. Channel scaling reproduces the
// library's brightness+1 arithmetic so the old solid/chase output is tested
// at the same transport boundary as the new independently scaled frames.
class Adafruit_NeoPixel {
 public:
  Adafruit_NeoPixel(uint16_t, int pin, uint16_t) : pin_(pin) {}
  void begin() {
    ++led_test::begin_calls;
    pinMode(pin_, OUTPUT);
  }
  void clear() { pixels_.fill(0); }
  void show() {
    led_test::frames.push_back({pixels_, static_cast<uint8_t>(brightness_ - 1)});
  }
  void setBrightness(uint8_t value) {
    const uint8_t next = static_cast<uint8_t>(value + 1);
    if (next == brightness_) return;
    const uint8_t previous = static_cast<uint8_t>(brightness_ - 1);
    const uint16_t scale = previous == 0 ? 0 : value == 255 ?
        static_cast<uint16_t>(65535 / previous) :
        static_cast<uint16_t>(((static_cast<uint16_t>(next) << 8) - 1) / previous);
    for (uint32_t& pixel : pixels_) {
      pixel = Color(static_cast<uint8_t>(((pixel >> 16) & 255) * scale >> 8),
                    static_cast<uint8_t>(((pixel >> 8) & 255) * scale >> 8),
                    static_cast<uint8_t>((pixel & 255) * scale >> 8));
    }
    brightness_ = next;
  }
  void setPixelColor(uint16_t index, uint32_t color) {
    if (index >= pixels_.size()) return;
    if (brightness_ != 0) {
      color = Color(static_cast<uint8_t>(((color >> 16) & 255) * brightness_ >> 8),
                    static_cast<uint8_t>(((color >> 8) & 255) * brightness_ >> 8),
                    static_cast<uint8_t>((color & 255) * brightness_ >> 8));
    }
    pixels_[index] = color;
  }
  static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
    return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | b;
  }

 private:
  int pin_;
  uint8_t brightness_ = 0;
  std::array<uint32_t, 12> pixels_{};
};

#endif

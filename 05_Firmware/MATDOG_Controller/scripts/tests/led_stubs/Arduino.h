#ifndef MATDOG_TEST_LED_ARDUINO_H
#define MATDOG_TEST_LED_ARDUINO_H

#include <stdint.h>
#include <vector>

// Host-only transport observations. The real ring/manager/policy are linked;
// neither this clock nor pinMode can touch a device.
constexpr uint8_t INPUT = 0;
constexpr uint8_t OUTPUT = 1;

namespace led_test {
struct PinModeCall { int pin; uint8_t mode; };
inline uint32_t now_ms = 0;
inline std::vector<PinModeCall> pin_modes;
}

inline uint32_t millis() { return led_test::now_ms; }
inline void pinMode(int pin, uint8_t mode) {
  led_test::pin_modes.push_back({pin, mode});
}

#endif

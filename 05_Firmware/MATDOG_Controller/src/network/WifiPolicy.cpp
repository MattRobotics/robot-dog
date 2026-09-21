#include "WifiPolicy.h"

namespace matdog {
namespace network {

void WifiPolicy::begin(const WifiPolicyConfig& config, bool credentials_present,
                       uint32_t now_ms) {
  config_ = config;
  credentials_present_ = credentials_present;

  // Enabled follows the credentials: a build with nothing configured must
  // never start the radio, and there is no state in which it silently does.
  // The operator can still switch it off later with @WIFI OFF; the operator
  // cannot switch it ON without credentials, because there is nothing to
  // connect to and pretending otherwise would just produce a retry storm.
  enabled_ = credentials_present;

  radio_started_ = false;
  state_ = WifiState::INACTIVE;
  fault_ = credentials_present ? WifiFault::NONE : WifiFault::NO_CREDENTIALS;
  state_since_ms_ = now_ms;
  backoff_ms_ = config_.backoff_initial_ms;
  counters_ = WifiCounters{0, 0, 0, 0, 0};
}

void WifiPolicy::setEnabled(bool enabled, uint32_t now_ms) {
  if (enabled && !credentials_present_) {
    // Fail closed, and say why: the refusal is reported through fault(),
    // not swallowed.
    fault_ = WifiFault::NO_CREDENTIALS;
    return;
  }
  if (enabled_ == enabled) return;
  enabled_ = enabled;
  state_since_ms_ = now_ms;
}

void WifiPolicy::enter(WifiState next, uint32_t now_ms) {
  state_ = next;
  state_since_ms_ = now_ms;
}

void WifiPolicy::enterBackoff(uint32_t now_ms) {
  enter(WifiState::BACKOFF, now_ms);
}

WifiAction WifiPolicy::update(uint32_t now_ms, bool link_up) {
  // ---- Gate: may this layer touch the radio at all? --------------------
  if (!enabled_ || !credentials_present_) {
    if (state_ == WifiState::INACTIVE) return WifiAction::NONE;
    const bool had_radio = radio_started_;
    radio_started_ = false;
    enter(WifiState::INACTIVE, now_ms);
    // Only ask for a teardown if something was actually brought up. STOP on
    // an already-stopped radio would be a pointless driver call in the
    // control loop.
    return had_radio ? WifiAction::STOP_RADIO : WifiAction::NONE;
  }

  switch (state_) {
    case WifiState::INACTIVE:
      // Re-armed (credentials appeared, or @WIFI ON). One transition per
      // tick: the radio starts on the next one.
      fault_ = WifiFault::NONE;
      enter(WifiState::IDLE, now_ms);
      return WifiAction::NONE;

    case WifiState::IDLE:
      if (radio_started_) {
        counters_.connect_attempts++;
        enter(WifiState::CONNECTING, now_ms);
        return WifiAction::START_CONNECT;
      }
      counters_.radio_starts++;
      radio_started_ = true;
      enter(WifiState::RADIO_STARTING, now_ms);
      return WifiAction::START_RADIO;

    case WifiState::RADIO_STARTING:
      // See WifiPolicyConfig::radio_settle_ms — this wait is what keeps the
      // 1000 ms blocking wait inside WiFi.begin() from ever being reached.
      if (now_ms - state_since_ms_ < config_.radio_settle_ms) return WifiAction::NONE;
      counters_.connect_attempts++;
      enter(WifiState::CONNECTING, now_ms);
      return WifiAction::START_CONNECT;

    case WifiState::CONNECTING:
      if (link_up) {
        counters_.connects++;
        backoff_ms_ = config_.backoff_initial_ms;
        fault_ = WifiFault::NONE;
        enter(WifiState::CONNECTED, now_ms);
        return WifiAction::NONE;
      }
      if (now_ms - state_since_ms_ >= config_.connect_timeout_ms) {
        counters_.connect_timeouts++;
        fault_ = WifiFault::CONNECT_TIMEOUT;
        enterBackoff(now_ms);
      }
      return WifiAction::NONE;

    case WifiState::CONNECTED:
      if (!link_up) {
        counters_.link_losses++;
        fault_ = WifiFault::LINK_LOST;
        // A link that was up and dropped is a fresh event (AP rebooted,
        // walked out of range). It does not inherit the retry pressure of
        // an earlier failure, so the ladder restarts from the bottom.
        backoff_ms_ = config_.backoff_initial_ms;
        enterBackoff(now_ms);
      }
      return WifiAction::NONE;

    case WifiState::BACKOFF:
      if (link_up) {
        // The driver got there on its own between ticks. Accept reality
        // rather than tearing down a working link to follow the plan.
        counters_.connects++;
        backoff_ms_ = config_.backoff_initial_ms;
        fault_ = WifiFault::NONE;
        enter(WifiState::CONNECTED, now_ms);
        return WifiAction::NONE;
      }
      if (now_ms - state_since_ms_ < backoff_ms_) return WifiAction::NONE;

      // Grow the wait for the NEXT failure, capped. Computed as a division
      // rather than a multiplication so it cannot overflow on the way to
      // the ceiling.
      {
        const uint32_t grown = (backoff_ms_ > config_.backoff_max_ms / 2)
                                   ? config_.backoff_max_ms
                                   : backoff_ms_ * 2;
        backoff_ms_ = grown;
      }

      if (!radio_started_) {
        // The radio never came up (RADIO_START_FAILED). Go round through
        // IDLE so the next action is START_RADIO, not an association
        // attempt against a driver that is not running.
        enter(WifiState::IDLE, now_ms);
        return WifiAction::NONE;
      }
      counters_.connect_attempts++;
      enter(WifiState::CONNECTING, now_ms);
      return WifiAction::START_CONNECT;
  }

  return WifiAction::NONE;
}

void WifiPolicy::reportActionFailed(WifiFault fault, uint32_t now_ms) {
  fault_ = fault;
  if (fault == WifiFault::RADIO_START_FAILED) {
    radio_started_ = false;
  }
  enterBackoff(now_ms);
}

const char* toString(WifiState state) {
  switch (state) {
    case WifiState::INACTIVE:       return "INACTIVE";
    case WifiState::IDLE:           return "IDLE";
    case WifiState::RADIO_STARTING: return "RADIO_STARTING";
    case WifiState::CONNECTING:     return "CONNECTING";
    case WifiState::CONNECTED:      return "CONNECTED";
    case WifiState::BACKOFF:        return "BACKOFF";
  }
  return "UNKNOWN";
}

const char* toString(WifiFault fault) {
  switch (fault) {
    case WifiFault::NONE:                  return "NONE";
    case WifiFault::NO_CREDENTIALS:        return "NO_CREDENTIALS";
    case WifiFault::CONNECT_TIMEOUT:       return "CONNECT_TIMEOUT";
    case WifiFault::LINK_LOST:             return "LINK_LOST";
    case WifiFault::RADIO_START_FAILED:    return "RADIO_START_FAILED";
    case WifiFault::CONNECT_CALL_REJECTED: return "CONNECT_CALL_REJECTED";
  }
  return "UNKNOWN";
}

void formatIpv4(uint32_t ipv4, char* out, uint32_t out_size) {
  if (out == nullptr || out_size == 0) return;

  const uint8_t octet[4] = {
      static_cast<uint8_t>(ipv4 & 0xFFu),
      static_cast<uint8_t>((ipv4 >> 8) & 0xFFu),
      static_cast<uint8_t>((ipv4 >> 16) & 0xFFu),
      static_cast<uint8_t>((ipv4 >> 24) & 0xFFu),
  };

  // Hand-rolled rather than snprintf: this runs in the control loop, the
  // output is bounded at 15 characters + NUL by construction, and it keeps
  // the unit free of <stdio.h>.
  uint32_t w = 0;
  for (uint32_t i = 0; i < 4; ++i) {
    if (i > 0 && w + 1 < out_size) out[w++] = '.';
    uint8_t v = octet[i];
    char digits[3];
    uint32_t n = 0;
    do {
      digits[n++] = static_cast<char>('0' + (v % 10));
      v = static_cast<uint8_t>(v / 10);
    } while (v != 0 && n < 3);
    while (n > 0 && w + 1 < out_size) out[w++] = digits[--n];
  }
  out[w < out_size ? w : out_size - 1] = '\0';
}

}  // namespace network
}  // namespace matdog

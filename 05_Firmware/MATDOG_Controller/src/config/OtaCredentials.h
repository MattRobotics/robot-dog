#ifndef MATDOG_CONFIG_OTA_CREDENTIALS_H
#define MATDOG_CONFIG_OTA_CREDENTIALS_H

#include <stddef.h>

// ---------------------------------------------------------------------------
// OTA SHARED SECRET — same resolution order and reasoning as
// WifiCredentials.h, extended to the OTA HMAC authentication key (I7,
// 2026-09-25 network transport correction).
// ---------------------------------------------------------------------------
//   1. -DMATDOG_OTA_SECRET build flag, if given.
//   2. otherwise src/config/OtaCredentials.local.h, if it exists.
//   3. otherwise EMPTY — and an empty secret means
//      OtaSession::authenticate() fails closed with REJECTED_NO_SECRET for
//      every request (the same fail-closed shape WifiPolicy uses for an
//      absent SSID). A checkout with no secret configured builds, boots,
//      and can still serve the read-only /status endpoint if the Web
//      server is explicitly started — it simply can never accept an OTA
//      update.
//
// src/config/OtaCredentials.local.h is listed in this sketch's .gitignore
// and is NOT tracked. scripts/static_audit.py fails the build if that
// .gitignore entry is ever removed. Use OtaCredentials.local.h.example as
// the starting point.
//
// This secret authenticates OTA session requests via HMAC-SHA256
// (update/Hmac256.h, update/OtaSession.h) — deliberately a SEPARATE secret
// from the Wi-Fi passphrase: knowing the Wi-Fi password only gets an
// attacker onto the LAN, not the ability to push firmware. See
// 09_Logs/Development_Log for the full evaluation of why HMAC over a
// pre-shared secret was chosen over full TLS for this transport.

#if defined(__has_include)
#if __has_include("OtaCredentials.local.h")
#include "OtaCredentials.local.h"
#endif
#endif

#ifndef MATDOG_OTA_SECRET
#define MATDOG_OTA_SECRET ""
#endif

namespace matdog {
namespace config {

// The ONE place the secret exists as a named symbol. It is passed straight
// into OtaSession::begin() in Controller.cpp and stored nowhere else — not
// in a status struct, not in an accessor, not in a log line.
// scripts/static_audit.py enforces that this symbol appears exactly once
// outside this header.
constexpr const char* kOtaSecret = MATDOG_OTA_SECRET;

// sizeof("") == 1, so this is a compile-time "a secret was configured"
// test that needs no strlen() and no runtime work.
constexpr bool kOtaSecretPresent = (sizeof(MATDOG_OTA_SECRET) > 1);

}  // namespace config
}  // namespace matdog

#endif  // MATDOG_CONFIG_OTA_CREDENTIALS_H

#ifndef MATDOG_CONFIG_WIFI_CREDENTIALS_H
#define MATDOG_CONFIG_WIFI_CREDENTIALS_H

#include <stddef.h>

// ---------------------------------------------------------------------------
// WI-FI CREDENTIALS — resolution order, and why secrets stay out of Git.
// ---------------------------------------------------------------------------
// The repository had no local/private configuration convention before W1
// (grep for secret/credential/password/Secrets.h found nothing), so this
// header establishes one, deliberately small:
//
//   1. -DMATDOG_WIFI_SSID / -DMATDOG_WIFI_PASSWORD build flags, if given.
//   2. otherwise src/config/WifiCredentials.local.h, if it exists.
//   3. otherwise EMPTY — and an empty SSID means the radio is never
//      started at all (network/WifiPolicy.cpp fails closed on
//      NO_CREDENTIALS). A checkout with no credentials builds, boots and
//      runs normally; it simply has no Wi-Fi.
//
// The local header is the PREFERRED mechanism for a workstation, and the
// one documented in the README, because a passphrase passed as -D lands in
// shell history, in `ps` output and in the build log that scripts/build.sh
// echoes. The build-flag path is kept for a future CI/secret-store that has
// no filesystem to write to, and it wins so it can override a stale local
// file.
//
// src/config/WifiCredentials.local.h is listed in this sketch's .gitignore
// and is NOT tracked. scripts/static_audit.py fails the build if that
// .gitignore entry is ever removed, so a real passphrase cannot become
// committable by an unreviewed edit. Use WifiCredentials.local.h.example as
// the starting point.
//
// NOTE ON WHAT THIS DOES NOT CLAIM: a credential compiled into an
// application image is readable by anyone who can read the flash. That is
// accepted for a home 2.4 GHz network on a bench robot. It is NOT an
// authentication story for OTA — OTA-A defines its own, separately (handoff
// section 17).

#if defined(__has_include)
#if __has_include("WifiCredentials.local.h")
#include "WifiCredentials.local.h"
#endif
#endif

#ifndef MATDOG_WIFI_SSID
#define MATDOG_WIFI_SSID ""
#endif

#ifndef MATDOG_WIFI_PASSWORD
#define MATDOG_WIFI_PASSWORD ""
#endif

namespace matdog {
namespace config {

constexpr const char* kWifiSsid = MATDOG_WIFI_SSID;

// The ONE place the passphrase exists as a named symbol. It is passed
// straight to the connect call in network/WifiManager.cpp and stored
// nowhere: not in WifiStatus, not in an accessor, not in a log line.
// scripts/static_audit.py enforces that this symbol appears exactly once
// outside this header.
constexpr const char* kWifiPassword = MATDOG_WIFI_PASSWORD;

// sizeof("") == 1, so this is a compile-time "an SSID was configured"
// test that needs no strlen() and no runtime work.
constexpr bool kWifiCredentialsPresent = (sizeof(MATDOG_WIFI_SSID) > 1);

}  // namespace config
}  // namespace matdog

#endif  // MATDOG_CONFIG_WIFI_CREDENTIALS_H

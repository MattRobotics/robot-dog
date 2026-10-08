#ifndef MATDOG_CONFIG_TLS_IDENTITY_H
#define MATDOG_CONFIG_TLS_IDENTITY_H
// Provision per-device identity through the authorized physical firmware path.
// PEM key/cert are intentionally absent in source. No shared/default identity.
#if __has_include("TlsIdentity.local.h")
#include "TlsIdentity.local.h"
#endif
#ifndef MATDOG_TLS_CERT_PEM
#define MATDOG_TLS_CERT_PEM ""
#endif
#ifndef MATDOG_TLS_KEY_PEM
#define MATDOG_TLS_KEY_PEM ""
#endif
namespace matdog {
namespace config {
static constexpr char kTlsCertificate[] = MATDOG_TLS_CERT_PEM;
static constexpr char kTlsPrivateKey[] = MATDOG_TLS_KEY_PEM;
static constexpr bool kTlsIdentityPresent = sizeof(kTlsCertificate) > 1 &&
                                            sizeof(kTlsPrivateKey) > 1;
} // namespace config
} // namespace matdog
#endif

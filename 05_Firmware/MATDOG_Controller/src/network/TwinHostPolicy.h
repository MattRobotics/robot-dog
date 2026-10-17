#ifndef MATDOG_TWIN_HOST_POLICY_H
#define MATDOG_TWIN_HOST_POLICY_H
namespace matdog { namespace network {
// Explicit deployment allowlist: PC on MATDOG AP (reserved .2) or its reviewed
// LAN address. Changing a host requires a reviewed config/build, never a wildcard.
// MotionDashboard's matching list is pinned against this header by regression.
static constexpr char kDashboardContentSecurityPolicy[] =
    "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; "
    "connect-src 'self'; frame-src http://192.168.4.2:8765 http://192.168.1.149:8765; "
    "frame-ancestors 'none'; base-uri 'none'; form-action 'self'";
} }
#endif

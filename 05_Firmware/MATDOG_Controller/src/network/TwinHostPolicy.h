#ifndef MATDOG_TWIN_HOST_POLICY_H
#define MATDOG_TWIN_HOST_POLICY_H
namespace matdog { namespace network {
// The Digital Twin is served by the Controller itself (/twin/), same origin:
// no PC bridge host is part of the operational Dashboard.
static constexpr char kDashboardContentSecurityPolicy[] =
    "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; "
    "connect-src 'self'; frame-src 'self'; img-src 'self'; "
    "frame-ancestors 'none'; base-uri 'none'; form-action 'self'";
} }
#endif

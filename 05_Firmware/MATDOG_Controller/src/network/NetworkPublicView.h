#ifndef MATDOG_NETWORK_PUBLIC_VIEW_H
#define MATDOG_NETWORK_PUBLIC_VIEW_H
#include "WifiPolicy.h"
namespace matdog { namespace network {
// The public status type has no password, token, admin digest or TLS key.
// Names/scans are additionally hidden outside an authenticated AP session.
inline void redactNetworkNames(WifiStatus& w){
 w.ssid[0]=w.bssid[0]=w.ap_ssid[0]=w.sta_test.ssid[0]=0;
 for(auto& p:w.profiles)p.ssid[0]=0;
 for(auto& p:w.scan){p.ssid[0]=p.bssid[0]=0;}
 w.scan_count=0;
}
} }
#endif

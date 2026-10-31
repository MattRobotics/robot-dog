#include "../../src/network/NetworkPublicView.h"
#include <cstring>
#include <cassert>
#include <cstdio>
int main(){matdog::network::WifiStatus original;strcpy(original.ssid,"PRIVATE_LAN");strcpy(original.ap_ssid,"PRIVATE_AP");strcpy(original.bssid,"11:22:33:44:55:66");for(auto& p:original.profiles)strcpy(p.ssid,"PRIVATE_PROFILE");strcpy(original.sta_test.ssid,"PRIVATE_TEST");original.scan_count=12;for(auto& p:original.scan){strcpy(p.ssid,"PRIVATE_SCAN");strcpy(p.bssid,"PRIVATE_BSSID");}auto view=original;matdog::network::redactNetworkNames(view);assert(*original.ssid&&*original.profiles[0].ssid);assert(!*view.ssid&&!*view.ap_ssid&&!*view.bssid&&!*view.sta_test.ssid&&view.scan_count==0);for(auto p:view.profiles)assert(!*p.ssid);for(auto p:view.scan)assert(!*p.ssid&&!*p.bssid);puts("NETWORK_PUBLIC_VIEW PASS names/scans redacted; source immutable; no secret fields in status type");}

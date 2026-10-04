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
        for mutation in mutations:
            with self.subTest(mutation=mutation):self.assertTrue(self.check(mutation))
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

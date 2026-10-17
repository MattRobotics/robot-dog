import importlib.util,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location('release_bridge',ROOT/'06_Software/Matdog_Core/host_bridge/matdog_bridge.py')
b=importlib.util.module_from_spec(spec);sys.modules[spec.name]=b;spec.loader.exec_module(b)
class Timestamp(unittest.TestCase):
 def test_log_declares_utc_rome_dst_source_and_keeps_raw_nonce(self):
  with tempfile.TemporaryDirectory() as d:
   log=b.SessionLog(d)
   for utc,off,zone in [('2026-10-16T18:29:00Z','+02:00','CEST'),('2026-10-26T18:29:00Z','+01:00','CET')]:
    with patch.object(b,'utc_now',return_value=utc):log.write('RX','@TEL {"boot_nonce":12,"now_ms":3000}')
   log.close();records=[json.loads(s) for s in Path(log.path).read_text().splitlines()]
   for record,zone in zip(records,['CEST','CET']):
    self.assertEqual(record['zone'],zone);self.assertEqual(record['time_source'],'host_wall_clock')
    self.assertTrue(record['utc'].endswith('Z'));self.assertIn('boot_nonce',record['line']);self.assertGreater(record['mono'],0)
   self.assertTrue(records[0]['rome'].endswith('+02:00'));self.assertTrue(records[1]['rome'].endswith('+01:00'))
if __name__=='__main__':unittest.main()

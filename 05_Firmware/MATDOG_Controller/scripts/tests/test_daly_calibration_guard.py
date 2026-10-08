#!/usr/bin/env python3
"""Pure DALY observer regressions: synthetic transcript, no serial access."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from daly_calibration_guard import DalyGuard, DalyGuardFailure

NORMAL = ["DALY   init=OK detected=ONLINE expected=REQUIRED result=PASS",
          "  comm=OK age_ms=100",
          "  pack_v=10.8 current_a=-1.0 soc=0.0% cells=3",
          "  cell_max_mv=3600 cell_min_mv=3600 delta_mv=0",
          "  charge_mos=ON discharge_mos=ON state=IDLE alarms=0000 0000 0000 0000"]

class GuardTests(unittest.TestCase):
    def sample(self, lines=NORMAL):
        guard=DalyGuard()
        for line in lines: guard.feed(line,10)
        return guard

    def test_exact_thresholds_and_soc_not_gate(self):
        g=self.sample(); g.check(10)
        self.assertEqual(g.snapshot()['soc_percent'],0)

    def test_low_pack_cell_alarm_comm_age_and_topology(self):
        cases=[(2,'10.8','10.7'),(3,'cell_min_mv=3600','cell_min_mv=3599'),
               (4,'alarms=0000','alarms=0001'),(1,'comm=OK','comm=TIMEOUT'),
               (1,'age_ms=100','age_ms=5001'),(2,'cells=3','cells=4'),
               (4,'discharge_mos=ON','discharge_mos=OFF'),(2,'pack_v=10.8','pack_v=nan'),
               (3,'delta_mv=0','delta_mv=9'),(0,'detected=ONLINE','detected=NO_RESPONSE')]
        for index,old,new in cases:
            with self.subTest(new=new), self.assertRaises(DalyGuardFailure):
                lines=NORMAL.copy(); lines[index]=lines[index].replace(old,new); self.sample(lines)

    def test_incomplete_and_missing_do_not_authorize(self):
        with self.assertRaises(DalyGuardFailure): DalyGuard().check(0)
        g=self.sample(NORMAL[:3])
        with self.assertRaisesRegex(DalyGuardFailure,'incomplete'): g.check(11.01)
        g=self.sample()
        with self.assertRaisesRegex(DalyGuardFailure,'stale'): g.check(15)

    def test_summary_does_not_refresh_old_sample(self):
        g=self.sample(); g.feed(NORMAL[0],14); g.feed('SERVO init=OK',14)
        self.assertEqual(g.snapshot()['pack_v'],10.8)
        with self.assertRaises(DalyGuardFailure): g.check(15)

    def test_fault_latches_even_if_followed_by_normal(self):
        g=self.sample(); lines=NORMAL.copy(); lines[2]=lines[2].replace('10.8','10.7')
        with self.assertRaises(DalyGuardFailure):
            for line in lines: g.feed(line,11)
        with self.assertRaises(DalyGuardFailure): g.feed(NORMAL[0],12)

    def test_partial_block_cannot_be_replaced(self):
        g=self.sample(NORMAL[:3])
        with self.assertRaisesRegex(DalyGuardFailure,'incomplete'): g.feed(NORMAL[0],10.5)

if __name__=='__main__': unittest.main()

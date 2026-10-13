#!/usr/bin/env python3
"""Real offline card generator -> real bounded firmware parser. No devices.
All approvals below are synthetic test fixtures, never operator evidence.
"""
import copy,hashlib,json,subprocess,sys,tempfile,unittest
from pathlib import Path
FW=Path(__file__).resolve().parents[2];sys.path.insert(0,str(FW/'scripts/release'))
import make_reviewed_profile as maker
class Tests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.temp=tempfile.TemporaryDirectory(prefix='matdog-cards-');cls.root=Path(cls.temp.name);unit=cls.root/'reader.cpp'
  unit.write_text('#include "'+str(FW/'src/execution/ValidationProfile.h')+'"\n#include <iostream>\n#include <string>\nint main(){matdog::execution::ValidationProfile p;std::string s;while(std::getline(std::cin,s))if(!p.card(s.c_str()))return 1;return p.sealed()?0:2;}\n')
  cls.binary=cls.root/'reader';subprocess.run(['g++','-std=c++17','-O2',str(unit),str(FW/'src/execution/ValidationProfile.cpp'),str(FW/'src/update/Sha256.cpp'),str(FW/'src/servo/ServoPopulation.cpp'),*map(str,sorted((FW/'src/motion').glob('*.cpp'))),'-o',str(cls.binary)],check=True)
 @classmethod
 def tearDownClass(cls):cls.temp.cleanup()
 def data(self,scope,clock=9):
  reviews={k:dict(approved=True,evidence_sha256='a'*64) for k in (*maker.REQUIRED,'external_bearing_support_observer','loaded_tracking','loaded_stability_contact_compliance')}
  return dict(generation=1,operator_name='SYNTHETIC_TEST_ONLY',review_utc='2026-10-13T12:00:00Z',current_record_sha256='d'*64,scope=scope,reviews=reviews,route='FIXTURE_LIMIT_COMPATIBLE' if scope in (3,4) else 'CANONICAL',corridor=[0 if scope in (3,4) else 114,251],execution=dict(velocity_rad_s=.065,acceleration_rad_s2=.3,tracking_rad=.0009,feedback_age_ms=40,batch_timeout_ms=2000,tracking_timeout_ms=2500,imu_age_ms=100,temperature_max_c=60,voltage_min_dv=50,voltage_max_dv=140,output_ceiling=500,transaction_timeout_ms=10,contact_inset_ticks=18),drive=dict(speed_raw=20,acceleration_raw=1,speed_rad_s_per_raw=.001533980787,acceleration_rad_s2_per_raw=.15),error_budget_rad=[.000766990394,.0003],max_joint_step_rad=.0015,joint_limits=[dict(reviewed=True,required_min_urad=-100000,required_max_urad=100000,margin_urad=1000) for _ in range(12)],stand_time_scale=9,measured_fixture_anchor_xyz_rpy=[0,0,.3,0,0,0],fixture_position_tube_m=.001,fixture_rotation_tube=.001,walk_time_scale=clock,walk_step_rad=.004)
 def parses(self,cards):return subprocess.run([str(self.binary)],input=cards,text=True).returncode==0
 def test_all_scopes_and_slow_clocks_roundtrip(self):
  for scope in (1,2,3,4):
   for clock in (9,100):
    cards=maker.create(self.data(scope,clock));self.assertTrue(self.parses(cards),(scope,clock))
    if scope==1:self.assertNotIn('DYNAMICS ',cards)
 def test_proof_quantization_budget_and_hash_mutations(self):
  cards=maker.create(self.data(4));self.assertFalse(self.parses(cards.replace('WALK 90 90 .005 .8 .04 8','WALK 90 90 .006 .8 .04 8')))
  self.assertFalse(self.parses('\n'.join((' '.join(line.split()[:-1])+ ' 0.006') if line.startswith('WALK ') else line for line in cards.splitlines())+'\n'))
  self.assertFalse(self.parses(cards.replace('CLOCK 9','CLOCK 8',1)))
  self.assertFalse(self.parses(cards.replace('RECORD '+'d'*64,'RECORD '+'e'*64)))
 def test_missing_physical_evidence_and_out_of_scope_fail(self):
  for key in ('physical_power_cut','12_joint_limits','sequential_bus_deadlines'):
   d=self.data(3);d['reviews'][key]['approved']=False
   with self.assertRaises(ValueError):maker.create(d)
  d=self.data(2);del d['reviews']['loaded_stability_contact_compliance']
  with self.assertRaises(ValueError):maker.create(d)
  d=self.data(4,8)
  with self.assertRaises(ValueError):maker.create(d)
if __name__=='__main__':unittest.main()

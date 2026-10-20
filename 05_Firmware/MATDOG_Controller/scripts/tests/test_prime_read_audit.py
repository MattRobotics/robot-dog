#!/usr/bin/env python3
"""Offline mutation proof of the detailed-parser audit. No device or build."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import static_audit as audit

class ReadAudit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.files=[(p,audit.strip_comments(p.read_text())) for p in audit.iter_source_files()]
    def check(self,name=None,old='',new=''):
        files=[];changed=False
        for p,code in self.files:
            if p.name==name:
                self.assertIn(old,code);code=code.replace(old,new,1);changed=True
            files.append((p,code))
        if name:self.assertTrue(changed)
        audit.failures.clear();audit.check_thermal_confirmation(files)
        return list(audit.failures)
    def test_reviewed_parser_and_whitespace(self):
        self.assertEqual(self.check(),[])
        self.assertEqual(self.check('ServoReadValidation.h','if(frame[2]!=id)','if ( frame [ 2 ] != id )'),[])
    def test_every_required_read_invariant_is_mutation_checked(self):
        changes=[
            ('ServoReadValidation.h','if(frame[2]!=id)return ReadValidity::ID;',''),
            ('ServoReadValidation.h','if(frame[3]!=width+2)return ReadValidity::LENGTH;',''),
            ('ServoReadValidation.h','if(frame[4]!=0)return ReadValidity::STATUS;',''),
            ('ServoReadValidation.h','if(frame[0]!=255||frame[1]!=255)return ReadValidity::HEADER;',''),
            ('ServoReadValidation.h','if(size!=width+6)return ReadValidity::SHORT_PACKET;',''),
            ('ServoReadValidation.h','return sum==255?ReadValidity::VALID:ReadValidity::CHECKSUM;','return ReadValidity::VALID;'),
            ('ServoReadValidation.h','using diagnostics::ReadValidity;','using diagnostics::ReadValidity; return ReadValidity::VALID;'),
            ('ServoReadValidation.h','static_cast<uint8_t>(sum + frame[i])','static_cast<uint8_t>(sum)'),
            ('ServoReadValidation.h','==diagnostics::ReadValidity::VALID','!=diagnostics::ReadValidity::VALID'),
            ('ValidatedServoRead.h','if (validity!=ReadValidity::VALID) return 0;',''),
            ('ValidatedServoRead.h','return ReadCaptured(id,address,data,width,nullptr);','return width;'),
            ('ValidatedServoRead.h','BusTransaction::kDrainMaxBytes','255'),
            ('ValidatedServoRead.h','if (!bus.mayWait()) { bus.waitFailed(); break; }',''),
            ('ValidatedServoRead.h','const bool tx_allowed = bus.allowTx();','const bool tx_allowed = true;'),
            ('ValidatedServoRead.h','uint8_t drained = 0;','return 10; uint8_t drained = 0;'),
            ('ValidatedServoRead.h','uint8_t drained = 0;','return width; uint8_t drained = 0;'),
            ('ValidatedServoRead.h','return Read(id, address, &data, 1) == 1 ? data : -1;','return data;'),
            ('ValidatedServoRead.h','return Read(id, address, data, 2) == 2 ? data[0] | (data[1] << 8) : -1;','return data[0];'),
        ]
        for name,old,new in changes:
            with self.subTest(file=name,mutation=old):self.assertTrue(self.check(name,old,new))

if __name__=='__main__':unittest.main(verbosity=2)

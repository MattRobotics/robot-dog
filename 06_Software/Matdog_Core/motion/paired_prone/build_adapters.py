#!/usr/bin/env python3
"""Build host-only G1/G2/G5/G4 adapters. No device transport is linked."""
import argparse,json,hashlib,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
MOTION=ROOT/'05_Firmware/MATDOG_Controller/src/motion'
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    common=['LegKinematics.cpp','LegInverseKinematics.cpp','FootContact.cpp','BodyPose.cpp']
    groups={'core_adapter':common+['TiltedContactIk.cpp'],'walk_adapter':common+['Gait.cpp','StandTrajectory.cpp']}
    receipt={}
    for name,sources in groups.items():
        inputs=[HERE/(name+'.cpp')]+[MOTION/s for s in sources]
        command=['g++','-std=c++17','-O2','-shared','-fPIC','-Wall','-Wextra','-Werror','-I',str(MOTION),*[str(f) for f in inputs],'-o',str(a.output/(name+'.so'))]
        subprocess.run(command,check=True)
        receipt[name]={'command':command,'source_sha256':{str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in inputs},'library_sha256':hashlib.sha256((a.output/(name+'.so')).read_bytes()).hexdigest()}
    (a.output/'adapter_build_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
if __name__=='__main__':main()

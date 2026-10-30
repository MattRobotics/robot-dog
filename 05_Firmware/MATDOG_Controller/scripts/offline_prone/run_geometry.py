#!/usr/bin/env python3
"""Reproduce offline trajectories with the existing C++ planner and FK/IK."""
from pathlib import Path
import subprocess,sys,json
here=Path(__file__).resolve().parent;fw=here.parents[1];out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=True)
names=['BodyPose','ContactMode','FootContact','Gait','GaitTiming','ImuAttitude','LegKinematics','LegInverseKinematics','StandTrajectory','TiltCompensation','TiltedContactIk','MotionContract']
for name,output in [('path_probe','path.csv'),('locked_contact_probe','locked_contact.log')]:
 subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',str(here/(name+'.cpp')),*[str(fw/'src/motion'/(n+'.cpp')) for n in names],'-o',str(out/name)],check=True)
 with (out/output).open('w') as f:subprocess.run([str(out/name)],stdout=f,check=True)
subprocess.run([sys.executable,str(here/'analyze_path.py'),str(out)],check=True)

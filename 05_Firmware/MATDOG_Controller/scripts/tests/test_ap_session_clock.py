#!/usr/bin/env python3
"""F02: compile the same acceptance test against HEAD or a local source ref."""
import argparse
from pathlib import Path
import subprocess
import tempfile
p=argparse.ArgumentParser()
p.add_argument('--source-ref')
a=p.parse_args()
here=Path(__file__).resolve().parent
root=here.parent.parent
src=root/'src'
with tempfile.TemporaryDirectory(prefix='matdog-f02-') as d:
    temp=Path(d)
    if a.source_ref:
        (temp/'network').mkdir()
        for name in ['PortalSecurity.h','PortalSecurity.cpp','NetworkConfig.h']:
            data=subprocess.check_output(['git','show',a.source_ref+':05_Firmware/MATDOG_Controller/src/network/'+name],cwd=root)
            (temp/'network'/name).write_bytes(data)
        (temp/'update').symlink_to(src/'update',target_is_directory=True)
        chosen=temp
    else:
        chosen=src
    exe=temp/'probe'
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-O1',
                    '-fsanitize=address,undefined','-fno-omit-frame-pointer',
                    '-I',str(chosen),str(here/'test_ap_session_clock.cpp'),
                    str(chosen/'network/PortalSecurity.cpp'),str(src/'network/NetworkConfig.cpp'),
                    str(src/'update/Sha256.cpp'),str(src/'update/Hmac256.cpp'),'-o',str(exe)],check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)

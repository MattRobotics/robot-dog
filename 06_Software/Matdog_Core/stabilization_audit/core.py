"""Host bridge to the production G5-A C++ units (content-addressed, atomically built shared library)."""
import ctypes as C
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MOTION = ROOT / '05_Firmware/MATDOG_Controller/src/motion'
OUT = ROOT / '09_Logs/Validation_Reports/G5A_Stabilization_Feasibility'
LEGS = ('lf', 'rf', 'rh', 'lh')
IK_STATUS = ['OK', 'INVALID_INPUT', 'NO_CONVERGENCE', 'SINGULAR', 'JOINT_LIMIT', 'CONTACT_MODE']
ATT_STATUS = ['OK', 'INVALID_POLICY', 'NONFINITE', 'BAD_QUATERNION_NORM', 'STALE', 'FUTURE_STAMP', 'TIME_REGRESSION', 'SEQUENCE_REGRESSION', 'ACCURACY_STATUS_LOW', 'ACCURACY_RAD_EXCEEDED', 'TILT_OUT_OF_RANGE']
STAB_STATE = ['DISABLED', 'ACTIVE', 'RAMPING_DOWN', 'FAULT']
VERDICT = ['REQUIRES_MEASUREMENT', 'EXCEEDS_LIMIT', 'PROVISIONALLY_SUPPORTED', 'VERIFIED']
PROVENANCE = {'UNMEASURED': 0, 'VENDOR_NOMINAL': 1, 'BENCH_NO_LOAD': 2, 'HARDWARE_LOADED_VERIFIED': 3}
P = C.POINTER(C.c_double)


def _ptr(a):
    return a.ctypes.data_as(P)


class Lib:
    def __init__(self):
        srcs = sorted(MOTION.glob('*.cpp')) + [HERE / 'bridge.cpp']
        srcs = [s for s in srcs if s.name not in ('Locomotion.cpp',) or True]
        digest = hashlib.sha256(b''.join(p.read_bytes() for p in srcs + sorted(MOTION.glob('*.h')))).hexdigest()[:20]
        path = Path(tempfile.gettempdir()) / ('matdog-g5a-' + digest + '.so')
        if not path.exists():
            part = path.with_suffix('.%d.tmp' % os.getpid())
            subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-fno-exceptions', '-fno-rtti', '-shared', '-fPIC', '-I', str(MOTION),
                            *map(str, srcs), '-o', str(part)], check=True)
            os.replace(part, path)
        self.so = C.CDLL(str(path))
        s = self.so
        s.g5a_tilt.argtypes = [C.c_double] * 4 + [P]; s.g5a_tilt.restype = C.c_int
        s.g5a_stand.argtypes = [P, P, P]
        s.g5a_compensate.argtypes = [C.c_double] * 3 + [P, C.c_int, P, P, P, C.POINTER(C.c_int)]; s.g5a_compensate.restype = C.c_int
        s.g5a_ik.argtypes = [C.c_int, P, P, P, C.c_int, P, P]; s.g5a_ik.restype = C.c_int
        s.g5a_loop_new.argtypes = [P, P, C.c_uint, C.POINTER(C.c_int)]; s.g5a_loop_new.restype = C.c_void_p
        s.g5a_loop_free.argtypes = [C.c_void_p]
        s.g5a_loop_enable.argtypes = [C.c_void_p, C.c_double]; s.g5a_loop_enable.restype = C.c_int
        s.g5a_loop_disable.argtypes = [C.c_void_p]; s.g5a_loop_reset.argtypes = [C.c_void_p]
        s.g5a_loop_step.argtypes = [C.c_void_p, P, C.c_double, P]; s.g5a_loop_step.restype = C.c_int
        s.g5a_stand_path.argtypes = [C.c_double, P, P, P]; s.g5a_stand_path.restype = C.c_int
        s.g5a_max_gain.argtypes = [C.c_double, C.c_uint]; s.g5a_max_gain.restype = C.c_double
        s.g5a_classify.argtypes = [C.c_double, C.c_double, C.c_int, C.c_int, C.c_double]; s.g5a_classify.restype = C.c_int

    def tilt(self, q):
        out = np.zeros(5)
        ok = self.so.g5a_tilt(*map(float, q), _ptr(out))
        return (out[0], out[1], out[2:5].copy()) if ok else None

    def stand(self):
        c, s, m = np.zeros(12), np.zeros(12), np.zeros(2)
        self.so.g5a_stand(_ptr(c), _ptr(s), _ptr(m))
        return c.reshape(4, 3), s.reshape(4, 3), float(m[0]), float(m[1])

    def compensate(self, roll, pitch, seeds=None, height=None, strip=True):
        contacts, standSeeds, h, _ = self.stand()
        seeds = standSeeds if seeds is None else np.asarray(seeds, dtype=np.float64).reshape(4, 3)
        j, c, m, info = np.zeros(12), np.zeros(12), np.zeros(4), (C.c_int * 2)()
        st = self.so.g5a_compensate(float(h if height is None else height), float(roll), float(pitch), _ptr(np.ascontiguousarray(seeds.ravel())), int(strip), _ptr(j), _ptr(c), _ptr(m), info)
        if st:
            return dict(status='INVALID_INPUT' if st == 1 else 'IK_FAILURE', failed_leg=int(info[0]), leg_status=IK_STATUS[info[1]])
        return dict(status='OK', joints=j.reshape(4, 3), contacts=c.reshape(4, 3), drift=m[0], margin=m[1], condition=m[2], iterations=int(m[3]))

    def ik(self, leg, target, normal, seed, strip=True):
        j, m = np.zeros(3), np.zeros(4)
        st = self.so.g5a_ik(leg, _ptr(np.asarray(target, dtype=np.float64)), _ptr(np.asarray(normal, dtype=np.float64)), _ptr(np.asarray(seed, dtype=np.float64)), int(strip), _ptr(j), _ptr(m))
        return IK_STATUS[st], j, m

    def stand_path(self, progress, seeds):
        j, h = np.zeros(12), np.zeros(1)
        st = self.so.g5a_stand_path(float(progress), _ptr(np.ascontiguousarray(np.asarray(seeds, dtype=np.float64).ravel())), _ptr(j), _ptr(h))
        return (st, None, None) if st else (0, j.reshape(4, 3), float(h[0]))

    def classify(self, required, limit, provenance, margin=1.0):
        return VERDICT[self.so.g5a_classify(required, 0.0 if limit is None else limit, PROVENANCE[provenance], 0 if limit is None else 1, margin)]


class Loop:
    """Monitor + stabilizer pair driven through the production code."""
    def __init__(self, lib, policy, cfg, delay_steps):
        ok = C.c_int(0)
        self.lib = lib
        self.h = lib.so.g5a_loop_new(_ptr(np.asarray(policy, dtype=np.float64)), _ptr(np.asarray(cfg, dtype=np.float64)), delay_steps, C.byref(ok))
        self.ok = bool(ok.value)

    def __del__(self):
        try:
            self.lib.so.g5a_loop_free(self.h)
        except Exception:
            pass

    def enable(self, now):
        return bool(self.lib.so.g5a_loop_enable(self.h, now))

    def disable(self):
        self.lib.so.g5a_loop_disable(self.h)

    def reset_fault(self):
        self.lib.so.g5a_loop_reset(self.h)

    def step(self, snapshot, now):
        out = np.zeros(5)
        st = self.lib.so.g5a_loop_step(self.h, _ptr(np.asarray(snapshot, dtype=np.float64)), now, _ptr(out))
        return dict(attitude=ATT_STATUS[st], roll_cmd=out[0], pitch_cmd=out[1], state=STAB_STATE[int(out[2])], saturated=bool(out[3]), roll_meas=out[4])

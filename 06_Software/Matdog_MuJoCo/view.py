#!/usr/bin/env python3
"""Look at the MATDOG twin: an open-loop physics replay of a sequence CSV
(fw_bridge/export_sequences output, a cosim.py CSV or a delivered mujoco_replay
CSV) with the nominal servo model.

  view.py <csv>                          interactive viewer, real time
  view.py <csv> --kinematic              joints imposed, no physics (M1 view)
  view.py <csv> --png <out> --at 0 2.5   offscreen snapshots (MUJOCO_GL=egl)
"""
import argparse
import pathlib
import struct
import time
import zlib

import mujoco
import mujoco.viewer
import numpy as np

import dynamics
import servo
import twin


def load(path):
    header = pathlib.Path(path).read_text().split("\n", 1)[0]
    if "raw0" in header:
        return twin.sequence(path)
    seq = twin.delivered(path)
    n = len(seq["q"])
    for key, value in (("state", "WALKING"), ("stop_active", "0")):
        seq.setdefault(key, np.full(n, value))
    return seq


def write_png(path, rgb):
    """RGB uint8 image to PNG with the standard library only."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    pathlib.Path(path).write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) +
                                   chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def camera(viewer_cam, data):
    viewer_cam.lookat[:] = data.qpos[0:3]
    viewer_cam.distance, viewer_cam.azimuth, viewer_cam.elevation = 0.75, 135, -20


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--kinematic", action="store_true")
    ap.add_argument("--png")
    ap.add_argument("--at", type=float, nargs="*", default=[0.0])
    args = ap.parse_args()
    seq = load(args.csv)
    tw = dynamics.Twin(servo.Params())
    speed = seq["drive_speed_step_s"].astype(int)
    acc = seq["drive_acc_100step_s2"].astype(int)
    tw.begin(seq["q"][0], speed[0], acc[0])

    def frame(k):
        if args.kinematic:
            tw.d.qpos[7:19] = seq["q"][k]
            mujoco.mj_forward(tw.m, tw.d)
        else:
            tw.release(seq["q"][k], speed[k], acc[k])

    if args.png:
        renderer = mujoco.Renderer(tw.m, 720, 1280)
        cam = mujoco.MjvCamera()
        targets = sorted(args.at)
        for k in range(len(seq["q"])):
            frame(k)
            while targets and seq["t"][k] >= targets[0] - 1e-9:
                camera(cam, tw.d)
                renderer.update_scene(tw.d, cam)
                out = pathlib.Path(args.png.replace(".png", f"_t{targets[0]:.2f}.png"))
                write_png(out, renderer.render())
                print(out)
                targets.pop(0)
        return

    with mujoco.viewer.launch_passive(tw.m, tw.d) as viewer:
        camera(viewer.cam, tw.d)
        for k in range(len(seq["q"])):
            started = time.time()
            frame(k)
            viewer.sync()
            if not viewer.is_running():
                break
            time.sleep(max(0.0, twin.RELEASE_S - (time.time() - started)))
        while viewer.is_running():
            time.sleep(0.05)


if __name__ == "__main__":
    main()

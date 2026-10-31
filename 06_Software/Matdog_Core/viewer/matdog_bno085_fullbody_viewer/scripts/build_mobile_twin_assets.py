#!/usr/bin/env python3
"""MATDOG Android direct twin: decimated, quantized collision meshes (MTW1).

READ-ONLY on the canonical model: reads the staged collision STLs (or the
canonical folder) and writes a compact binary mesh pack plus a size report to
an OUTPUT directory you name. Nothing here touches firmware, flash or hardware.

Decimation: uniform vertex clustering on a grid of `cell_mm` (one vertex per
occupied cell = mean of its members; degenerate triangles dropped). Visual
display geometry only: NEVER a collision/clearance model (the canonical STL
and the offline FCL certificates remain the only geometry evidence).

MTW1 layout (little endian):
  magic 'MTW1', u16 mesh_count, then per mesh:
    u8 name_len, name bytes (link name), f32 min[3], f32 scale[3],
    u32 vertex_count, u32 index_count,
    u16 quantized xyz * vertex_count  (q = round((p-min)/scale), 0..65535)
    u16 or u32 indices (u16 when vertex_count <= 65535)
Positions are STL millimetres, like the existing STL loader.
"""
import argparse, gzip, hashlib, json, struct, sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

def read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) >= 84:
        n = struct.unpack_from('<I', data, 80)[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n, offset=84)
            return rec['v'].astype(np.float64)
    tris, cur = [], []
    for line in data.decode('ascii', 'replace').splitlines():
        t = line.strip().split()
        if t and t[0] == 'vertex':
            cur.append([float(x) for x in t[1:4]])
            if len(cur) == 3: tris.append(cur); cur = []
    return np.array(tris, dtype=np.float64)

def cluster(tris: np.ndarray, cell: float):
    verts = tris.reshape(-1, 3)
    if cell <= 0:
        uniq, inv = np.unique(verts.round(6), axis=0, return_inverse=True)
        return uniq, inv.reshape(-1, 3)
    keys = np.floor(verts / cell).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(uniq), 3)); np.add.at(sums, inv, verts)
    counts = np.bincount(inv, minlength=len(uniq))[:, None]
    pos = sums / counts
    idx = inv.reshape(-1, 3)
    keep = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])
    idx = idx[keep]
    # drop duplicate faces regardless of winding duplicates
    if len(idx):
        _, first = np.unique(np.sort(idx, axis=1), axis=0, return_index=True)
        idx = idx[np.sort(first)]
    used = np.unique(idx)
    remap = -np.ones(len(pos), dtype=np.int64); remap[used] = np.arange(len(used))
    return pos[used], remap[idx]

def encode(name: str, pos: np.ndarray, idx: np.ndarray) -> bytes:
    lo = pos.min(axis=0) if len(pos) else np.zeros(3)
    span = (pos.max(axis=0) - lo) if len(pos) else np.ones(3)
    scale = np.where(span > 0, span / 65535.0, 1.0)
    q = np.clip(np.round((pos - lo) / scale), 0, 65535).astype('<u2')
    b = name.encode()
    out = struct.pack('<B', len(b)) + b + struct.pack('<3f3fII', *lo, *scale, len(pos), idx.size)
    out += q.tobytes()
    out += idx.astype('<u2' if len(pos) <= 65535 else '<u4').tobytes()
    return out

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--cells', default='0,1,2,3,4,6', help='comma list of grid cells in mm (0 = lossless dedupe)')
    args = ap.parse_args()
    root = ET.parse(args.urdf).getroot()
    base = args.urdf.parent
    links = []
    for link in root.findall('link'):
        mesh = link.find('collision/geometry/mesh')
        if mesh is not None:
            fn = mesh.get('filename').split('package://', 1)[-1]
            fn = fn.split('/', 1)[1] if '://' in mesh.get('filename') else fn
            links.append((link.get('name'), base / fn))
    args.out.mkdir(parents=True, exist_ok=True)
    source = {name: read_stl(path) for name, path in links}
    report = {'schema': 'MATDOG_MOBILE_TWIN_BUDGET_V1', 'urdf_sha256': hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
              'links': len(links), 'stl_bytes': sum(p.stat().st_size for _, p in links),
              'stl_triangles': int(sum(len(t) for t in source.values())), 'variants': []}
    for cell in [float(c) for c in args.cells.split(',')]:
        blob = b'MTW1' + struct.pack('<H', len(links)); tris = 0; worst = 0.0; per = {}
        for name, _ in links:
            pos, idx = cluster(source[name], cell)
            tris += len(idx); per[name] = int(len(idx))
            blob += encode(name, pos, idx)
            worst = max(worst, cell * np.sqrt(3) / 2 if cell > 0 else 0.0)
        gz = gzip.compress(blob, 9, mtime=0)
        tag = 'lossless' if cell == 0 else f'{cell:g}mm'
        (args.out / f'matdog_collision_{tag}.mtw').write_bytes(blob)
        (args.out / f'matdog_collision_{tag}.mtw.gz').write_bytes(gz)
        report['variants'].append({'cell_mm': cell, 'triangles': tris, 'raw_bytes': len(blob), 'gzip_bytes': len(gz),
                                   'max_vertex_shift_mm': round(worst, 3), 'gzip_sha256': hashlib.sha256(gz).hexdigest(),
                                   'per_link_triangles': per})
    (args.out / 'MOBILE_TWIN_BUDGET.json').write_text(json.dumps(report, indent=2) + '\n')
    for v in report['variants']:
        print(f"cell={v['cell_mm']:>4} mm  tris={v['triangles']:>7}  raw={v['raw_bytes']:>9}  gz={v['gzip_bytes']:>8}  shift<= {v['max_vertex_shift_mm']} mm")
    print(f"STL: {report['stl_bytes']} bytes, {report['stl_triangles']} triangles, {report['links']} links")
    return 0

if __name__ == '__main__':
    sys.exit(main())

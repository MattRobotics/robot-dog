#!/usr/bin/env python3
"""Structural OpenXML / formula / chart / table integrity check of a workbook.

Usage: python3 -I check_workbook_openxml.py <file.xlsx> [--evaluator <path-to-xl.py>]
Not a substitute for opening the file in Excel (never performed here).
"""
import sys, re, zipfile, importlib.util, posixpath
from lxml import etree

M = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
R = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
PR = "http://schemas.openxmlformats.org/package/2006/relationships"
CT = "http://schemas.openxmlformats.org/package/2006/content-types"
problems, notes = [], []
def bad(m): problems.append(m)
def X(t): return "{%s}%s" % (M, t)

def colnum(c):
    n = 0
    for ch in c: n = n * 26 + ord(ch) - 64
    return n

def main(path, evaluator=None):
    z = zipfile.ZipFile(path)
    if z.testzip(): bad("ZIP CRC failure")
    names = [i.filename for i in z.infolist()]
    if len(names) != len(set(names)): bad("duplicate part names")
    parts = {}
    for n in names:
        try: parts[n] = etree.fromstring(z.read(n))
        except Exception as e: bad(f"{n}: not well-formed: {e}")
    notes.append(f"{len(names)} parts, all well-formed" if not problems else "")
    # content types
    ct = parts["[Content_Types].xml"]
    defaults = {d.get("Extension"): d.get("ContentType") for d in ct.findall("{%s}Default" % CT)}
    overrides = {o.get("PartName"): o.get("ContentType") for o in ct.findall("{%s}Override" % CT)}
    for o in overrides:
        if o.lstrip("/") not in names: bad(f"content-type override for missing part {o}")
    for n in names:
        if n == "[Content_Types].xml": continue
        ext = posixpath.basename(n).rsplit(".", 1)[-1]
        if "/" + n not in overrides and ext not in defaults:
            bad(f"part without content type: {n}")
    # relationships
    def resolve(base, tgt):
        if tgt.startswith("/"): return tgt.lstrip("/")
        return posixpath.normpath(posixpath.join(posixpath.dirname(base), tgt))
    relsets = {}
    for n in names:
        if n.endswith(".rels"):
            src = n.replace("_rels/", "", 1)[:-5]
            src = posixpath.normpath(src)
            ids = {}
            for r in parts[n]:
                if r.get("TargetMode") == "External": continue
                tgt = resolve(src if src != "." else "_rels/.rels", r.get("Target"))
                if n == "_rels/.rels": tgt = r.get("Target").lstrip("/")
                if tgt not in names: bad(f"{n}: relationship {r.get('Id')} -> missing {tgt}")
                if r.get("Id") in ids: bad(f"{n}: duplicate Id {r.get('Id')}")
                ids[r.get("Id")] = tgt
            relsets[src] = ids
    # workbook
    wb = parts["xl/workbook.xml"]
    sheets = [(s.get("name"), s.get("sheetId"), s.get("{%s}id" % R)) for s in wb.find(X("sheets"))]
    if len({s[0].lower() for s in sheets}) != len(sheets): bad("duplicate sheet names")
    if len({s[1] for s in sheets}) != len(sheets): bad("duplicate sheetId")
    wbrels = relsets["xl/workbook.xml"]
    sheetparts = {}
    for name, sid, rid in sheets:
        if len(name) > 31 or re.search(r"[\[\]:*?/\\]", name): bad(f"invalid sheet name {name}")
        if rid not in wbrels: bad(f"{name}: dangling r:id"); continue
        sheetparts[name] = wbrels[rid]
    notes.append(f"{len(sheets)} sheets: " + ", ".join(s[0] for s in sheets))
    for dn in wb.findall(".//" + X("definedName")):
        bad(f"definedName present (check manually): {dn.get('name')}")
    # styles
    st = parts["xl/styles.xml"]
    nxf = len(st.find(X("cellXfs")))
    sst = parts.get("xl/sharedStrings.xml"); nsst = 0 if sst is None else len(sst)
    # sheets
    nform = nvals = 0
    for name, part in sheetparts.items():
        t = parts[part]
        last = 0; seen = set()
        for row in t.find(X("sheetData")):
            r = int(row.get("r"))
            if r <= last: bad(f"{name}: rows out of order at {r}")
            last = r; lc = 0
            for c in row:
                m = re.fullmatch(r"([A-Z]+)(\d+)", c.get("r"))
                if not m or int(m.group(2)) != r: bad(f"{name}!{c.get('r')}: bad ref in row {r}"); continue
                cn = colnum(m.group(1))
                if cn <= lc: bad(f"{name}!{c.get('r')}: cells out of order")
                lc = cn
                if c.get("s") and int(c.get("s")) >= nxf: bad(f"{name}!{c.get('r')}: style index out of range")
                if c.get("t") == "s" and int(c.find(X("v")).text) >= nsst: bad(f"{name}!{c.get('r')}: shared string out of range")
                v = c.find(X("v")); f = c.find(X("f"))
                if f is not None:
                    nform += 1
                    if v is None and c.get("t") != "str": bad(f"{name}!{c.get('r')}: formula without cached value")
                if v is not None and c.get("t") == "e": bad(f"{name}!{c.get('r')}: error value {v.text}")
                if v is not None and v.text in ("#REF!", "#NAME?", "#VALUE!", "#DIV/0!", "#N/A"): bad(f"{name}!{c.get('r')}: error text")
                if f is not None and f.text and "#REF!" in f.text: bad(f"{name}!{c.get('r')}: #REF! in formula")
                if f is not None and f.text:
                    for sn in re.findall(r"'([^']+)'!", f.text) + re.findall(r"(?<![\w'])([A-Za-z_][\w.]*)!", f.text):
                        if sn not in sheetparts: bad(f"{name}!{c.get('r')}: formula references missing sheet {sn}")
                nvals += 1
        mc = t.find(X("mergeCells"))
        if mc is not None:
            rects = []
            for m in mc:
                a, b = m.get("ref").split(":")
                ma = re.fullmatch(r"([A-Z]+)(\d+)", a); mb = re.fullmatch(r"([A-Z]+)(\d+)", b)
                rects.append((colnum(ma[1]), int(ma[2]), colnum(mb[1]), int(mb[2]), m.get("ref")))
            for i in range(len(rects)):
                for j in range(i + 1, len(rects)):
                    p, q = rects[i], rects[j]
                    if not (p[2] < q[0] or q[2] < p[0] or p[3] < q[1] or q[3] < p[1]):
                        bad(f"{name}: overlapping merges {p[4]} / {q[4]}")
        # tables / drawings referenced from sheet
        rl = relsets.get(part, {})
        for tp in t.findall(".//" + X("tablePart")):
            if tp.get("{%s}id" % R) not in rl: bad(f"{name}: dangling tablePart")
        dr = t.find(X("drawing"))
        if dr is not None and dr.get("{%s}id" % R) not in rl: bad(f"{name}: dangling drawing r:id")
    notes.append(f"{nform} formulas, all with cached values" if not any("cached" in p for p in problems) else "")
    # tables: header cells vs table columns
    for tn in [n for n in names if n.startswith("xl/tables/")]:
        tb = parts[tn]
        owner = next((p for p, rs in relsets.items() if tn in rs.values()), None)
        sname = next((s for s, p in sheetparts.items() if p == owner), None)
        if sname is None: bad(f"{tn}: no owning sheet"); continue
        a, b = tb.get("ref").split(":"); ma = re.fullmatch(r"([A-Z]+)(\d+)", a); mb = re.fullmatch(r"([A-Z]+)(\d+)", b)
        cols = [c.get("name") for c in tb.find("{%s}tableColumns" % M)]
        if len(cols) != colnum(mb[1]) - colnum(ma[1]) + 1: bad(f"{tn}: column count != ref width")
        if len(set(cols)) != len(cols): bad(f"{tn}: duplicate column names")
        hdr = int(ma[2]); got = {}
        for row in parts[sheetparts[sname]].find(X("sheetData")):
            if int(row.get("r")) == hdr:
                for c in row:
                    v = c.find(X("v")); isx = c.find(X("is"))
                    txt = "".join(isx.itertext()) if isx is not None else (v.text if v is not None else None)
                    if c.get("t") == "s" and v is not None: txt = "".join(sst[int(v.text)].itertext())
                    got[colnum(re.match(r"[A-Z]+", c.get("r"))[0])] = txt
        for i, cn in enumerate(cols):
            h = got.get(colnum(ma[1]) + i)
            if h != cn: bad(f"{tn}: header cell {i} {h!r} != column {cn!r}")
        notes.append(f"table {tb.get('name')} on {sname}: {len(cols)} columns match header row {hdr}")
    # charts
    for cn in [n for n in names if n.startswith("xl/drawings/charts/")]:
        ch = parts[cn]
        for f in ch.iter("{http://schemas.openxmlformats.org/drawingml/2006/chart}f"):
            m = re.fullmatch(r"'?([^'!]+)'?!\$([A-Z]+)\$(\d+):\$([A-Z]+)\$(\d+)", f.text)
            if not m or m[1] not in sheetparts: bad(f"{cn}: bad series ref {f.text}"); continue
            notes.append(f"chart {cn}: {f.text}")
            t = parts[sheetparts[m[1]]]; cnt = 0
            for row in t.find(X("sheetData")):
                if int(m[3]) <= int(row.get("r")) <= int(m[5]):
                    for c in row:
                        if m[2] == re.match(r"[A-Z]+", c.get("r"))[0] and c.find(X("v")) is not None: cnt += 1
            if cnt != int(m[5]) - int(m[3]) + 1: bad(f"{cn}: {f.text} has {cnt} populated cells of {int(m[5])-int(m[3])+1}")
        ax = [a.get("val") for a in ch.iter("{http://schemas.openxmlformats.org/drawingml/2006/chart}axId")]
        if len(ax) != 4 or len(set(ax)) != 2: bad(f"{cn}: axis ids inconsistent {ax}")
    # recalculation with the in-house evaluator
    if evaluator:
        spec = importlib.util.spec_from_file_location("xl", evaluator); xl = importlib.util.module_from_spec(spec); spec.loader.exec_module(xl)
        book = xl.Book(path)
        try:
            formula, changed = book.recalc()
            notes.append(f"evaluator: {len(formula)} formulas recomputed, {len(changed)} differ from cached values")
            for ch_ in changed[:20]: bad(f"cached != recomputed: {ch_}")
        except Exception as e:
            bad(f"evaluator could not process the workbook: {e!r}")
    for n in notes:
        if n: print("  ok:", n)
    for p in problems: print("  PROBLEM:", p)
    print("RESULT:", "PASS" if not problems else f"FAIL ({len(problems)} problems)")
    return 0 if not problems else 1

if __name__ == "__main__":
    ev = sys.argv[sys.argv.index("--evaluator") + 1] if "--evaluator" in sys.argv else None
    sys.exit(main(sys.argv[1], ev))

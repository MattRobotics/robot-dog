"""Minimal OOXML workbook editor + formula evaluator (no third-party dependency).
Edits sheet XML in place so styles, merges, tables and formulas of untouched cells are preserved byte-for-byte."""
import re, math, zipfile, copy, io
from lxml import etree
NS_X="http://schemas.openxmlformats.org/spreadsheetml/2006/main"
NS_R="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
NS_REL="http://schemas.openxmlformats.org/package/2006/relationships"
X=lambda t:"{%s}%s"%(NS_X,t)
def col2n(c):
    n=0
    for ch in c: n=n*26+ord(ch)-64
    return n
def n2col(n):
    s=""
    while n: n,r=divmod(n-1,26); s=chr(65+r)+s
    return s
def split_ref(ref):
    m=re.fullmatch(r"\$?([A-Z]+)\$?(\d+)",ref); return col2n(m.group(1)),int(m.group(2))
class Sheet:
    def __init__(self,book,name,part,tree):
        self.book,self.name,self.part,self.tree=book,name,part,tree
        self.data=tree.find(X("sheetData"))
    def rows(self): return {int(r.get("r")):r for r in self.data}
    def row(self,n,create=True):
        for r in self.data:
            if int(r.get("r"))==n: return r
            if int(r.get("r"))>n:
                if not create: return None
                new=etree.Element(X("row"),r=str(n)); r.addprevious(new); return new
        if not create: return None
        new=etree.SubElement(self.data,X("row"),r=str(n)); return new
    def cell(self,ref,create=True):
        c,r=split_ref(ref); row=self.row(r,create)
        if row is None: return None
        for e in row:
            ec,_=split_ref(e.get("r"))
            if ec==c: return e
            if ec>c:
                if not create: return None
                new=etree.Element(X("c"),r=ref); e.addprevious(new); return new
        if not create: return None
        return etree.SubElement(row,X("c"),r=ref)
    def _clear(self,c):
        for ch in list(c): c.remove(ch)
        if "t" in c.attrib: del c.attrib["t"]
    def set_text(self,ref,text,style=None):
        c=self.cell(ref); self._clear(c); c.set("t","str")
        if style is not None: c.set("s",str(style))
        etree.SubElement(c,X("v")).text=text
    def set_num(self,ref,val,style=None):
        c=self.cell(ref); self._clear(c)
        if style is not None: c.set("s",str(style))
        etree.SubElement(c,X("v")).text=repr(float(val)) if not float(val).is_integer() else str(int(val))
    def set_formula(self,ref,formula,style=None):
        c=self.cell(ref); self._clear(c)
        if style is not None: c.set("s",str(style))
        etree.SubElement(c,X("f")).text=formula.lstrip("=")
        etree.SubElement(c,X("v")).text="0"
    def style_of(self,ref):
        c=self.cell(ref,create=False); return None if c is None else c.get("s")
    def cells(self):
        for row in self.data:
            for c in row: yield c
class Book:
    def __init__(self,path):
        z=zipfile.ZipFile(path); self.order=[i.filename for i in z.infolist()]
        self.parts={n:z.read(n) for n in self.order}
        self.wb=etree.fromstring(self.parts["xl/workbook.xml"])
        self.rels=etree.fromstring(self.parts["xl/_rels/workbook.xml.rels"])
        self.ct=etree.fromstring(self.parts["[Content_Types].xml"])
        self.relmap={r.get("Id"):r.get("Target") for r in self.rels}
        self.sheets={}; self.sheet_order=[]
        for s in self.wb.find(X("sheets")):
            tgt=self.relmap[s.get("{%s}id"%NS_R)].lstrip("/")
            part=tgt if tgt.startswith("xl/") else "xl/"+tgt
            self.sheets[s.get("name")]=Sheet(self,s.get("name"),part,etree.fromstring(self.parts[part]))
            self.sheet_order.append(s.get("name"))
        self.sst=[]
        if "xl/sharedStrings.xml" in self.parts:
            t=etree.fromstring(self.parts["xl/sharedStrings.xml"])
            self.sst=["".join(si.itertext()) for si in t.findall(X("si"))]
    def __getitem__(self,n): return self.sheets[n]
    def add_sheet(self,name,tree):
        idx=max(int(re.search(r"sheet(\d+)\.xml",p).group(1)) for p in set(self.parts)|set(self.order) if re.match(r"xl/worksheets/sheet\d+\.xml",p))+1
        part=f"xl/worksheets/sheet{idx}.xml"; rid=f"Rnew{idx}"
        etree.SubElement(self.rels,"{%s}Relationship"%NS_REL,Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet",Target="/"+part,Id=rid)
        sid=max(int(s.get("sheetId")) for s in self.wb.find(X("sheets")))+1
        el=etree.SubElement(self.wb.find(X("sheets")),X("sheet"),name=name,sheetId=str(sid)); el.set("{%s}id"%NS_R,rid)
        ov=etree.SubElement(self.ct,"{http://schemas.openxmlformats.org/package/2006/content-types}Override",PartName="/"+part,ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml")
        self.order.append(part); self.parts[part]=b""; self.sheets[name]=Sheet(self,name,part,tree); self.sheet_order.append(name); return self.sheets[name]
    def copy_sheet(self,src,new):
        t=copy.deepcopy(self.sheets[src].tree)
        for tp in t.findall(X("tableParts")): t.remove(tp)         # tables are not duplicated
        return self.add_sheet(new,t)
    def save(self,path):
        out=zipfile.ZipFile(path,"w",zipfile.ZIP_DEFLATED)
        self.parts["xl/workbook.xml"]=etree.tostring(self.wb,xml_declaration=True,encoding="utf-8")
        self.parts["xl/_rels/workbook.xml.rels"]=etree.tostring(self.rels,xml_declaration=True,encoding="utf-8")
        self.parts["[Content_Types].xml"]=etree.tostring(self.ct,xml_declaration=True,encoding="utf-8")
        for s in self.sheets.values(): self.parts[s.part]=etree.tostring(s.tree,xml_declaration=True,encoding="utf-8")
        for n in self.order: out.writestr(n,self.parts[n])
        out.close()
    # ---------------------------------------------------------------- evaluation
    def raw(self,sheet,ref):
        c=self.sheets[sheet].cell(ref,create=False)
        if c is None: return None
        v=c.find(X("v"))
        if c.find(X("is")) is not None: return "".join(c.find(X("is")).itertext())
        if v is None: return None
        t=c.get("t")
        if t=="s": return self.sst[int(v.text)]
        if t in ("str","inlineStr"): return v.text or ""
        if t=="b": return bool(int(v.text))
        try: return float(v.text)
        except: return v.text
    def formulas(self):
        """yield (sheet, ref, formula-text) with shared formulas expanded."""
        for sn in self.sheet_order:
            masters={}
            for c in self.sheets[sn].cells():
                f=c.find(X("f"))
                if f is None: continue
                if f.get("t")=="shared":
                    si=f.get("si")
                    if f.text: masters[si]=(c.get("r"),f.text); txt=f.text
                    else:
                        mref,mtxt=masters[si]; mc,mr=split_ref(mref); cc,cr=split_ref(c.get("r")); txt=shift(mtxt,cc-mc,cr-mr)
                else: txt=f.text
                yield sn,c.get("r"),txt,c
    def recalc(self):
        cache={}; formula={}
        for sn,ref,txt,c in self.formulas(): formula[(sn,ref)]=(txt,c)
        def val(sn,ref):
            key=(sn,ref)
            if key in cache: return cache[key]
            if key in formula:
                cache[key]=0.0; cache[key]=Evaluator(self,val,sn).eval(formula[key][0])
            else: cache[key]=self.raw(sn,ref)
            return cache[key]
        changed=[]
        for (sn,ref),(txt,c) in formula.items():
            v=val(sn,ref); old=self.raw(sn,ref)
            same=(old==v) or (old in (None,"") and v=="") or (isinstance(old,float) and isinstance(v,(int,float)) and not isinstance(v,bool) and abs(old-v)<=1e-9*max(1,abs(old)))
            if same: continue
            vv=c.find(X("v"))
            if vv is None: vv=etree.SubElement(c,X("v"))
            if isinstance(v,str): c.set("t","str"); vv.text=v
            elif isinstance(v,bool): c.set("t","b"); vv.text="1" if v else "0"
            else:
                if "t" in c.attrib: del c.attrib["t"]
                vv.text=repr(float(v)) if not float(v).is_integer() else str(int(v))
            changed.append((sn,ref,old,v))
        return formula,changed
def shift(txt,dc,dr):
    def rep(m):
        cabs,col,rabs,row=m.group(1),m.group(2),m.group(3),m.group(4)
        c=col if cabs else n2col(col2n(col)+dc); r=row if rabs else str(int(row)+dr)
        return f"{cabs}{c}{rabs}{r}"
    return re.sub(r"(?<![A-Za-z_!])(\$?)([A-Z]{1,3})(\$?)(\d+)(?![\w(])",rep,txt)
class Evaluator:
    TOK=re.compile(r"\s*(?:(?P<num>\d+\.?\d*(?:[eE][+-]?\d+)?)|(?P<str>\"[^\"]*\")|(?P<ref>(?:'[^']+'|[A-Za-z_][\w.]*)?!?\$?[A-Z]{1,3}\$?\d+(?::\$?[A-Z]{1,3}\$?\d+)?)|(?P<fn>[A-Z][A-Z0-9.]*)\(|(?P<op><>|<=|>=|[-+*/^(),=<>&]))")
    def __init__(self,book,val,sheet): self.book,self.val,self.sheet=book,val,sheet
    def eval(self,txt):
        self.toks=[]; pos=0; txt=txt.strip()
        while pos<len(txt):
            m=self.TOK.match(txt,pos)
            if not m or m.end()==pos: raise ValueError("cannot tokenize %r at %d"%(txt,pos))
            k=m.lastgroup; self.toks.append((k,m.group(k))); pos=m.end()
        self.i=0; v=self.cmp(); assert self.i==len(self.toks),(txt,self.toks[self.i:]); return v
    def peek(self): return self.toks[self.i] if self.i<len(self.toks) else (None,None)
    def take(self): t=self.toks[self.i]; self.i+=1; return t
    def cmp(self):
        a=self.add()
        while self.peek()[0]=="op" and self.peek()[1] in ("=","<>","<",">","<=",">="):
            op=self.take()[1]; b=self.add()
            if isinstance(a,str)!=isinstance(b,str): a=str(a if a is not None else "") ; b=str(b if b is not None else "")
            a={"=":a==b,"<>":a!=b,"<":a<b,">":a>b,"<=":a<=b,">=":a>=b}[op]
        return a
    def add(self):
        a=self.mul()
        while self.peek()[0]=="op" and self.peek()[1] in "+-" and self.peek()[1]:
            op=self.take()[1]; b=self.mul(); a=num(a)+num(b) if op=="+" else num(a)-num(b)
        return a
    def mul(self):
        a=self.pw()
        while self.peek()[0]=="op" and self.peek()[1] in ("*","/"):
            op=self.take()[1]; b=self.pw(); a=num(a)*num(b) if op=="*" else num(a)/num(b)
        return a
    def pw(self):
        a=self.un()
        while self.peek()==("op","^"): self.take(); a=num(a)**num(self.un())
        return a
    def un(self):
        if self.peek()==("op","-"): self.take(); return -num(self.un())
        if self.peek()==("op","+"): self.take(); return num(self.un())
        return self.atom()
    def atom(self):
        k,v=self.take()
        if k=="num": return float(v)
        if k=="str": return v[1:-1]
        if k=="ref": return self.ref(v)
        if k=="fn":
            args=[]
            if self.peek()!=("op",")"):
                args.append(self.arg())
                while self.peek()==("op",","): self.take(); args.append(self.arg())
            assert self.take()==("op",")")
            return self.fn(v,args)
        if (k,v)==("op","("):
            r=self.cmp(); assert self.take()==("op",")"); return r
        raise ValueError((k,v))
    def arg(self): return self.cmp()
    def ref(self,r):
        sheet=self.sheet
        if "!" in r: sheet,r=r.rsplit("!",1); sheet=sheet.strip("'")
        if ":" in r:
            a,b=r.split(":"); ac,ar=split_ref(a); bc,br=split_ref(b)
            return [self.val(sheet,f"{n2col(c)}{rr}") for rr in range(ar,br+1) for c in range(ac,bc+1)]
        return self.val(sheet,r.replace("$",""))
    def fn(self,name,a):
        flat=lambda xs:[x for y in xs for x in (y if isinstance(y,list) else [y])]
        if name=="SUM": return sum(x for x in flat(a) if isinstance(x,(int,float)) and not isinstance(x,bool))
        if name=="ABS": return abs(num(a[0]))
        if name=="PI": return math.pi
        if name=="IF": return a[1] if a[0] else (a[2] if len(a)>2 else False)
        if name=="MAX": return max(x for x in flat(a) if isinstance(x,(int,float)))
        if name=="MIN": return min(x for x in flat(a) if isinstance(x,(int,float)))
        if name=="ROUND": return round(num(a[0]),int(num(a[1])))
        raise ValueError("unsupported function "+name)
def num(x):
    if x is None or x=="": return 0.0
    if isinstance(x,bool): return float(x)
    return float(x)

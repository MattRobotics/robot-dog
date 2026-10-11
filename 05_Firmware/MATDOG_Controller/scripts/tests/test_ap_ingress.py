#!/usr/bin/env python3
"""Historical AP+STA F01 gap detector pinned to b7516c39, NOT a simulation/proof of actual interface ingress.

Labels describe the required policy. Production sees only socket endpoints and
headers, not these labels. An actual stack-level regression is required before
closing F01. Exit 1 deliberately keeps that finding BLOCKED on this architecture.
No network/device APIs are called; generated C++ exists only in a host temp dir.
"""
import ast
from pathlib import Path
import subprocess
import tempfile
here=Path(__file__).resolve().parent
src=here.parent.parent/'src'
tree=ast.parse((here/'test_ap_transport.py').read_text())
fixture=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign)
             and any(isinstance(t,ast.Name) and t.id=='fixture' for t in n.targets))
fixture=fixture[:fixture.index('int main(int argc,char**)')]+r'''
int main() {
 unsigned gaps=0, positives=0;
 struct Scenario { const char *label; uint32_t peer; bool allowed; };
 for(bool mapped:{false,true}) for(const auto &s:{
     Scenario{"AP legitimate",0xc0a80402u,true},
     Scenario{"STA to AP destination",0xc0a80132u,false},
     Scenario{"STA with spoofed AP-subnet peer",0xc0a80402u,false},
     Scenario{"unattestable ingress",0xc0a80402u,false}}) {
   endpoints(mapped,0xc0a80401u,s.peer);
   HttpTransport service;provision(service);
   auto login=request(service,"/wifi/login","token="+token);run(login);
   auto action=request(service,"/wifi/scan","");
   action.headers["Cookie"]="md_session="+std::string(service.portal_.cookie());
   action.headers["X-Csrf-Token"]=service.portal_.csrf();run(action);
   const bool accepted=login.status==200 && action.status==200 && service.service.mutations==1;
   if(s.allowed && accepted) ++positives;
   if(!s.allowed && accepted) ++gaps;
   printf("mapped=%d scenario=%s accepted=%d policy_allows=%d\n",mapped,s.label,accepted,s.allowed);
 }
 printf("F01_UNATTESTED_INGRESS: legitimate=%u/2 unresolved_negative_controls=%u/6\n",positives,gaps);
 return positives==2 && gaps==0?0:1;
}
'''
def section(text,start,end):
    assert text.count(start)==1 and text.count(end)==1
    return text.split(start,1)[1].split(end,1)[0]
baseline='b7516c39c9fcba09992acd07377facd5ae6e8588'
c,h=[subprocess.check_output(['git','show',baseline+':05_Firmware/MATDOG_Controller/src/network/'+name],cwd=src,text=True) for name in ['HttpTransport.cpp','HttpTransport.h']]
helpers=section(c,'\nbool readHexHeader','\nvoid jsonString')
helpers=helpers[helpers.index('\n}')+2:]
unit=fixture.replace('// PRODUCTION_TYPES','enum class HttpRequestKind'+section(h,'enum class HttpRequestKind','// Two optional listeners'))
unit=unit.replace('// PRODUCTION_HELPERS',helpers)
unit=unit.replace('// PRODUCTION_HANDLER','esp_err_t HttpTransport::handleWifi'+section(c,'esp_err_t HttpTransport::handleWifi','\n\n} // namespace network'))
unit=unit.replace('// PRODUCTION_SERVICE_CASES','case HttpRequestKind::WIFI_STATUS:'+section(c,'case HttpRequestKind::WIFI_STATUS:','case HttpRequestKind::OTA_CHALLENGE:'))
with tempfile.TemporaryDirectory(prefix='matdog-f01-') as d:
    cpp,exe=Path(d)/'probe.cpp',Path(d)/'probe'
    cpp.write_text(unit)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-O1',
                    '-fsanitize=address,undefined','-fno-omit-frame-pointer','-I',str(src),str(cpp),
                    str(src/'network/PortalSecurity.cpp'),str(src/'network/NetworkConfig.cpp'),
                    str(src/'update/Sha256.cpp'),'-o',str(exe)],check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)

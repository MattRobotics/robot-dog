#!/usr/bin/env python3
"""Historical V2 extracted-production transport test (candidate: test_ap_only_transport.py). No sockets/devices are opened.

Compiles the unmodified socket helpers, handleWifi, request/response structures,
and WIFI cases of serviceRequest from the selected checkout. Only HTTPD, lwIP,
clock/random, mailbox delivery and ControllerService are fakes. Real security,
form parser and SHA256 are linked. Extraction fails if source boundaries change.
The socket fake models the installed lwIP ABI and short-buffer copy semantics;
it is not execution of lwIP/HTTPD or hardware validation.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
p = argparse.ArgumentParser()
p.add_argument('--source-root', type=Path, default=ROOT)
p.add_argument('--expect-original-defect', action='store_true')
p.add_argument('--source-ref', default='b7516c39c9fcba09992acd07377facd5ae6e8588', help='Read transport source/types from a local Git commit')
a = p.parse_args()
src = a.source_root / 'src'
c = (src / 'network/HttpTransport.cpp').read_text()
h = (src / 'network/HttpTransport.h').read_text()
if a.source_ref:
    prefix = '05_Firmware/MATDOG_Controller/src/network/'
    c, h = [subprocess.check_output(['git', 'show', a.source_ref + ':' + prefix + n], cwd=ROOT, text=True) for n in ['HttpTransport.cpp', 'HttpTransport.h']]
def between(text, start, end):
    assert text.count(start) == 1 and text.count(end) == 1, (start, end)
    return text.split(start, 1)[1].split(end, 1)[0]
helpers = between(c, '\nbool readHexHeader', '\nvoid jsonString')
# Omit unrelated readHexHeader; keep every helper immediately before socketFacts.
helpers = helpers[helpers.index('\n}') + 2:]
handler = 'esp_err_t HttpTransport::handleWifi' + between(c, 'esp_err_t HttpTransport::handleWifi', '\n\n} // namespace network')
cases = 'case HttpRequestKind::WIFI_STATUS:' + between(c, 'case HttpRequestKind::WIFI_STATUS:', 'case HttpRequestKind::OTA_CHALLENGE:')
types = 'enum class HttpRequestKind' + between(h, 'enum class HttpRequestKind', '// Two optional listeners')
# Generated C++ is a host-only fixture, not an Arduino sketch header.
fixture = r'''
// Test fixture only: all credentials are synthetic, never read from the host.
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <map>
#include <string>
#include "network/PortalSecurity.h"
#include "update/OtaSession.h"
using socklen_t = uint32_t;
constexpr int AF_INET=2, AF_INET6=10;
struct in_addr { uint32_t s_addr; };
struct in6_addr { uint8_t s6_addr[16]; };
struct sockaddr { uint8_t sa_len, sa_family; char data[14]; };
struct sockaddr_in { uint8_t sin_len, sin_family; uint16_t sin_port; in_addr sin_addr; char zero[8]; };
struct sockaddr_in6 { uint8_t sin6_len, sin6_family; uint16_t sin6_port; uint32_t sin6_flowinfo; in6_addr sin6_addr; uint32_t sin6_scope_id; };
struct sockaddr_storage { uint8_t len, ss_family; char pad[2]; uint32_t data[6]; };
static_assert(sizeof(sockaddr_in)==16 && sizeof(sockaddr_in6)==28 && sizeof(sockaddr_storage)==28);
uint16_t ntohs(uint16_t v) { return __builtin_bswap16(v); }
uint32_t ntohl(uint32_t v) { return __builtin_bswap32(v); }
uint16_t htons(uint16_t v) { return ntohs(v); }
uint32_t htonl(uint32_t v) { return ntohl(v); }
#define IN6_IS_ADDR_V4MAPPED(a) (memcmp((a)->s6_addr, "\0\0\0\0\0\0\0\0\0\0\xff\xff", 12)==0)
using esp_err_t=int;
constexpr int ESP_OK=0, HTTP_GET=0, HTTP_POST=1, HTTPD_403_FORBIDDEN=403, HTTPD_400_BAD_REQUEST=400, HTTPD_500_INTERNAL_SERVER_ERROR=500;
uint64_t clock_ms=10000;
uint32_t millis() { return static_cast<uint32_t>(clock_ms); }
int64_t esp_timer_get_time() { return static_cast<int64_t>(clock_ms * 1000); }
void esp_fill_random(void *p,size_t n) { memset(p,0x42,n); }
struct httpd_req_t {
 void *user_ctx=nullptr; int method=HTTP_POST; std::string uri_store, body;
 const char *uri=nullptr; size_t content_len=0, consumed=0;
 std::map<std::string,std::string> headers,response_headers;
 int status=200; std::string response;
};
sockaddr_storage local_addr{}, peer_addr{};
socklen_t local_len=28,peer_len=28;
bool socket_error=false,header_error=false;
int getsockname(int, sockaddr *out,socklen_t *n) {
 if(socket_error)return -1;
 *n=std::min(*n,local_len); memcpy(out,&local_addr,*n); return 0;
}
int getpeername(int, sockaddr *out,socklen_t *n) {
 if(socket_error)return -1;
 *n=std::min(*n,peer_len); memcpy(out,&peer_addr,*n); return 0;
}
int httpd_req_to_sockfd(httpd_req_t*) { return 7; }
int httpd_req_get_hdr_value_str(httpd_req_t*r,const char*k,char*out,size_t n) {
 if(header_error)return -1;
 auto it=r->headers.find(k); if(it==r->headers.end())return -1;
 size_t z=std::min(n-1,it->second.size()); memcpy(out,it->second.data(),z);out[z]=0;
 return z==it->second.size()?ESP_OK:-2;
}
int httpd_req_recv(httpd_req_t*r,char*out,size_t n) {
 size_t z=std::min(n,r->body.size()-r->consumed);memcpy(out,r->body.data()+r->consumed,z);r->consumed+=z;return int(z);
}
int httpd_resp_send_err(httpd_req_t*r,int status,const char*m) { r->status=status;r->response=m;return 0; }
int httpd_resp_set_hdr(httpd_req_t*r,const char*k,const char*v) { r->response_headers[k]=v;return 0; }
int httpd_resp_set_type(httpd_req_t*r,const char*v) { return httpd_resp_set_hdr(r,"Content-Type",v); }
int httpd_resp_send(httpd_req_t*r,const char*s,size_t n) { r->response.assign(s,n);return 0; }
int httpd_resp_sendstr(httpd_req_t*r,const char*s) { r->response=s;return 0; }
namespace matdog { namespace network {
// PRODUCTION_TYPES
struct FakeService {
 uint8_t digest[32]{}; unsigned mutations=0;
 const uint8_t* networkAdminDigest(){return digest;}
 struct { bool holds_actuator_inhibit=false; } ota;
 auto& otaStatus(){return ota;}
 bool configureNetwork(const ConfigPatch&){++mutations;return true;}
 bool scanNetwork(){++mutations;return true;}
 bool accessPoint(bool){++mutations;return true;}
};
class HttpTransport {
 public:
 FakeService service; FakeService*service_=&service; PortalSession portal_;
 struct {bool sessionAuthenticated(){return false;}}session_;
 static esp_err_t handleWifi(httpd_req_t*);
 void wifiResponse(HttpTransportResponse&r){r.ok=true;strcpy(r.text,"{}");r.text_len=2;}
 bool dispatch(const HttpTransportRequest&req,HttpTransportResponse*out){
  HttpTransportResponse resp{};[[maybe_unused]] const uint32_t now_ms=static_cast<uint32_t>(clock_ms);
  switch(req.kind){
   // PRODUCTION_SERVICE_CASES
   default: return false;
  }
  *out=resp;return true;
 }
};
// PRODUCTION_HELPERS
// PRODUCTION_HANDLER
}}
using namespace matdog::network;
unsigned checks=0;
void check(bool ok,const char*name){++checks;if(!ok){fprintf(stderr,"FAIL %s\n",name);exit(1);}}
void endpoint(sockaddr_storage&out,socklen_t&len,uint32_t ip,bool mapped,uint16_t port){
 memset(&out,0,sizeof(out));
 if(mapped){sockaddr_in6 a{};a.sin6_len=28;a.sin6_family=AF_INET6;a.sin6_port=htons(port);a.sin6_addr.s6_addr[10]=a.sin6_addr.s6_addr[11]=255;uint32_t n=htonl(ip);memcpy(a.sin6_addr.s6_addr+12,&n,4);memcpy(&out,&a,28);len=28;}
 else{sockaddr_in a{};a.sin_len=16;a.sin_family=AF_INET;a.sin_port=htons(port);a.sin_addr.s_addr=htonl(ip);memcpy(&out,&a,16);len=16;}
}
void endpoints(bool mapped=true,uint32_t local=0xc0a80401u,uint32_t peer=0xc0a80402u){endpoint(local_addr,local_len,local,mapped,80);endpoint(peer_addr,peer_len,peer,mapped,49152);socket_error=header_error=false;}
httpd_req_t request(HttpTransport&s,std::string path,std::string body,std::string host="192.168.4.1",std::string origin="http://192.168.4.1"){
 // Serialize and parse a complete browser/Python-equivalent HTTP/1.1 fixture.
 // This parser is a fake HTTPD boundary, not validation of the ESP HTTP parser.
 std::string wire="POST "+path+" HTTP/1.1\r\nHost: "+host+"\r\nOrigin: "+origin+"\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
 httpd_req_t r;r.user_ctx=&s;r.uri_store=path;r.body=wire.substr(wire.find("\r\n\r\n")+4);r.content_len=r.body.size();
 size_t i=wire.find("\r\n")+2;while(wire.substr(i,2)!="\r\n"){size_t e=wire.find("\r\n",i),c=wire.find(':',i);r.headers[wire.substr(i,c-i)]=wire.substr(c+2,e-c-2);i=e+2;}return r;
}
void run(httpd_req_t&r){r.uri=r.uri_store.c_str();HttpTransport::handleWifi(&r);}
std::string token(64,'a');
void provision(HttpTransport&s){matdog::update::Sha256 sha;sha.update(reinterpret_cast<const uint8_t*>(token.data()),token.size());sha.finish(s.service.digest);}
int main(int argc,char**){
 bool original=argc>1; endpoints();HttpTransport s;provision(s);
 auto login=request(s,"/wifi/login","token="+token);run(login);
 if(original){check(login.status==403 && login.response=="Protected AP origin required","original mapped AP reproduces recorded 403");HttpTransportRequest facts{};check(socketFacts(&login,facts)&&!facts.ap_socket&&facts.peer==0,"original reads IPv6 flowinfo as IPv4 and loses peer");printf("ORIGINAL_DEFECT_REPRODUCED: %u checks; correct headers, mapped AP accepted socket, zero mutations\n",checks);return 0;}
 check(login.status==200,"mapped AP browser login");
 check(login.response_headers["Set-Cookie"].find("HttpOnly; SameSite=Strict; Path=/wifi/; Max-Age=300")!=std::string::npos,"cookie protections");
 HttpTransportRequest facts{};check(socketFacts(&login,facts)&&facts.ap_socket&&facts.peer==htonl(0xc0a80402u),"mapped AP and peer extracted");
 for(bool mapped:{false,true}){endpoints(mapped);for(const char*host:{"192.168.4.1","192.168.4.1:80"})for(const char*origin:{"http://192.168.4.1","http://192.168.4.1:80"}){HttpTransport x;provision(x);auto r=request(x,"/wifi/login","token="+token,host,origin);run(r);check(r.status==200,"IPv4/mapped explicit allowed header variants");}}
 endpoints();
 for(auto pair:{std::pair<const char*,const char*>{"Host","evil.invalid"},{"Host","192.168.4.1:81"},{"Origin","http://evil.invalid"},{"Origin","null"},{"Origin","https://192.168.4.1"},{"Origin","http://192.168.4.1.evil"},{"Host",""},{"Origin",""}}){auto r=request(s,"/wifi/login","token="+token);r.headers[pair.first]=pair.second;run(r);check(r.status==403,"forged/empty header");}
 for(const char*k:{"Host","Origin"}){auto r=request(s,"/wifi/login","token="+token);r.headers.erase(k);run(r);check(r.status==403,"missing header");}
 for(uint32_t ip:{0xc0a80110u,0x7f000001u,0x0a000001u,0u})for(bool mapped:{false,true}){endpoints(mapped,ip);for(const char*path:{"/wifi/config","/wifi/ap","/wifi/scan"}){auto r=request(s,path,"");run(r);check(r.status==403&&r.response=="Protected AP origin required","non-AP local socket denies writes");}}
 endpoints();for(const char*k:{"Host","Origin"}){auto r=request(s,"/wifi/login","token="+token);r.headers[k]=std::string(100,'a');run(r);check(r.status==403,"oversize header");}
 header_error=true;auto hr=request(s,"/wifi/login","token="+token);run(hr);check(hr.status==403,"header retrieval error");header_error=false;
 socket_error=true;auto sr=request(s,"/wifi/login","token="+token);run(sr);check(sr.status==403&&sr.response=="Socket unavailable","socket error");endpoints();
 local_len=8;auto shortaddr=request(s,"/wifi/login","token="+token);run(shortaddr);check(shortaddr.status==403,"short sockaddr");endpoints();
 for(bool mapped:{false,true}){endpoints(mapped);endpoint(local_addr,local_len,0xc0a80401u,mapped,443);HttpTransportRequest f{};check(socketFacts(&login,f)&&f.tls&&f.ap_socket,"TLS AP remains AP for OTA exclusion");endpoint(local_addr,local_len,0xc0a80110u,mapped,443);check(socketFacts(&login,f)&&f.tls&&!f.ap_socket,"TLS STA classification");}
 endpoints();peer_len=8;auto shortpeer=request(s,"/wifi/login","token="+token);run(shortpeer);check(shortpeer.status==403,"short peer sockaddr");endpoints();
 local_addr.ss_family=99;auto unknown=request(s,"/wifi/login","token="+token);run(unknown);check(unknown.status==403,"unknown address family");endpoints();
 // Native IPv6, including a flowinfo value resembling AP IPv4, must fail closed.
 reinterpret_cast<sockaddr_in6*>(&local_addr)->sin6_addr.s6_addr[0]=0x20;
 reinterpret_cast<sockaddr_in6*>(&local_addr)->sin6_flowinfo=htonl(0xc0a80401u);
 auto ipv6=request(s,"/wifi/login","token="+token);run(ipv6);check(ipv6.status==403,"native IPv6 rejected");endpoints();
 for(std::string bad:{std::string(64,'b'),std::string(64,'z'),std::string()}){HttpTransport x;provision(x);auto r=request(x,"/wifi/login","token="+bad);run(r);check(r.status==(bad.empty()?400:403),"wrong/malformed/empty token");}
 auto write=[&](std::string cookie,std::string csrf){auto r=request(s,"/wifi/ap","on=1");r.headers["Cookie"]=cookie;r.headers["X-Csrf-Token"]=csrf;run(r);return r.status;};
 std::string cookie="md_session="+std::string(s.portal_.cookie()),csrf=s.portal_.csrf();
 check(write("",csrf)==403,"missing cookie");check(write("md_session="+std::string(64,'0'),csrf)==403,"forged cookie");check(write(cookie,"")==403,"missing csrf");check(write(cookie,std::string(64,'0'))==403,"bad csrf");
 endpoints(true,0xc0a80401u,0xc0a80403u);check(write(cookie,csrf)==403,"peer mismatch");endpoints();clock_ms+=300000;check(write(cookie,csrf)==403,"expired TTL exact boundary");auto renewed=request(s,"/wifi/login","token="+token);run(renewed);check(renewed.status==200,"fresh login required after irreversible expiry");
 check(s.service.mutations==0,"all adversarial requests zero service mutations");check(write(cookie,csrf)==200&&s.service.mutations==1,"authorized AP action");
 auto config=request(s,"/wifi/config","usb_sleep=0");config.headers["Cookie"]=cookie;config.headers["X-Csrf-Token"]=csrf;run(config);check(config.status==200&&config.response=="CONFIG_PENDING","authorized config reaches fake service");
 auto badform=request(s,"/wifi/config","usb_sleep=0&usb_sleep=1");run(badform);check(badform.status==400,"duplicate form key");
 auto ct=request(s,"/wifi/login","token="+token);ct.headers["Content-Type"]="application/json";run(ct);check(ct.status==400,"strict content type");
 HttpTransport throttle;provision(throttle);for(int i=0;i<5;++i){clock_ms+=1000;auto r=request(throttle,"/wifi/login","token="+std::string(64,'b'));run(r);check(r.status==403,"wrong login throttle setup");}clock_ms+=1000;auto limited=request(throttle,"/wifi/login","token="+token);run(limited);check(limited.status==403,"rate limit after five failures");clock_ms+=60000;auto recovered=request(throttle,"/wifi/login","token="+token);run(recovered);check(recovered.status==200,"rate limit recovery");
 endpoints(true,0xc0a80110u);auto status=request(s,"/wifi/status","");status.method=HTTP_GET;run(status);check(status.status==200,"STA read-only status dispatch");
 endpoints();HttpTransport uptime;provision(uptime);clock_ms=(uint64_t{1}<<32)-1000;
 auto longlogin=request(uptime,"/wifi/login","token="+token);run(longlogin);check(longlogin.status==200,"64-bit transport login before wrap");
 auto longwrite=[&](){auto r=request(uptime,"/wifi/ap","on=1");r.headers["Cookie"]="md_session="+std::string(uptime.portal_.cookie());r.headers["X-Csrf-Token"]=uptime.portal_.csrf();run(r);return r.status;};
 clock_ms+=2000;check(longwrite()==200,"transport uses wide timer across millis wrap");
 clock_ms+=300000;check(longwrite()==403,"transport wide TTL expires");
 clock_ms=(uint64_t{2}<<32)-1000;check(longwrite()==403,"transport no expired replay next cycle");
 check(uptime.service.mutations==1,"expired transport replay has no mutation");
 printf("AP_TRANSPORT_PASS: %u checks; real extracted transport/auth, fake HTTPD/lwIP/service; no device I/O\n",checks);
}
'''

unit = fixture.replace('// PRODUCTION_TYPES', types).replace('// PRODUCTION_HELPERS', helpers).replace('// PRODUCTION_HANDLER', handler).replace('// PRODUCTION_SERVICE_CASES', cases)
with tempfile.TemporaryDirectory(prefix='matdog-ap-offline-') as d:
    cpp, exe = Path(d)/'test.cpp', Path(d)/'test'
    cpp.write_text(unit)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer','-I',str(src),str(cpp),str(src/'network/PortalSecurity.cpp'),str(src/'network/NetworkConfig.cpp'),str(src/'update/Sha256.cpp'),'-o',str(exe)],check=True)
    subprocess.run([str(exe), *(['original'] if a.expect_original_defect else [])],check=True)

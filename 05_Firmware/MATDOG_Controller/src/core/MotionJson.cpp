#include "MotionJson.h"
#include <cstring>
#include <cstdlib>
#include <cmath>
namespace matdog { namespace core {
namespace {
struct Parser {
 const char* p;const char* end;
 void ws(){while(p<end&&(*p==' '||*p=='\n'||*p=='\r'||*p=='\t'))p++;}
 bool token(char c){ws();if(p==end||*p!=c)return false;p++;return true;}
 bool string(char* out,size_t cap){ws();if(p==end||*p++!='"')return false;size_t n=0;while(p<end&&*p!='"'){unsigned char c=*p++;if(c<32)return false;if(c=='\\'){if(p==end)return false;c=*p++;if(c=='u'){for(unsigned i=0;i<4;i++){if(p==end||!((*p>='0'&&*p<='9')||(*p>='a'&&*p<='f')||(*p>='A'&&*p<='F')))return false;p++;}c='?';}else if(c!='"'&&c!='\\'&&c!='/')return false;}if(n+1>=cap)return false;out[n++]=c;}if(p==end)return false;p++;out[n]=0;return true;}
 bool number(double& v){ws();const char* start=p;if(p<end&&*p=='-')p++;if(p==end)return false;if(*p=='0')p++;else {if(*p<'1'||*p>'9')return false;while(p<end&&*p>='0'&&*p<='9')p++;}if(p<end&&*p=='.'){p++;const char* d=p;while(p<end&&*p>='0'&&*p<='9')p++;if(p==d)return false;}if(p<end&&(*p=='e'||*p=='E')){p++;if(p<end&&(*p=='+'||*p=='-'))p++;const char* d=p;while(p<end&&*p>='0'&&*p<='9')p++;if(p==d)return false;}const size_t n=p-start;if(!n||n>48)return false;char text[49]{};memcpy(text,start,n);char* tail=nullptr;v=strtod(text,&tail);return tail==text+n&&std::isfinite(v);}
 bool uint(uint32_t& v){double d;if(!number(d)||d<0||d>4294967295.0||std::floor(d)!=d)return false;v=static_cast<uint32_t>(d);return true;}
};
bool params(Parser& p,motion::MotionCommand& c,uint32_t& seen){
 if(!p.token('{'))return false;
 p.ws();if(p.p<p.end&&*p.p=='}'){p.p++;return true;}
 for(unsigned count=0;count<16;count++){
  char key[32]{};if(!p.string(key,sizeof(key))||!p.token(':'))return false;
  const char* keys[]={"target_height_m","vx","vy","yaw_rate","x","y","z","roll","pitch","yaw","gait","action_id","reason"};unsigned index=13;for(unsigned i=0;i<13;i++)if(!strcmp(key,keys[i]))index=i;
  if(index==13||(seen&(1u<<index)))return false;
  seen|=1u<<index;
  double* values[]={&c.target_height_m,&c.vx,&c.vy,&c.yaw_rate,&c.x,&c.y,&c.z,&c.roll,&c.pitch,&c.yaw};
  if(index<10){if(!p.number(*values[index]))return false;}
  else if(index==10){if(!p.string(c.gait,sizeof(c.gait)))return false;}
  else if(index==11){if(!p.string(c.action_id,sizeof(c.action_id)))return false;}
  else {char unused[96];if(!p.string(unused,sizeof(unused)))return false;}
  if(p.token('}'))return true;
  if(!p.token(','))return false;
 }
 return false;
}
}
bool parseMotionJson(const char* input,size_t len,motion::MotionCommand* out,const char** reason){
 if(reason)*reason="INVALID_JSON_SCHEMA";
 if(!input||!out||!len||len>1024||memchr(input,0,len))return false;
 Parser p{input,input+len};motion::MotionCommand c;uint32_t seen=0,ps=0;bool closed=false;
 if(!p.token('{'))return false;
 for(unsigned count=0;count<8;count++){
  char key[32]{};if(!p.string(key,sizeof(key))||!p.token(':'))return false;
  const char* keys[]={"version","id","sequence","issued_at_ms","ttl_ms","owner","type","params"};unsigned i=8;for(unsigned k=0;k<8;k++)if(!strcmp(key,keys[k]))i=k;
  if(i==8||(seen&(1u<<i)))return false;
  seen|=1u<<i;
  uint32_t n=0;char text[49]{};
  switch(i){case 0:if(!p.uint(n)||n>65535)return false;c.version=n;break;case 1:if(!p.string(c.id,sizeof(c.id)))return false;break;case 2:if(!p.uint(c.sequence))return false;break;case 3:if(!p.uint(c.issued_at_ms))return false;break;case 4:if(!p.uint(c.ttl_ms))return false;break;case 5:if(!p.string(text,sizeof(text)))return false;c.owner=motion::parseOwner(text);break;case 6:if(!p.string(text,sizeof(text)))return false;c.intent=motion::parseIntent(text);break;case 7:if(!params(p,c,ps))return false;break;default:return false;}
  if(p.token('}')){closed=true;break;}
  if(!p.token(','))return false;
 }
 p.ws();if(!closed||p.p!=p.end||seen!=255)return false;
 uint32_t allowed=0,required=0;
 switch(c.intent){case motion::Intent::STAND:allowed=required=1;break;case motion::Intent::LIE_DOWN:allowed=1;break;case motion::Intent::GAIT_SELECT:allowed=required=1u<<10;break;case motion::Intent::BASE_VELOCITY:allowed=required=14;break;case motion::Intent::BODY_POSE:allowed=1008;break;case motion::Intent::ACTION_EXECUTE:allowed=(1u<<11)|1;required=1u<<11;break;case motion::Intent::STOP:allowed=1u<<12;break;case motion::Intent::STATUS:case motion::Intent::CAPABILITIES:break;default:return false;}
 if((ps&~allowed)||(ps&required)!=required)return false;
 *out=c;if(reason)*reason="OK";return true;
}
} }

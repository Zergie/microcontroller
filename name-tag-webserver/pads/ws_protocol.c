#include "ws_protocol.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>

static uint32_t rotate(uint32_t x,unsigned n) { return (x<<n)|(x>>(32-n)); }
// SHA-1 is used only for RFC 6455's public handshake token, not authentication.
// The input is always the 24-character key followed by the 36-character GUID.
static void accept_token(const char *key,char out[29]) {
    uint8_t bytes[128]={0}, digest[20];
    uint32_t h[]={0x67452301,0xefcdab89,0x98badcfe,0x10325476,0xc3d2e1f0};
    memcpy(bytes,key,24); memcpy(bytes+24,"258EAFA5-E914-47DA-95CA-C5AB0DC85B11",36);
    bytes[60]=0x80; bytes[126]=1; bytes[127]=0xe0; // 60 * 8 bits, big endian.
    for (unsigned block=0;block<2;++block) {
        uint32_t w[80];
        for (unsigned i=0;i<16;++i) {
            const uint8_t *p=bytes+block*64+i*4;
            w[i]=((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
        }
        for (unsigned i=16;i<80;++i) w[i]=rotate(w[i-3]^w[i-8]^w[i-14]^w[i-16],1);
        uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4];
        for (unsigned i=0;i<80;++i) {
            uint32_t f,k;
            if (i<20) { f=(b&c)|(~b&d); k=0x5a827999; }
            else if (i<40) { f=b^c^d; k=0x6ed9eba1; }
            else if (i<60) { f=(b&c)|(b&d)|(c&d); k=0x8f1bbcdc; }
            else { f=b^c^d; k=0xca62c1d6; }
            uint32_t t=rotate(a,5)+f+e+k+w[i]; e=d; d=c; c=rotate(b,30); b=a; a=t;
        }
        h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d; h[4]+=e;
    }
    for (unsigned i=0;i<20;++i) digest[i]=(uint8_t)(h[i/4]>>(24-8*(i%4)));
    const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    unsigned used=0;
    for (unsigned i=0;i<20;i+=3) {
        uint32_t n=(uint32_t)digest[i]<<16;
        if (i+1<20) n|=(uint32_t)digest[i+1]<<8;
        if (i+2<20) n|=digest[i+2];
        out[used++]=alphabet[n>>18]; out[used++]=alphabet[(n>>12)&63];
        out[used++]=alphabet[(n>>6)&63]; out[used++]=i+2<20 ? alphabet[n&63] : '=';
    }
    out[28]=0;
}

static bool header(const char *request,const char *name,char *out,size_t capacity) {
    size_t n=strlen(name); bool found=false;
    const char *line=strstr(request,"\r\n");
    while (line && line[2] && line[2]!='\r') {
        line+=2; const char *end=strstr(line,"\r\n");
        if (!end) return false;
        if (!strncasecmp(line,name,n) && line[n]==':') {
            if (found) return false;
            const char *value=line+n+1;
            while (value<end && (*value==' ' || *value=='\t')) ++value;
            while (end>value && (end[-1]==' ' || end[-1]=='\t')) --end;
            size_t length=(size_t)(end-value);
            if (length>=capacity) return false;
            memcpy(out,value,length); out[length]=0; found=true;
        }
        line=strstr(line,"\r\n");
    }
    return found;
}
static bool contains_token(const char *value,const char *token) {
    size_t n=strlen(token);
    while (*value) {
        while (*value==' ' || *value=='\t' || *value==',') ++value;
        const char *end=strchr(value,','); if (!end) end=value+strlen(value);
        const char *trim=end; while (trim>value && (trim[-1]==' ' || trim[-1]=='\t')) --trim;
        if ((size_t)(trim-value)==n && !strncasecmp(value,token,n)) return true;
        value=*end ? end+1 : end;
    }
    return false;
}
static bool handshake(pad_ws_protocol_t *ws) {
    char key[25],value[160],host[128];
    ws->input[ws->used]=0;
    const char *request=(char *)ws->input;
    if (strncmp(request,"GET /api/pads/ws HTTP/1.1\r\n",27) ||
        !header(request,"Sec-WebSocket-Key",key,sizeof(key)) || strlen(key)!=24 ||
        key[22]!='=' || key[23]!='=' ||
        !header(request,"Upgrade",value,sizeof(value)) || strcasecmp(value,"websocket") ||
        !header(request,"Connection",value,sizeof(value)) || !contains_token(value,"upgrade") ||
        !header(request,"Sec-WebSocket-Version",value,sizeof(value)) || strcmp(value,"13") ||
        !header(request,"Host",host,sizeof(host)) || !header(request,"Origin",value,sizeof(value))) return false;
    const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (unsigned i=0;i<22;++i) if (!strchr(alphabet,key[i])) return false;
    if (!strchr("AQgw",key[21])) return false;
    // Only same-device browser origins; the trusted dev proxy rewrites these.
    char *port=strchr(host,':'); if (port) *port=0;
    if (!*host || strncasecmp(value,"http://",7) || strcasecmp(value+7,host)) return false;
    char accept[29]; accept_token(key,accept);
    // Reuse the consumed receive buffer to keep the RP2040 callback stack small.
    int n=snprintf((char *)ws->input,sizeof(ws->input),"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n",accept);
    if (n<0 || (size_t)n>=sizeof(ws->input) || !ws->write(ws->user,ws->input,(size_t)n)) return false;
    ws->upgraded=true; ws->used=0; return true;
}
static bool frame_out(pad_ws_protocol_t *ws,uint8_t opcode,const uint8_t *data,size_t length) {
    uint8_t frame[127]={0x80|opcode,(uint8_t)length};
    memcpy(frame+2,data,length); return ws->write(ws->user,frame,length+2);
}
static bool close_error(pad_ws_protocol_t *ws,unsigned code) {
    uint8_t payload[]={(uint8_t)(code>>8),(uint8_t)code};
    frame_out(ws,8,payload,2); return false;
}
static bool valid_close(const uint8_t *p,unsigned length) {
    if (!length) return true;
    if (length==1) return false;
    unsigned code=(unsigned)p[0]*256+p[1];
    if (!((code>=1000 && code<=1014 && code!=1004 && code!=1005 && code!=1006) ||
          (code>=3000 && code<=4999))) return false;
    for (unsigned i=2;i<length;) {
        unsigned c=p[i++],remaining=0,min=0,value=0;
        if (c<128) continue;
        if (c>=0xc2 && c<=0xdf) { remaining=1; min=0x80; value=c&31; }
        else if (c>=0xe0 && c<=0xef) { remaining=2; min=0x800; value=c&15; }
        else if (c>=0xf0 && c<=0xf4) { remaining=3; min=0x10000; value=c&7; }
        else return false;
        if (i+remaining>length) return false;
        while (remaining--) { c=p[i++]; if ((c&0xc0)!=0x80) return false; value=(value<<6)|(c&63); }
        if (value<min || value>0x10ffff || (value>=0xd800 && value<=0xdfff)) return false;
    }
    return true;
}
static bool frame_in(pad_ws_protocol_t *ws) {
    uint8_t *p=ws->input; unsigned opcode=p[0]&15, length=p[1]&127;
    bool final=(p[0]&128)!=0;
    if ((p[0]&0x70) || !(p[1]&128) || length>125) return close_error(ws,1002);
    if (opcode>=8 && !final) return close_error(ws,1002);
    if (ws->used<length+6) return true;
    uint8_t payload[125];
    for (unsigned i=0;i<length;++i) payload[i]=p[6+i]^p[2+i%4];
    ws->used=0;
    if (opcode==8) return close_error(ws,valid_close(payload,length) ? 1000 : 1002);
    if (opcode==9) return frame_out(ws,10,payload,length);
    if (opcode==10) return true;
    if (opcode!=0 && opcode!=2) return close_error(ws,1003);
    if ((opcode==0)!=ws->fragmented) return close_error(ws,1002);
    if (ws->message_used+length>sizeof(ws->message)) return close_error(ws,1009);
    memcpy(ws->message+ws->message_used,payload,length); ws->message_used+=length;
    ws->fragmented=!final;
    if (!final) return true;
    uint8_t kind=ws->message[0];
    unsigned mask=ws->message[1];
    if (ws->message_used>=3 && (kind==3 || kind==4)) mask|=(unsigned)ws->message[2]<<8;
    bool valid=false;
    if (ws->message_used==2 && kind==1 && ws->press)
        valid=ws->press(ws->user,mask);
    if (ws->message_used==4 && kind==2 && ws->pulse)
        valid=ws->pulse(ws->user,mask,(unsigned)ws->message[2]|((unsigned)ws->message[3]<<8));
    if (ws->message_used==3 && kind==3 && ws->press && mask<=1023)
        valid=ws->press(ws->user,mask);
    if (ws->message_used==5 && kind==4 && ws->pulse && mask<=1023)
        valid=ws->pulse(ws->user,mask,(unsigned)ws->message[3]|((unsigned)ws->message[4]<<8));
    // Read-only diagnostic echo: no pad state, lease or settings changes.
    if (ws->message_used==2 && kind==5) valid=true;
    if (!valid) return close_error(ws,1008);
    ws->message_used=0;
    // Tiny acknowledgement also lets the browser detect stalled transports.
    uint8_t ack[]={kind,(uint8_t)mask,(uint8_t)(mask>>8)};
    return frame_out(ws,2,ack,(kind==3 || kind==4) ? 3 : 2);
}
bool pad_ws_feed(pad_ws_protocol_t *ws,const uint8_t *data,size_t length) {
    for (size_t i=0;i<length;++i) {
        if (ws->used>=sizeof(ws->input)-1 || (!ws->upgraded && data[i]==0)) return false;
        ws->input[ws->used++]=data[i];
        if (!ws->upgraded) {
            if (ws->used>=4 && !memcmp(ws->input+ws->used-4,"\r\n\r\n",4) && !handshake(ws)) return false;
        } else if (ws->used>=2 && !frame_in(ws)) return false;
    }
    return true;
}

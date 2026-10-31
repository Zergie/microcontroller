#include "pads/ws_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t output[4096];
static size_t output_used;
static unsigned pressed,commands,duration;
static bool allow_write=true,allow_press=true;
static bool write_bytes(void *user,const uint8_t *data,size_t length) {
    (void)user;
    if (!allow_write) return false;
    assert(output_used+length<sizeof(output)); memcpy(output+output_used,data,length); output_used+=length; return true;
}
static bool press(void *user,uint8_t mask) { (void)user; pressed=mask; ++commands; return allow_press; }
static bool pulse(void *user,uint8_t mask,unsigned ms) { duration=ms; return press(user,mask); }
static pad_ws_protocol_t protocol(void) {
    output_used=commands=pressed=0; allow_write=allow_press=true;
    return (pad_ws_protocol_t){.write=write_bytes,.press=press,.pulse=pulse};
}
static void request(char *buffer,size_t capacity,const char *key,const char *origin) {
    snprintf(buffer,capacity,"GET /api/pads/ws HTTP/1.1\r\nHost: rp2040.local:81\r\nUpgrade: WebSocket\r\nConnection: keep-alive, Upgrade\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: %s\r\nOrigin: %s\r\n\r\n",key,origin);
}
static pad_ws_protocol_t upgraded(void) {
    pad_ws_protocol_t ws=protocol(); char buffer[512]; request(buffer,sizeof(buffer),"dGhlIHNhbXBsZSBub25jZQ==","http://rp2040.local");
    // TCP may deliver one byte at a time.
    for (size_t i=0;i<strlen(buffer);++i) assert(pad_ws_feed(&ws,(uint8_t *)buffer+i,1));
    assert(ws.upgraded && !commands);
    assert(strstr((char *)output,"s3pPLMBiTxaQ9kYGzzhZRbK+xOo=")); output_used=0;
    return ws;
}
static size_t frame(uint8_t *out,uint8_t opcode,bool final,const uint8_t *payload,size_t length) {
    out[0]=(final ? 128:0)|opcode; out[1]=128|(uint8_t)length;
    for (unsigned i=0;i<4;++i) out[2+i]=(uint8_t)(i*37+11);
    for (size_t i=0;i<length;++i) out[6+i]=payload[i]^out[2+i%4];
    return length+6;
}
int main(int argc,char **argv) {
    if (argc==2) {
        pad_ws_protocol_t ws=protocol(); char buffer[512]; request(buffer,sizeof(buffer),argv[1],"http://rp2040.local");
        assert(pad_ws_feed(&ws,(uint8_t *)buffer,strlen(buffer))); fwrite(output,1,output_used,stdout); return 0;
    }
    pad_ws_protocol_t ws=upgraded(); uint8_t data[300],payload[]={1,129};
    size_t n=frame(data,2,true,payload,2);
    for (size_t i=0;i<n;++i) assert(pad_ws_feed(&ws,data+i,1));
    assert(commands==1 && pressed==129 && output_used==4 && output[0]==0x82 && output[3]==129);
    // Coalesced TCP frames and independent command processing, no ACK wait.
    payload[1]=64; n=frame(data,2,true,payload,2); payload[1]=0; n+=frame(data+n,2,true,payload,2);
    assert(pad_ws_feed(&ws,data,n) && commands==3 && pressed==0);
    // Fragmentation with interleaved ping; control payload must not enter a mask.
    n=frame(data,2,false,payload,1); n+=frame(data+n,9,true,(uint8_t *)"ping",4); n+=frame(data+n,0,true,payload+1,1);
    assert(pad_ws_feed(&ws,data,n) && commands==4);
    assert(memchr(output,0x8a,output_used));
    uint8_t note[]={2,64,0x2f,0x04}; // 1071 ms, little endian.
    n=frame(data,2,false,note,2); n+=frame(data+n,0,true,note+2,2);
    assert(pad_ws_feed(&ws,data,n) && commands==5 && pressed==64 && duration==1071);
    assert(output[output_used-2]==2 && output[output_used-1]==64);
    for (unsigned kind=0;kind<10;++kind) {
        ws=upgraded(); payload[0]=1; payload[1]=255; n=frame(data,2,true,payload,2);
        switch (kind) {
        case 0: data[1]&=127; break; // Unmasked client.
        case 1: data[0]|=0x40; break; // Unnegotiated extension.
        case 2: data[0]=0x80; break; // Unexpected continuation.
        case 3: data[1]=0xfe; break; // Oversized/extended payload.
        case 4: data[0]=0x09; break; // Fragmented control frame.
        case 5: payload[0]=2; n=frame(data,2,true,payload,2); break;
        case 6: n=frame(data,2,true,(uint8_t *)"abc",3); break;
        case 7: n=frame(data,1,true,payload,2); break;
        case 8: n=frame(data,8,true,(uint8_t *)"x",1); break;
        case 9: allow_press=false; break;
        }
        assert(!pad_ws_feed(&ws,data,n));
    }
    ws=upgraded(); allow_write=false; n=frame(data,2,true,payload,2); assert(!pad_ws_feed(&ws,data,n));
    const char *origins[]={"http://evil.example","null","http://rp2040.local.evil","https://rp2040.local"};
    for (unsigned i=0;i<4;++i) {
        ws=protocol(); char buffer[512]; request(buffer,sizeof(buffer),"dGhlIHNhbXBsZSBub25jZQ==",origins[i]);
        assert(!pad_ws_feed(&ws,(uint8_t *)buffer,strlen(buffer)) && !commands);
    }
    ws=protocol(); memset(data,'X',sizeof(data));
    assert(pad_ws_feed(&ws,data,sizeof(data))); assert(pad_ws_feed(&ws,data,sizeof(data))); assert(pad_ws_feed(&ws,data,sizeof(data)));
    assert(!pad_ws_feed(&ws,data,sizeof(data)));
    // Deterministic malformed-input stress under ASan/UBSan.
    uint32_t rng=17;
    for (unsigned run=0;run<5000;++run) {
        ws=protocol(); ws.upgraded=true;
        for (unsigned i=0;i<sizeof(data);++i) { rng=rng*1664525u+1013904223u; data[i]=(uint8_t)(rng>>24); }
        (void)pad_ws_feed(&ws,data,sizeof(data));
    }
    puts("Pad WebSocket: RFC handshake, bounded streaming, masks, fragmentation, ping, origin and malformed frames OK");
}

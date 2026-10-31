#include "pads/ws_server.h"
#include "pads/pads.h"
#include "lwip/tcp.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static struct tcp_pcb listener;
static uint64_t now;
static bool fail_new,fail_bind,fail_listen,fail_write,fail_close;
static unsigned freed,written;
uint64_t time_us_64(void) { return now*1000; }
struct tcp_pcb *tcp_new_ip_type(unsigned type) { assert(type==IPADDR_TYPE_V4); return fail_new ? NULL : &listener; }
err_t tcp_bind(struct tcp_pcb *p,const void *address,u16_t port) { (void)p; (void)address; assert(port==81); return fail_bind ? ERR_MEM : ERR_OK; }
struct tcp_pcb *tcp_listen_with_backlog(struct tcp_pcb *p,unsigned backlog) { assert(backlog==1); return fail_listen ? NULL : p; }
err_t tcp_close(struct tcp_pcb *p) { if (fail_close) return ERR_MEM; p->closed=1; return ERR_OK; }
void tcp_abort(struct tcp_pcb *p) { p->aborted=1; if (p->error) p->error(p->arg,ERR_ABRT); }
err_t tcp_write(struct tcp_pcb *p,const void *data,u16_t length,unsigned flags) {
    (void)p; (void)data; assert(flags==TCP_WRITE_FLAG_COPY); written+=length; return fail_write ? ERR_MEM : ERR_OK;
}
err_t tcp_output(struct tcp_pcb *p) { (void)p; return ERR_OK; }
void pbuf_free(struct pbuf *p) { (void)p; ++freed; }
static err_t input(struct tcp_pcb *p,const uint8_t *data,size_t length) {
    struct pbuf tail={.payload=(void *)(data+1),.len=(u16_t)(length-1),.tot_len=(u16_t)(length-1)};
    struct pbuf head={.next=&tail,.payload=(void *)data,.len=1,.tot_len=(u16_t)length};
    return p->recv(p->arg,p,&head,ERR_OK);
}
static struct tcp_pcb open_client(void) {
    return (struct tcp_pcb){0};
}
static void upgrade(struct tcp_pcb *p) {
    const char request[]="GET /api/pads/ws HTTP/1.1\r\nHost: 192.168.7.1:81\r\nOrigin: http://192.168.7.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n";
    assert(listener.accept(NULL,p,ERR_OK)==ERR_OK && p->nodelay);
    assert(input(p,(const uint8_t *)request,strlen(request))==ERR_OK);
    assert(written>100);
}
int main(void) {
    pads_init(); pads_set_active(true,0);
    fail_new=true; assert(!pad_ws_server_init()); fail_new=false;
    fail_bind=true; assert(!pad_ws_server_init()); fail_bind=false;
    fail_listen=true; assert(!pad_ws_server_init()); fail_listen=false;
    assert(pad_ws_server_init());
    struct tcp_pcb a=open_client(); upgrade(&a);
    const uint8_t mask[]={0x82,0x82,0,0,0,0,1,129};
    assert(input(&a,mask,sizeof(mask))==ERR_OK && pads_pressed(now)==129);
    struct tcp_pcb b=open_client(); assert(listener.accept(NULL,&b,ERR_OK)==ERR_ABRT && b.aborted);
    assert(pads_pressed(now)==129); // Rejected second client cannot release the owner.
    assert(a.recv(a.arg,&a,NULL,ERR_OK)==ERR_OK && a.closed && !pads_pressed(now));
    a=open_client(); upgrade(&a); assert(input(&a,mask,sizeof(mask))==ERR_OK);
    now=1201; assert(!pads_pressed(now)); // Mask lease is independent of TCP's longer timeout.
    now=5000; assert(a.poll(a.arg,&a)==ERR_OK && a.closed);
    a=open_client(); assert(listener.accept(NULL,&a,ERR_OK)==ERR_OK);
    now=7000; assert(a.poll(a.arg,&a)==ERR_OK && a.closed); // Incomplete handshake timeout.
    a=open_client(); upgrade(&a); assert(input(&a,mask,sizeof(mask))==ERR_OK);
    a.error(a.arg,ERR_ABRT); assert(!pads_pressed(now));
    a=open_client(); upgrade(&a);
    fail_write=fail_close=true;
    assert(input(&a,mask,sizeof(mask))==ERR_ABRT && a.aborted && !pads_pressed(now));
    fail_write=fail_close=false;
    a=open_client(); upgrade(&a);
    pads_set_active(false,now); assert(input(&a,mask,sizeof(mask))==ERR_OK && a.closed && !pads_active());
    assert(freed>=8);
    puts("Pad TCP adapter: chained buffers, no-delay writes, ownership, lease, close/error and resource failures OK");
}

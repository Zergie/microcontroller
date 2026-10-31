#include "network.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

struct fake_network_state fake_network;
static struct udp_pcb *bound[8];
struct udp_pcb *udp_new(void) { return calloc(1,sizeof(struct udp_pcb)); }
void udp_remove(struct udp_pcb *pcb) {
    for (unsigned i=0;i<8;++i) if (bound[i]==pcb) bound[i]=NULL;
    free(pcb);
}
void udp_recv(struct udp_pcb *pcb,udp_recv_fn fn,void *arg) { pcb->receive=fn;pcb->arg=arg; }
err_t udp_bind(struct udp_pcb *pcb,const ip_addr_t *addr,u16_t port) {
    (void)addr;
    for (unsigned i=0;i<8;++i) if (!bound[i]) { bound[i]=pcb;pcb->port=port;return ERR_OK; }
    return -1;
}
struct udp_pcb *fake_udp_bound(u16_t port) {
    for (unsigned i=0;i<8;++i) if (bound[i] && bound[i]->port==port) return bound[i];
    return NULL;
}
struct pbuf *pbuf_alloc(int layer,u16_t length,int type) {
    (void)layer;(void)type;struct pbuf *p=calloc(1,sizeof(*p));assert(p);
    p->payload=calloc(1,length ? length : 1);assert(p->payload);p->len=p->tot_len=length;return p;
}
void pbuf_free(struct pbuf *p) { free(p->payload);free(p); }
u16_t pbuf_copy_partial(const struct pbuf *p,void *dest,u16_t length,u16_t offset) {
    assert(offset<=p->tot_len);
    size_t size=p->tot_len-offset;if(size>length) size=length;memcpy(dest,(uint8_t *)p->payload+offset,size);return size;
}
err_t pbuf_take(struct pbuf *p,const void *data,u16_t length) {
    assert(length<=p->len);memcpy(p->payload,data,length);return ERR_OK;
}
struct netif *ip_current_input_netif(void) { return NULL; }
err_t udp_sendto(struct udp_pcb *pcb,struct pbuf *p,const ip_addr_t *addr,u16_t port) {
    (void)pcb;assert(p->tot_len<=sizeof(fake_network.response));
    ++fake_network.responses;fake_network.response_size=p->tot_len;
    if (addr) fake_network.destination=*addr;
    fake_network.port=port;
    memcpy(fake_network.response,p->payload,p->tot_len);return ERR_OK;
}
err_t udp_sendto_if(struct udp_pcb *pcb,struct pbuf *p,const ip_addr_t *addr,u16_t port,struct netif *nif) {
    (void)nif;return udp_sendto(pcb,p,addr,port);
}
void fake_udp_inject(u16_t port,const void *data,size_t size,const ip_addr_t *source,u16_t source_port) {
    struct udp_pcb *pcb=fake_udp_bound(port);assert(pcb);
    struct pbuf *p=pbuf_alloc(0,size,0);pbuf_take(p,data,size);
    pcb->receive(pcb->arg,pcb,p,source,source_port);
}

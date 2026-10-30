#ifndef TEST_UDP_H
#define TEST_UDP_H
#include "ip_addr.h"
#include <stddef.h>
typedef int err_t;
#define ERR_OK 0
#define PBUF_TRANSPORT 0
#define PBUF_RAM 0
struct netif { int unused; };
struct pbuf { void *payload; u16_t len, tot_len; };
struct udp_pcb { void *arg; void (*receive)(void *,struct udp_pcb *,struct pbuf *,const ip_addr_t *,u16_t); u16_t port; };
typedef void (*udp_recv_fn)(void *,struct udp_pcb *,struct pbuf *,const ip_addr_t *,u16_t);
struct udp_pcb *udp_new(void);
void udp_remove(struct udp_pcb *pcb);
void udp_recv(struct udp_pcb *pcb,udp_recv_fn fn,void *arg);
err_t udp_bind(struct udp_pcb *pcb,const ip_addr_t *addr,u16_t port);
struct pbuf *pbuf_alloc(int layer,u16_t length,int type);
void pbuf_free(struct pbuf *p);
u16_t pbuf_copy_partial(const struct pbuf *p,void *dest,u16_t length,u16_t offset);
err_t pbuf_take(struct pbuf *p,const void *data,u16_t length);
struct netif *ip_current_input_netif(void);
err_t udp_sendto_if(struct udp_pcb *,struct pbuf *,const ip_addr_t *,u16_t,struct netif *);
err_t udp_sendto(struct udp_pcb *,struct pbuf *,const ip_addr_t *,u16_t);
#endif

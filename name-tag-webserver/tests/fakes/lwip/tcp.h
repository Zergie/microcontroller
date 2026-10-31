#ifndef PAD_TEST_TCP_H
#define PAD_TEST_TCP_H
#include <stddef.h>
#include <stdint.h>
typedef int err_t;
typedef uint16_t u16_t;
#define ERR_OK 0
#define ERR_ABRT -1
#define ERR_MEM -2
#define TCP_WRITE_FLAG_COPY 1
#define IPADDR_TYPE_V4 0
#define IP_ANY_TYPE NULL
struct pbuf { struct pbuf *next; void *payload; u16_t len,tot_len; };
struct tcp_pcb {
    void *arg;
    err_t (*recv)(void *,struct tcp_pcb *,struct pbuf *,err_t);
    void (*error)(void *,err_t);
    err_t (*poll)(void *,struct tcp_pcb *);
    err_t (*accept)(void *,struct tcp_pcb *,err_t);
    unsigned closed,aborted,nodelay,recved;
};
struct tcp_pcb *tcp_new_ip_type(unsigned type);
err_t tcp_bind(struct tcp_pcb *pcb,const void *address,u16_t port);
struct tcp_pcb *tcp_listen_with_backlog(struct tcp_pcb *pcb,unsigned backlog);
err_t tcp_close(struct tcp_pcb *pcb);
void tcp_abort(struct tcp_pcb *pcb);
err_t tcp_write(struct tcp_pcb *pcb,const void *data,u16_t length,unsigned flags);
err_t tcp_output(struct tcp_pcb *pcb);
void pbuf_free(struct pbuf *buffer);
static inline void tcp_arg(struct tcp_pcb *p,void *arg) { p->arg=arg; }
static inline void tcp_recv(struct tcp_pcb *p,err_t (*fn)(void *,struct tcp_pcb *,struct pbuf *,err_t)) { p->recv=fn; }
static inline void tcp_err(struct tcp_pcb *p,void (*fn)(void *,err_t)) { p->error=fn; }
static inline void tcp_poll(struct tcp_pcb *p,err_t (*fn)(void *,struct tcp_pcb *),unsigned interval) { (void)interval; p->poll=fn; }
static inline void tcp_accept(struct tcp_pcb *p,err_t (*fn)(void *,struct tcp_pcb *,err_t)) { p->accept=fn; }
static inline void tcp_nagle_disable(struct tcp_pcb *p) { p->nodelay=1; }
static inline void tcp_recved(struct tcp_pcb *p,u16_t n) { p->recved+=n; }
#endif

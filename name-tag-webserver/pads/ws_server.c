#include "ws_server.h"
#include "ws_protocol.h"
#include "pads.h"
#include "lwip/tcp.h"
#include "pico/stdlib.h"
#include <string.h>

// One interactive controller, bounded storage, no allocation per mask/frame.
static struct {
    struct tcp_pcb *pcb;
    pad_ws_protocol_t protocol;
    uint64_t received_at;
} client;

static err_t disconnect(void) {
    struct tcp_pcb *pcb=client.pcb;
    if (client.protocol.upgraded) pads_release(time_us_64()/1000);
    client.pcb=NULL;
    tcp_arg(pcb,NULL); tcp_recv(pcb,NULL); tcp_err(pcb,NULL); tcp_poll(pcb,NULL,0);
    if (tcp_close(pcb)==ERR_OK) return ERR_OK;
    tcp_abort(pcb); return ERR_ABRT;
}
static bool write_bytes(void *user,const uint8_t *data,size_t length) {
    (void)user;
    if (tcp_write(client.pcb,data,(u16_t)length,TCP_WRITE_FLAG_COPY)!=ERR_OK) return false;
    return tcp_output(client.pcb)==ERR_OK;
}
static bool press(void *user,unsigned mask) {
    (void)user;
    return pads_set_pressed(mask,time_us_64()/1000);
}
static bool pulse(void *user,unsigned mask,unsigned duration_ms) {
    (void)user;
    return pads_pulse(mask,duration_ms,time_us_64()/1000);
}
static void failed(void *arg,err_t error) {
    (void)arg; (void)error;
    if (client.protocol.upgraded) pads_release(time_us_64()/1000);
    client.pcb=NULL;
}
static err_t receive(void *arg,struct tcp_pcb *pcb,struct pbuf *buffer,err_t error) {
    (void)arg;
    if (!buffer) return disconnect();
    bool valid=error==ERR_OK;
    tcp_recved(pcb,buffer->tot_len);
    client.received_at=time_us_64()/1000;
    for (struct pbuf *part=buffer;part && valid;part=part->next)
        valid=pad_ws_feed(&client.protocol,part->payload,part->len);
    pbuf_free(buffer);
    return valid ? ERR_OK : disconnect();
}
static err_t poll_client(void *arg,struct tcp_pcb *pcb) {
    (void)arg; (void)pcb;
    unsigned timeout=client.protocol.upgraded ? 5000 : 2000;
    return time_us_64()/1000-client.received_at>=timeout ? disconnect() : ERR_OK;
}
static err_t accept_client(void *arg,struct tcp_pcb *pcb,err_t error) {
    (void)arg;
    if (error!=ERR_OK || client.pcb) { tcp_abort(pcb); return ERR_ABRT; }
    memset(&client,0,sizeof(client)); client.pcb=pcb;
    client.protocol.write=write_bytes; client.protocol.press=press; client.protocol.pulse=pulse;
    client.received_at=time_us_64()/1000;
    tcp_nagle_disable(pcb);
    tcp_arg(pcb,&client); tcp_recv(pcb,receive); tcp_err(pcb,failed); tcp_poll(pcb,poll_client,1);
    return ERR_OK;
}
bool pad_ws_server_init(void) {
    struct tcp_pcb *pcb=tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb) return false;
    if (tcp_bind(pcb,IP_ANY_TYPE,81)!=ERR_OK) { tcp_close(pcb); return false; }
    struct tcp_pcb *listener=tcp_listen_with_backlog(pcb,1);
    if (!listener) { tcp_close(pcb); return false; }
    tcp_accept(listener,accept_client); return true;
}

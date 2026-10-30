#ifndef TEST_NETWORK_H
#define TEST_NETWORK_H
#include "lwip/udp.h"

struct fake_network_state {
    unsigned responses;
    uint8_t response[1024];
    size_t response_size;
    ip_addr_t destination;
    u16_t port;
};
extern struct fake_network_state fake_network;
struct udp_pcb *fake_udp_bound(u16_t port);
void fake_udp_inject(u16_t port, const void *data, size_t size, const ip_addr_t *source, u16_t source_port);
#endif

#ifndef NETWORK_STATUS_H
#define NETWORK_STATUS_H

enum network_phase {
    // Keep legacy DNS values: persisted watchdog phases from older firmware
    // must retain their numeric meaning even though the DNS responder is gone.
    NETWORK_BOOT, NETWORK_USB_START, NETWORK_DNS_START, NETWORK_DHCP_START,
    NETWORK_MDNS_START, NETWORK_HTTP_START, NETWORK_USB_POLL, NETWORK_FRAME,
    NETWORK_TIMERS, NETWORK_USB_TX, NETWORK_DNS_RX, NETWORK_DNS_TX
};

#if defined(PICO_ON_DEVICE) && PICO_ON_DEVICE
void network_status_init(void);
void network_status_mark(enum network_phase phase);
void network_status_poll(void);
int network_status_portal_enabled(void);
const char *network_status_response(void);
#else
static inline void network_status_mark(enum network_phase phase) { (void)phase; }
#endif
#endif

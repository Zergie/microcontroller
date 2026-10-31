#include "network_runtime.h"
#include "usb_network.h"
#include "network_status.h"
#include "system_control.h"
#include "dhcpserver/dhcpserver.h"
#include <lwip/apps/mdns.h>
#include <lwip/apps/httpd.h>

static dhcp_server_t dhcp;

bool network_runtime_init(const char *hostname) {
    static const ip4_addr_t address = IPADDR4_INIT_BYTES(192, 168, 7, 1);
    static const ip4_addr_t mask = IPADDR4_INIT_BYTES(255, 255, 255, 0);
    static const ip4_addr_t gateway = IPADDR4_INIT_BYTES(0, 0, 0, 0);
    network_status_init();
    network_status_mark(NETWORK_USB_START);
    if (!usb_network_init(&address, &mask, &gateway, true)) return false;
    network_status_mark(NETWORK_DHCP_START);
    dhcp_server_init(&dhcp, (ip_addr_t *)&address, (ip_addr_t *)&mask, DHCP_ADVERTISE_NONE);
    network_status_mark(NETWORK_MDNS_START);
    mdns_resp_init();
    mdns_resp_add_netif(netif_default, hostname);
    network_status_mark(NETWORK_HTTP_START);
    httpd_init();
    return true;
}

void network_runtime_poll(void) {
    usb_network_update();
    system_control_update();
    network_status_poll();
}

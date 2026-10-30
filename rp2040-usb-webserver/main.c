#include <hardware/uart.h>
#include <lwip/apps/mdns.h>
#include <lwip/apps/httpd.h>
#include <lwip/ip.h>
#include <pico/stdlib.h>
#include <stdio.h>

#include "dhcpserver/dhcpserver.h"
#include "usb_network.h"
#include "gpio_control.h"
#include "system_control.h"
#include "network_status.h"

// usb network addresses
static const ip4_addr_t ownip = IPADDR4_INIT_BYTES(192, 168, 7, 1);
static const ip4_addr_t netmask = IPADDR4_INIT_BYTES(255, 255, 255, 0);
static const ip4_addr_t gateway = IPADDR4_INIT_BYTES(0, 0, 0, 0);

int main() {
  gpio_control_init();
  network_status_init();

  // setup USB network
  network_status_mark(NETWORK_USB_START);
  if (!usb_network_init(&ownip, &netmask, &gateway, true)) {
    printf("failed to start usb network\n");
    return -1;
  }

  // setup DHCP server
  dhcp_server_t dhcp_server;
  // This USB link is a controller, not an Internet DNS server. Advertising a
  // wildcard DNS resolver can hijack host-wide lookups even without a gateway.
  network_status_mark(NETWORK_DHCP_START);
  dhcp_server_init(&dhcp_server, (ip_addr_t *)&ownip, (ip_addr_t *)&netmask, DHCP_ADVERTISE_NONE);

  // enable mDNS
  network_status_mark(NETWORK_MDNS_START);
  mdns_resp_init();
  mdns_resp_add_netif(netif_default, "rp2040");

  network_status_mark(NETWORK_HTTP_START);
  httpd_init();
  // Signal that network setup and HTTP server initialization are complete.
  gpio_control_boot_ready();

  // enter main loop
  printf("setup complete, entering main loop\n");
  while (true) {
    usb_network_update();
    system_control_update();
    network_status_poll();
  }

  printf("shutting down\n");
  mdns_resp_remove_netif(netif_default);
  dhcp_server_deinit(&dhcp_server);
  usb_network_deinit();

  return 0;
}

#include "network_status.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include <stdio.h>

#define NETWORK_MAGIC 0x4e455431u
static int portal_enabled;
static char response[384];

void network_status_init(void) {
    bool recovered = watchdog_enable_caused_reboot() && watchdog_hw->scratch[0] == NETWORK_MAGIC;
    unsigned previous = recovered ? watchdog_hw->scratch[1] : 0;
    static const char *phases[] = {"boot","usb-start","dns-start","dhcp-start",
        "mdns-start","http-start","usb-poll","frame-input","timers","usb-tx","dns-rx","dns-tx"};
    portal_enabled = !recovered;
    snprintf(response, sizeof(response),
        "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n"
        "{\"captive_enabled\":%s,\"dns_advertised\":false,\"watchdog_recovery\":%s,\"previous_phase\":\"%s\"}",
        portal_enabled ? "true" : "false", recovered ? "true" : "false",
        previous < sizeof(phases)/sizeof(phases[0]) ? phases[previous] : "unknown");
    watchdog_hw->scratch[0] = NETWORK_MAGIC;
    network_status_mark(NETWORK_BOOT);
    // A stalled USB/lwIP callback must not leave the board unreachable forever.
    watchdog_enable(8000, true);
}

void network_status_mark(enum network_phase phase) { watchdog_hw->scratch[1] = phase; }
void network_status_poll(void) { watchdog_update(); }
int network_status_portal_enabled(void) { return portal_enabled; }
const char *network_status_response(void) { return response; }

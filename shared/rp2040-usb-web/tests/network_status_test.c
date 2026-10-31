#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "network_status.h"
#include "hardware/structs/watchdog.h"

static struct test_watchdog_hw registers;
struct test_watchdog_hw *watchdog_hw=&registers;
static bool rebooted;
static unsigned updates;
bool watchdog_enable_caused_reboot(void) { return rebooted; }
void watchdog_enable(uint32_t delay,bool pause) { assert(delay==8000 && pause); }
void watchdog_update(void) { ++updates; }

int main(void) {
    network_status_init();
    assert(network_status_portal_enabled());
    assert(strstr(network_status_response(),"\"watchdog_recovery\":false"));
    assert(strstr(network_status_response(),"\"dns_advertised\":false"));
    network_status_mark(NETWORK_DNS_TX);
    rebooted=true;
    network_status_init();
    assert(!network_status_portal_enabled());
    assert(strstr(network_status_response(),"\"previous_phase\":\"dns-tx\""));
    assert(strstr(network_status_response(),"\"watchdog_recovery\":true"));
    network_status_poll();assert(updates==1);
    rebooted=false; // Normal boot/BOOTSEL does not lock the portal out.
    network_status_init();
    assert(network_status_portal_enabled());
    assert(strstr(network_status_response(),"\"watchdog_recovery\":false"));
    puts("Network recovery: watchdog fallback, persisted phase, normal re-enable and heartbeat OK");
}

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "network_status.h"

#define __unused __attribute__((unused))
typedef int err_t;
enum { ERR_OK=0, ERR_USE=-1, ERR_MEM=-2 };
struct netif { int unused; };
struct pbuf { uint16_t tot_len; };
static bool ready, writable;
static unsigned writes;
static bool tud_ready(void) { return ready; }
static bool tud_network_can_xmit(uint16_t length) { assert(length==100);return writable; }
static void tud_network_xmit(void *p,uint16_t arg) { assert(p && arg==0);++writes; }
static void __unused tud_task(void) {
    fputs("FAIL: USB backpressure enters a nested wait instead of returning to the main loop\n",stderr);
    assert(false);
}

// Generated verbatim from usb_network.c by run_native.py, not a second copy.
#include "usb_tx_under_test.h"

int main(void) {
    struct pbuf packet={100};
    assert(tud_output(NULL,&packet)==ERR_USE && writes==0);
    ready=true;
    assert(tud_output(NULL,&packet)==ERR_MEM && writes==0);
    writable=true;
    assert(tud_output(NULL,&packet)==ERR_OK && writes==1);
    puts("USB send backpressure: disconnected/busy return immediately, ready packet sent OK");
}

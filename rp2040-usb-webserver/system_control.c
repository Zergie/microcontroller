#include "system_control.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"

static bool bootsel_pending;
static absolute_time_t bootsel_deadline;

void system_request_bootsel(void) {
    if (bootsel_pending) return;
    // Keep servicing USB/lwIP so the HTTP acknowledgement can leave the device.
    bootsel_deadline = make_timeout_time_ms(1500);
    bootsel_pending = true;
}

void system_control_update(void) {
    if (bootsel_pending && time_reached(bootsel_deadline)) {
        // Same interfaces as a physical BOOTSEL boot: mass storage and PICOBOOT.
        reset_usb_boot(0, 0);
    }
}

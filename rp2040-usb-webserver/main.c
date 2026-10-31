#include "gpio_control.h"
#include "network_runtime.h"

int main(void) {
    gpio_control_init();
    if (!network_runtime_init("rp2040")) return -1;
    gpio_control_boot_ready();
    while (true) network_runtime_poll();
}

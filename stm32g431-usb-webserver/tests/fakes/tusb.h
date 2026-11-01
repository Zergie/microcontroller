#ifndef TEST_TUSB_H
#define TEST_TUSB_H
#include <stdbool.h>
#include <stdint.h>
#include "tusb_config.h"
#define TUSB_ROLE_DEVICE 1
#define TUSB_SPEED_FULL 1
typedef struct { int role; int speed; } tusb_rhport_init_t;
bool tusb_init(uint8_t port, const tusb_rhport_init_t *init);
bool tud_ready(void);
void tud_task(void);
bool tud_network_can_xmit(uint16_t size);
void tud_network_xmit(void *ref, uint16_t arg);
void tud_network_recv_renew(void);
#endif

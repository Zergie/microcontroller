#ifndef BOARD_H
#define BOARD_H
#include <stdint.h>
void board_init(void);
void board_reset_to_dfu(void);
uint32_t sys_now(void);
void platform_panic(void) __attribute__((noreturn));
#endif

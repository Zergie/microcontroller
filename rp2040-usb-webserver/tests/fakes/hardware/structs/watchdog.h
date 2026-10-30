#ifndef TEST_WATCHDOG_STRUCTS_H
#define TEST_WATCHDOG_STRUCTS_H
#include <stdint.h>
struct test_watchdog_hw { uint32_t scratch[8]; };
extern struct test_watchdog_hw *watchdog_hw;
#endif

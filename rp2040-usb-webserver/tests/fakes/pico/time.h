#ifndef TEST_TIME_H
#define TEST_TIME_H
#include <stdint.h>
static inline uint64_t get_absolute_time(void) { return 1000; }
static inline uint32_t to_ms_since_boot(uint64_t value) { return value; }
#endif

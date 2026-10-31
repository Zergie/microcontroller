#include "pixel_sdk.h"
#define ws2812_T1 3
#define ws2812_T2 3
#define ws2812_T3 4
static const int ws2812_program = 1;
static inline pio_sm_config ws2812_program_get_default_config(unsigned offset) {
    assert(offset == 0); return (pio_sm_config){0};
}

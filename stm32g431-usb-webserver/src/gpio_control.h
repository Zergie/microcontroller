#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define GPIO_COUNT 41
typedef struct {
    uint8_t mode, pull, value, fast, duty;
    uint32_t frequency;
} gpio_state_t;
extern gpio_state_t gpio_states[GPIO_COUNT];
bool gpio_available(unsigned pin);
bool gpio_level(unsigned pin);
int gpio_apply(unsigned pin, gpio_state_t state);
#endif

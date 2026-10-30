#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

enum pin_mode { PIN_INPUT, PIN_OUTPUT, PIN_PWM, PIN_ANALOG, PIN_OPEN_DRAIN };
enum pin_pull { PULL_NONE, PULL_UP, PULL_DOWN, PULL_KEEPER };
enum pad_setting { PAD_KEEP, PAD_DRIVE, PAD_HYSTERESIS };
enum gpio_result { GPIO_OK, GPIO_INVALID, GPIO_RESERVED, GPIO_CONFLICT };

struct gpio_config {
    unsigned pin;
    enum pin_mode mode;
    enum pin_pull pull;
    bool level; // Output HIGH, or Open Drain released (Hi-Z).
    unsigned frequency, duty;
    enum pad_setting pad;
    unsigned drive_ma;
    bool fast_slew, hysteresis;
};

struct gpio_snapshot {
    bool available;
    const char *reason;
    enum pin_mode mode;
    enum pin_pull pull;
    bool value;
    uint32_t frequency, actual_frequency;
    unsigned duty, slice, channel;
    unsigned drive_ma;
    bool fast_slew, hysteresis;
    unsigned adc_raw, millivolts;
    bool output_enabled, output_latch;
};

bool gpio_control_available(unsigned pin);
void gpio_control_init(void);
void gpio_control_boot_ready(void);
bool gpio_control_led_on(void);
// Validates the complete configuration before any hardware writes.
enum gpio_result gpio_control_apply(const struct gpio_config *config);
void gpio_control_read(unsigned pin, struct gpio_snapshot *snapshot);

#endif

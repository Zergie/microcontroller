#ifndef TEST_SDK_H
#define TEST_SDK_H
#include <stdbool.h>
#include <stdint.h>

#define PICO_DEFAULT_LED_PIN 25
#define GPIO_IN false
#define GPIO_OUT true
#define GPIO_FUNC_SIO 5
#define GPIO_FUNC_PWM 4
#define clk_sys 0
enum gpio_drive_strength { GPIO_DRIVE_STRENGTH_2MA, GPIO_DRIVE_STRENGTH_4MA, GPIO_DRIVE_STRENGTH_8MA, GPIO_DRIVE_STRENGTH_12MA };
enum gpio_slew_rate { GPIO_SLEW_RATE_SLOW, GPIO_SLEW_RATE_FAST };
struct fake_pin { unsigned function; bool output, latch, input, up, down, hysteresis; enum gpio_drive_strength drive; enum gpio_slew_rate slew; };
extern struct fake_pin fake_pins[30];
typedef struct { unsigned div, frac, wrap; } pwm_config;
struct fake_pwm { pwm_config config; unsigned level[2]; bool enabled; };
extern struct fake_pwm fake_pwms[8];
enum fake_event_kind {
    EVENT_GPIO_INIT, EVENT_DIR, EVENT_LATCH, EVENT_FUNCTION, EVENT_INPUT,
    EVENT_PULLS, EVENT_DRIVE, EVENT_SLEW, EVENT_HYSTERESIS,
    EVENT_PWM_LEVEL, EVENT_PWM_ENABLED, EVENT_PWM_INIT, EVENT_PWM_BOTH,
    EVENT_ADC_INIT, EVENT_ADC_GPIO
};
struct fake_event { enum fake_event_kind kind; unsigned target, a, b, c; };
#define FAKE_EVENT_CAPACITY 4096
extern struct fake_event fake_events[FAKE_EVENT_CAPACITY];
extern unsigned fake_event_count;
void fake_clear_events(void);
void gpio_init(unsigned pin);
void gpio_set_dir(unsigned pin, bool output);
void gpio_put(unsigned pin, bool value);
void gpio_set_function(unsigned pin, unsigned function);
void gpio_set_input_enabled(unsigned pin, bool enabled);
void gpio_disable_pulls(unsigned pin);
void gpio_set_pulls(unsigned pin, bool up, bool down);
bool gpio_get_out_level(unsigned pin);
bool gpio_get(unsigned pin);
bool gpio_is_dir_out(unsigned pin);
bool gpio_is_pulled_up(unsigned pin);
bool gpio_is_pulled_down(unsigned pin);
void gpio_set_drive_strength(unsigned pin, enum gpio_drive_strength strength);
enum gpio_drive_strength gpio_get_drive_strength(unsigned pin);
void gpio_set_slew_rate(unsigned pin, enum gpio_slew_rate slew);
enum gpio_slew_rate gpio_get_slew_rate(unsigned pin);
void gpio_set_input_hysteresis_enabled(unsigned pin, bool enabled);
bool gpio_is_input_hysteresis_enabled(unsigned pin);
unsigned pwm_gpio_to_slice_num(unsigned pin);
unsigned pwm_gpio_to_channel(unsigned pin);
void pwm_set_chan_level(unsigned slice, unsigned channel, unsigned value);
void pwm_set_enabled(unsigned slice, bool enabled);
pwm_config pwm_get_default_config(void);
void pwm_config_set_clkdiv_int_frac(pwm_config *config, unsigned div, unsigned frac);
void pwm_config_set_wrap(pwm_config *config, unsigned wrap);
void pwm_init(unsigned slice, const pwm_config *config, bool enabled);
void pwm_set_both_levels(unsigned slice, unsigned a, unsigned b);
unsigned clock_get_hz(unsigned clock);
void adc_init(void);
void adc_gpio_init(unsigned pin);
void adc_select_input(unsigned input);
unsigned adc_read(void);
#endif

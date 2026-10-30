#include "gpio_control.h"

#include <stdint.h>
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

struct pin_state {
    enum pin_mode mode;
    uint32_t frequency;
    uint32_t actual_frequency;
    uint8_t duty;
};
static struct pin_state pins[30];

bool gpio_control_available(unsigned pin) {
    // Pico GPIO23: SMPS, GPIO24: VBUS sense, GPIO29: VSYS sense.
    return pin < 30 && pin != 23 && pin != 24 && pin != 29;
}

static void release_pwm(unsigned pin) {
    if (pins[pin].mode != PIN_PWM) return;
    unsigned slice = pwm_gpio_to_slice_num(pin);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0);
    bool other = false;
    for (unsigned i = 0; i < 30; ++i) {
        if (i != pin && pins[i].mode == PIN_PWM && pwm_gpio_to_slice_num(i) == slice) other = true;
    }
    if (!other) pwm_set_enabled(slice, false);
}

static void prepare_digital(unsigned pin, bool latch) {
    release_pwm(pin);
    // Release the driver before switching the function, then preload the latch.
    gpio_set_dir(pin, GPIO_IN);
    gpio_put(pin, latch);
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_input_enabled(pin, true);
    gpio_disable_pulls(pin);
}

// Boot initialization uses the same non-driving foundation as API transitions.
static void set_digital(unsigned pin, bool output, bool value) {
    prepare_digital(pin, value);
    gpio_set_dir(pin, output ? GPIO_OUT : GPIO_IN);
    pins[pin] = (struct pin_state){.mode = output ? PIN_OUTPUT : PIN_INPUT};
}

struct pwm_plan {
    unsigned slice, channel, div16, period, actual_frequency;
    bool shared;
};

static enum gpio_result prepare_pwm(unsigned pin, unsigned frequency, unsigned duty, struct pwm_plan *plan) {
    if (frequency < 10 || frequency > 100000 || duty > 100) return GPIO_INVALID;
    unsigned slice = pwm_gpio_to_slice_num(pin);
    unsigned channel = pwm_gpio_to_channel(pin);
    bool shared = false;
    for (unsigned i = 0; i < 30; ++i) {
        if (i == pin || pins[i].mode != PIN_PWM || pwm_gpio_to_slice_num(i) != slice) continue;
        if (pwm_gpio_to_channel(i) == channel || pins[i].frequency != frequency) return GPIO_CONFLICT;
        shared = true;
    }
    uint64_t scaled_clock = (uint64_t)clock_get_hz(clk_sys) * 16;
    unsigned div16 = (scaled_clock + (uint64_t)frequency * 65535 - 1) / ((uint64_t)frequency * 65535);
    if (div16 < 16) div16 = 16;
    if (div16 > 4095) return GPIO_INVALID;
    uint64_t denominator = (uint64_t)frequency * div16;
    unsigned period = (scaled_clock + denominator / 2) / denominator;
    if (period < 2 || period > 65535) return GPIO_INVALID;
    *plan = (struct pwm_plan){slice, channel, div16, period,
        (scaled_clock + (uint64_t)div16 * period / 2) / ((uint64_t)div16 * period), shared};
    return GPIO_OK;
}

static void enable_pwm(const struct gpio_config *request, const struct pwm_plan *plan) {
    if (!plan->shared) {
        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv_int_frac(&config, plan->div16 >> 4, plan->div16 & 15);
        pwm_config_set_wrap(&config, plan->period - 1);
        pwm_init(plan->slice, &config, false);
        pwm_set_both_levels(plan->slice, 0, 0);
    }
    pwm_set_chan_level(plan->slice, plan->channel, (plan->period * request->duty + 50) / 100);
    gpio_set_function(request->pin, GPIO_FUNC_PWM);
    pwm_set_enabled(plan->slice, true);
}

void gpio_control_init(void) {
    adc_init();
    for (unsigned pin = 0; pin < 30; ++pin) {
        if (gpio_control_available(pin)) set_digital(pin, false, false);
    }
    gpio_init(PICO_DEFAULT_LED_PIN);
    set_digital(PICO_DEFAULT_LED_PIN, true, false);
}

void gpio_control_boot_ready(void) {
    set_digital(PICO_DEFAULT_LED_PIN, true, true);
}

bool gpio_control_led_on(void) {
    return gpio_get_out_level(PICO_DEFAULT_LED_PIN);
}

enum gpio_result gpio_control_apply(const struct gpio_config *config) {
    unsigned pin = config->pin;
    if (pin >= 30) return GPIO_INVALID;
    if (!gpio_control_available(pin)) return GPIO_RESERVED;
    if (config->mode < PIN_INPUT || config->mode > PIN_OPEN_DRAIN ||
        config->pull < PULL_NONE || config->pull > PULL_KEEPER ||
        config->pad < PAD_KEEP || config->pad > PAD_HYSTERESIS) return GPIO_INVALID;
    if (config->mode != PIN_INPUT && config->pull != PULL_NONE) return GPIO_INVALID;
    if (config->mode == PIN_ANALOG && (pin < 26 || pin > 28)) return GPIO_INVALID;
    bool output = config->mode == PIN_OUTPUT || config->mode == PIN_PWM || config->mode == PIN_OPEN_DRAIN;
    if (config->pad == PAD_DRIVE && (!output ||
        (config->drive_ma != 2 && config->drive_ma != 4 && config->drive_ma != 8 && config->drive_ma != 12))) return GPIO_INVALID;
    if (config->pad == PAD_HYSTERESIS && config->mode != PIN_INPUT) return GPIO_INVALID;
    struct pwm_plan plan;
    if (config->mode == PIN_PWM) {
        enum gpio_result result = prepare_pwm(pin, config->frequency, config->duty, &plan);
        if (result != GPIO_OK) return result;
    }
    // Disconnect the old signal before applying pads; only enable the new mode
    // after all validation and pad settings are complete. No failure past here.
    prepare_digital(pin, config->mode == PIN_OUTPUT && config->level);
    if (config->pad == PAD_DRIVE) {
        enum gpio_drive_strength strength = config->drive_ma == 2 ? GPIO_DRIVE_STRENGTH_2MA :
            config->drive_ma == 4 ? GPIO_DRIVE_STRENGTH_4MA :
            config->drive_ma == 8 ? GPIO_DRIVE_STRENGTH_8MA : GPIO_DRIVE_STRENGTH_12MA;
        gpio_set_drive_strength(pin, strength);
        gpio_set_slew_rate(pin, config->fast_slew ? GPIO_SLEW_RATE_FAST : GPIO_SLEW_RATE_SLOW);
    } else if (config->pad == PAD_HYSTERESIS) gpio_set_input_hysteresis_enabled(pin, config->hysteresis);
    switch (config->mode) {
        case PIN_INPUT:
            gpio_set_pulls(pin, config->pull == PULL_UP || config->pull == PULL_KEEPER,
                config->pull == PULL_DOWN || config->pull == PULL_KEEPER);
            break;
        case PIN_OUTPUT: gpio_set_dir(pin, GPIO_OUT); break;
        case PIN_PWM: enable_pwm(config, &plan); break;
        case PIN_ANALOG:
            adc_gpio_init(pin);
            break;
        case PIN_OPEN_DRAIN:
            // The foundation always preloads LOW; release never drives HIGH.
            gpio_set_dir(pin, config->level ? GPIO_IN : GPIO_OUT);
            break;
    }
    pins[pin] = (struct pin_state){.mode = config->mode};
    if (config->mode == PIN_PWM) {
        pins[pin].frequency = config->frequency;
        pins[pin].actual_frequency = plan.actual_frequency;
        pins[pin].duty = config->duty;
    }
    return GPIO_OK;
}

void gpio_control_read(unsigned pin, struct gpio_snapshot *snapshot) {
    const struct pin_state *state = &pins[pin];
    bool accessible = gpio_control_available(pin);
    bool up = accessible && gpio_is_pulled_up(pin);
    bool down = accessible && gpio_is_pulled_down(pin);
    static const unsigned drive_milliamps[] = {2, 4, 8, 12};
    *snapshot = (struct gpio_snapshot){
        .available = accessible,
        .reason = pin == 23 ? "SMPS" : pin == 24 ? "VBUS" : pin == 29 ? "VSYS" : "",
        .mode = state->mode, .pull = up && down ? PULL_KEEPER : up ? PULL_UP : down ? PULL_DOWN : PULL_NONE,
        .value = accessible && (state->mode == PIN_OUTPUT ? gpio_get_out_level(pin) : gpio_get(pin)),
        .frequency = state->frequency, .actual_frequency = state->actual_frequency, .duty = state->duty,
        .slice = pwm_gpio_to_slice_num(pin), .channel = pwm_gpio_to_channel(pin),
        .drive_ma = drive_milliamps[gpio_get_drive_strength(pin)],
        .fast_slew = gpio_get_slew_rate(pin) == GPIO_SLEW_RATE_FAST,
        .hysteresis = gpio_is_input_hysteresis_enabled(pin),
        .output_enabled = gpio_is_dir_out(pin), .output_latch = gpio_get_out_level(pin)
    };
    if (state->mode == PIN_ANALOG) {
        adc_select_input(pin - 26);
        snapshot->adc_raw = adc_read();
        snapshot->millivolts = (snapshot->adc_raw * 3300 + 2048) / 4096;
    }
}

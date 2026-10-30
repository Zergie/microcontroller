#include "gpio_api.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static bool number(const char **text, unsigned *value) {
    const char *p = *text;
    if (*p < '0' || *p > '9') return false;
    unsigned n = 0;
    do {
        unsigned digit = *p++ - '0';
        if (n > (UINT32_MAX - digit) / 10) return false;
        n = n * 10 + digit;
    } while (*p >= '0' && *p <= '9');
    *text = p;
    *value = n;
    return true;
}

// Match whole path components, never a partial command such as "onward".
static bool token(const char **text, const char *word) {
    size_t length = strlen(word);
    if (strncmp(*text, word, length) != 0 || ((*text)[length] && (*text)[length] != '/')) return false;
    *text += length;
    return true;
}

int gpio_api_parse(const char *uri, struct gpio_config *config, bool *led_command) {
    *config = (struct gpio_config){0};
    *led_command = false;
    if (strcmp(uri, "/api/led/on") == 0 || strcmp(uri, "/api/led/off") == 0) {
        // This API targets the ordinary Pico; the onboard LED is GP25.
        *config = (struct gpio_config){.pin = 25, .mode = PIN_OUTPUT, .level = strcmp(uri, "/api/led/on") == 0};
        *led_command = true;
        return 200;
    }
    const char prefix[] = "/api/gpio/";
    if (strncmp(uri, prefix, sizeof(prefix) - 1) != 0) return 404;
    const char *p = uri + sizeof(prefix) - 1;
    if (!number(&p, &config->pin) || *p != '/' || config->pin >= 30) return 400;
    ++p;
    if (!gpio_control_available(config->pin)) return 403;
    if (token(&p, "input")) config->mode = PIN_INPUT;
    else if (token(&p, "input-pullup")) config->pull = PULL_UP;
    else if (token(&p, "input-pulldown")) config->pull = PULL_DOWN;
    else if (token(&p, "input-buskeeper")) config->pull = PULL_KEEPER;
    else if (token(&p, "on")) { config->mode = PIN_OUTPUT; config->level = true; }
    else if (token(&p, "off")) config->mode = PIN_OUTPUT;
    else if (token(&p, "analog")) config->mode = PIN_ANALOG;
    else if (token(&p, "open-drain")) {
        config->mode = PIN_OPEN_DRAIN;
        if (*p != '/') return 404;
        ++p;
        if (token(&p, "release")) config->level = true;
        else if (!token(&p, "low")) return 404;
    } else if (token(&p, "pwm")) {
        config->mode = PIN_PWM;
        if (*p != '/') return 400;
        ++p;
        if (!number(&p, &config->frequency) || *p != '/') return 400;
        ++p;
        if (!number(&p, &config->duty)) return 400;
    } else return 404;
    if (*p == '\0') return 200;
    if (*p != '/') return 400;
    ++p;
    if (token(&p, "drive")) {
        config->pad = PAD_DRIVE;
        if (*p != '/') return 400;
        ++p;
        if (!number(&p, &config->drive_ma) || *p != '/') return 400;
        ++p;
        if (token(&p, "fast")) config->fast_slew = true;
        else if (!token(&p, "slow")) return 400;
    } else if (token(&p, "hysteresis")) {
        config->pad = PAD_HYSTERESIS;
        if (*p != '/') return 400;
        ++p;
        if (token(&p, "on")) config->hysteresis = true;
        else if (!token(&p, "off")) return 400;
    } else return 400;
    return *p == '\0' ? 200 : 400;
}

int gpio_api_command(const char *uri, bool *led_command) {
    struct gpio_config config;
    int status = gpio_api_parse(uri, &config, led_command);
    if (status != 200) return status;
    switch (gpio_control_apply(&config)) {
        case GPIO_OK: return 200;
        case GPIO_INVALID: return 400;
        case GPIO_RESERVED: return 403;
        case GPIO_CONFLICT: return 409;
    }
    return 503;
}

struct json_writer {
    char *buffer;
    size_t capacity;
    size_t used;
    bool failed;
};

static void appendf(struct json_writer *writer, const char *format, ...) {
    if (writer->failed) return;
    va_list arguments;
    va_start(arguments, format);
    int length = vsnprintf(writer->buffer + writer->used,
        writer->capacity - writer->used, format, arguments);
    va_end(arguments);
    if (length < 0 || (size_t)length >= writer->capacity - writer->used) {
        writer->failed = true;
        return;
    }
    writer->used += length;
}

bool gpio_api_snapshot(char *buffer, size_t capacity) {
    struct json_writer writer = {.buffer = buffer, .capacity = capacity};
    static const char *modes[] = {"input", "output", "pwm", "analog", "open-drain"};
    static const char *pulls[] = {"none", "up", "down", "keeper"};
    appendf(&writer, "{\"pins\":[");
    for (unsigned pin = 0; pin < 30 && !writer.failed; ++pin) {
        struct gpio_snapshot state;
        gpio_control_read(pin, &state);
        appendf(&writer, "%s{\"pin\":%u,\"available\":%s,\"reason\":\"%s\",",
            pin ? "," : "", pin, state.available ? "true" : "false", state.reason);
        appendf(&writer, "\"mode\":\"%s\",\"pull\":\"%s\",\"value\":%s,",
            modes[state.mode], pulls[state.pull],
            state.mode == PIN_ANALOG ? "null" : state.value ? "true" : "false");
        appendf(&writer, "\"frequency_hz\":%lu,\"actual_frequency_hz\":%lu,\"duty_percent\":%u,",
            (unsigned long)state.frequency, (unsigned long)state.actual_frequency, state.duty);
        appendf(&writer, "\"slice\":%u,\"channel\":\"%c\",", state.slice, 'A' + state.channel);
        appendf(&writer, "\"drive_strength_ma\":%u,\"slew_rate\":\"%s\",\"hysteresis\":%s",
            state.drive_ma, state.fast_slew ? "fast" : "slow", state.hysteresis ? "true" : "false");
        if (state.mode == PIN_ANALOG) {
            appendf(&writer, ",\"adc_raw\":%u,\"millivolts\":%u", state.adc_raw, state.millivolts);
        } else if (state.mode == PIN_OPEN_DRAIN) {
            appendf(&writer, ",\"output_enabled\":%s,\"output_latch\":%s",
                state.output_enabled ? "true" : "false", state.output_latch ? "true" : "false");
        }
        appendf(&writer, "}");
    }
    appendf(&writer, "]}");
    return !writer.failed;
}

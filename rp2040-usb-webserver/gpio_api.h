#ifndef GPIO_API_H
#define GPIO_API_H

#include <stdbool.h>
#include <stddef.h>
#include "gpio_control.h"

// Parsing and HTTP/JSON compatibility live here, independently of hardware I/O.
int gpio_api_parse(const char *uri, struct gpio_config *config, bool *led_command);
int gpio_api_command(const char *uri, bool *led_command);
bool gpio_api_snapshot(char *buffer, size_t capacity);

#endif

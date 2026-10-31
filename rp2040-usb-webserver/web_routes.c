#include <string.h>
#include "lwip/apps/fs.h"
#include "gpio_control.h"
#include "gpio_api.h"
#include "web_http.h"

#define SNAPSHOT_CAPACITY 12288

static const struct {
    const char *uri;
    const char *data;
} errors[] = {
    {"/_api/400", "HTTP/1.0 400 Bad Request\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"Invalid command: pin 0-29, PWM 10-100000 Hz and 0-100 percent; body must be empty.\"}"},
    {"/_api/403", "HTTP/1.0 403 Forbidden\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"This pin is reserved for Pico board hardware.\"}"},
    {"/_api/404", "HTTP/1.0 404 Not Found\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"Unknown API command.\"}"},
    {"/_api/409", "HTTP/1.0 409 Conflict\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"PWM conflict: slice frequencies must match; a channel can only be assigned once.\"}"},
};
#define ERROR_COUNT (sizeof(errors) / sizeof(errors[0]))

static const char led_on[] = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nContent-Length: 11\r\n\r\n{\"on\":true}";
static const char led_off[] = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nContent-Length: 12\r\n\r\n{\"on\":false}";
static const char *error_response(const char *uri) {
    for (size_t i = 0; i < ERROR_COUNT; ++i) {
        if (strcmp(errors[i].uri, uri) == 0) return errors[i].data;
    }
    return NULL;
}

int app_http_open(struct fs_file *file, const char *uri) {
    if (strcmp(uri, "/api/gpio.json") == 0)
        return web_http_json(file, SNAPSHOT_CAPACITY, gpio_api_snapshot);
    if (strcmp(uri, "/api/led.json") == 0)
        return web_http_text(file, gpio_control_led_on() ? led_on : led_off);
    const char *error = error_response(uri);
    return error ? web_http_text(file, error) : 0;
}

int app_http_command(const char *uri, const char **response_uri) {
    bool led_command;
    int status = gpio_api_command(uri, &led_command);
    if (status == 200) *response_uri = led_command ? "/api/led.json" : "/api/gpio.json";
    return status;
}

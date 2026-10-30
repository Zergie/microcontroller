#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lwip/apps/fs.h"
#include "lwip/apps/httpd.h"
#include "gpio_control.h"
#include "gpio_api.h"
#include "system_control.h"
#include "web_assets.h"
#include "captive_portal.h"
#include "network_status.h"

#define SNAPSHOT_CAPACITY 12288

// The last entry is the fallback for unknown errors and allocation failures.
static const struct {
    int status;
    const char *uri;
    const char *data;
} errors[] = {
    {400, "/_api/400", "HTTP/1.0 400 Bad Request\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"Invalid command: pin 0-29, PWM 10-100000 Hz and 0-100 percent; body must be empty.\"}"},
    {403, "/_api/403", "HTTP/1.0 403 Forbidden\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"This pin is reserved for Pico board hardware.\"}"},
    {404, "/_api/404", "HTTP/1.0 404 Not Found\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"Unknown API command.\"}"},
    {409, "/_api/409", "HTTP/1.0 409 Conflict\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"PWM conflict: slice frequencies must match; a channel can only be assigned once.\"}"},
    {503, "/_api/503", "HTTP/1.0 503 Service Unavailable\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"No free connections or memory.\"}"},
};
#define ERROR_COUNT (sizeof(errors) / sizeof(errors[0]))

static const char led_on[] = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nContent-Length: 11\r\n\r\n{\"on\":true}";
static const char led_off[] = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nContent-Length: 12\r\n\r\n{\"on\":false}";
static const char bootsel_accepted[] = "HTTP/1.0 202 Accepted\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n{\"status\":\"rebooting\",\"mode\":\"bootsel\",\"delay_ms\":1500}";

// Response choice belongs to the connection, never to a global 'last command'.
static struct {
    void *connection;
    const char *response;
} posts[8];

static const char *error_uri(int status) {
    for (size_t i = 0; i < ERROR_COUNT; ++i) {
        if (errors[i].status == status) return errors[i].uri;
    }
    return errors[ERROR_COUNT - 1].uri;
}

static const char *error_response(const char *uri) {
    for (size_t i = 0; i < ERROR_COUNT; ++i) {
        if (strcmp(errors[i].uri, uri) == 0) return errors[i].data;
    }
    return NULL;
}

static int finish_file(struct fs_file *file, const char *data, size_t length) {
    file->data = data;
    file->len = length;
    file->index = file->len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return 1;
}

static const char *gpio_snapshot_response(struct fs_file *file) {
    // A response owns its snapshot until fs_close_custom. The lwIP TCP copy
    // policy in lwipopts.h remains necessary even after the file is closed.
    const char header[] = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n";
    char *snapshot = malloc(sizeof(header) + SNAPSHOT_CAPACITY);
    if (snapshot) {
        memcpy(snapshot, header, sizeof(header) - 1);
        if (gpio_api_snapshot(snapshot + sizeof(header) - 1, SNAPSHOT_CAPACITY)) {
            file->pextension = snapshot;
            return snapshot;
        }
        free(snapshot);
    }
    return errors[ERROR_COUNT - 1].data;
}

int fs_open_custom(struct fs_file *file, const char *name) {
    file->pextension = NULL;
    const char *data = NULL;
    if ((data = network_status_portal_enabled() ? captive_portal_redirect(name) : NULL) != NULL) {
        // A probe redirect opens the canonical origin, where assets and API
        // requests remain same-origin in the captive browser.
    } else if (strcmp(name, "/api/system/network.json") == 0) {
        data = network_status_response();
    } else if (strcmp(name, "/api/gpio.json") == 0) {
        data = gpio_snapshot_response(file);
    } else if (strcmp(name, "/api/led.json") == 0) {
        data = gpio_control_led_on() ? led_on : led_off;
    } else if (strcmp(name, "/_api/bootsel-accepted") == 0) {
        data = bootsel_accepted;
    } else if ((data = error_response(name)) != NULL) {
        // Internal error response selected for this POST connection.
    } else {
        if (strcmp(name, "/") == 0) name = "/index.html";
        for (size_t i = 0; i < sizeof(web_assets) / sizeof(web_assets[0]); ++i) {
            if (strcmp(name, web_assets[i].name) == 0) {
                return finish_file(file, (const char *)web_assets[i].data, web_assets[i].length);
            }
        }
        return 0;
    }
    return finish_file(file, data, strlen(data));
}

void fs_close_custom(struct fs_file *file) {
    free(file->pextension);
    file->pextension = NULL;
}

err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                      u16_t http_request_len, int content_len, char *response_uri,
                      u16_t response_uri_len, u8_t *post_auto_wnd) {
    (void)http_request;
    (void)http_request_len;
    (void)post_auto_wnd;
    int status = 400;
    if (content_len == 0) {
        unsigned slot;
        for (slot = 0; slot < 8 && posts[slot].connection; ++slot) {}
        if (slot == 8) status = 503;
        else {
            if (strcmp(uri, "/api/system/bootsel") == 0) {
                system_request_bootsel();
                posts[slot].connection = connection;
                posts[slot].response = "/_api/bootsel-accepted";
                return ERR_OK;
            }
            bool led_command;
            status = gpio_api_command(uri, &led_command);
            if (status == 200) {
                posts[slot].connection = connection;
                posts[slot].response = led_command ? "/api/led.json" : "/api/gpio.json";
                return ERR_OK;
            }
        }
    }
    snprintf(response_uri, response_uri_len, "%s", error_uri(status));
    return ERR_ARG;
}

err_t httpd_post_receive_data(void *connection, struct pbuf *p) {
    (void)connection;
    bool empty = p->tot_len == 0;
    pbuf_free(p);
    return empty ? ERR_OK : ERR_ARG;
}

void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len) {
    for (unsigned i = 0; i < 8; ++i) {
        if (posts[i].connection == connection) {
            snprintf(response_uri, response_uri_len, "%s", posts[i].response);
            posts[i].connection = NULL;
            return;
        }
    }
}

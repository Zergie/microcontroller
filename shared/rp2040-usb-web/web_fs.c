#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lwip/apps/httpd.h"
#include "web_http.h"
#include "web_assets.h"
#include "captive_portal.h"
#include "network_status.h"
#include "system_control.h"

static const char unavailable[] = "HTTP/1.0 503 Service Unavailable\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"No free connections or memory.\"}";
static const char invalid[] = "HTTP/1.0 400 Bad Request\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"Invalid command; body must be empty.\"}";
static const char not_found[] = "HTTP/1.0 404 Not Found\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"error\":\"Unknown API command.\"}";
static const char bootsel[] = "HTTP/1.0 202 Accepted\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n{\"status\":\"rebooting\",\"mode\":\"bootsel\",\"delay_ms\":1500}";

// Each pending POST owns its response choice. No global last-command state.
static struct { void *connection; const char *response; } posts[8];

static int finish_file(struct fs_file *file, const char *data, size_t length) {
    file->data = data;
    file->len = length;
    file->index = file->len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return 1;
}

int web_http_text(struct fs_file *file, const char *response) {
    return finish_file(file, response, strlen(response));
}

int web_http_json(struct fs_file *file, size_t capacity, bool (*write)(char *, size_t)) {
    static const char header[] = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n";
    char *response = malloc(sizeof(header) + capacity);
    if (response) {
        memcpy(response, header, sizeof(header) - 1);
        if (write(response + sizeof(header) - 1, capacity)) {
            file->pextension = response;
            return web_http_text(file, response);
        }
        free(response);
    }
    return web_http_text(file, unavailable);
}

int fs_open_custom(struct fs_file *file, const char *uri) {
    file->pextension = NULL;
    const char *redirect = network_status_portal_enabled() ? captive_portal_redirect(uri) : NULL;
    if (redirect) return web_http_text(file, redirect);
    if (strcmp(uri, "/api/system/network.json") == 0)
        return web_http_text(file, network_status_response());
    if (strcmp(uri, "/_api/bootsel-accepted") == 0) return web_http_text(file, bootsel);
    if (strcmp(uri, "/_api/503") == 0) return web_http_text(file, unavailable);
    if (app_http_open(file, uri)) return 1;
    if (strcmp(uri, "/_api/400") == 0) return web_http_text(file, invalid);
    if (strcmp(uri, "/_api/404") == 0) return web_http_text(file, not_found);
    if (strcmp(uri, "/") == 0) uri = "/index.html";
    for (size_t i = 0; i < sizeof(web_assets) / sizeof(web_assets[0]); ++i) {
        if (strcmp(uri, web_assets[i].name) == 0)
            return finish_file(file, (const char *)web_assets[i].data, web_assets[i].length);
    }
    return 0;
}

void fs_close_custom(struct fs_file *file) {
    free(file->pextension);
    file->pextension = NULL;
}

err_t httpd_post_begin(void *connection, const char *uri, const char *request,
                      u16_t request_len, int content_len, char *response_uri,
                      u16_t response_uri_len, u8_t *auto_window) {
    (void)request; (void)request_len; (void)auto_window;
    int status = 400;
    if (content_len == 0) {
        unsigned slot;
        for (slot = 0; slot < 8 && posts[slot].connection; ++slot) {}
        if (slot == 8) status = 503;
        else {
            const char *response = NULL;
            if (strcmp(uri, "/api/system/bootsel") == 0) {
                system_request_bootsel();
                response = "/_api/bootsel-accepted";
                status = 202;
            } else status = app_http_command(uri, &response);
            if (status >= 200 && status < 300 && response) {
                posts[slot].connection = connection;
                posts[slot].response = response;
                return ERR_OK;
            }
        }
    }
    snprintf(response_uri, response_uri_len, "/_api/%d", status);
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

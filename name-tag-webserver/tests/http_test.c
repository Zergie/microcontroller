#include "web_http.h"
#include "lwip/apps/httpd.h"
#include "animations/animation_engine.h"
#include "pads/pads.h"
#include "api/system_api.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned bootsel_requests;
static bool fail_malloc;
uint64_t time_us_64(void) { return 1000000; }
void system_request_bootsel(void) { ++bootsel_requests; }
int network_status_portal_enabled(void) { return 0; }
const char *network_status_response(void) { return "HTTP/1.0 200 OK\r\n\r\n{}"; }
void *__real_malloc(size_t size);
void *__wrap_malloc(size_t size) {
    if (fail_malloc) { fail_malloc = false; return NULL; }
    return __real_malloc(size);
}

static struct fs_file open_file(const char *uri) {
    struct fs_file file = {0};
    assert(fs_open_custom(&file, uri));
    assert(file.len > 0 && file.index == file.len && file.flags == FS_FILE_FLAGS_HEADER_INCLUDED);
    return file;
}

static err_t begin(void *connection, const char *uri, int length, char *response) {
    u8_t window = 1;
    return httpd_post_begin(connection, uri, "", 0, length, response, 80, &window);
}

int main(void) {
    system_api_init("0123456789abcdef", 125000000, 2097152, "test-sdk");
    char tiny[8];
    assert(!system_api_snapshot(tiny, sizeof(tiny), 0));
    struct fs_file diagnostic = open_file("/api/system/info.json");
    assert(strstr(diagnostic.data, "\"board_id\":\"0123456789abcdef\""));
    assert(strstr(diagnostic.data, "\"uptime_ms\":1000"));
    assert(strstr(diagnostic.data, "\"cpu_hz\":125000000"));
    assert(strstr(diagnostic.data, "\"flash_bytes\":2097152"));
    fs_close_custom(&diagnostic);
    diagnostic = open_file("/api/system/benchmark");
    assert(!diagnostic.pextension);
    const char *body = strstr(diagnostic.data, "\r\n\r\n") + 4;
    assert(strlen(body) == 16384 && strstr(diagnostic.data, "Content-Length: 16384\r\n"));
    fs_close_custom(&diagnostic);
    animation_init(0, 42);
    pads_init();
    struct fs_file before = open_file("/api/animation.json");
    assert(before.pextension && strstr(before.data, "\"selection\":\"random\""));
    int connections[9]; char response[80];
    assert(begin(&connections[0], "/api/animation/set/solid/40/100/300/ff0000", 0, response) == ERR_OK);
    httpd_post_finished(&connections[0], response, sizeof(response));
    assert(!strcmp(response, "/api/animation.json"));
    struct fs_file after = open_file(response);
    assert(after.pextension != before.pextension && strstr(after.data, "\"selection\":\"solid\""));
    assert(strstr(before.data, "\"selection\":\"random\""));
    fs_close_custom(&after); assert(!after.pextension);
    fs_close_custom(&before); assert(!before.pextension);
    fail_malloc = true;
    after = open_file("/api/animation.json");
    assert(!after.pextension && strstr(after.data, "503 Service Unavailable"));
    fs_close_custom(&after);
    assert(begin(&connections[0], "/api/animation/set/solid/999/100/300/ff0000", 0, response) == ERR_ARG);
    assert(!strcmp(response, "/_api/400") && animation_config().brightness_percent == 40);
    after = open_file(response); assert(strstr(after.data, "400 Bad Request")); fs_close_custom(&after);
    assert(begin(&connections[0], "/api/animation/set/solid/0/100/300/ff0000", 1, response) == ERR_ARG);
    assert(animation_config().brightness_percent == 40);
    assert(begin(&connections[0], "/api/unknown", 0, response) == ERR_ARG);
    after = open_file(response); assert(strstr(after.data, "404 Not Found")); fs_close_custom(&after);
    for (unsigned i = 0; i < 8; ++i)
        assert(begin(&connections[i], "/api/animation/set/random/20/100/300/00ff64", 0, response) == ERR_OK);
    assert(begin(&connections[8], "/api/system/bootsel", 0, response) == ERR_ARG);
    assert(!strcmp(response, "/_api/503") && !bootsel_requests);
    for (unsigned i = 0; i < 8; ++i) httpd_post_finished(&connections[i], response, sizeof(response));
    assert(begin(&connections[0], "/api/system/bootsel", 0, response) == ERR_OK && bootsel_requests == 1);
    httpd_post_finished(&connections[0], response, sizeof(response));
    after = open_file(response); assert(strstr(after.data, "202 Accepted")); fs_close_custom(&after);
    assert(begin(&connections[0], "/api/pads/set/3/1500/ff000000ff000000ffffffff00ff00ff00ffffffffff0000", 0, response) == ERR_OK);
    httpd_post_finished(&connections[0], response, sizeof(response));
    assert(!strcmp(response,"/api/pads.json"));
    after=open_file(response); assert(strstr(after.data,"\"pressed_mask\":3")); fs_close_custom(&after);
    after=open_file("/api/animation.json"); assert(strstr(after.data,"\"control_view\":\"pads\"")); fs_close_custom(&after);
    assert(begin(&connections[0], "/api/pads/release", 1, response) == ERR_ARG && pads_pressed(1000)==3);
    pads_config_t preferences=pads_config();
    assert(begin(&connections[0], "/api/pads/pulse/4/536", 0, response) == ERR_OK);
    httpd_post_finished(&connections[0],response,sizeof(response));
    after=open_file(response); assert(strstr(after.data,"\"fade_ms\":1500")); fs_close_custom(&after);
    pads_config_t unchanged=pads_config(); assert(pads_config_equal(&preferences,&unchanged));
    assert(begin(&connections[0], "/api/pads/pulse/4/2001", 0, response) == ERR_ARG);
    assert(begin(&connections[0], "/api/view/animations", 0, response) == ERR_OK);
    httpd_post_finished(&connections[0],response,sizeof(response));
    after=open_file(response); assert(strstr(after.data,"\"active\":false")); fs_close_custom(&after);
    const char *assets[] = {"/", "/app.js", "/style.css"};
    for (unsigned i = 0; i < 3; ++i) {
        after = open_file(assets[i]); assert(!after.pextension); fs_close_custom(&after);
    }
    puts("Name Tag HTTP: real routes/assets, snapshot ownership, rejection, memory/connection limits and BOOTSEL OK");
}

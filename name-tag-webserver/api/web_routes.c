#include "web_http.h"
#include "animation_api.h"
#include "pads_api.h"
#include "system_api.h"
#include "pico/stdlib.h"
#include <string.h>

static bool write_pads(char *buffer,size_t capacity) {
    return pads_api_snapshot(buffer,capacity,time_us_64()/1000);
}
static bool write_system(char *buffer, size_t capacity) {
    return system_api_snapshot(buffer, capacity, time_us_64()/1000);
}
// Fixed flash-backed payload: no heap allocation or settings writes during tests.
#define BLOCK64 "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
#define BLOCK1024 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64 BLOCK64
#define BLOCK4096 BLOCK1024 BLOCK1024 BLOCK1024 BLOCK1024
static const char benchmark[] = "HTTP/1.0 200 OK\r\nContent-Type: application/octet-stream\r\nCache-Control: no-store\r\nContent-Length: 16384\r\n\r\n"
    BLOCK4096 BLOCK4096 BLOCK4096 BLOCK4096;

int app_http_open(struct fs_file *file, const char *uri) {
    if (!strcmp(uri, "/api/system/info.json")) return web_http_json(file, 768, write_system);
    if (!strcmp(uri, "/api/system/benchmark")) return web_http_text(file, benchmark);
    if (!strcmp(uri,"/api/pads.json")) return web_http_json(file,1024,write_pads);
    if (!strcmp(uri, "/api/animation.json"))
        return web_http_json(file, ANIMATION_SNAPSHOT_CAPACITY, animation_api_snapshot);
    return 0;
}

int app_http_command(const char *uri, const char **response_uri) {
    bool is_pads=!strncmp(uri,"/api/pads/",10) || !strncmp(uri,"/api/view/",10);
    int status = is_pads ? pads_api_command(uri,time_us_64()/1000) : animation_api_command(uri, time_us_64() / 1000);
    if (status == 200) *response_uri = is_pads ? "/api/pads.json" : "/api/animation.json";
    return status;
}

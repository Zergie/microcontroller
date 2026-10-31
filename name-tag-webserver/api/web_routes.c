#include "web_http.h"
#include "animation_api.h"
#include "pads_api.h"
#include "pico/stdlib.h"
#include <string.h>

static bool write_pads(char *buffer,size_t capacity) {
    return pads_api_snapshot(buffer,capacity,time_us_64()/1000);
}
int app_http_open(struct fs_file *file, const char *uri) {
    if (!strcmp(uri,"/api/pads.json")) return web_http_json(file,1024,write_pads);
    if (!strcmp(uri, "/api/animation.json"))
        return web_http_json(file, 1024, animation_api_snapshot);
    return 0;
}

int app_http_command(const char *uri, const char **response_uri) {
    bool is_pads=!strncmp(uri,"/api/pads/",10) || !strncmp(uri,"/api/view/",10);
    int status = is_pads ? pads_api_command(uri,time_us_64()/1000) : animation_api_command(uri, time_us_64() / 1000);
    if (status == 200) *response_uri = is_pads ? "/api/pads.json" : "/api/animation.json";
    return status;
}

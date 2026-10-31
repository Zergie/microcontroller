#include "animation_api.h"
#include "animations/animation_engine.h"
#include "board_config.h"
#include "settings/settings.h"
#include "pads/pads.h"
#include <stdio.h>
#include <string.h>

static uint64_t boot_id;
void animation_api_set_boot_id(uint64_t id) { boot_id=id; }

static bool number(const char *text, unsigned *result) {
    unsigned value = 0;
    if (!*text) return false;
    for (; *text; ++text) {
        if (*text < '0' || *text > '9' || value > 100000) return false;
        value = value * 10 + (unsigned)(*text - '0');
    }
    *result = value;
    return true;
}

static int hex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int animation_api_command(const char *uri, uint64_t now_ms) {
    const char prefix[] = "/api/animation/set/";
    if (strncmp(uri, prefix, sizeof(prefix) - 1)) return 404;
    const char *command = uri + sizeof(prefix) - 1;
    char copy[128];
    if (strlen(command) >= sizeof(copy)) return 400;
    strcpy(copy, command);
    char *parts[5] = {copy};
    for (unsigned i = 1; i < 5; ++i) {
        char *slash = strchr(parts[i - 1], '/');
        if (!slash) return 400;
        *slash = '\0'; parts[i] = slash + 1;
    }
    animation_config_t candidate = animation_config();
    char *suffix = strchr(parts[4], '/');
    candidate.color_mode = COLOR_CUSTOM;
    if (suffix) {
        if (!strcmp(suffix, "/original")) candidate.color_mode=COLOR_ORIGINAL;
        else if (!strcmp(suffix, "/random")) candidate.color_mode=COLOR_RANDOM;
        else return 400;
        *suffix = '\0';
    }
    candidate.random = strcmp(parts[0], "random") == 0;
    if (!candidate.random) {
        effect_t found = EFFECT_COUNT;
        for (effect_t i = 0; i < EFFECT_COUNT; ++i)
            if (!strcmp(parts[0], effect_id(i))) found = i;
        if (found == EFFECT_COUNT) return 400;
        candidate.effect = found;
    }
    if (!number(parts[1], &candidate.brightness_percent) ||
        !number(parts[2], &candidate.speed_percent) ||
        !number(parts[3], &candidate.interval_seconds) || strlen(parts[4]) != 6) return 400;
    uint8_t *channels[] = {&candidate.color.r, &candidate.color.g, &candidate.color.b};
    for (unsigned i = 0; i < 3; ++i) {
        int high = hex(parts[4][i * 2]), low = hex(parts[4][i * 2 + 1]);
        if (high < 0 || low < 0) return 400;
        *channels[i] = (uint8_t)(high * 16 + low);
    }
    return animation_configure(&candidate, now_ms) ? 200 : 400;
}

bool animation_api_snapshot(char *buffer, size_t capacity) {
    animation_config_t config = animation_config();
    rgb_t current=animation_color();
    const char *modes[]={"custom","original","random"};
    int written = snprintf(buffer, capacity,
        "{\"boot_id\":\"%08x%08x\",\"selection\":\"%s\",\"current\":\"%s\",\"brightness_percent\":%u,"
        "\"speed_percent\":%u,\"interval_seconds\":%u,\"color\":\"#%02x%02x%02x\","
        "\"original_colors\":%s,\"color_mode\":\"%s\",\"current_color\":\"#%02x%02x%02x\","
        "\"settings_saved\":%s,\"settings_saving\":%s,\"settings_error\":%s,"
        "\"pads_supported\":true,\"pads_mask_bits\":10,\"pads_transport\":\"websocket-v1\",\"control_view\":\"%s\",\"pixel_count\":%u,\"pin\":%u,\"effects\":[",
        (unsigned)(boot_id>>32), (unsigned)boot_id,
        config.random ? "random" : effect_id(config.effect), effect_id(animation_current()),
        config.brightness_percent, config.speed_percent, config.interval_seconds,
        config.color.r, config.color.g, config.color.b,
        config.color_mode==COLOR_ORIGINAL ? "true" : "false", modes[config.color_mode],
        current.r,current.g,current.b, settings_saved() ? "true" : "false",
        settings_saving() ? "true" : "false",
        settings_error() ? "true" : "false", pads_active() ? "pads":"animations", PIXEL_COUNT, PIXEL_PIN);
    if (written < 0 || (size_t)written >= capacity) return false;
    size_t used = (size_t)written;
    for (effect_t i = 0; i < EFFECT_COUNT; ++i) {
        written = snprintf(buffer + used, capacity - used, "%s\"%s\"", i ? "," : "", effect_id(i));
        if (written < 0 || (size_t)written >= capacity - used) return false;
        used += (size_t)written;
    }
    written = snprintf(buffer + used, capacity - used, "],\"default_colors\":[");
    if (written < 0 || (size_t)written >= capacity - used) return false;
    used += (size_t)written;
    for (effect_t i = 0; i < EFFECT_COUNT; ++i) {
        rgb_t color = effect_default_color(i);
        written = snprintf(buffer + used, capacity - used, "%s\"#%02x%02x%02x\"", i ? "," : "", color.r, color.g, color.b);
        if (written < 0 || (size_t)written >= capacity - used) return false;
        used += (size_t)written;
    }
    return capacity - used >= 3 && (memcpy(buffer + used, "]}", 3), true);
}

#include "system_api.h"
#include "board_config.h"
#include <stdio.h>

static char board_id[17] = "unknown";
static unsigned clock_hz, flash_bytes;
static const char *sdk_version = "unknown";

void system_api_init(const char *id, unsigned clock, unsigned flash, const char *sdk) {
    snprintf(board_id, sizeof(board_id), "%s", id);
    clock_hz = clock; flash_bytes = flash; sdk_version = sdk;
}

bool system_api_snapshot(char *buffer, size_t capacity, uint64_t now_ms) {
    int size = snprintf(buffer, capacity,
        "{\"chip\":\"RP2040\",\"board_id\":\"%s\",\"cpu_hz\":%u,\"flash_bytes\":%u,"
        "\"sram_bytes\":270336,\"uptime_ms\":%llu,\"firmware\":\"Name Tag Webserver\","
        "\"build\":\"%s %s\",\"sdk\":\"%s\",\"usb\":\"USB 1.1 Full Speed / NCM\","
        "\"pixel_count\":%u,\"pin\":%u}",
        board_id, clock_hz, flash_bytes, (unsigned long long)now_ms,
        __DATE__, __TIME__, sdk_version, PIXEL_COUNT, PIXEL_PIN);
    return size >= 0 && (size_t)size < capacity;
}

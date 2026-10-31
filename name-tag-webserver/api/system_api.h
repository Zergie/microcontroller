#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void system_api_init(const char *board_id, unsigned clock_hz, unsigned flash_bytes, const char *sdk);
bool system_api_snapshot(char *buffer, size_t capacity, uint64_t now_ms);

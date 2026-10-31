#ifndef ANIMATION_API_H
#define ANIMATION_API_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
// Empty-body POST URL. Parse and validate the entire command before mutation.
int animation_api_command(const char *uri, uint64_t now_ms);
// Ephemeral identity of this boot, independent of saved settings.
void animation_api_set_boot_id(uint64_t boot_id);
bool animation_api_snapshot(char *buffer, size_t capacity);
#endif

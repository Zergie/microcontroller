#ifndef SETTINGS_H
#define SETTINGS_H
#include <stdbool.h>
#include <stdint.h>

#define SETTINGS_PAGE_SIZE 256u
#define SETTINGS_SECTOR_SIZE 4096u
#define SETTINGS_SECTORS 2u
#define SETTINGS_BYTES (SETTINGS_SECTOR_SIZE * SETTINGS_SECTORS)

// Storage adapter: erased bytes are 0xff; program may only clear bits. Both
// callbacks complete synchronously and return false on failure. The mapped
// pages must reflect completed writes. Offsets are relative to this region.
typedef struct {
    const uint8_t *pages;
    bool (*erase)(unsigned sector);
    bool (*program)(unsigned page, const uint8_t data[SETTINGS_PAGE_SIZE]);
} settings_storage_t;

// Call after animation_init. Loads the newest valid versioned/CRC record, or
// keeps defaults. Poll from the main loop, never from an HTTP/USB callback.
void settings_init(const settings_storage_t *storage, uint64_t now_ms);
void settings_poll(uint64_t now_ms);
bool settings_saved(void);
bool settings_saving(void);
bool settings_error(void);
#endif

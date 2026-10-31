#include "flash_storage.h"
#include "settings.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "pico/flash.h"
#include "pico/stdlib.h"

#define STORAGE_OFFSET (PICO_FLASH_SIZE_BYTES - SETTINGS_BYTES)
_Static_assert(SETTINGS_SECTOR_SIZE==FLASH_SECTOR_SIZE && SETTINGS_PAGE_SIZE==FLASH_PAGE_SIZE,
               "Settings geometry must match flash");
typedef struct { uint32_t offset; const uint8_t *data; } operation_t;

static void __not_in_flash_func(write_flash)(void *argument) {
    operation_t *op=argument;
    if (op->data) flash_range_program(op->offset,op->data,FLASH_PAGE_SIZE);
    else flash_range_erase(op->offset,FLASH_SECTOR_SIZE);
}
static bool erase(unsigned sector) {
    if (sector>=SETTINGS_SECTORS) return false;
    operation_t op={STORAGE_OFFSET+sector*FLASH_SECTOR_SIZE,NULL};
    return flash_safe_execute(write_flash,&op,100)==PICO_OK;
}
static bool program(unsigned page, const uint8_t data[SETTINGS_PAGE_SIZE]) {
    if (page>=SETTINGS_BYTES/SETTINGS_PAGE_SIZE) return false;
    operation_t op={STORAGE_OFFSET+page*FLASH_PAGE_SIZE,data};
    return flash_safe_execute(write_flash,&op,100)==PICO_OK;
}
void settings_flash_init(uint64_t now_ms) {
    static const settings_storage_t storage={
        .pages=(const uint8_t *)(XIP_BASE+STORAGE_OFFSET), .erase=erase, .program=program
    };
    settings_init(&storage,now_ms);
}

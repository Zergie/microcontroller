#include "settings.h"
#include "animations/animation_engine.h"
#include <string.h>
#include "pads/pads.h"

#define PAGES (SETTINGS_BYTES / SETTINGS_PAGE_SIZE)
#define PAGES_PER_SECTOR (SETTINGS_SECTOR_SIZE / SETTINGS_PAGE_SIZE)
#define CRC_OFFSET (SETTINGS_PAGE_SIZE - 4)
#define SAVE_DELAY_MS 10000u
#define RETRY_MS 5000u

static const settings_storage_t *storage;
typedef struct { animation_config_t animation; pads_config_t pads; bool pads_active; } stored_settings_t;
static stored_settings_t saved,pending;
static stored_settings_t current_settings(void) {
    return (stored_settings_t){animation_config(),pads_config(),pads_active()};
}
static bool have_saved, dirty, failed, saving;
static unsigned latest_page;
static uint32_t generation;
static uint64_t changed_at, retry_at;

static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static void write32(uint8_t *p, uint32_t n) {
    for (unsigned i=0;i<4;++i) p[i]=(uint8_t)(n>>(i*8));
}
static uint32_t crc32(const uint8_t *data, unsigned length) {
    uint32_t crc=0xffffffffu;
    for (unsigned i=0;i<length;++i) {
        crc^=data[i];
        for (unsigned bit=0;bit<8;++bit) crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);
    }
    return ~crc;
}
static bool same_animation(const animation_config_t *a, const animation_config_t *b) {
    return a->random==b->random && (a->random || a->effect==b->effect) &&
        a->brightness_percent==b->brightness_percent && a->speed_percent==b->speed_percent &&
        a->interval_seconds==b->interval_seconds && a->color_mode==b->color_mode &&
        a->color.r==b->color.r && a->color.g==b->color.g && a->color.b==b->color.b;
}
static bool same(const stored_settings_t *a,const stored_settings_t *b) {
    return same_animation(&a->animation,&b->animation) && pads_config_equal(&a->pads,&b->pads) && a->pads_active==b->pads_active;
}
static bool decode(const uint8_t *p, stored_settings_t *stored) {
    // Bytes 4, 7 and 23 are reserved and must remain zero.
    if (memcmp(p,"NTCF",4) || p[4] || (p[5]&~7u) || ((p[5]&6u)==6u) || p[7] || p[23] ||
        read32(p+CRC_OFFSET)!=crc32(p,CRC_OFFSET) || !memchr(p+24,0,32)) return false;
    effect_t effect=EFFECT_COUNT;
    for (effect_t i=0;i<EFFECT_COUNT;++i) if (!strcmp((const char *)p+24,effect_id(i))) effect=i;
    animation_config_t *config=&stored->animation;
    *config=(animation_config_t){
        .random=(p[5]&1)!=0, .effect=effect, .brightness_percent=p[6],
        .speed_percent=read32(p+12), .interval_seconds=read32(p+16),
        .color={p[20],p[21],p[22]}, .color_mode=(p[5]&4) ? COLOR_RANDOM : (p[5]&2) ? COLOR_ORIGINAL : COLOR_CUSTOM
    };
    stored->pads.fade_ms=read32(p+80);
    if (p[84]>1) return false;
    stored->pads_active=p[84]!=0;
    for (unsigned i=0;i<PAD_COUNT;++i) stored->pads.colors[i]=(rgb_t){p[56+i*3],p[57+i*3],p[58+i*3]};
    return animation_config_valid(config) && pads_config_valid(&stored->pads);
}
static void encode(uint8_t *p, const stored_settings_t *stored, uint32_t sequence) {
    const animation_config_t *config=&stored->animation;
    memset(p,0,SETTINGS_PAGE_SIZE);
    memcpy(p,"NTCF",4);
    p[5]=(config->random ? 1 : 0)|(config->color_mode==COLOR_ORIGINAL ? 2 : config->color_mode==COLOR_RANDOM ? 4 : 0);
    p[6]=(uint8_t)config->brightness_percent;
    write32(p+8,sequence); write32(p+12,config->speed_percent); write32(p+16,config->interval_seconds);
    p[20]=config->color.r; p[21]=config->color.g; p[22]=config->color.b;
    const char *id=effect_id(config->effect);
    memcpy(p+24,id,strlen(id)+1);
    for (unsigned i=0;i<PAD_COUNT;++i) {
        p[56+i*3]=stored->pads.colors[i].r; p[57+i*3]=stored->pads.colors[i].g; p[58+i*3]=stored->pads.colors[i].b;
    }
    write32(p+80,stored->pads.fade_ms); p[84]=stored->pads_active ? 1:0;
    write32(p+CRC_OFFSET,crc32(p,CRC_OFFSET));
}
static bool erased(unsigned page) {
    const uint8_t *data=storage->pages+page*SETTINGS_PAGE_SIZE;
    for (unsigned i=0;i<SETTINGS_PAGE_SIZE;++i) if (data[i]!=0xff) return false;
    return true;
}

void settings_init(const settings_storage_t *adapter, uint64_t now_ms) {
    storage=adapter;
    have_saved=false; dirty=false; failed=false; saving=false;
    latest_page=0; generation=0; retry_at=0;
    for (unsigned page=0;page<PAGES;++page) {
        stored_settings_t candidate;
        const uint8_t *record=storage->pages+page*SETTINGS_PAGE_SIZE;
        if (!decode(record,&candidate)) continue;
        uint32_t sequence=read32(record+8);
        if (!have_saved || (sequence!=generation && sequence-generation<0x80000000u)) {
            have_saved=true; saved=candidate; latest_page=page; generation=sequence;
        }
    }
    if (have_saved) {
        animation_configure(&saved.animation,now_ms);
        pads_configure(&saved.pads);
        pads_set_active(saved.pads_active,now_ms);
    }
    pending=current_settings();
    changed_at=now_ms;
    dirty=!have_saved;
}

bool settings_saved(void) {
    stored_settings_t current=current_settings();
    return have_saved && same(&saved,&current);
}
bool settings_error(void) { return failed; }
bool settings_saving(void) { return saving; }

static bool commit(void) {
    unsigned sector=have_saved ? latest_page/PAGES_PER_SECTOR : 0;
    unsigned first=have_saved ? latest_page+1 : 0;
    unsigned end=(sector+1)*PAGES_PER_SECTOR;
    unsigned page=first;
    while (page<end && !erased(page)) ++page;
    if (page==end) {
        sector=have_saved ? 1-sector : 0;
        // The newest valid record remains in the other sector throughout erase
        // and program. A torn write is ignored by CRC on the next boot.
        if (!storage->erase(sector)) return false;
        page=sector*PAGES_PER_SECTOR;
    }
    uint8_t record[SETTINGS_PAGE_SIZE];
    uint32_t sequence=have_saved ? generation+1 : 1;
    encode(record,&pending,sequence);
    // Readback is authoritative, including a callback reporting failure after
    // all bytes were written. Otherwise a retry could reuse its generation.
    storage->program(page,record);
    stored_settings_t verified;
    const uint8_t *written=storage->pages+page*SETTINGS_PAGE_SIZE;
    if (!decode(written,&verified) || read32(written+8)!=sequence || !same(&verified,&pending)) return false;
    saved=verified; generation=sequence; latest_page=page; have_saved=true;
    return true;
}

void settings_poll(uint64_t now_ms) {
    if (!storage) return;
    stored_settings_t current=current_settings();
    if (have_saved && same(&saved,&current)) { dirty=false; failed=false; return; }
    if (!dirty) { dirty=true; changed_at=now_ms; pending=current; }
    if (!same(&pending,&current)) { pending=current; changed_at=now_ms; }
    if (now_ms<retry_at || now_ms-changed_at<SAVE_DELAY_MS) return;
    saving=true;
    bool committed=commit();
    saving=false;
    if (committed) { dirty=false; failed=false; }
    else { failed=true; retry_at=now_ms+RETRY_MS; }
}

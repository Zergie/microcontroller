#include "pads.h"
#include <string.h>

_Static_assert(PIXEL_COUNT>=PAD_COUNT,"LED pads require at least eight pixels");
static pads_config_t config;
static bool active, dirty;
static unsigned pressed,last_brightness;
static uint64_t lease_at, next_frame, released_at[PAD_COUNT];
static unsigned release_duration[PAD_COUNT];
static uint16_t last_level[PAD_COUNT];

void pads_init(void) {
    config=(pads_config_t){.colors={{0,255,40},{0,255,255},{0,0,255},{180,0,255},
        {255,0,20},{255,40,0},{255,150,0},{255,255,255}},.fade_ms=1000};
    active=false; dirty=true; pressed=0; last_brightness=101; lease_at=next_frame=0;
    memset(release_duration,0,sizeof(release_duration));
    memset(released_at,0,sizeof(released_at)); memset(last_level,0,sizeof(last_level));
}
pads_config_t pads_config(void) { return config; }
bool pads_config_valid(const pads_config_t *candidate) { return candidate && candidate->fade_ms<=PAD_MAX_FADE_MS; }
bool pads_config_equal(const pads_config_t *a,const pads_config_t *b) {
    return a->fade_ms==b->fade_ms && !memcmp(a->colors,b->colors,sizeof(a->colors));
}
bool pads_configure(const pads_config_t *candidate) {
    if (!pads_config_valid(candidate)) return false;
    if (!pads_config_equal(&config,candidate)) { config=*candidate; dirty=true; }
    return true;
}
static void set_mask(unsigned mask,uint64_t now) {
    for (unsigned i=0;i<PAD_COUNT;++i) {
        unsigned bit=1u<<i;
        if ((pressed&bit) && !(mask&bit)) { released_at[i]=now; release_duration[i]=config.fade_ms; }
        if (mask&bit) release_duration[i]=0;
    }
    if (pressed!=mask) dirty=true;
    pressed=mask;
}
static void expire(uint64_t now) {
    if (pressed && now>=lease_at) set_mask(0,lease_at);
}
void pads_release(uint64_t now) { expire(now); set_mask(0,now); }
void pads_set_active(bool enabled,uint64_t now) {
    if (active==enabled) return;
    pads_release(now);
    memset(release_duration,0,sizeof(release_duration));
    active=enabled; dirty=true; next_frame=0;
}
bool pads_active(void) { return active; }
bool pads_apply(unsigned mask,const pads_config_t *candidate,uint64_t now) {
    if (mask>255 || !pads_config_valid(candidate)) return false;
    expire(now);
    pads_configure(candidate);
    pads_set_active(true,now);
    set_mask(mask,now);
    lease_at=now+PAD_LEASE_MS;
    return true;
}
unsigned pads_pressed(uint64_t now) { expire(now); return pressed; }
bool pads_set_pressed(unsigned mask,uint64_t now) {
    if (!active || mask>255) return false;
    expire(now); set_mask(mask,now); lease_at=now+PAD_LEASE_MS;
    return true;
}
bool pads_pulse(unsigned mask,unsigned duration_ms,uint64_t now) {
    if (!active || !mask || mask>255 || duration_ms>PAD_MAX_FADE_MS) return false;
    expire(now);
    for (unsigned i=0;i<PAD_COUNT;++i) if (mask&(1u<<i)) {
        pressed&=~(1u<<i);
        released_at[i]=now; release_duration[i]=duration_ms;
    }
    dirty=true;
    return true;
}
bool pads_render(uint64_t now,unsigned brightness,effect_frame_t *frame) {
    if (!active) return false;
    expire(now);
    if (!dirty && brightness==last_brightness && now<next_frame) return false;
    next_frame=now+10;
    uint16_t levels[PAD_COUNT];
    bool changed=dirty || brightness!=last_brightness;
    for (unsigned i=0;i<PAD_COUNT;++i) {
        uint64_t elapsed=now-released_at[i];
        levels[i]=(pressed&(1u<<i)) ? 65535 :
            release_duration[i] && elapsed<release_duration[i] ?
            (uint16_t)((release_duration[i]-elapsed)*65535/release_duration[i]) : 0;
        changed|=levels[i]!=last_level[i];
    }
    if (!changed) return false;
    for (unsigned i=0;i<PIXEL_COUNT;++i) frame->pixels[0][i]=(rgb_t){0};
    if (brightness>100) brightness=100;
    for (unsigned i=0;i<PAD_COUNT;++i) {
        uint32_t level=(uint32_t)levels[i]*brightness;
        frame->pixels[0][i]=(rgb_t){config.colors[i].r*level/6553500u,
            config.colors[i].g*level/6553500u,config.colors[i].b*level/6553500u};
    }
    memcpy(last_level,levels,sizeof(levels)); last_brightness=brightness; dirty=false; frame->count=1;
    return true;
}

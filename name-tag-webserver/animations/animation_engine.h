#ifndef ANIMATION_ENGINE_H
#define ANIMATION_ENGINE_H
#include <stdbool.h>
#include "effects.h"

typedef enum { COLOR_CUSTOM, COLOR_ORIGINAL, COLOR_RANDOM } color_mode_t;

typedef struct {
    bool random;
    effect_t effect;
    unsigned brightness_percent; // 1..100
    unsigned speed_percent;      // 5..800
    unsigned interval_seconds;   // 5..3600; used by random loop
    rgb_t color;
    color_mode_t color_mode;
} animation_config_t;

// One owner (main loop). now_ms must be monotonic; tests supply a fake clock.
void animation_init(uint64_t now_ms, uint32_t seed);
bool animation_config_valid(const animation_config_t *config);
bool animation_configure(const animation_config_t *config, uint64_t now_ms);
animation_config_t animation_config(void);
effect_t animation_current(void);
rgb_t animation_color(void);
void animation_refresh(void);
// Returns one or two ordered frames only when an effect advances or settings
// change. Drain the returned burst through PIO/DMA before calling again.
bool animation_next(uint64_t now_ms, effect_frame_t *frame);
#endif

#ifndef EFFECTS_H
#define EFFECTS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "board_config.h"

typedef struct { uint8_t r, g, b; } rgb_t;
typedef enum {
    EFFECT_BLINK, EFFECT_COLOR_CYCLE, EFFECT_COMET, EFFECT_CHASE, EFFECT_PULSE,
    EFFECT_SPARKLE, EFFECT_SOLID, EFFECT_RAINBOW, EFFECT_SPARKLE_PULSE,
    EFFECT_RAINBOW_COMET, EFFECT_RAINBOW_CHASE, EFFECT_RAINBOW_SPARKLE,
    EFFECT_CUSTOM_COLOR_CHASE,
    EFFECT_AURORA, EFFECT_TIDAL, EFFECT_FIREFLIES, EFFECT_MOLTEN,
    EFFECT_RIPPLE, EFFECT_HEARTBEAT, EFFECT_PRISM_WEAVE, EFFECT_TWIN_ORBIT,
    EFFECT_COUNT
} effect_t;

// Sparkle's after_draw emits an intermediate show before Animation.show.
// Both frames must reach the serializer, in order, without overwriting DMA.
typedef struct { rgb_t pixels[2][PIXEL_COUNT]; unsigned count; } effect_frame_t;
typedef struct {
    rgb_t pixels[PIXEL_COUNT];
    rgb_t color;
    uint64_t next_due, generator_started_at;
    uint32_t draws;
    int tail_start;
    unsigned offset, color_index;
    bool reverse, generator_started;
} effect_state_t;

const char *effect_id(effect_t effect);
rgb_t effect_default_color(effect_t effect);
rgb_t effect_colorwheel(unsigned position);
unsigned effect_interval_ms(effect_t effect);
void effect_init(effect_state_t *state, effect_t effect);
// Apply the original color setter, including Sparkle's background conversion.
void effect_set_color(effect_state_t *state, effect_t effect, rgb_t color);
// Stateful port of Adafruit LED Animation 2.11.1 for this RGB NeoPixel strip.
// virtual_ms is monotonic, speed-scaled time. Late calls draw once (no catch-up),
// matching Animation.animate. No allocation, hardware access or waiting.
bool effect_step(effect_state_t *state, effect_t effect, uint64_t virtual_ms,
                 rgb_t color, bool original_colors, uint32_t *random_state, effect_frame_t *frame);
#endif

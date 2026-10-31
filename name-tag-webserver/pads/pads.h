#ifndef PADS_H
#define PADS_H
#include "animations/effects.h"
#define PAD_COUNT 10u
#define PAD_MASK ((1u << PAD_COUNT) - 1u)
#define PAD_MAX_FADE_MS 2000u
#define PAD_LEASE_MS 1200u
typedef struct { rgb_t colors[PAD_COUNT]; unsigned fade_ms; } pads_config_t;
void pads_init(void);
pads_config_t pads_config(void);
bool pads_config_valid(const pads_config_t *config);
bool pads_config_equal(const pads_config_t *a, const pads_config_t *b);
bool pads_configure(const pads_config_t *config);
void pads_set_active(bool active, uint64_t now_ms);
bool pads_active(void);
bool pads_apply(unsigned mask, const pads_config_t *config, uint64_t now_ms);
// Fast transport: never changes settings or reactivates a closed pad view.
bool pads_set_pressed(unsigned mask, uint64_t now_ms);
// A transient note envelope. Never changes colors, saved fade or active view.
bool pads_pulse(unsigned mask, unsigned duration_ms, uint64_t now_ms);
void pads_release(uint64_t now_ms);
unsigned pads_pressed(uint64_t now_ms);
// One frame for all ten held/fading LEDs.
bool pads_render(uint64_t now_ms, unsigned brightness, effect_frame_t *frame);
#endif

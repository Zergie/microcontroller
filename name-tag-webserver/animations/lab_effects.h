#ifndef LAB_EFFECTS_H
#define LAB_EFFECTS_H
#include "effects.h"
// Pure bounded renderer for the eight Animation Lab concepts. Original mode
// keeps their palettes; custom/random mode recolors the same light envelopes.
void lab_effect_render(effect_t effect, uint64_t elapsed_ms, rgb_t color,
                       bool original_colors, rgb_t pixels[PIXEL_COUNT]);
#endif

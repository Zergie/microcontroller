// SPDX-FileCopyrightText: 2020 Kattni Rembor for Adafruit Industries
// SPDX-FileCopyrightText: 2020 Connie Sieh for Adafruit Industries
// SPDX-FileCopyrightText: 2021 Kattni Rembor (CircuitPython rainbowio)
// SPDX-License-Identifier: MIT
// C port of the bundled adafruit_led_animation 2.11.1. See LICENSE.txt.
#include "effects.h"
#include "circuitpython_math.h"
#include <string.h>

static const struct {
    const char *id;
    unsigned interval_ms;
    rgb_t color;
} defaults[] = {
    // int(speed * 1000) after CircuitPython OBJ_REPR_C float boxing.
    // Comet deliberately uses the nominal 0.01 s instead of the rounded 9 ms.
    {"blink", 500, {0,255,40}}, {"color-cycle", 399, {255,0,20}},
    {"comet", 10, {180,0,255}}, {"chase", 99, {255,255,255}},
    {"pulse", 99, {255,100,0}}, {"sparkle", 99, {180,0,255}},
    {"solid", 1000, {0,255,40}}, {"rainbow", 99, {0,0,0}},
    {"sparkle-pulse", 99, {0,255,40}}, {"rainbow-comet", 99, {0,0,0}},
    {"rainbow-chase", 99, {0,0,0}}, {"rainbow-sparkle", 99, {0,0,0}},
    {"custom-color-chase", 99, {0,0,0}}
};
_Static_assert(sizeof(defaults) / sizeof(defaults[0]) == EFFECT_COUNT, "Effect defaults incomplete");

const char *effect_id(effect_t effect) {
    return (unsigned)effect < EFFECT_COUNT ? defaults[effect].id : NULL;
}
rgb_t effect_default_color(effect_t effect) {
    return (unsigned)effect < EFFECT_COUNT ? defaults[effect].color : (rgb_t){0};
}
unsigned effect_interval_ms(effect_t effect) {
    return (unsigned)effect < EFFECT_COUNT ? defaults[effect].interval_ms : 0;
}

// CircuitPython rainbowio.colorwheel, integer positions 0..255.
rgb_t effect_colorwheel(unsigned position) {
    position %= 256;
    if (position < 85) return (rgb_t){255-position*3,position*3,0};
    if (position < 170) { position -= 85; return (rgb_t){0,255-position*3,position*3}; }
    position -= 170;
    unsigned value = position*3 < 256 ? position*3 : 255;
    return (rgb_t){value,0,255-value};
}

static rgb_t intensity(rgb_t color, float level) {
    return (rgb_t){(uint8_t)cp_multiply(color.r,level),
                   (uint8_t)cp_multiply(color.g,level),
                   (uint8_t)cp_multiply(color.b,level)};
}
static void fill(rgb_t *pixels, rgb_t color) {
    for (unsigned i=0;i<PIXEL_COUNT;++i) pixels[i]=color;
}
static unsigned random_pixel(uint32_t *state) {
    *state ^= *state << 13; *state ^= *state >> 17; *state ^= *state << 5;
    return *state % PIXEL_COUNT;
}

void effect_init(effect_state_t *state, effect_t effect) {
    memset(state, 0, sizeof(*state));
    state->color=effect_default_color(effect);
    state->tail_start = -(effect == EFFECT_RAINBOW_COMET ? 7 : 10)-1;
}

static bool same_color(rgb_t a, rgb_t b) { return a.r==b.r && a.g==b.g && a.b==b.b; }

void effect_set_color(effect_state_t *state, effect_t effect, rgb_t color) {
    if (same_color(state->color,color)) return;
    if (effect==EFFECT_SPARKLE) {
        rgb_t old_half={state->color.r/4,state->color.g/4,state->color.b/4};
        rgb_t old_dim={state->color.r/10,state->color.g/10,state->color.b/10};
        rgb_t half={color.r/4,color.g/4,color.b/4};
        rgb_t dim={color.r/10,color.g/10,color.b/10};
        for (unsigned i=0;i<PIXEL_COUNT;++i) {
            if (same_color(state->pixels[i],old_half)) state->pixels[i]=half;
            else if (same_color(state->pixels[i],old_dim)) state->pixels[i]=dim;
        }
    }
    state->color=color;
}

static void comet(effect_state_t *state, effect_t effect, rgb_t color) {
    int tail = effect == EFFECT_RAINBOW_COMET ? 7 : 10;
    rgb_t colors[11] = {{0}};
    float step=cp_divide(cp_float(0.95f),tail);
    for (int n=0;n<tail;++n) {
        rgb_t base = effect == EFFECT_RAINBOW_COMET ? effect_colorwheel((tail-n-1)*(256/tail)) : color;
        colors[n+1] = intensity(base,cp_add(cp_multiply(n,step),cp_float(0.05f)));
    }
    for (int n=0;n<=tail;++n) {
        int pixel = state->tail_start+n;
        if (pixel >= PIXEL_COUNT) break;
        if (pixel >= 0) state->pixels[pixel] = colors[state->reverse ? tail-n : n];
    }
    state->tail_start += state->reverse ? -1 : 1;
    if (state->tail_start < -tail || (state->tail_start >= PIXEL_COUNT && !state->reverse))
        state->reverse = !state->reverse;
}

static rgb_t bar_color(effect_t effect, rgb_t color, unsigned index, unsigned bar) {
    static const rgb_t palette[] = {{255,40,0},{255,255,255},{0,255,40}};
    if (effect == EFFECT_RAINBOW_CHASE) return effect_colorwheel(((index+64-bar%64)%64)*8);
    if (effect == EFFECT_CUSTOM_COLOR_CHASE) return palette[(index+3-bar%3)%3];
    return color;
}
static void chase(effect_state_t *state, effect_t effect, rgb_t color) {
    unsigned size = effect == EFFECT_CUSTOM_COLOR_CHASE ? 2 : 3;
    unsigned spacing = effect == EFFECT_CHASE ? 6 : effect == EFFECT_RAINBOW_CHASE ? 2 : 3;
    unsigned pixel=0,bar=0;
    for (unsigned i=state->offset;i>0 && pixel<PIXEL_COUNT;--i) {
        state->pixels[pixel++] = i>spacing ? bar_color(effect,color,state->color_index,bar) : (rgb_t){0};
        if (i<=spacing) bar=1;
    }
    while (pixel<PIXEL_COUNT) {
        for (unsigned i=0;i<size && pixel<PIXEL_COUNT;++i)
            state->pixels[pixel++] = bar_color(effect,color,state->color_index,bar);
        for (unsigned i=0;i<spacing && pixel<PIXEL_COUNT;++i) state->pixels[pixel++]=(rgb_t){0};
        ++bar;
    }
    state->offset = (state->offset+1)%(size+spacing);
    if (state->draws%PIXEL_COUNT==0) {
        unsigned colors = effect == EFFECT_RAINBOW_CHASE ? 64 : 3;
        state->color_index = (state->color_index+1)%colors;
    }
}

bool effect_step(effect_state_t *state, effect_t effect, uint64_t now,
                 rgb_t color, uint32_t *random_state, effect_frame_t *frame) {
    if ((unsigned)effect >= EFFECT_COUNT || now < state->next_due) return false;
    effect_set_color(state,effect,color);
    if (!state->generator_started) {
        state->generator_started_at=now; state->generator_started=true;
    }
    uint64_t elapsed=now-state->generator_started_at;
    ++state->draws;
    frame->count=1;
    unsigned sparkle_pixels[15];
    unsigned sparkles=0;
    switch (effect) {
        case EFFECT_BLINK:
            fill(state->pixels, state->offset ? (rgb_t){0} : color);
            state->offset ^= 1; break;
        case EFFECT_COLOR_CYCLE:
            fill(state->pixels, state->offset ? (rgb_t){255,40,0} : color);
            state->offset ^= 1; break;
        case EFFECT_SOLID: fill(state->pixels,color); break;
        case EFFECT_COMET: case EFFECT_RAINBOW_COMET: comet(state,effect,color); break;
        case EFFECT_CHASE: case EFFECT_RAINBOW_CHASE: case EFFECT_CUSTOM_COLOR_CHASE:
            chase(state,effect,color); break;
        case EFFECT_PULSE: case EFFECT_SPARKLE_PULSE: {
            unsigned pos=elapsed%3000;
            if (pos>1500) pos=3000-pos;
            rgb_t pulse=intensity(color,cp_divide(pos,1500));
            if (effect==EFFECT_PULSE) fill(state->pixels,pulse);
            else { sparkles=1; sparkle_pixels[0]=random_pixel(random_state); state->pixels[sparkle_pixels[0]]=pulse; }
            break;
        }
        case EFFECT_SPARKLE:
            sparkles=10;
            for (unsigned i=0;i<sparkles;++i) {
                sparkle_pixels[i]=random_pixel(random_state); state->pixels[sparkle_pixels[i]]=color;
            }
            break;
        case EFFECT_RAINBOW: case EFFECT_RAINBOW_SPARKLE: {
            unsigned period=effect==EFFECT_RAINBOW ? 2000 : 5000;
            unsigned index=(unsigned)cp_multiply(cp_divide(elapsed%period,period),256);
            for (unsigned i=0;i<PIXEL_COUNT;++i) {
                rgb_t base=effect_colorwheel(index+i);
                state->pixels[i]=effect==EFFECT_RAINBOW_SPARKLE ? intensity(base,cp_float(0.2f)) : base;
            }
            if (effect==EFFECT_RAINBOW_SPARKLE) {
                // after_draw shows the dim background before adding sparkles.
                memcpy(frame->pixels[0],state->pixels,sizeof(state->pixels));
                for (unsigned i=0;i<15;++i) {
                    unsigned pixel=random_pixel(random_state); state->pixels[pixel]=effect_colorwheel(index+pixel);
                }
                frame->count=2;
            }
            break;
        }
        default: return false;
    }
    if (effect==EFFECT_SPARKLE || effect==EFFECT_SPARKLE_PULSE) {
        memcpy(frame->pixels[0],state->pixels,sizeof(state->pixels));
        if (effect==EFFECT_SPARKLE) {
            // The bundled empty mask dims only selected pixels to color//4.
            rgb_t half={color.r/4,color.g/4,color.b/4};
            for (unsigned i=0;i<sparkles;++i) state->pixels[sparkle_pixels[i]]=half;
        }
        frame->count=2;
    }
    memcpy(frame->pixels[frame->count-1],state->pixels,sizeof(state->pixels));
    state->next_due=now+effect_interval_ms(effect);
    return true;
}

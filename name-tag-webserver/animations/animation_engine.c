#include "animation_engine.h"

static animation_config_t config;
static effect_t current;
static effect_t bag[EFFECT_COUNT];
static effect_state_t effects[EFFECT_COUNT];
static unsigned remaining;
static uint32_t random_state, color_random_state;
static rgb_t random_color;

static void choose_color(void) {
    color_random_state ^= color_random_state << 13;
    color_random_state ^= color_random_state >> 17;
    color_random_state ^= color_random_state << 5;
    random_color=effect_colorwheel(color_random_state%256);
}
static uint64_t started, real_anchor, virtual_anchor;
static bool initialized, clear_pending, refresh_pending;
// Replay one draw for color edits without advancing its phase or consuming RNG.
// Only the active effect needs a checkpoint; all effects retain their own state.
static effect_state_t before_draw;
static uint64_t drawn_at;
static uint32_t draw_random;
static bool have_draw, recolor_pending;

static uint64_t scaled_time(uint64_t now) {
    // Hundredths of a millisecond preserve fractions across configuration edits.
    return virtual_anchor+(now-real_anchor)*config.speed_percent;
}
static void activate(effect_t effect, uint64_t now) {
    current=effect;
    if (config.color_mode==COLOR_RANDOM) choose_color();
    started=now;
    // AnimationSequence(auto_clear=True, auto_reset=False): position/cycle
    // state survives revisits, but the shared pixel buffer starts black.
    for (unsigned i=0;i<PIXEL_COUNT;++i) effects[current].pixels[i]=(rgb_t){0};
    effects[current].next_due=0;
    have_draw=false;
    recolor_pending=false;
    refresh_pending=false;
    clear_pending=true;
}

static uint32_t random_next(void) {
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return random_state;
}

static effect_t next_effect(void) {
    if (!remaining) {
        for (unsigned i = 0; i < EFFECT_COUNT; ++i) bag[i] = (effect_t)i;
        for (unsigned i = EFFECT_COUNT - 1; i > 0; --i) {
            unsigned j = random_next() % (i + 1);
            effect_t swap = bag[i]; bag[i] = bag[j]; bag[j] = swap;
        }
        // Do not repeat the last effect across round boundaries.
        if (bag[EFFECT_COUNT - 1] == current) {
            effect_t swap = bag[0]; bag[0] = bag[EFFECT_COUNT - 1]; bag[EFFECT_COUNT - 1] = swap;
        }
        remaining = EFFECT_COUNT;
    }
    return bag[--remaining];
}

bool animation_config_valid(const animation_config_t *candidate) {
    return candidate && (unsigned)candidate->effect < EFFECT_COUNT &&
        candidate->brightness_percent >= 1 && candidate->brightness_percent <= 100 && candidate->speed_percent >= 5 &&
        candidate->speed_percent <= 800 && candidate->interval_seconds >= 5 &&
        candidate->interval_seconds <= 3600 && (unsigned)candidate->color_mode <= COLOR_RANDOM;
}

bool animation_configure(const animation_config_t *candidate, uint64_t now_ms) {
    if (!animation_config_valid(candidate)) return false;
    rgb_t old_color=initialized ? animation_color() : (rgb_t){0};
    bool palette_changed=initialized && ((config.color_mode==COLOR_ORIGINAL)!=(candidate->color_mode==COLOR_ORIGINAL));
    bool brightness_changed=!initialized || candidate->brightness_percent!=config.brightness_percent;
    bool selection_changed = !initialized || candidate->random != config.random ||
        (!candidate->random && candidate->effect != current);
    bool random_color_selected=candidate->color_mode==COLOR_RANDOM &&
        (!initialized || config.color_mode!=COLOR_RANDOM);
    virtual_anchor=initialized ? scaled_time(now_ms) : 0;
    real_anchor=now_ms;
    config = *candidate;
    if (selection_changed) {
        if (config.random) remaining=0;
        activate(config.random ? next_effect() : config.effect, now_ms);
    }
    if (random_color_selected && !selection_changed) choose_color();
    rgb_t color=animation_color();
    if (have_draw && (palette_changed || old_color.r!=color.r || old_color.g!=color.g || old_color.b!=color.b))
        recolor_pending=true;
    refresh_pending |= brightness_changed;
    initialized=true;
    return true;
}

void animation_init(uint64_t now_ms, uint32_t seed) {
    random_state = seed ? seed : 0x6d2b79f5u;
    color_random_state=seed^0xa511e9b3u;
    if (!color_random_state) color_random_state=1;
    current = EFFECT_COUNT;
    remaining=0;
    initialized=false;
    for (effect_t i=0;i<EFFECT_COUNT;++i) effect_init(&effects[i],i);
    animation_config_t defaults = {
        .random=true, .effect=EFFECT_RAINBOW_SPARKLE, .brightness_percent=20,
        .speed_percent=100, .interval_seconds=300, .color={0,255,40}, .color_mode=COLOR_ORIGINAL
    };
    animation_configure(&defaults, now_ms);
}

animation_config_t animation_config(void) { return config; }
void animation_refresh(void) { refresh_pending=true; }
effect_t animation_current(void) { return current; }
rgb_t animation_color(void) {
    if (config.color_mode==COLOR_ORIGINAL) return effect_default_color(current);
    return config.color_mode==COLOR_RANDOM ? random_color : config.color;
}

bool animation_next(uint64_t now_ms, effect_frame_t *frame) {
    if (config.random && now_ms-started >= config.interval_seconds*1000u)
        activate(next_effect(),now_ms);
    if (clear_pending) {
        for (unsigned i=0;i<PIXEL_COUNT;++i) frame->pixels[0][i]=(rgb_t){0};
        frame->count=1;
        clear_pending=false;
        return true;
    }
    rgb_t color=animation_color();
    uint64_t now=scaled_time(now_ms)/100;
    bool drawn=false;
    if (now>=effects[current].next_due) {
        effect_set_color(&effects[current],current,color);
        before_draw=effects[current];
        drawn_at=now;
        draw_random=random_state;
        drawn=effect_step(&effects[current],current,now,color,config.color_mode==COLOR_ORIGINAL,&random_state,frame);
        have_draw=true;
    } else if (recolor_pending && have_draw) {
        effect_set_color(&before_draw,current,color);
        effect_state_t replay=before_draw;
        uint32_t replay_random=draw_random;
        drawn=effect_step(&replay,current,drawn_at,color,config.color_mode==COLOR_ORIGINAL,&replay_random,frame);
        effects[current]=replay;
    }
    if (!drawn) {
        if (!refresh_pending) return false;
        frame->count=1;
        for (unsigned i=0;i<PIXEL_COUNT;++i) frame->pixels[0][i]=effects[current].pixels[i];
    }
    refresh_pending=false;
    recolor_pending=false;
    // PixelBuf uses a 0..256 integer scale rather than a floating multiplier.
    unsigned brightness=config.brightness_percent*256/100;
    for (unsigned n=0;n<frame->count;++n) {
        for (unsigned i=0;i<PIXEL_COUNT;++i) {
            frame->pixels[n][i].r=(uint8_t)(frame->pixels[n][i].r*brightness/256);
            frame->pixels[n][i].g=(uint8_t)(frame->pixels[n][i].g*brightness/256);
            frame->pixels[n][i].b=(uint8_t)(frame->pixels[n][i].b*brightness/256);
        }
    }
    return true;
}

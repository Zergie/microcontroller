#include "animations/animation_engine.h"
#include <stdio.h>
#include <string.h>

static unsigned failures;
#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr,"Animation edit regression at line %d: %s\n",__LINE__,#condition); \
    ++failures; } } while (0)

static animation_config_t select_effect(effect_t effect, unsigned speed) {
    animation_init(0,42);
    animation_config_t config=animation_config();
    config.random=false; config.effect=effect; config.speed_percent=speed;
    config.brightness_percent=100;
    CHECK(animation_configure(&config,0));
    effect_frame_t frame;
    CHECK(animation_next(0,&frame)); // sequence clear
    CHECK(animation_next(0,&frame)); // first effect draw
    return config;
}

static void immediate_color_and_phase(void) {
    animation_config_t config=select_effect(EFFECT_SOLID,5);
    config.color_mode=COLOR_CUSTOM; config.color=(rgb_t){255,0,0};
    CHECK(animation_configure(&config,100));
    effect_frame_t frame;
    CHECK(animation_next(100,&frame));
    CHECK(frame.pixels[0][0].r==255 && frame.pixels[0][0].g==0);
    CHECK(!animation_next(101,&frame));
    CHECK(!animation_next(19999,&frame));
    CHECK(animation_next(20000,&frame)); // redraw kept the original deadline

    config=select_effect(EFFECT_BLINK,5);
    config.color_mode=COLOR_CUSTOM; config.color=(rgb_t){255,0,0};
    CHECK(animation_configure(&config,5000));
    CHECK(animation_next(5000,&frame));
    CHECK(frame.pixels[0][0].r==255 && frame.pixels[0][0].g==0); // stays in ON phase
    CHECK(!animation_next(9999,&frame));
    CHECK(animation_next(10000,&frame));
    CHECK(frame.pixels[0][0].r==0 && frame.pixels[0][0].g==0); // original OFF deadline
}

static void revisit_without_black_wait(void) {
    animation_config_t config=select_effect(EFFECT_SOLID,5);
    effect_frame_t frame;
    config.effect=EFFECT_CHASE;
    CHECK(animation_configure(&config,100));
    CHECK(animation_next(100,&frame)); CHECK(animation_next(100,&frame));
    config.effect=EFFECT_SOLID;
    CHECK(animation_configure(&config,200));
    CHECK(animation_next(200,&frame)); // clear remains ordered before draw
    CHECK(animation_next(200,&frame));
    CHECK(frame.pixels[0][0].g==255);
    CHECK(!animation_next(201,&frame));
}

static void sparkle_color_and_random_stream(void) {
    effect_state_t state;
    effect_init(&state,EFFECT_SPARKLE);
    effect_frame_t frame;
    uint32_t random=42;
    for (unsigned now=0;now<=1000;now+=100)
        CHECK(effect_step(&state,EFFECT_SPARKLE,now,(rgb_t){180,0,255},&random,&frame));
    CHECK(effect_step(&state,EFFECT_SPARKLE,1100,(rgb_t){0,255,0},&random,&frame));
    for (unsigned n=0;n<frame.count;++n)
        for (unsigned i=0;i<PIXEL_COUNT;++i)
            CHECK(frame.pixels[n][i].r==0 && frame.pixels[n][i].b==0);

    // Recoloring must replay the current sparkle selection, not consume RNG.
    rgb_t control[PIXEL_COUNT];
    animation_config_t config=select_effect(EFFECT_SPARKLE,5);
    CHECK(animation_next(1980,&frame));
    memcpy(control,frame.pixels[1],sizeof(control));
    config=select_effect(EFFECT_SPARKLE,5);
    config.color_mode=COLOR_CUSTOM; config.color=(rgb_t){0,255,0};
    CHECK(animation_configure(&config,100)); CHECK(animation_next(100,&frame));
    config.color_mode=COLOR_ORIGINAL;
    CHECK(animation_configure(&config,200)); CHECK(animation_next(200,&frame));
    CHECK(animation_next(1980,&frame));
    CHECK(!memcmp(control,frame.pixels[1],sizeof(control)));
}

static void fractional_clock_and_noop(void) {
    animation_config_t config=select_effect(EFFECT_BLINK,5);
    effect_frame_t frame={0};
    for (unsigned now=1;now<=10000;++now) {
        config.brightness_percent=now%2 ? 20 : 21;
        CHECK(animation_configure(&config,now));
        animation_next(now,&frame);
    }
    CHECK(frame.pixels[0][0].g==0);
    config=select_effect(EFFECT_SPARKLE,5);
    CHECK(animation_configure(&config,1));
    CHECK(!animation_next(1,&frame)); // identical settings produce no extra show
}

static void all_effect_recolor_replays(void) {
    for (effect_t effect=0;effect<EFFECT_COUNT;++effect) {
        effect_frame_t control,edited;
        animation_config_t config=select_effect(effect,5);
        uint64_t due=effect_interval_ms(effect)*20u;
        CHECK(animation_next(due,&control));
        config=select_effect(effect,5);
        config.color_mode=COLOR_CUSTOM; config.color=(rgb_t){21,199,70};
        CHECK(animation_configure(&config,100)); CHECK(animation_next(100,&edited));
        config.color_mode=COLOR_ORIGINAL;
        CHECK(animation_configure(&config,150)); CHECK(animation_next(150,&edited));
        CHECK(animation_next(due,&edited));
        CHECK(control.count==edited.count);
        for (unsigned n=0;n<control.count;++n)
            CHECK(!memcmp(control.pixels[n],edited.pixels[n],sizeof(control.pixels[n])));
    }
}

static void brightness_scaling(void) {
    animation_config_t config=select_effect(EFFECT_SOLID,100);
    config.color_mode=COLOR_CUSTOM; config.color=(rgb_t){255,127,40};
    effect_frame_t frame;
    for (unsigned percent=1;percent<=100;++percent) {
        config.brightness_percent=percent;
        CHECK(animation_configure(&config,percent));
        CHECK(animation_next(percent,&frame));
        unsigned scaled=percent*256/100;
        CHECK(frame.pixels[0][0].r==255*scaled/256);
        CHECK(frame.pixels[0][0].g==127*scaled/256);
        CHECK(frame.pixels[0][0].b==40*scaled/256);
    }
}

int main(void) {
    immediate_color_and_phase(); revisit_without_black_wait();
    sparkle_color_and_random_stream(); fractional_clock_and_noop();
    all_effect_recolor_replays();
    brightness_scaling();
    if (failures) { fprintf(stderr,"%u animation edit checks failed\n",failures); return 1; }
    puts("Animation edits: immediate colors, preserved phase/RNG, immediate revisits, fractional clock and brightness scaling OK");
}

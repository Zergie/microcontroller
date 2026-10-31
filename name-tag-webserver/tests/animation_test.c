#include "animations/animation_engine.h"
#include "api/animation_api.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void expect_unchanged(const char *uri, int status) {
    char before[ANIMATION_SNAPSHOT_CAPACITY], after[ANIMATION_SNAPSHOT_CAPACITY];
    assert(animation_api_snapshot(before, sizeof(before)));
    assert(animation_api_command(uri, 9000) == status);
    assert(animation_api_snapshot(after, sizeof(after)));
    assert(strcmp(before, after) == 0);
}

static void random_rounds(void) {
    animation_init(0, 1234);
    animation_config_t config = animation_config();
    assert(config.random && config.interval_seconds == 300 && config.brightness_percent == 20);
    config.interval_seconds = 5;
    assert(animation_configure(&config, 0));
    effect_frame_t frame;
    effect_t previous = EFFECT_COUNT;
    for (unsigned round = 0; round < 100; ++round) {
        unsigned seen = 0;
        for (unsigned step = 0; step < EFFECT_COUNT; ++step) {
            uint64_t now = (round * EFFECT_COUNT + step) * 5000ull;
            animation_next(now, &frame);
            effect_t current = animation_current();
            assert(current != previous && !(seen & (1u << current)));
            seen |= 1u << current;
            animation_next(now + 4999, &frame);
            assert(animation_current() == current);
            previous = current;
        }
        assert(seen == (1u << EFFECT_COUNT) - 1);
    }
    // A delayed loop changes only once, with no unbounded catch-up work.
    animation_next(UINT64_C(1000000000000), &frame);
    assert(animation_current() != previous);
}

static void effect_contracts(void) {
    animation_init(0,42);
    animation_config_t config=animation_config();
    config.random=false; config.effect=EFFECT_SOLID;
    assert(animation_configure(&config,0));
    effect_frame_t frame;
    assert(animation_next(0,&frame)); // auto_clear
    assert(animation_next(0,&frame)); // original Jade at 20%
    for (unsigned i=0;i<PIXEL_COUNT;++i) {
        assert(frame.pixels[0][i].r==0 && frame.pixels[0][i].g==50 && frame.pixels[0][i].b==7);
    }
    assert(!animation_next(999,&frame));
    config.brightness_percent=1;
    assert(animation_configure(&config,999));
    assert(animation_next(999,&frame)); // brightness change need not reset phase
    for (unsigned i=0;i<PIXEL_COUNT;++i)
        assert(frame.pixels[0][i].r==0 && frame.pixels[0][i].g==1 && frame.pixels[0][i].b==0);

    animation_init(0,42);
    config=animation_config(); config.random=false; config.effect=EFFECT_COMET;
    config.brightness_percent=100;
    assert(animation_configure(&config,0));
    assert(animation_next(0,&frame));
    assert(animation_next(0,&frame)); // first Comet frame is black
    assert(!animation_next(9,&frame));
    assert(animation_next(10,&frame)); // Comet uses the nominal 10 ms cadence.
    assert(frame.pixels[0][0].r>0);
    config.speed_percent=200;
    assert(animation_configure(&config,10));
    assert(!animation_next(10,&frame)); // speed-only edits do not redraw or reset phase
    assert(!animation_next(14,&frame));
    assert(animation_next(15,&frame)); // next 10 virtual milliseconds

    animation_init(0,42);
    config=animation_config(); config.random=false; config.effect=EFFECT_SPARKLE;
    assert(animation_configure(&config,0));
    assert(animation_next(0,&frame));
    assert(animation_next(0,&frame) && frame.count==2);
    assert(memcmp(frame.pixels[0],frame.pixels[1],sizeof(frame.pixels[0])));
    assert(!animation_next(98,&frame));
    assert(animation_next(99,&frame) && frame.count==2);
    assert(effect_id(EFFECT_COUNT)==NULL && effect_id((effect_t)-1)==NULL);
}

static void api_contracts(void) {
    animation_init(0,42);
    animation_api_set_boot_id(UINT64_C(0x0123456789abcdef));
    char status[ANIMATION_SNAPSHOT_CAPACITY];
    assert(animation_api_snapshot(status,sizeof(status)));
    assert(strstr(status,"\"boot_id\":\"0123456789abcdef\""));
    animation_api_set_boot_id(UINT64_C(0xfedcba9876543210));
    assert(animation_api_snapshot(status,sizeof(status)));
    assert(strstr(status,"\"boot_id\":\"fedcba9876543210\""));
    const char *invalid[] = {
        "/api/animation/set/solid/0/100/300/00ff64",
        "/api/animation/set/unknown/20/100/300/00ff64", "/api/animation/set/solid/101/100/300/00ff64",
        "/api/animation/set/solid/-1/100/300/00ff64", "/api/animation/set/solid/20/4/300/00ff64",
        "/api/animation/set/solid/20/801/300/00ff64", "/api/animation/set/solid/20/100/4/00ff64",
        "/api/animation/set/solid/20/100/3601/00ff64", "/api/animation/set/solid/20/100/300/ggffff",
        "/api/animation/set/solid/20/100/300/ffffff/extra", "/api/animation/set/solid/20/100/300/fffff",
        "/api/animation/set/solid//100/300/ffffff", "/api/animation/set/solid/20x/100/300/ffffff",
        "/api/animation/set/solid/999999999999999999999999999/100/300/ffffff",
        "/api/animation/set/solid/20/100", "/api/animation/set/", "/api/animation/set/////",
        "/api/animation/set/solid/20/100/300/ffffff?bad=1"
    };
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) expect_unchanged(invalid[i], 400);
    expect_unchanged("/api/no-such-command", 404);
    for (effect_t effect=EFFECT_AURORA;effect<EFFECT_COUNT;++effect) {
        char command[128];
        snprintf(command,sizeof(command),"/api/animation/set/%s/20/100/300/00ff28/original",effect_id(effect));
        assert(animation_api_command(command,9000)==200 && animation_current()==effect);
    }
    expect_unchanged("/api/animation/set/meteor-shower/20/100/300/00ff28/original",400);
    expect_unchanged("/api/animation/set/raindrops/20/100/300/00ff28/original",400);
    char long_uri[256]; memset(long_uri, 'a', sizeof(long_uri));
    memcpy(long_uri, "/api/animation/set/", 19); long_uri[255] = 0;
    expect_unchanged(long_uri, 400);
    assert(animation_api_command("/api/animation/set/random/100/800/3600/aAbBcC", 0) == 200);
    animation_config_t config = animation_config();
    assert(config.random && config.color.r == 170 && config.color.g == 187 && config.color.b == 204);
    assert(animation_api_command("/api/animation/set/pulse/1/5/5/000000", 0) == 200);
    assert(animation_config().color_mode==COLOR_CUSTOM && animation_config().speed_percent==5);
    assert(animation_api_command("/api/animation/set/random/20/100/300/00ff28/original", 1)==200);
    assert(animation_config().color_mode==COLOR_ORIGINAL);
    expect_unchanged("/api/animation/set/solid/20/100/300/ffffff/original/extra",400);
    assert(!animation_configure(NULL, 1));
}

static bool equal_color(rgb_t a, rgb_t b) { return a.r==b.r && a.g==b.g && a.b==b.b; }

static void random_color_contracts(void) {
    animation_init(0,42);
    animation_config_t config=animation_config();
    config.random=false; config.effect=EFFECT_SOLID; config.color_mode=COLOR_RANDOM;
    config.brightness_percent=100;
    assert(animation_configure(&config,0));
    rgb_t initial=animation_color();
    assert((unsigned)initial.r+initial.g+initial.b==255);
    effect_frame_t frame;
    animation_next(0,&frame); animation_next(0,&frame);
    assert(equal_color(frame.pixels[0][0],initial));
    for (uint64_t now=1;now<100;++now) {
        config.brightness_percent=(unsigned)now;
        config.speed_percent=100+(unsigned)now;
        assert(animation_configure(&config,now));
        animation_next(now,&frame);
        assert(equal_color(initial,animation_color()));
    }
    config.color_mode=(color_mode_t)42;
    assert(!animation_configure(&config,100));
    config.color_mode=COLOR_RANDOM; config.random=true; config.interval_seconds=5;
    assert(animation_configure(&config,100));
    unsigned changes=0;
    rgb_t previous=animation_color();
    for (unsigned i=1;i<=30;++i) {
        animation_next(100+i*5000ull,&frame);
        rgb_t next=animation_color();
        changes+=!equal_color(previous,next);
        animation_next(101+i*5000ull,&frame);
        assert(equal_color(next,animation_color()));
        previous=next;
    }
    assert(changes>20);
    assert(animation_api_command("/api/animation/set/solid/20/100/300/123456/random",160000)==200);
    assert(animation_config().color_mode==COLOR_RANDOM);
    assert(animation_api_command("/api/animation/set/solid/20/100/300/123456/random/extra",160000)==400);
}

static void snapshot_bounds(void) {
    char expected[ANIMATION_SNAPSHOT_CAPACITY]; assert(animation_api_snapshot(expected, sizeof(expected)));
    size_t required = strlen(expected) + 1;
    for (size_t capacity = 0; capacity <= ANIMATION_SNAPSHOT_CAPACITY; ++capacity) {
        unsigned char guarded[ANIMATION_SNAPSHOT_CAPACITY+2]; memset(guarded, 0xa5, sizeof(guarded));
        bool success = animation_api_snapshot((char *)guarded + 1, capacity);
        assert(success == (capacity >= required));
        assert(guarded[0] == 0xa5 && guarded[capacity + 1] == 0xa5);
        if (success) assert(!strcmp((char *)guarded + 1, expected));
    }
}

int main(int argc, char **argv) {
    animation_init(0, 42);
    if (argc == 2 && !strcmp(argv[1], "--json")) {
        char json[ANIMATION_SNAPSHOT_CAPACITY]; assert(animation_api_snapshot(json, sizeof(json))); puts(json); return 0;
    }
    random_rounds(); effect_contracts(); api_contracts(); random_color_contracts(); snapshot_bounds();
    puts("Animations/API: all 21 effects, 100 shuffle rounds, intervals, brightness, atomic rejection and JSON bounds OK");
}

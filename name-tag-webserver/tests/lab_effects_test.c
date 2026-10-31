#include "animations/effects.h"
#include "animations/lab_effects.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void contracts(void) {
    static const char *ids[]={"aurora","tidal","fireflies","molten","ripple","heartbeat","prism-weave","twin-orbit"};
    const uint64_t times[]={0,137,500,1000,1600,3200,4800,6000,8000,12345};
    _Static_assert(EFFECT_COUNT==21,"Only the eight selected concepts are added");
    for (unsigned e=0;e<8;++e) {
        effect_t effect=(effect_t)(EFFECT_AURORA+e);
        assert(!strcmp(effect_id(effect),ids[e]));
        assert(effect_interval_ms(effect)==33);
        rgb_t first[PIXEL_COUNT],pixels[PIXEL_COUNT]; bool changed=false;
        for (unsigned n=0;n<sizeof(times)/sizeof(*times);++n) {
            lab_effect_render(effect,times[n],effect_default_color(effect),true,pixels);
            if (!n) memcpy(first,pixels,sizeof(first));
            else changed|=memcmp(first,pixels,sizeof(first))!=0;
            // Custom green preserves each LED's envelope and never leaks other colors.
            lab_effect_render(effect,times[n],(rgb_t){0,255,0},false,pixels);
            for (unsigned i=0;i<PIXEL_COUNT;++i) assert(!pixels[i].r && !pixels[i].b);
            lab_effect_render(effect,times[n],(rgb_t){0},false,pixels);
            for (unsigned i=0;i<PIXEL_COUNT;++i) assert(!pixels[i].r && !pixels[i].g && !pixels[i].b);
        }
        assert(changed);
        lab_effect_render(effect,UINT64_MAX,(rgb_t){255,255,255},true,pixels);
        uint64_t clock=17;
        for (unsigned n=0;n<1000;++n) {
            clock=clock*UINT64_C(6364136223846793005)+1;
            lab_effect_render(effect,clock,(rgb_t){255,255,255},true,pixels);
        }
        effect_state_t state; effect_frame_t frame; uint32_t rng=42;
        effect_init(&state,effect);
        assert(effect_step(&state,effect,0,effect_default_color(effect),true,&rng,&frame));
        assert(frame.count==1 && rng==42);
        assert(!effect_step(&state,effect,32,effect_default_color(effect),true,&rng,&frame));
        assert(effect_step(&state,effect,33,effect_default_color(effect),true,&rng,&frame));
    }
}
int main(int argc,char **argv) {
    if (argc==2 && !strcmp(argv[1],"--trace")) {
        const uint64_t times[]={0,137,500,1000,1600,3200,4800,6000,8000,12345};
        for (effect_t e=EFFECT_AURORA;e<EFFECT_COUNT;++e) for (unsigned n=0;n<sizeof(times)/sizeof(*times);++n) {
            // Keep the frozen concept frames; compare at proportionally earlier
            // integer timestamps to prove speed changes without palette drift.
            unsigned speed=(e==EFFECT_AURORA || e==EFFECT_TIDAL) ? 6 :
                (e==EFFECT_FIREFLIES || e==EFFECT_MOLTEN || e==EFFECT_RIPPLE || e==EFFECT_PRISM_WEAVE) ? 2 : 1;
            if (times[n]%speed) continue;
            rgb_t pixels[PIXEL_COUNT]; lab_effect_render(e,times[n]/speed,effect_default_color(e),true,pixels);
            printf("%s %llu",effect_id(e),(unsigned long long)times[n]);
            for (unsigned i=0;i<PIXEL_COUNT;++i) printf(" %u %u %u",pixels[i].r,pixels[i].g,pixels[i].b);
            putchar('\n');
        }
        return 0;
    }
    contracts();
    puts("Animation Lab: eight selected effects, independent envelopes, custom colors, bounded clocks and 33 ms pacing OK");
}

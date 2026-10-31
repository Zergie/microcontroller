// Original Animation Lab concepts; bounded single-frame rendering on core 0.
#include "lab_effects.h"
#include "lab_curves.h"
#include "firefly_parameters.h"

#define TAU 6.283185307179586f
#define TURN UINT64_C(628318531)
typedef struct { float r,g,b; } light_t;
static float clamp(float x) { return x<0 ? 0 : x>1 ? 1 : x; }
static float absolute(float x) { return x<0 ? -x : x; }
static float fraction(float x) { x-=(int64_t)x; return x<0 ? x+1 : x; }
static float wave(float angle) {
    float at=fraction(angle/TAU)*256;
    unsigned i=(unsigned)at;
    if (i>=256) return 0; // A tiny negative angle can round its wrapped phase to 1.
    return (sine_curve[i]+(sine_curve[i+1]-sine_curve[i])*(at-i))/32767.0f;
}
// Reduce time in integer arithmetic before converting to float. This keeps
// smooth phase after days of uptime and never overflows even at UINT64_MAX.
static float moving_wave(float spatial,int rate,uint64_t ms) {
    unsigned magnitude=(unsigned)(rate<0 ? -rate : rate);
    float phase=(float)(((ms%TURN)*magnitude*1000)%TURN)/100000000.0f;
    return wave(spatial+(rate<0 ? -phase : phase));
}
static float gaussian(float distance,float width) {
    float at=absolute(distance)/width*16;
    if (at>=128) return 0;
    unsigned i=(unsigned)at;
    return (gaussian_curve[i]+((float)gaussian_curve[i+1]-gaussian_curve[i])*(at-i))/65535.0f;
}
static light_t blend(light_t a,light_t b,float p) {
    p=clamp(p);
    return (light_t){a.r+(b.r-a.r)*p,a.g+(b.g-a.g)*p,a.b+(b.b-a.b)*p};
}
static light_t dim(light_t c,float v) { v=clamp(v); return (light_t){c.r*v,c.g*v,c.b*v}; }
static light_t hue(float h,float saturation) {
    const float offsets[]={0,2.0f/3,1.0f/3};
    float channels[3];
    for (unsigned i=0;i<3;++i)
        channels[i]=255*(1-saturation+saturation*clamp(absolute(fraction(h+offsets[i])*6-3)-1));
    return (light_t){channels[0],channels[1],channels[2]};
}
static light_t render(effect_t effect,uint64_t ms,float x) {
    switch (effect) {
    case EFFECT_AURORA: {
        float a=(moving_wave(x*.52f,-312,ms)+1)/2;
        float b=(moving_wave(x*.93f,186,ms)+1)/2;
        return dim(blend((light_t){39,240,167},(light_t){157,60,242},b),.08f+.75f*a*a);
    }
    case EFFECT_TIDAL: {
        float front=(wave(((ms%8000)*6%8000)*TAU/8000-TAU/4)+1)*4.5f;
        light_t water=dim((light_t){10,83,210},clamp(front-x+1)*.23f);
        return blend(water,(light_t){111,255,219},gaussian(x-front,.65f)*.85f);
    }
    case EFFECT_FIREFLIES: {
        unsigned index=(unsigned)(x+.5f);
        if (index>9) index=9;
        const float *fly=fireflies[index];
        uint64_t period=(uint64_t)(fly[0]*1000000+.5f);
        float phase=fraction((float)(((ms%period)*2000)%period)/period+fly[1]);
        float level=phase<.36f ? wave(phase/.36f*TAU/2) : 0;
        return dim(blend((light_t){171,255,44},(light_t){255,182,45},fly[2]),level*level*.95f);
    }
    case EFFECT_MOLTEN: {
        float heat=clamp(.25f+.24f*moving_wave(x*1.7f,440,ms)+
            .18f*moving_wave(x*.8f,-620,ms)+.2f*moving_wave(x*2.9f,180,ms));
        return dim(blend((light_t){155,9,1},(light_t){255,174,19},heat*heat),.22f+.78f*heat);
    }
    case EFFECT_RIPPLE: {
        float p=((ms%4800)*2%4800)*.0016f,d=absolute(x-4.5f);
        return blend(dim((light_t){55,47,190},gaussian(d-p+1.3f,.6f)*.38f),
                     (light_t){126,221,255},gaussian(d-p,.48f)*clamp(1-p/8));
    }
    case EFFECT_HEARTBEAT: {
        float d=absolute(x-4.5f),p=(ms%3200)*.001f-d*.035f;
        float level=gaussian(p-.32f,.14f)+.65f*gaussian(p-.76f,.16f);
        return dim(blend((light_t){247,47,107},(light_t){255,201,209},level*.5f),
                   clamp(level)*(.7f+.3f*(1-d/5)));
    }
    case EFFECT_PRISM_WEAVE: {
        bool odd=((unsigned)(x+.5f)%2)!=0;
        float level=.2f+.8f*(moving_wave(x*.7f,odd ? -180 : 180,ms)+1)/2;
        float h=.47f+.46f*(moving_wave(x*.42f,odd ? -76 : 140,ms)+1)/2;
        return dim(hue(h,.78f),level);
    }
    case EFFECT_TWIN_ORBIT: {
        float a=(ms%6000)*10.0f/6000,b=10-a;
        if (b>=10) b=0;
        float da=absolute(x-a),db=absolute(x-b);
        if (da>5) da=10-da;
        if (db>5) db=10-db;
        light_t warm=dim((light_t){255,148,44},gaussian(da,.8f));
        light_t cool=dim((light_t){34,176,255},gaussian(db,.8f));
        return (light_t){warm.r+cool.r,warm.g+cool.g,warm.b+cool.b};
    }
    default: return (light_t){0};
    }
}
static uint8_t channel(float value) { return (uint8_t)(255*clamp(value/255)+.5f); }
void lab_effect_render(effect_t effect,uint64_t ms,rgb_t color,bool original_colors,rgb_t pixels[PIXEL_COUNT]) {
    for (unsigned pin=0;pin<PIXEL_COUNT;++pin) {
        float x=PIXEL_COUNT>1 ? (PIXEL_COUNT-1-pin)*9.0f/(PIXEL_COUNT-1) : 4.5f;
        light_t light=render(effect,ms,x);
        if (!original_colors) {
            float peak=light.r>light.g ? light.r : light.g;
            if (light.b>peak) peak=light.b;
            light=dim((light_t){color.r,color.g,color.b},peak/255);
        }
        pixels[pin]=(rgb_t){channel(light.r),channel(light.g),channel(light.b)};
    }
}

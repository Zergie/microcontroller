#include "settings/settings.h"
#include "pads/pads.h"
#include "animations/animation_engine.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t flash[SETTINGS_BYTES];
static unsigned programs, erases;
static unsigned cut=SETTINGS_PAGE_SIZE;
static bool fail_erase, report_program_failure;
static bool erase(unsigned sector) {
    assert(settings_saving());
    assert(sector<2);
    ++erases;
    if (fail_erase) return false;
    memset(flash+sector*SETTINGS_SECTOR_SIZE,0xff,SETTINGS_SECTOR_SIZE);
    return true;
}
static bool program(unsigned page, const uint8_t *data) {
    assert(settings_saving());
    assert(page<32);
    ++programs;
    for (unsigned i=0;i<cut;++i) {
        uint8_t *byte=flash+page*SETTINGS_PAGE_SIZE+i;
        assert((*byte&data[i])==data[i]); // Flash cannot change 0 back to 1.
        *byte&=data[i];
    }
    return cut==SETTINGS_PAGE_SIZE && !report_program_failure;
}
static const settings_storage_t storage={flash,erase,program};
static void boot(uint64_t now) {
    animation_init(now,42);
    pads_init();
    settings_init(&storage,now);
}
static void fresh(void) {
    memset(flash,0xff,sizeof(flash)); programs=erases=0;
    cut=SETTINGS_PAGE_SIZE; fail_erase=report_program_failure=false;
    boot(0);
}
static void change(unsigned brightness, uint64_t now) {
    animation_config_t config=animation_config(); config.brightness_percent=brightness;
    assert(animation_configure(&config,now)); settings_poll(now);
}

static void lifecycle(void) {
    fresh();
    assert(!settings_saved());
    assert(!settings_saving());
    settings_poll(9990); assert(programs==0);
    assert(!settings_saving());
    settings_poll(10000); assert(programs==1 && settings_saved());
    assert(!settings_saving());
    animation_config_t config={
        .random=false, .effect=EFFECT_CUSTOM_COLOR_CHASE, .brightness_percent=7,
        .speed_percent=800, .interval_seconds=123, .color={47,202,109}, .color_mode=COLOR_CUSTOM
    };
    assert(animation_configure(&config,11000));
    assert(!settings_saved()); settings_poll(11000);
    settings_poll(20990); assert(programs==1);
    settings_poll(21000); assert(programs==2 && settings_saved());
    boot(30000);
    animation_config_t restored=animation_config();
    assert(settings_saved() && !restored.random && restored.effect==EFFECT_CUSTOM_COLOR_CHASE);
    assert(restored.brightness_percent==7 && restored.speed_percent==800 && restored.interval_seconds==123);
    assert(restored.color.r==47 && restored.color.g==202 && restored.color.b==109 && restored.color_mode==COLOR_CUSTOM);
    settings_poll(100000); assert(programs==2); // No write just for boot/polling.
    change(8,110000); change(7,115000); settings_poll(200000);
    assert(programs==2 && settings_saved()); // Changing back cancels pending save.
    config=animation_config(); config.random=true; config.color_mode=COLOR_ORIGINAL;
    assert(animation_configure(&config,210000)); settings_poll(210000); settings_poll(220000);
    boot(230000); restored=animation_config();
    assert(restored.random && restored.color_mode==COLOR_ORIGINAL && restored.brightness_percent==7);
    config=animation_config(); config.color_mode=COLOR_RANDOM;
    assert(animation_configure(&config,240000)); settings_poll(240000); settings_poll(250000);
    boot(260000); assert(animation_config().color_mode==COLOR_RANDOM && settings_saved());
    unsigned committed=programs;
    effect_frame_t frame;
    for (unsigned i=1;i<5;++i) {
        animation_next(260000+i*123000ull,&frame); settings_poll(260000+i*123000ull);
        assert(settings_saved() && programs==committed); // Runtime color is not persisted.
    }

}

static void batching(void) {
    fresh(); settings_poll(10000);
    for (unsigned i=0;i<10;++i) change(i+1,11000+i*100);
    assert(programs==1);
    settings_poll(21899); assert(programs==1);
    settings_poll(21900); assert(programs==2);
    boot(25000); assert(animation_config().brightness_percent==10);

    // A change one millisecond before the deadline starts a full new 10 s.
    change(20,30000); settings_poll(39999); assert(programs==2);
    change(21,39999);
    settings_poll(40000); settings_poll(49998); assert(programs==2);
    settings_poll(49999); assert(programs==3 && settings_saved());

    // Editing for a full minute must never trigger a forced intermediate save.
    for (unsigned i=0;i<=60;++i) {
        change(30+i,60000+i*1000);
        assert(programs==3);
    }
    settings_poll(129999); assert(programs==3);
    settings_poll(130000); assert(programs==4 && settings_saved());
    boot(140000); assert(animation_config().brightness_percent==90);

    // Repeating the same value is not a change and does not restart the timer.
    change(31,150000); change(31,159999);
    settings_poll(160000); assert(programs==5 && settings_saved());
}

static void pads_persistence(void) {
    fresh(); settings_poll(10000);
    pads_config_t config=pads_config(); config.fade_ms=2000;
    for (unsigned i=0;i<PAD_COUNT;++i) config.colors[i]=(rgb_t){i*17,255-i*10,i*21};
    assert(pads_apply(3,&config,11000)); settings_poll(11000);
    assert(!settings_saved());
    assert(pads_apply(3,&config,11500)); settings_poll(11500);
    assert(pads_apply(3,&config,12000)); settings_poll(12000);
    settings_poll(20999); assert(programs==1);
    settings_poll(21000); assert(programs==2 && settings_saved());
    boot(30000);
    pads_config_t restored=pads_config();
    assert(pads_active() && pads_config_equal(&config,&restored));
    assert(pads_pressed(30000)==0 && settings_saved()); // Never restore held fingers.
    effect_frame_t frame; assert(pads_render(30000,100,&frame));
    for (unsigned i=0;i<PIXEL_COUNT;++i) assert(!frame.pixels[0][i].r && !frame.pixels[0][i].g && !frame.pixels[0][i].b);
    pads_set_active(false,31000); settings_poll(31000); settings_poll(41000);
    boot(50000); assert(!pads_active());
    restored=pads_config(); assert(pads_config_equal(&config,&restored));
}

static void song_preferences(void) {
    fresh();
    pads_config_t config=pads_config(); config.fade_ms=1700;
    assert(pads_configure(&config)); pads_set_active(true,0);
    settings_poll(0); settings_poll(10000);
    assert(settings_saved()); unsigned baseline=programs;
    for (uint64_t now=11000;now<60000;now+=500) {
        assert(pads_pulse(1u<<((now/500)%8),536,now));
        settings_poll(now);
        assert(settings_saved() && programs==baseline);
    }
    pads_release(60000); settings_poll(70000); // Background/connection interruption.
    boot(80000);
    pads_config_t restored=pads_config();
    assert(pads_config_equal(&config,&restored) && !pads_pressed(80000));
    assert(programs==baseline && settings_saved());
}

static void lab_selection_persistence(void) {
    for (effect_t effect=EFFECT_AURORA;effect<EFFECT_COUNT;++effect) {
        fresh();
        animation_config_t config=animation_config();
        config.random=false; config.effect=effect;
        config.color_mode=COLOR_RANDOM; config.speed_percent=800;
        assert(animation_configure(&config,0));
        settings_poll(0); settings_poll(10000);
        boot(20000);
        animation_config_t restored=animation_config();
        assert(!restored.random && restored.effect==effect && restored.color_mode==COLOR_RANDOM);
        assert(restored.speed_percent==800 && settings_saved());
    }
}

static void torn_writes(void) {
    fresh(); settings_poll(10000);
    uint8_t baseline[SETTINGS_BYTES]; memcpy(baseline,flash,sizeof(flash));
    for (unsigned length=0;length<SETTINGS_PAGE_SIZE;++length) {
        memcpy(flash,baseline,sizeof(flash)); cut=length;
        boot(0); change(37,100); settings_poll(10100);
        assert(settings_error() && !settings_saved());
        cut=SETTINGS_PAGE_SIZE;
        boot(20000); assert(animation_config().brightness_percent==20 && settings_saved());
        change(38,20100); settings_poll(30100); assert(settings_saved());
        boot(40000); assert(animation_config().brightness_percent==38);
    }
    fresh(); settings_poll(10000);
    report_program_failure=true;
    change(37,11000); settings_poll(21000);
    assert(settings_saved() && !settings_error()); // Readback proves completed write.
    report_program_failure=false;
    change(38,22000); settings_poll(32000);
    boot(40000); assert(animation_config().brightness_percent==38);
}

static void rotation_and_corruption(void) {
    fresh(); settings_poll(10000);
    uint64_t now=20000;
    for (unsigned i=0;i<80;++i) {
        change(1+i%100,now); settings_poll(now+10000); now+=20000;
        boot(now); assert(animation_config().brightness_percent==1+i%100);
    }
    assert(erases==5); // One erase per sector rotation, not per setting.
    // Start with a full first sector: power loss during rotation retains it.
    fresh(); settings_poll(10000);
    for (unsigned i=0;i<15;++i) { change(i+1,20000+i*20000); settings_poll(30000+i*20000); }
    uint8_t baseline[SETTINGS_BYTES]; memcpy(baseline,flash,sizeof(flash));
    for (unsigned length=0;length<SETTINGS_PAGE_SIZE;++length) {
        memcpy(flash,baseline,sizeof(flash)); cut=length;
        boot(400000); change(50,400100); settings_poll(410100);
        cut=SETTINGS_PAGE_SIZE; boot(420000);
        assert(animation_config().brightness_percent==15);
    }
    memcpy(flash,baseline,sizeof(flash)); boot(0); fail_erase=true;
    change(50,100); settings_poll(10100); assert(settings_error());
    unsigned attempts=erases; settings_poll(10110); assert(erases==attempts);
    fail_erase=false; settings_poll(15099); assert(!settings_saved());
    settings_poll(15100); assert(settings_saved());
    boot(70000); assert(animation_config().brightness_percent==50);

    fresh(); settings_poll(10000); change(30,11000); settings_poll(21000);
    uint8_t valid[SETTINGS_BYTES]; memcpy(valid,flash,sizeof(flash));
    for (unsigned byte=0;byte<SETTINGS_PAGE_SIZE;++byte) {
        memcpy(flash,valid,sizeof(flash)); flash[SETTINGS_PAGE_SIZE+byte]^=1;
        boot(30000); assert(animation_config().brightness_percent==20);
    }
    memset(flash,0x35,sizeof(flash)); boot(0);
    assert(animation_config().brightness_percent==20 && !settings_saved());
    settings_poll(10000); assert(settings_saved()); // Recover from invalid data.
}

int main(void) {
    lifecycle(); batching(); pads_persistence(); song_preferences(); lab_selection_persistence(); torn_writes(); rotation_and_corruption();
    puts("Settings: reboot restoration, batching, no-op writes, 512 torn writes, CRC corruption, sector rotation and failed-write recovery OK");
}

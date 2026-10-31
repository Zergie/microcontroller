#include "pads/pads.h"
#include "api/pads_api.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

// Pads API exposes storage status but does not own persistence.
bool settings_saved(void) { return true; }
bool settings_error(void) { return false; }
int main(void) {
    pads_init();
    pads_config_t config=pads_config();
    for (unsigned i=0;i<PAD_COUNT;++i) config.colors[i]=(rgb_t){255,100,0};
    config.fade_ms=1000;
    effect_frame_t frame;
    assert(pads_apply(0x81,&config,0)); // First and eighth together.
    assert(pads_render(0,100,&frame));
    assert(frame.pixels[0][0].r==255 && frame.pixels[0][7].r==255);
    for (unsigned i=1;i<7;++i) assert(!frame.pixels[0][i].r);
    for (unsigned i=8;i<PIXEL_COUNT;++i) assert(!frame.pixels[0][i].r);
    assert(pads_apply(0x80,&config,100)); // Release one while the other stays held.
    assert(pads_render(600,100,&frame));
    assert(frame.pixels[0][0].r==127 && frame.pixels[0][7].r==255);
    assert(pads_apply(0x80,&config,700)); // Heartbeat must not restart the first fade.
    assert(pads_render(1100,100,&frame)); assert(!frame.pixels[0][0].r);
    assert(pads_render(1100,50,&frame)); assert(frame.pixels[0][7].r==127);
    assert(pads_apply(0,&config,1200));
    assert(pads_render(1700,100,&frame)); assert(frame.pixels[0][7].r==127);
    assert(pads_apply(0x80,&config,1750)); // Re-press interrupts fade at full brightness.
    assert(pads_render(1750,100,&frame)); assert(frame.pixels[0][7].r==255);
    // No release arrives: lease expires at 2950, then fade finishes at 3950.
    assert(pads_render(3450,100,&frame)); assert(frame.pixels[0][7].r==127);
    assert(pads_pressed(3450)==0);
    assert(pads_render(3950,100,&frame)); assert(!frame.pixels[0][7].r);
    assert(!pads_render(5000,100,&frame));
    config.fade_ms=0; assert(pads_apply(255,&config,6000));
    assert(pads_apply(0,&config,6001));
    assert(pads_render(6001,100,&frame));
    for (unsigned i=0;i<PIXEL_COUNT;++i) assert(!frame.pixels[0][i].r);
    pads_set_active(false,7000); assert(!pads_active() && !pads_render(7000,100,&frame));

    const char *valid="/api/pads/set/129/1500/ff000000ff000000ffffffff00ff00ff00ffffffffff0000";
    assert(pads_api_command(valid,8000)==200 && pads_pressed(8000)==129);
    pads_config_t before=pads_config();
    const char *bad[]={"/api/pads/set/1024/1000/ff000000ff000000ffffffff00ff00ff00ffffffffff0000",
        "/api/pads/set/0/2001/ff000000ff000000ffffffff00ff00ff00ffffffffff0000",
        "/api/pads/set/-1/1000/ff000000ff000000ffffffff00ff00ff00ffffffffff0000",
        "/api/pads/set/1/1000/ff0000", "/api/pads/set/0/1000/zz000000ff000000ffffffff00ff00ff00ffffffffff0000"};
    for (unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) {
        assert(pads_api_command(bad[i],8001)==400);
        pads_config_t after=pads_config(); assert(pads_config_equal(&before,&after) && pads_pressed(8001)==129);
    }
    char json[1024]; assert(pads_api_snapshot(json,sizeof(json),8001));
    size_t needed=strlen(json)+1;
    for (size_t capacity=0;capacity<=needed;++capacity) {
        unsigned char guard[1026]; memset(guard,0xa5,sizeof(guard));
        assert(pads_api_snapshot((char *)guard+1,capacity,8001)==(capacity>=needed));
        assert(guard[0]==0xa5 && guard[capacity+1]==0xa5);
    }
    assert(pads_api_command("/api/pads/config/2000/ff000000ff000000ffffffff00ff00ff00ffffffffff0000",8050)==200);
    assert(pads_pressed(8050)==129 && pads_config().fade_ms==2000);
    assert(pads_set_pressed(8,8060) && pads_pressed(8060)==8);
    before=pads_config();
    assert(pads_set_pressed(16,8070));
    pads_config_t unchanged=pads_config(); assert(pads_config_equal(&before,&unchanged));
    assert(pads_api_command("/api/pads/config/2001/ff000000ff000000ffffffff00ff00ff00ffffffffff0000",8080)==400);
    assert(pads_pressed(8080)==16);
    assert(pads_api_command("/api/pads/release",8100)==200 && !pads_pressed(8100));
    assert(pads_api_command("/api/view/animations",8200)==200 && !pads_active());
    assert(!pads_set_pressed(255,8201) && !pads_active());
    // A song pulse releases immediately and fades for the captured note length,
    // even if the next note's duration is configured while it is fading.
    const unsigned note_lengths[]={536,1071,1607};
    for (unsigned i=0;i<3;++i) {
        pads_init(); config=pads_config(); config.fade_ms=note_lengths[i];
        config.colors[0]=(rgb_t){255,0,0};
        assert(pads_apply(1,&config,0));
        assert(pads_set_pressed(0,0));
        assert(pads_render(0,100,&frame) && frame.pixels[0][0].r==255);
        config.fade_ms=1; assert(pads_configure(&config));
        assert(pads_render(note_lengths[i]/2,100,&frame));
        assert(frame.pixels[0][0].r>=127 && frame.pixels[0][0].r<=128);
        assert(pads_render(note_lengths[i],100,&frame) && frame.pixels[0][0].r==0);
    }
    pads_init(); config=pads_config(); config.colors[0]=(rgb_t){255,0,0};
    assert(pads_configure(&config));
    assert(!pads_pulse(1,500,0)); // A note cannot activate a closed view.
    pads_set_active(true,0);
    assert(pads_api_command("/api/pads/pulse/1/500",100)==200);
    assert(pads_pressed(100)==0);
    assert(pads_render(100,100,&frame) && frame.pixels[0][0].r==255);
    assert(pads_render(350,100,&frame) && frame.pixels[0][0].r==127);
    assert(pads_pulse(2,1000,350)); // Another note does not restart the first envelope.
    assert(pads_render(600,100,&frame) && !frame.pixels[0][0].r);
    pads_config_t after_pulse=pads_config(); assert(pads_config_equal(&config,&after_pulse));
    const char *bad_pulses[]={"/api/pads/pulse/0/100", "/api/pads/pulse/1024/100",
        "/api/pads/pulse/1/2001", "/api/pads/pulse/1/500/", "/api/pads/pulse/-1/500"};
    for (unsigned i=0;i<sizeof(bad_pulses)/sizeof(*bad_pulses);++i)
        assert(pads_api_command(bad_pulses[i],600)==400);
    assert(pads_pulse(1,0,700));
    assert(pads_render(700,100,&frame) && !frame.pixels[0][0].r);
    // Ten-color HTTP configuration, upper mask bits, fade and atomic rejection.
    const char *ten="/api/pads/set/1023/1000/ff000000ff000000ffffffff00ff00ff00ffffffffff0000123456abcdef";
    assert(pads_api_command(ten,9000)==200 && pads_pressed(9000)==1023);
    assert(pads_render(9000,100,&frame));
    assert(frame.pixels[0][8].r==0x12 && frame.pixels[0][9].b==0xef);
    before=pads_config();
    assert(pads_api_command(valid,9010)==200); // Legacy colors retain LED 9/10.
    config=pads_config(); assert(!memcmp(config.colors+8,before.colors+8,2*sizeof(rgb_t)));
    assert(pads_set_pressed(768,9020));
    assert(pads_pulse(768,1000,9030) && !pads_pressed(9030));
    assert(pads_render(9530,100,&frame) && frame.pixels[0][9].r==0xab/2);
    assert(!pads_set_pressed(1024,9540) && !pads_pulse(1024,100,9540));
    assert(pads_api_snapshot(json,sizeof(json),9540) && strstr(json,"\"pad_count\":10"));
    puts("LED pads: concurrent holds, independent fades, re-press, global brightness, lease expiry, ten-LED masks/colors, atomic API and JSON bounds OK");
}

#include "network_runtime.h"
#include "animations/animation_engine.h"
#include "api/animation_api.h"
#include "api/system_api.h"
#include "hardware/clocks.h"
#include "pico/version.h"
#include "pixels/pixels.h"
#include "board_config.h"
#include "settings/flash_storage.h"
#include "settings/settings.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "pico/rand.h"
#include "pads/pads.h"
#include "pads/ws_server.h"

int main(void) {
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    uint32_t seed = 0;
    for (unsigned i = 0; i < sizeof(id.id); ++i) seed = seed * 33u + id.id[i];
    char board_id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
    pico_get_unique_board_id_string(board_id, sizeof(board_id));
    system_api_init(board_id, clock_get_hz(clk_sys), PICO_FLASH_SIZE_BYTES, PICO_SDK_VERSION_STRING);
    pixels_init();
    animation_api_set_boot_id(((uint64_t)get_rand_32()<<32)|get_rand_32());
    animation_init(time_us_64() / 1000, seed ^ get_rand_32() ^ (uint32_t)time_us_64());
    pads_init();
    settings_flash_init(time_us_64()/1000);
    if (!network_runtime_init("rp2040")) return -1;
    if (!pad_ws_server_init()) return -1;
    effect_frame_t frame={0};
    unsigned transmitting=0;
    bool manual=pads_active();
    pixels_set_pad_mode(manual);
    while (true) {
        network_runtime_poll();
        settings_poll(time_us_64()/1000);
        pixels_poll();
        if (manual!=pads_active()) {
            manual=pads_active(); frame.count=transmitting=0;
            pixels_set_pad_mode(manual);
            if (!manual) animation_refresh();
        }
        if (manual) {
            if (pads_render(time_us_64()/1000,animation_config().brightness_percent,&frame)) transmitting=0;
        } else if (transmitting == frame.count) {
            bool drawn=animation_next(time_us_64()/1000,&frame);
            if (drawn) transmitting=0;
        }
        if (transmitting < frame.count && pixels_submit(frame.pixels[transmitting])) ++transmitting;
    }
}

#include "pixels.h"
#include "board_config.h"
#include "ws2812.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

_Static_assert(PIXEL_COUNT > 0 && PIXEL_COUNT <= 256, "Supported pixel count: 1..256");
static PIO pio = pio0;
static unsigned sm;
static int dma;
static uint32_t words[PIXEL_COUNT];
static bool in_flight;
static bool draining;
static uint64_t ready_at;
static uint64_t finished_at;
static bool pad_mode;

static uint64_t next_start_time(uint64_t finished_at) {
    // CircuitPython 9.2.4's RP2040 neopixel_write waits until raw_ticks + 2
    // after TXSTALL. Use its supervisor/port.c tick conversion, including the
    // slightly shorter final tick of each second, rather than a 300 us delay.
    uint64_t ticks=1024*(finished_at/1000000)+(finished_at%1000000)/977+2;
    return (ticks/1024)*1000000+(ticks%1024)*977;
}

void pixels_set_pad_mode(bool enabled) {
    if (pad_mode==enabled) return;
    pad_mode=enabled;
    if (!in_flight) ready_at=enabled ? finished_at+PIXEL_PAD_RESET_US : next_start_time(finished_at);
}

void pixels_init(void) {
    sm = pio_claim_unused_sm(pio, true);
    unsigned offset = pio_add_program(pio, &ws2812_program);
    pio_gpio_init(pio, PIXEL_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, PIXEL_PIN, 1, true);
    pio_sm_config config = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&config, PIXEL_PIN);
    sm_config_set_out_shift(&config, false, true, PIXEL_RGBW ? 32 : 24);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
    unsigned cycles = ws2812_T1 + ws2812_T2 + ws2812_T3;
    sm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / (800000 * cycles));
    pio_sm_init(pio, sm, offset, &config);
    pio_sm_set_enabled(pio, sm, true);

    dma = dma_claim_unused_channel(true);
    dma_channel_config transfer = dma_channel_get_default_config(dma);
    channel_config_set_transfer_data_size(&transfer, DMA_SIZE_32);
    channel_config_set_read_increment(&transfer, true);
    channel_config_set_write_increment(&transfer, false);
    channel_config_set_dreq(&transfer, pio_get_dreq(pio, sm, true));
    dma_channel_configure(dma, &transfer, &pio->txf[sm], words, PIXEL_COUNT, false);
}

void pixels_poll(void) {
    if (!in_flight) return;
    const uint32_t stalled=1u << (PIO_FDEBUG_TXSTALL_LSB+sm);
    if (!draining) {
        if (dma_channel_is_busy(dma)) return;
        // Clear a possibly stale stall only after DMA has supplied the final
        // word. A new TXSTALL then proves FIFO AND shift register have drained.
        pio->fdebug=stalled; // write-one-to-clear
        draining=true;
        return;
    }
    if (!(pio->fdebug&stalled)) return;
    finished_at=time_us_64();
    ready_at=pad_mode ? finished_at+PIXEL_PAD_RESET_US : next_start_time(finished_at);
    in_flight=false;
    draining=false;
}

bool pixels_submit(const rgb_t *frame) {
    pixels_poll();
    if (in_flight || time_us_64() < ready_at) return false;
    for (unsigned i = 0; i < PIXEL_COUNT; ++i) {
        words[i] = ((uint32_t)frame[i].g << 24) | ((uint32_t)frame[i].r << 16) |
                   ((uint32_t)frame[i].b << 8);
    }
    dma_channel_set_read_addr(dma, words, false);
    dma_channel_set_trans_count(dma, PIXEL_COUNT, true);
    in_flight = true;
    return true;
}

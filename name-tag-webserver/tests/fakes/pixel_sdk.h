#ifndef TEST_PIXEL_SDK_H
#define TEST_PIXEL_SDK_H
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "board_config.h"
typedef struct { uint32_t txf[4]; uint32_t fdebug; } pio_hw_t;
#define PIO_FDEBUG_TXSTALL_LSB 24
typedef pio_hw_t *PIO;
extern pio_hw_t fake_pio;
#define pio0 (&fake_pio)
#define PIO_FIFO_JOIN_TX 1
#define DMA_SIZE_32 2
#define clk_sys 0
typedef struct { unsigned pin, shift, join; float divider; } pio_sm_config;
typedef struct { bool read_increment, write_increment; unsigned size, dreq; } dma_channel_config;
typedef struct {
    pio_sm_config pio;
    dma_channel_config dma;
    const uint32_t *source;
    unsigned count, starts;
    bool busy, enabled;
} pixel_sdk_t;
extern pixel_sdk_t fake_pixels;
static inline unsigned pio_claim_unused_sm(PIO p, bool required) { assert(p == pio0 && required); return 0; }
static inline unsigned pio_add_program(PIO p, const void *program) { assert(p == pio0 && program); return 0; }
static inline void pio_gpio_init(PIO p, unsigned pin) { assert(p == pio0 && pin == PIXEL_PIN); }
static inline void pio_sm_set_consecutive_pindirs(PIO p, unsigned sm, unsigned pin, unsigned count, bool out) {
    assert(p == pio0 && sm == 0 && pin == PIXEL_PIN && count == 1 && out);
}
static inline void sm_config_set_sideset_pins(pio_sm_config *c, unsigned pin) { c->pin = pin; }
static inline void sm_config_set_out_shift(pio_sm_config *c, bool right, bool autopull, unsigned bits) {
    assert(!right && autopull); c->shift = bits;
}
static inline void sm_config_set_fifo_join(pio_sm_config *c, unsigned join) { c->join = join; }
static inline void sm_config_set_clkdiv(pio_sm_config *c, float divider) { c->divider = divider; }
static inline void pio_sm_init(PIO p, unsigned sm, unsigned offset, const pio_sm_config *c) {
    assert(p == pio0 && sm == 0 && offset == 0); fake_pixels.pio = *c;
}
static inline void pio_sm_set_enabled(PIO p, unsigned sm, bool enabled) {
    assert(p == pio0 && sm == 0); fake_pixels.enabled = enabled;
}
static inline unsigned pio_get_dreq(PIO p, unsigned sm, bool tx) { assert(p == pio0 && sm == 0 && tx); return 7; }
static inline int dma_claim_unused_channel(bool required) { assert(required); return 1; }
static inline dma_channel_config dma_channel_get_default_config(int channel) {
    assert(channel == 1); return (dma_channel_config){0};
}
static inline void channel_config_set_transfer_data_size(dma_channel_config *c, unsigned size) { c->size = size; }
static inline void channel_config_set_read_increment(dma_channel_config *c, bool value) { c->read_increment = value; }
static inline void channel_config_set_write_increment(dma_channel_config *c, bool value) { c->write_increment = value; }
static inline void channel_config_set_dreq(dma_channel_config *c, unsigned dreq) { c->dreq = dreq; }
static inline void dma_channel_configure(int channel, const dma_channel_config *c, volatile void *dest,
                                         const void *source, unsigned count, bool start) {
    assert(channel == 1 && dest == &fake_pio.txf[0] && !start);
    fake_pixels.dma = *c; fake_pixels.source = source; fake_pixels.count = count;
}
static inline bool dma_channel_is_busy(int channel) { assert(channel == 1); return fake_pixels.busy; }
static inline void dma_channel_set_read_addr(int channel, const void *source, bool start) {
    assert(channel == 1 && !start); fake_pixels.source = source;
}
static inline void dma_channel_set_trans_count(int channel, unsigned count, bool start) {
    assert(channel == 1 && start); fake_pixels.count = count; fake_pixels.busy = true; ++fake_pixels.starts;
}
static inline unsigned clock_get_hz(unsigned clock) { assert(clock == clk_sys); return 125000000; }
#endif
